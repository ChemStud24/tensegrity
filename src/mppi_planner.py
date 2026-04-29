#!/usr/bin/env python3
"""
MPPI planner node. Requires the tensegrity catkin workspace to be built and sourced.

From the tensegrity package directory:
  1. If you haven't already: ./setup_workspace.sh
  2. source ~/catkin_ws_tensegrity/devel/setup.bash
  3. python src/mppi_planner.py
     (or run from workspace root: rosrun tensegrity mppi_planner.py)
"""
import os
import time
import copy
import numpy as np
import pickle
import rospy
import rospkg
import rosnode
import logging
import threading
import json
from collections import deque
from geometry_msgs.msg import Point, PoseStamped
from scipy.spatial.transform import Rotation as R

try:
    from tensegrity.msg import ActionHybridMPPI, PoseStateStamped, State
except ModuleNotFoundError:
    print(
        "ModuleNotFoundError: No module named 'tensegrity'.\n"
        "Source the catkin workspace first:\n"
        "  source ~/catkin_ws_tensegrity/devel/setup.bash\n"
        "If the workspace does not exist, run from the package root:\n"
        "  ./setup_workspace.sh"
    )
    raise
from scipy.spatial.transform import Rotation as R

# Try to import perception services, create mocks if not available
try:
    from tensegrity.srv import GetPose, GetPoseRequest, GetPoseResponse, GetBarHeight
except ImportError:
    print("Warning: tensegrity services not available in mppi_planner, using mocks")
    from unittest.mock import MagicMock
    GetPose = GetPoseRequest = GetPoseResponse = GetBarHeight = MagicMock
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from Tensegrity_model_inputs import *
from gnn_simulator.model_predictive_control.hybrid_planner import HybridAStarMPPIPlanner
import torch

torch.backends.cuda.matmul.allow_tf32 = True
torch._dynamo.config.cache_size_limit = 512


class PlannerMPPI:

	def __init__(self, start, goal, boundary, init_cable_lengths, obstacles=[], grid_step=10, tol=0.1,
	            planner_params={}, pose_queue_size=20):

		start = (start[0] / 1000, start[1] / 1000, start[2])
		goal = (goal[0] / 1000, goal[1] / 1000, goal[2])
		boundary = (boundary[0] / 1000, boundary[1] / 1000, boundary[2] / 1000, boundary[3] / 1000)
		obstacles = [(obs[0] / 1000, obs[1] / 1000, obs[2] / 1000, obs[3] / 1000) for obs in obstacles]
		grid_step = grid_step / 1000
		# init_cable_lengths = [cable_length / 1000 for cable_length in init_cable_lengths]

		self.grid_step = grid_step
		self.tol = tol

		pub_topic = '/action_mppi_msg'
		self.pub = rospy.Publisher(pub_topic, ActionHybridMPPI, queue_size=10)

		sub_topic = '/state_msg'
		self.sub = rospy.Subscriber(sub_topic, State, self.state_callback)
		self.ready_for_next_action = False

		# Subscriber for pose messages with queue
		pose_topic = '/pose_msg'
		self.pose_queue_size = pose_queue_size
		self.pose_queue = deque([], maxlen=pose_queue_size)
		self.pose_queue_lock = threading.Lock()
		self.pose_sub = rospy.Subscriber(pose_topic, PoseStateStamped, self.pose_callback, queue_size=pose_queue_size)

		# Setup logger
		self.logger = logging.getLogger('PlannerMPPI')
		self.logger.setLevel(logging.INFO)
		if not self.logger.handlers:
			handler = logging.StreamHandler()
			formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
			handler.setFormatter(formatter)
			self.logger.addHandler(handler)

		# Store mppi_params for later access
		self.mppi_params = planner_params['mppi_params']

		# Convert obstacles from 2-tuple (center) to 4-tuple (bounding box) format if needed
		# converted_obstacles = self._convert_obstacles(obstacles, obstacle_size)
		converted_obstacles = obstacles

		# Initialize HybridAStarMPPIPlanner
		self.planner = HybridAStarMPPIPlanner(
			obstacles=tuple(tuple(o * 10.0 for o in obs) for obs in converted_obstacles),
			boundary=tuple(b * 10.0 for b in boundary),
			goal=(goal[0] * 10.0, goal[1] * 10.0, goal[2]),
			grid_step=[grid_step * 10.0, grid_step * 10.0],
			tol=tol * 10.0,
			logger=self.logger,
			**planner_params
		)
		self.tol = tol	

		self.init_rest_lens = self._compute_init_rest_lens(init_cable_lengths)
		if planner_params['mppi_params']['torch_compile']:
			self._init_dummy_torch_compile()
		# self.init_rest_lens = self._compute_init_rest_lens_mjc(init_cable_lengths)

		# Used for restarting sim
		self.start_up_sim_state = self.planner.mppi_controller.sim.get_curr_state()
		self.start_up_rest_lens = torch.hstack([
			c.rest_length for c in self.planner.mppi_controller.sim.robot.actuated_cables.values()
		])
		self.disconnect_time = 0

		# Store configuration
		self.current_state = start
		self.goal = goal
		self.boundary = boundary
		self.obstacles = obstacles
		self.heur_type = planner_params['astar_params']['heur_type']

		# Store pose history for MPPI planner
		self.prev_pose_and_t = deque([], maxlen=20)
		self.env_dt = 0.01

		# Initialize pose from start state
		# self._initialize_pose_from_state(start)

		self.count = 0
		self.COMs = []
		self.endcaps = []
		self.PAs = []

	def _init_dummy_torch_compile(self):
		self.planner.mppi_controller.sim.run_compile()
		init_state = self.planner.mppi_controller.sim.get_curr_state()
		init_pose = init_state.reshape(-1, 13, 1)[:, :7].reshape(init_state.shape[0], -1, 1)
		poses = [(init_pose.clone(), 0.0), (init_pose.clone(), 0.01)]
		curr_rest_lens = torch.hstack([
			c.rest_length for c in self.planner.mppi_controller.sim.robot.actuated_cables.values()
		]).cpu().numpy()
		motor_speeds = np.zeros_like(curr_rest_lens)

		for _ in range(3):
			_ = self.planner._plan_mppi_only(
				poses, curr_rest_lens.copy(), motor_speeds.copy()
			) # Trigger torch compile with dummy planning

		self.logger.info("Dummy JIT compile done.")

		# reset rest lengths to original values
		curr_rest_lens = torch.from_numpy(curr_rest_lens).to(init_state.device)
		for i, c in enumerate(self.planner.mppi_controller.sim.robot.actuated_cables.values()):
			c.set_rest_length(curr_rest_lens[:, i:i+1])

	def _convert_obstacles(self, obstacles, obstacle_size):
		"""Convert obstacles from 2-tuple (center) to 4-tuple (bounding box) format.

		Args:
			obstacles: List of obstacles, either as 2-tuples (x, y) or 4-tuples (x_min, x_max, y_min, y_max)
			obstacle_size: (width, height) of obstacles when given as 2-tuples

		Returns:
			List of obstacles in 4-tuple (x_min, x_max, y_min, y_max) format
		"""
		if not obstacles:
			return []

		converted = []
		obs_w, obs_h = obstacle_size

		for obs in obstacles:
			if len(obs) == 2:
				# Convert from center (x, y) to bounding box (x_min, x_max, y_min, y_max)
				ox, oy = obs
				converted.append((ox - obs_w/2, ox + obs_w/2, oy - obs_h/2, oy + obs_h/2))
			elif len(obs) == 4:
				# Already in bounding box format
				converted.append(obs)
			else:
				raise ValueError(f"Obstacle must be 2-tuple (center) or 4-tuple (bounding box), got {len(obs)}-tuple")

		return converted

	def _compute_init_rest_lens(self, init_cable_lengths, min_length=80, range_=120, tol=0.05):
		sim = self.planner.mppi_controller.sim
		init_cable_lengths = torch.tensor(
			init_cable_lengths,
			dtype=sim.dtype,
			device=sim.device
		).reshape(1, -1, 1)

		target_gaits = ((init_cable_lengths - min_length) / range_).reshape(1, -1, 1)
		sim.run_target_gait(
			sim.get_curr_state(),
			target_gaits,
			pid_min_length=min_length,
			pid_ranges=[range_] * init_cable_lengths.shape[1],
			pid_tol=tol,
			state_to_graph_kwargs={'dataset_idx': 8},
			max_steps=1000
		)

		rest_lens = torch.hstack([
			c.rest_length for c in sim.robot.actuated_cables.values()
		]).flatten().cpu().numpy() / 10

		return rest_lens

	def _compute_init_rest_lens_mjc(self, init_cable_lengths, min_length=80, range_=120, tol=0.1):
		sim = self.planner.mppi_controller.sim
		scale = sim.scale_factor  # typically 10.0

		# Scale cable lengths to MuJoCo units and compute target gaits
		scaled_cable_lengths = np.array(init_cable_lengths, dtype=np.float64) * scale
		target_gait = ((scaled_cable_lengths * 100 - min_length) / range_).tolist()

		# Configure PIDs on the sim
		for i, pid in enumerate(sim.pids):
			pid.min_length = min_length / 100.0
			pid.RANGE = range_ / 100.0
			pid.tol = tol
			pid.reset()

		# Run PID convergence loop (mirrors ThreeBarTensegrityMuJoCoSimulator.run_w_target_gaits)
		import mujoco
		max_steps = int(10.0 / sim.dt)
		controls = [1.0]  # non-zero to enter loop
		step = 0

		while any(c != 0 for c in controls) and step < max_steps:
			step += 1
			mujoco.mj_forward(sim.mjc_model, sim.mjc_data)

			if step % int(sim.pid_freq / sim.dt) == 0 or step == 1:
				controls = []
				for i in range(len(target_gait)):
					pid = sim.pids[i]
					rest_length = sim.mjc_model.tendon_lengthspring[i, 0]
					key = sim.cable_map[i] if hasattr(sim, "cable_map") and sim.cable_map else i
					s0 = sim.mjc_data.sensor(f"pos_{sim.cable_sites[key][0]}").data
					s1 = sim.mjc_data.sensor(f"pos_{sim.cable_sites[key][1]}").data
					curr_length = np.linalg.norm(s1 - s0)

					ctrl, _ = pid.update_control_by_target_gait(
						curr_length, target_gait[i], rest_length
					)
					controls.append(ctrl)

			sim.sim_step(np.array(controls).reshape(1, -1))

		# Read converged rest lengths, unscale back to meters
		rest_lens = sim.mjc_model.tendon_lengthspring[:sim.n_actuators, 0].copy() / scale
		return rest_lens

	def state_callback(self, msg):
		"""Callback for state messages - runs asynchronously from main callback
		Receives State message containing the current state of the robot.
		"""
		if msg.prev_action:
			self.ready_for_next_action = True

	def compute_and_publish_control(self):
		"""Continuously compute controls based on latest pose_queue data"""
		if not self.ready_for_next_action:
			# self.logger.info("Not ready for next action")
			return

		# Get all latest poses from the queue
		latest_poses = self.get_latest_poses()

		# Only run planning if poses are available
		if len(latest_poses) == 0:
			self.logger.info("No poses available")
			return

		# Check if we've reached the goal
		last_pose = latest_poses[-1][0].reshape(3, 7)  # (3 rods, 7D)
		com_xy = last_pose[:, :2].mean(axis=0)  # centroid x, y
		dist_to_goal = np.linalg.norm(com_xy - np.array(self.goal[:2]))
		if dist_to_goal < self.tol:
			self.logger.info(f"Goal reached (dist={dist_to_goal:.4f} < tol={self.tol})")
			action_msg = ActionHybridMPPI()
			action_msg.header.stamp = rospy.Time.now()
			action_msg.control_type = 'astar'
			action_msg.primitive_actions = ['rest']
			action_msg.control_interval = -1.0
			self.pub.publish(action_msg)
			self.ready_for_next_action = False
			return
		else:
			self.logger.info(f"Goal not reached, dist_to_goal: {dist_to_goal:.4f}")

		# Update prev_pose_and_t from queue data
		# Each queue entry contains: (concatenated_pose_21D, timestamp, encoder_lengths, motor_speeds)
		# The concatenated pose (21D = 3*7D) represents 3 poses at one time step
		prev_pose_and_t = []
		encoder_lengths_list = []
		for concatenated_pose, timestamp, encoder_lengths, motor_speeds in latest_poses:
			# Reshape concatenated pose (21D) to (3, 7) format expected by planner
			# This represents 3 poses (rods) at this time step
			pose = concatenated_pose.reshape(3, 7)
			prev_pose_and_t.append((pose, timestamp))
			encoder_lengths_list.append(encoder_lengths)

		# Get encoder_lengths and motor_speeds from the most recent queue entry
		_, _, latest_encoder_lengths, latest_motor_speeds = latest_poses[-1]

		# Use the latest encoder_lengths as current cable rest lengths
		# encoder_lengths from the message represent absolute cable lengths in meters
		if len(latest_encoder_lengths) > 0:
			rest_lens = self.init_rest_lens + np.array(latest_encoder_lengths)
		motor_speeds = latest_motor_speeds if len(latest_motor_speeds) > 0 else np.zeros_like(self.init_rest_lens)

		# Plan using MPPI planner
		# try:
		# Scale dimensions by 10x
		scaled_rest_lens = rest_lens.copy() * 10.0
		scaled_prev_pose_and_t = []
		for pose, timestamp in prev_pose_and_t:
			scaled_pose = pose.reshape(-1, 7)
			scaled_pose[:, :3] *= 10.0
			scaled_prev_pose_and_t.append((scaled_pose, timestamp))
		with torch.no_grad():
			step_type, actions, _ = self.planner.plan(
				scaled_prev_pose_and_t, scaled_rest_lens, motor_speeds
			)
		# except Exception as e:
		# 	rospy.logerr(f"Planning failed: {e}")
		# 	return

		# publish results
		action_msg = ActionHybridMPPI()
		action_msg.header.stamp = rospy.Time.now()
		action_msg.control_type = step_type
		if step_type == 'mppi':
			ctrl_int = self.planner.mppi_controller.ctrl_interval
			action_msg.mppi_actions = actions[0, :, ::ctrl_int].T.flatten().tolist()
			action_msg.control_interval = float(ctrl_int * self.planner.mppi_controller.dt)
		else:
			# primitive_actions is a string[] - wrap in list if needed
			prim_action = actions[0] if 'cw' in actions[0] else str(actions[1]) + '_' + str(actions[2])
			action_msg.primitive_actions = [prim_action] if isinstance(prim_action, str) else prim_action
			action_msg.control_interval = -1.0  # not used for primitive actions
			self.ready_for_next_action = False

		self.pub.publish(action_msg)
		self.logger.info(f"Published action")

	def compute_end_pts(self, stacked_pose):
		poses = [stacked_pose[i:i+1] for i in range(stacked_pose.shape[0])]
		end_pts = []
		for pose in poses:
			pose = pose.reshape(1, 7)
			q = np.hstack([pose[:, 4:], pose[:, 3:4]]).flatten()
			rot_mat = R.from_quat(q).as_matrix()
			unit_vector = rot_mat[:, 2:3]
			end_pt0 = pose[:, :3] - 0.5 * unit_vector * 0.295
			end_pt1 = pose[:, :3] + 0.5 * unit_vector * 0.295
			end_pts.append(end_pt0)
			end_pts.append(end_pt1)
		return np.vstack(end_pts)

	def pose_callback(self, msg):
		"""Callback for pose messages - runs asynchronously from main callback
		Receives PoseStateStamped message containing a Pose[] array (expected length 3),
		encoder_counts, and motor_speeds.
		"""
		# Convert geometry_msgs/Pose to numpy [x, y, z, qw, qx, qy, qz]
		def pose_to_array(pose):
			return np.array(
				[
					pose.position.x,
					pose.position.y,
					pose.position.z,
					pose.orientation.w,
					pose.orientation.x,
					pose.orientation.y,
					pose.orientation.z,
				],
				dtype=np.float64,
			)

		# Expect 3 poses (one per rod). If fewer are present, pad with zeros; if more, truncate.
		# print(f"Received pose message with {msg.poses} poses")
		# quit()
		poses = list(msg.poses) if msg.poses else []
		pose_arrays = [pose_to_array(p) for p in poses]
		concatenated_pose = np.vstack(pose_arrays)  # Should be 21D (3 * 7)

		end_pts = self.compute_end_pts(concatenated_pose)
		min_z = end_pts[:, 2].min(axis=0).item()
		concatenated_pose[:, 2] += 0.0175 - min_z

		end_pts = self.compute_end_pts(concatenated_pose)
		concatenated_pose = concatenated_pose.flatten()

		# Timestamp from header
		timestamp = msg.header.stamp.to_sec()

		encoder_lengths = np.array(msg.encoder_lengths, dtype=np.float64) / 1000
		motor_speeds = np.array(msg.motor_speeds, dtype=np.float64)
		
		# Thread-safe append to queue
		# Store: (concatenated_pose, timestamp, encoder_lengths, motor_speeds)
		with self.pose_queue_lock:
			self.pose_queue.append((concatenated_pose, timestamp, encoder_lengths, motor_speeds))
	
	def get_latest_poses(self, n=None):
		"""Get the latest n poses from the queue, or all if n is None.
		Each pose is a concatenation of 3 individual poses (21D: 3 * 7D).
		Returns list of tuples: (concatenated_pose_array, timestamp, encoder_counts, motor_speeds)
		"""
		with self.pose_queue_lock:
			if n is None:
				return list(self.pose_queue)
			else:
				return list(self.pose_queue)[-n:] if len(self.pose_queue) >= n else list(self.pose_queue)

	def _convert_mppi_actions_to_sequence(self, actions):
		"""Convert MPPI actions (numpy array) to action sequence format"""
		# This is a placeholder - actual conversion depends on your action format
		# MPPI actions are typically motor commands, need to convert to primitive names
		# For now, return a dummy sequence
		return ['100_100']  # Replace with actual conversion logic

	def run(self):
		"""Continuously compute and publish controls based on latest pose_queue"""
		rospy.loginfo("MPPI planner started, waiting for controller to connect...")
		was_connected = False
		while not rospy.is_shutdown():
			if self.pub.get_num_connections() > 0:
				if not was_connected:
					self.logger.info("Controller connected, starting planning loop")
					was_connected = True
	
				self.disconnect_time = 0
				self.ready_for_next_action = True
				self.compute_and_publish_control()
			elif was_connected and self.pub.get_num_connections() == 0:
				if self.disconnect_time == 0:
					self.disconnect_time = time.time()
					self.logger.warning("Controller disconnected, if not reconnected within 5 secs, the planner will reset, starting timer...")
				elif time.time() - self.disconnect_time > 5:
					self.logger.warning("Controller disconnected for >5s, resetting sim state...")
					motor_speeds = np.zeros(self.start_up_rest_lens.shape)
					self.planner.reset_sim_state(
						self.start_up_sim_state.clone(),
						motor_speeds,
						self.start_up_rest_lens.clone()
					)
					self.prev_pose_and_t.clear()
					was_connected = False
					self.disconnect_time = 0
			rospy.sleep(0.5)


if __name__ == '__main__':
	# Measurements in mm
	# start = (-700, 0, 0)
	# goal = (2700, -1200, 0)
	# obstacles = ((0,250, -650, 800), (1550,1800, -1600, -600))
	# boundary = (-1000, 3500, -1800, 800)

	start = (400, 1300, -np.pi/2)
	goal = (1900, 500, -np.pi/2)
	# obstacles = ((550,650, 1200,2400), (1750,1850, 600,1600))
	# obstacles = ((550,650, 300,500), (1750,1850, 1300,1500))
	obstacles = ((460,730,430,800),(1330,1600,1030,1400))
	boundary = (200, 2100, 300, 1400)

	# start = (1, 1.4, -np.pi/2)
	# goal = (1.9, 0.4, -np.pi/2)
	# obstacles = ((1.6, 0.5), (1.6, 0.7), (1.6, 0.8), (1.6, 0.7))
	# boundary = (-3, 2.2, 0.1, 1.7)

	def rotation_angle_from_matrix( matrix):
		"""
		Calculate the rotation angle in radians from a 2D rotation matrix using NumPy.

		Args:
			matrix (numpy.ndarray): A 2x2 rotation matrix
									[[cos(theta), -sin(theta)],
									[sin(theta),  cos(theta)]]

		Returns:
			float: The rotation angle in radians.
		"""
		# Ensure the input is a NumPy array
		matrix = np.array(matrix)
		
		# Extract sine and cosine from the matrix
		cos_theta = matrix[0, 0]
		sin_theta = matrix[1, 0]

		# Calculate the angle using arctan2
		angle_radians = np.arctan2(sin_theta, cos_theta)
		if angle_radians < 0: angle_radians += 2*np.pi

		return float(angle_radians)

	_script_dir = os.path.dirname(os.path.abspath(__file__))
	filepath = os.path.join(os.path.dirname(_script_dir), 'calibration', 'new_platform_transformation_table.pkl')
	with open(filepath,'rb') as f:
		action_dict = pickle.load(f)
	

	primitives = ['100_100','120_120','140_140','100_120','120_100',
			   '100_140','140_100','120_140','140_120','ccw','cw']

	primitive_workspace = []
	for prim in primitives:
		full_prim = action_dict[prim+"__"+prim]
		angle = rotation_angle_from_matrix(full_prim[0])
		simple_prim = [float(full_prim[1][0]), float(full_prim[1][1]),angle]
		primitive_workspace.append(simple_prim)	

	# astar_params = {
	# 	'gaits': [
	# 		('roll', 100, 100),
	# 		('roll', 120, 120),
	# 		('roll', 140, 140),
	# 		('roll', 100, 120),
	# 		('roll', 120, 100),
	# 		('roll', 100, 140),
	# 		('roll', 140, 100),
	# 		('roll', 120, 140),
	# 		('roll', 140, 120),
	# 		('ccw', 100, 100),
	# 		('cw', 100, 100)
			
	# 	],
	# 	'gait_deltas': primitive_workspace,
	# 	'heur_type': 'wave',
	# 	'repeat_tol': 0.7,
	# }
	# MPPI and A* parameters (these should be configured based on your needs)
	# astar_params = {
	# 	'gaits': [
	# 		('cw', 120, 120),
	# 		('ccw', 100, 100),
	# 		('roll', 100, 100),
	# 		('roll', 100, 120),
	# 		('roll', 100, 140),
	# 		('roll', 120, 100),
	# 		('roll', 120, 120),
	# 		('roll', 120, 140),
	# 		('roll', 140, 100),
	# 		('roll', 140, 120),
	# 		('roll', 140, 140)
	# 	],
	# 	'gait_deltas': [
	# 		(-0.40565651655197144, -1.0375710725784302, -0.5563808083534241),
    #         (0.41965270042419434, 0.34847545623779297, -0.07213731110095978),
    #         (-0.20067229866981506, -1.7011791467666626, -0.2842606008052826),
    #         (-0.05496932938694954, -1.8011555671691895, -0.2927650809288025),
    #         (0.11246095597743988, -1.9315996170043945, -0.29069265723228455),
    #         (-0.2660592198371887, -1.7847661972045898, -0.2607172429561615),
    #         (-0.1397669017314911, -1.876606822013855, -0.22433887422084808),
    #         (0.08572909981012344, -1.9767913818359375, -0.23118142783641815),
    #         (-0.2793423533439636, -1.6882871389389038, -0.2550457715988159),
    #         (-0.16679707169532776, -1.8680886030197144, -0.22707201540470123),
    #         (-0.05866171792149544, -2.0437886714935303, -0.1928362399339676)
	# 	],
	# 	'heur_type': 'wave',
	# 	'repeat_tol': 0.4,
	# }
	gait_deltas = [
			(-0.37157151103019714, -1.1484227180480957, -0.454834908246994),
            (0.6893242597579956, 0.6269674897193909, 0.2180231213569641),
            (-0.1630459725856781, -1.6645100116729736, -0.2102857530117035),
            (-0.027910035103559494, -1.7374625205993652, -0.2437712699174881),
            (0.2542741298675537, -1.8830678462982178, -0.21396899223327637),
            (-0.2743910253047943, -1.642714023590088, -0.1737537384033203),
            (-0.08029751479625702, -1.8038218021392822, -0.16179166734218597),
            (0.15351246297359467, -1.901036024093628, -0.14742010831832886),
            (-0.4188455045223236, -1.7940902709960938, -0.1983223855495453),
            (-0.19466280937194824, -1.8823869228363037, -0.18343089520931244),
            (0.043375056236982346, -2.0279226303100586, -0.16346679627895355)
		]
	gait_deltas = [(g[1] * -1.0, g[0], g[2]) for g in gait_deltas]

	astar_params = {
		'gaits': [
			('cw', 120, 120),
			('ccw', 100, 100),
			('roll', 100, 100),
			('roll', 100, 120),
			('roll', 100, 140),
			('roll', 120, 100),
			('roll', 120, 120),
			('roll', 120, 140),
			('roll', 140, 100),
			('roll', 140, 120),
			('roll', 140, 140)
		],
		'gait_deltas': gait_deltas,
		'heur_type': 'wave',
		'repeat_tol': 0.4,
	}
	_repo_root = os.path.dirname(_script_dir)
	_model_path = os.path.join(_repo_root, '../data_sets/tensegrity_real_datasets/new_platform_models/3bar_co_train_high_fric_real_dl', 'best_rollout_model.pt')
	mppi_params = {
		"sim": _model_path,
		'strategy': 'min',
		'device': 'cuda',
		'cost_weights': (1.0, 0.0, 0.0),
		'ctrl_interval': 0.5,
		'horizon': 2.0,
		'n_samples': 100,
		'use_motion_prim_heuristic': False,
		'torch_compile': True
	}
	
	planner_params = {
		'planner_type': 'mppi_turn_prims',  # astar_only or mppi_turn_prims
		'mppi_idle_time': 1e10,#change for astar only: -1e10, for mppi: 1e10
        'mppi_idle_dist': 1.0,
		'mppi_params': mppi_params,
		'astar_params': astar_params,
	}

	# Initial cable lengths (in mm) - adjust based on your robot's initial state
	# This should match the actual initial cable lengths of the robot
	init_cable_lengths = [180, 180, 180, 180, 180, 180]

	rospy.init_node('mppi_planner')
	with torch.no_grad():
		planner = PlannerMPPI(start, goal, boundary, init_cable_lengths, obstacles=obstacles, 
							planner_params=planner_params)
		# rate = rospy.Rate(50)
		planner.run()

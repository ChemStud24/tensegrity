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

class PlannerMPPI:

	def __init__(self, start, goal, boundary, init_cable_lengths, obstacles=[], grid_step=10, tol=0.1,
	             planner_params={}, pose_queue_size=20):

		start = (start[0] / 1000, start[1] / 1000, start[2])
		goal = (goal[0] / 1000, goal[1] / 1000, goal[2])
		boundary = (boundary[0] / 1000, boundary[1] / 1000, boundary[2] / 1000, boundary[3] / 1000)
		obstacles = [(obs[0] / 1000, obs[1] / 1000, obs[2] / 1000, obs[3] / 1000) for obs in obstacles]
		grid_step = grid_step / 1000

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
		# Make a copy of astar_params to avoid modifying the original
		astar_params_scaled = copy.deepcopy(planner_params['astar_params'])
		astar_params_scaled['gait_deltas'] = [(g[0] * 10.0, g[1] * 10.0, g[2]) for g in planner_params['astar_params']['gait_deltas']]
		self.astar_params = astar_params_scaled

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
		# self.init_rest_lens = self._compute_init_rest_lens_mjc(init_cable_lengths)

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
		init_cable_lengths = 10 * torch.tensor(
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

	def pose_callback(self, msg):
		"""Callback for pose messages - runs asynchronously from main callback
		Receives PoseStateStamped message containing a Pose[] array (expected length 3),
		encoder_counts, and motor_speeds.
		"""
		# Convert geometry_msgs/Pose to numpy [x, y, z, qw, qx, qy, qz]
		def pose_to_array(pose):
			return np.array(
				[
					pose.position.x / 1000,
					pose.position.y / 1000,
					pose.position.z / 1000,
					pose.orientation.w,
					pose.orientation.x,
					pose.orientation.y,
					pose.orientation.z,
				],
				dtype=np.float64,
			)

		# Expect 3 poses (one per rod). If fewer are present, pad with zeros; if more, truncate.
		poses = list(msg.poses) if msg.poses else []
		pose_arrays = [pose_to_array(p) for p in poses]
		concatenated_pose = np.concatenate(pose_arrays, axis=0)  # Should be 21D (3 * 7)

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
		while not rospy.is_shutdown():
			if self.pub.get_num_connections() > 0:
				self.ready_for_next_action = True
				self.logger.info("Controller connected, starting planning loop")
				break
			rospy.sleep(0.5)

		while not rospy.is_shutdown():
			self.compute_and_publish_control()


if __name__ == '__main__':
	# Measurements in mm
	start = (700, 0, 0)
	goal = (-2700, -1200, 0)
	obstacles = ((-250, 0, -650, 800), (-1800, -1550, -1600, -600))
	boundary = (-3500, 1000, -1800, 800)

	# MPPI and A* parameters (these should be configured based on your needs)
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
		'gait_deltas': [
			(-0.40565651655197144, -1.0375710725784302, -0.5563808083534241),
            (0.41965270042419434, 0.34847545623779297, -0.07213731110095978),
            (-0.20067229866981506, -1.7011791467666626, -0.2842606008052826),
            (-0.05496932938694954, -1.8011555671691895, -0.2927650809288025),
            (0.11246095597743988, -1.9315996170043945, -0.29069265723228455),
            (-0.2660592198371887, -1.7847661972045898, -0.2607172429561615),
            (-0.1397669017314911, -1.876606822013855, -0.22433887422084808),
            (0.08572909981012344, -1.9767913818359375, -0.23118142783641815),
            (-0.2793423533439636, -1.6882871389389038, -0.2550457715988159),
            (-0.16679707169532776, -1.8680886030197144, -0.22707201540470123),
            (-0.05866171792149544, -2.0437886714935303, -0.1928362399339676)
		],
		'heur_type': 'wave',
	}

	mppi_params = {
		"sim": "3bar_ds8_multi_8_mppi_turn_prims_v2.2/best_rollout_model.pt",
		'strategy': 'min',
		'device': 'cuda',
		'cost_weights': (1.0, 0.0, 0.0),
		'ctrl_interval': 1.0,
		'horizon': 4.0,
		'n_samples': 200,
		'use_motion_prim_heuristic': False,
	}
	
	planner_params = {
		'mppi_idle_time': 1e10,
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

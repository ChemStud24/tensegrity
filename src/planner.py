#!/usr/bin/env python3
import os
import numpy as np
import pickle
import rospy
import rospkg
import rosnode
from geometry_msgs.msg import Point
from tensegrity.msg import State, Action
from tensegrity_perception.srv import GetPose, GetPoseRequest, GetPoseResponse, GetBarHeight
from astar import astar
from scipy.spatial.transform import Rotation as R
from Tensegrity_model_inputs import *

class MotionPlanner:

	def __init__(self, start, goal, boundary, obstacles=[], heur_type = "dist"):
		sub_topic = '/state_msg'
		pub_topic = '/action_msg'
		self.sub = rospy.Subscriber(sub_topic,State,self.callback)
		self.pub = rospy.Publisher(pub_topic,Action,queue_size=10)

		# load state-action transition dictionary
		package_path = rospkg.RosPack().get_path('tensegrity')
		filepath = os.path.join(package_path,'calibration/legacy_motion_primitives.pkl')
		with open(filepath,'rb') as f:
			self.action_dict = pickle.load(f)
		# print(self.action_dict)
		# print(np.array(self.action_dict.values()))

		# ordered list of motion primitives
		self.primitives = ['100_100','120_120','140_140','100_120','120_100','100_140','140_100','120_140','140_120','ccw','cw']

		self.primitive_workspace = []
		for prim in self.primitives:
			full_prim = self.action_dict[prim+"__"+prim]
			angle = self.rotation_angle_from_matrix(full_prim[0])
			simple_prim = [float(full_prim[1][0]), float(full_prim[1][1]),angle]
			self.primitive_workspace.append(simple_prim)

		# define start, goal, and obstacles
		self.current_state = start
		self.goal = goal
		self.boundary = boundary
		self.obstacles = obstacles

		#dist is straight line distance to goal
		#wave is obstacle informed heuristic
		self.heur_type = heur_type

		#Tunable parameters
		self.obstacle_dim = (0.2,0.2)
		self.goal_tol = 0.1
		self.goal_rot_tol = np.pi/2
		self.repeat_tol = 0.04 #Won't resample states this close
		self.grid_step = 0.01 #Only used in wave heuristic

		# run A star planner for the known start, goal, and obstacles
		self.Astar()

		self.count = 0
		self.COMs = []
		self.endcaps = []

		# determine if the tracking service is available for feedback
		# if '/tracking_service' in rosnode.get_node_names():
		# 	self.closed_loop = True
		# else:
		# 	self.closed_loop = False

	def callback(self,msg):
		# receiving a state message means the robot is ready for the next action
		print('Action ' + str(self.count))

		# if tracking is available
		if '/tracking_service' in rosnode.get_node_names():
			# get pose to check if we deviated from the plan
			COM,axis,self.endcaps = self.get_pose()
		
			# if we did, re-run the planner
			print('I could replan here')
			if True: # replace this with a reasonable condition
				self.current_state = (float(COM[0]),float(COM[1]),float(np.arctan2(axis[1],axis[0])))
				self.Astar()

				# add current COM
				self.COMs = [COM] + [[x,y] for x,y,_ in self.expected_path]

		# publish results
		action_msg = Action()
		for act in self.action_sequence:
			action_msg.actions.append(act)
		for x,y in self.COMs:
			point = Point()
			point.x = x
			point.y = y
			action_msg.COMs.append(point)
		for x,y,z in self.endcaps:
			point = Point()
			point.x = x
			point.y = y
			point.z = z
			action_msg.endcaps.append(point)
		self.pub.publish(action_msg)
		self.action_sequence.pop(0)
		self.count += 1
	
	def int_path_to_string_path(self, int_path):
		return [self.primitives[i] for i in int_path]
	
	def Astar(self):
		# put A star planning code here
		# as a stand-in, we just do ccw 15 times
		# plan = [9 for x in range(15)]
		# self.action_sequence = [self.primitives[p] for p in plan]
		self.expected_path, path = astar(self.current_state,self.goal, self.primitive_workspace, \
			tolerance=self.goal_tol, rot_tol=self.goal_rot_tol, obstacles=self.obstacles,\
			repeat_tol = self.repeat_tol, single_push=False, \
        	stochastic=False,heur_type="dist", boundary=self.boundary, obstacle_dims=self.obstacle_dim,\
			grid_step=self.grid_step)

		print('Path: ',path)
		
		# only update the path if we found a valid path
		if len(path) > 0:
			self.action_sequence = self.int_path_to_string_path(path)

	def run(self, rate):
		while not rospy.is_shutdown():
			rate.sleep()

	def get_pose(self):
		service_name = "get_pose"
		
		vectors = np.array([[0.0,0.0,0.0]])
		centers = []
		endcaps = []
		try:
			request = GetPoseRequest()
			get_pose_srv = rospy.ServiceProxy(service_name, GetPose)
			rospy.loginfo("Request sent. Waiting for response...")
			response: GetPoseResponse = get_pose_srv(request)
			rospy.loginfo(f"Got response. Request success: {response.success}")
			if response.success:
				for pose in response.poses:
					
					rotation_matrix = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z,pose.orientation.w]).as_matrix()
					
					unit_vector = rotation_matrix[:,2]
					center = [pose.position.x,pose.position.y,pose.position.z]
					endcaps.append(np.array(center) + L/2*unit_vector)
					endcaps.append(np.array(center) - L/2*unit_vector)
					
					centers.append(center)
					vectors += unit_vector
			COM = np.mean(np.array(centers),axis=0)
			principal_axis = vectors/np.linalg.norm(vectors)
			endcaps = np.array(endcaps)/100 # convert to meters
			# reformat COM and PA
			COM = np.reshape(COM[0:2],(2,1))
			principal_axis = principal_axis[:,0:2]
			principal_axis = np.reshape(principal_axis,(2,1))
		except rospy.ServiceException as e:
			rospy.loginfo(f"Service call failed: {e}")
		return COM, principal_axis, endcaps
	
	def rotation_angle_from_matrix(self, matrix):
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


if __name__ == '__main__':
	# start = (0.1, -1.0, np.pi/2)
	# goal = (-2, -0.2, np.pi/2)
	# obstacles = ((-0.3,-0.2), (-0.3,-0.6), (-1.5, -1.0), (-1.5,-0.6))
	# boundary = (-3, 1, -1.4, 0.2)

	start = (-0.15, 1.1, -np.pi/2)
	goal = (2, 0.2, -np.pi/2)
	obstacles = ((0.3,0.2), (0.3,0.6), (1.5, 1.0), (1.5,0.6))
	boundary = (-1, 3, -0.2, 1.4)

	rospy.init_node('motion_planner')
	planner = MotionPlanner(start, goal, boundary, obstacles)
	rate = rospy.Rate(30)
	planner.run(rate)
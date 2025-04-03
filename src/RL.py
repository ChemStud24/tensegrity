#!/usr/bin/env python3
import os
import numpy as np
import rospy
import rospkg
import rosnode
from geometry_msgs.msg import Point
from tensegrity.msg import State, Action
from tensegrity_perception.srv import GetPose, GetPoseRequest, GetPoseResponse
from scipy.spatial.transform import Rotation as R
from Tensegrity_model_inputs import *
from policy import ctrl_policy


class MotionPlanner:

	def __init__(self,fps=7,path_to_model="actor_4000000.pth"):
		sub_topic = '/state_msg'
		pub_topic = '/action_msg'
		self.sub = rospy.Subscriber(sub_topic,State,self.callback)
		self.pub = rospy.Publisher(pub_topic,Action,queue_size=10)

		# define the control policy from the RL model
		self.CTRL = ctrl_policy(fps,path_to_model)
		self.count = 0

	def callback(self,msg):
		# receiving a state message means the robot is ready for the next action
		print('Action ' + str(self.count))

		# get pose
		COM,axis,self.endcaps = self.get_pose()
	
		# get next action
		action = self.CTRL.get_action(self.endcaps)

		# publish results
		action_msg = Action()
		self.pub.publish(action_msg)
		self.count += 1

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

if __name__ == '__main__':
	rospy.init_node('motion_planner')
	planner = MotionPlanner()
	rate = rospy.Rate(30)
	planner.run(rate)
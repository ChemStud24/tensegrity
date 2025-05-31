#!/usr/bin/env python3
import os
import cv2
import sys
import json
import rospy
import rospkg
import numpy as np
from math import ceil, sqrt
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class Visualizer:
	def __init__(self,trajectory,cam_intr,cam_extr):
		# ROS node
		rospy.init_node('test_trajectory')
		sub_topic = '/rgb_images'
		pub_topic = '/trajectory_test'
		self.sub = rospy.Subscriber(sub_topic,Image,self.callback)
		self.trajectory_pub = rospy.Publisher(pub_topic,Image,queue_size=10)
		self.bridge = CvBridge()

		# save trajectory in pixel coordinates 
		self.trajectory = []
		H = np.array(cam_extr)
		E = np.zeros((4,4))
		E[:3,:3] = H[:3,:3].T
		E[:3,3:] = -np.matmul(H[:3,:3].T,H[:3,3:])
		E[3,3] = 1
		cam_intr = np.array(cam_intr)
		fx, fy = cam_intr[0, 0], cam_intr[1, 1]
		cx, cy = cam_intr[0, 2], cam_intr[1, 2]
		for X,Y in trajectory:
			this_point = np.array([[X],[Y],[0],[1]])
			XYZ = np.matmul(E,this_point)
			x = int(np.round((XYZ[0] * fx / XYZ[2]) + cx))
			y = int(np.round((XYZ[1] * fy / XYZ[2]) + cy))
			self.trajectory.append([x,y])

	def callback(self,msg):

		# get image from messagemsg.header.stamp
		traj_im = self.bridge.imgmsg_to_cv2(msg,'rgb8')

		# superimpose trajectory on image
		for i,p in enumerate(self.trajectory):
			x,y = p
			traj_im = cv2.circle(traj_im, (x,y), radius=2, color=(255*i/len(self.trajectory), 255, 0), thickness=-1)

		# publish superimposed image
		trajectory_im_msg = self.bridge.cv2_to_imgmsg(traj_im,'rgb8')
		trajectory_im_msg.header.stamp = msg.header.stamp
		self.trajectory_pub.publish(trajectory_im_msg)

def make_circle(center_point,radius,num_points=100,starting_angle=0,swept_angle=2*np.pi):
	# circle
	center_point = np.array(center_point)
	t = np.linspace(starting_angle,starting_angle+swept_angle,ceil(num_points))
	trajectory = np.array([[radius*np.cos(T) + center_point[0],radius*np.sin(T) + center_point[1]] for T in t])
	return trajectory

def make_square(center_point,edge_length,num_points=100):
	# circle
	center_point = np.array(center_point)
	vertices = [[center_point[0] - edge_length/2, center_point[1] - edge_length/2],[center_point[0] - edge_length/2, center_point[1] + edge_length/2],[center_point[0] + edge_length/2, center_point[1] + edge_length/2],[center_point[0] + edge_length/2, center_point[1] - edge_length/2]]
	pairs = [(vertices[0],vertices[1]),(vertices[1],vertices[2]),(vertices[2],vertices[3]),(vertices[3],vertices[0])]
	trajectory_sequence = [np.linspace(v1,v2,ceil(num_points/4)) for v1,v2 in pairs]
	return np.vstack([segment for segment in trajectory_sequence])

# normal obstacles
# starting_point = [-0.15,1.1]
# ending_point = [1.6,0.2]
# robot_length = 0.30
# edge_length = 0.2
# # obstacle_point_1 = [0.68,0.1]
# # obstacle_point_2 = [1.48,1.0]
# obstacles = [[0.3,0.2],[0.3,0.6],[1.5,1.0],[1.5,0.6]]
# obstacles = [[0.5,0.0],[0.5,0.4],[1.5,1.0],[1.5,0.6]]
# # t = np.linspace(0,2*np.pi,20)
# robot_start = np.linspace(np.array(starting_point) - np.array([0,robot_length/2]),np.array(starting_point) + np.array([0,robot_length/2]),8)
# robot_end = np.linspace(np.array(ending_point) - np.array([0,robot_length/2]),np.array(ending_point) + np.array([0,robot_length/2]),8)
# # obstacle_1 = np.array([[radius*np.cos(T) + obstacle_point_1[0],radius*np.sin(T) + obstacle_point_1[1]] for T in t])
# # obstacle_2 = np.array([[radius*np.cos(T) + obstacle_point_2[0],radius*np.sin(T) + obstacle_point_2[1]] for T in t])
# trajectory_sequence = [robot_start]
# trajectory_sequence.extend([make_square(obs,edge_length,20) for obs in obstacles])
# trajectory_sequence.append(robot_end)
# obstacle_trajectory = np.vstack([segment for segment in trajectory_sequence])

# meteoroid
# starting_point = [-0.1,0.8]
# ending_point = [2,0.8]
# robot_length = 0.30
# edge_length = 0.2
# obstacles = [[0.4,0.2],[0.9,0.1],[0.6,1.2],[1.4,0.6]]
# robot_start = np.linspace(np.array(starting_point) - np.array([0,robot_length/2]),np.array(starting_point) + np.array([0,robot_length/2]),8)
# robot_end = np.linspace(np.array(ending_point) - np.array([0,robot_length/2]),np.array(ending_point) + np.array([0,robot_length/2]),8)
# trajectory_sequence = [robot_start]
# trajectory_sequence.extend([make_square(obs,edge_length,20) for obs in obstacles])
# trajectory_sequence.append(robot_end)
# obstacle_trajectory = np.vstack([segment for segment in trajectory_sequence])

# funnel
# starting_point = [-0.1,0.6]
# ending_point = [2,0.6]
# robot_length = 0.30
# edge_length = 0.2
# obstacles = [[0.9,0],[1.4,0.3],[0.9,1.2],[1.4,0.9]]
# robot_start = np.linspace(np.array(starting_point) - np.array([0,robot_length/2]),np.array(starting_point) + np.array([0,robot_length/2]),8)
# robot_end = np.linspace(np.array(ending_point) - np.array([0,robot_length/2]),np.array(ending_point) + np.array([0,robot_length/2]),8)
# trajectory_sequence = [robot_start]
# trajectory_sequence.extend([make_square(obs,edge_length,20) for obs in obstacles])
# trajectory_sequence.append(robot_end)
# obstacle_trajectory = np.vstack([segment for segment in trajectory_sequence])

# dead end
# starting_point = [-0.1,0.8]
# ending_point = [1.8,0.5]
# robot_length = 0.30
# edge_length = 0.2
# obstacles = [[0.9,0.5],[1.2,0.5],[0.9,1.1],[1.2,1.1],[1.3,0.8]]
# robot_start = np.linspace(np.array(starting_point) - np.array([0,robot_length/2]),np.array(starting_point) + np.array([0,robot_length/2]),8)
# robot_end = np.linspace(np.array(ending_point) - np.array([0,robot_length/2]),np.array(ending_point) + np.array([0,robot_length/2]),8)
# trajectory_sequence = [robot_start]
# trajectory_sequence.extend([make_square(obs,edge_length,20) for obs in obstacles])
# trajectory_sequence.append(robot_end)
# obstacle_trajectory = np.vstack([segment for segment in trajectory_sequence])

# outside
# starting_point = [0.4,1.2]
# ending_point = [1.7,0.4]
# robot_length = 0.30
# edge_length = 0.2
# obstacles = [[0.8,0.6],[0.8,0.4],[0.8,0.2],[0.8,0],[0.8,-0.2]]
# robot_start = np.linspace(np.array(starting_point) - np.array([0,robot_length/2]),np.array(starting_point) + np.array([0,robot_length/2]),8)
# robot_end = np.linspace(np.array(ending_point) - np.array([robot_length/2,0]),np.array(ending_point) + np.array([robot_length/2,0]),8)
# trajectory_sequence = [robot_start]
# trajectory_sequence.extend([make_square(obs,edge_length,20) for obs in obstacles])
# trajectory_sequence.append(robot_end)
# obstacle_trajectory = np.vstack([segment for segment in trajectory_sequence])

# outside
# starting_point = [0.15,1.0]
# ending_point = [1.7,0.2]
# robot_length = 0.30
# edge_length = 0.2
# # obstacle_point_1 = [0.68,0.1]
# # obstacle_point_2 = [1.48,1.0]
# # obstacles = [[0.3,0.2],[0.3,0.6],[1.5,1.0],[1.5,0.6]]
# obstacles = [[0.5,0.3],[0.5,0.5],[1.5,1.05],[1.5,0.95]]
# # t = np.linspace(0,2*np.pi,20)
# robot_start = np.linspace(np.array(starting_point) - np.array([0,robot_length/2]),np.array(starting_point) + np.array([0,robot_length/2]),8)
# robot_end = np.linspace(np.array(ending_point) - np.array([0,robot_length/2]),np.array(ending_point) + np.array([0,robot_length/2]),8)
# # obstacle_1 = np.array([[radius*np.cos(T) + obstacle_point_1[0],radius*np.sin(T) + obstacle_point_1[1]] for T in t])
# # obstacle_2 = np.array([[radius*np.cos(T) + obstacle_point_2[0],radius*np.sin(T) + obstacle_point_2[1]] for T in t])
# trajectory_sequence = [robot_start]
# trajectory_sequence.extend([make_square(obs,edge_length,20) for obs in obstacles])
# trajectory_sequence.append(robot_end)
# obstacle_trajectory = np.vstack([segment for segment in trajectory_sequence])

# outside2
starting_point = [0.15,1.1]
ending_point = [1.7,0.2]
robot_length = 0.30
edge_length = 0.2
# obstacle_point_1 = [0.68,0.1]
# obstacle_point_2 = [1.48,1.0]
# obstacles = [[0.3,0.2],[0.3,0.6],[1.5,1.0],[1.5,0.6]]
obstacles = [[0.5,0.3],[0.5,0.5],[1.1,0.5],[1.1,0.4]]
# t = np.linspace(0,2*np.pi,20)
robot_start = np.linspace(np.array(starting_point) - np.array([0,robot_length/2]),np.array(starting_point) + np.array([0,robot_length/2]),8)
robot_end = np.linspace(np.array(ending_point) - np.array([robot_length/2,0]),np.array(ending_point) + np.array([robot_length/2,0]),8)
# obstacle_1 = np.array([[radius*np.cos(T) + obstacle_point_1[0],radius*np.sin(T) + obstacle_point_1[1]] for T in t])
# obstacle_2 = np.array([[radius*np.cos(T) + obstacle_point_2[0],radius*np.sin(T) + obstacle_point_2[1]] for T in t])
trajectory_sequence = [robot_start]
trajectory_sequence.extend([make_square(obs,edge_length,20) for obs in obstacles])
trajectory_sequence.append(robot_end)
obstacle_trajectory = np.vstack([segment for segment in trajectory_sequence])

if __name__ == '__main__':
	
	# choose one of the following trajectories

	traj_name = sys.argv[1]

	if traj_name == "straight":
		# straight line
		ppm = 20 # points per meter
		starting_point = [0.1,0.6]
		ending_point = [1.7,0.6]
		num_points = ceil(ppm*(ending_point[0] - starting_point[0]))
		trajectory = np.linspace(ending_point,starting_point,num_points)
	elif traj_name == "short":
		# short straight line
		ppm = 20 # points per meter
		starting_point = [1.1,0.6]
		ending_point = [1.5,0.6]
		num_points = ceil(ppm*(ending_point[0] - starting_point[0]))
		trajectory = np.linspace(ending_point,starting_point,num_points)
	elif traj_name == "frame":
		# world frame
		trajectory = [[0,0],[0.1,0],[0.2,0],[0.3,0],[0,0.4],[0,0.1],[0,0.2],[0,0.3]]
	elif traj_name == "circle":
		# circle
		# num_points = 50
		ppm = 20
		center_point = np.array([0.9,0.2])
		radius = 0.9
		num_points = ceil(np.pi*radius*ppm)
		# t = np.linspace(0,-7*np.pi/4,num_points)
		t = np.linspace(-np.pi,-2*np.pi,num_points)
		trajectory = np.array([[radius*np.cos(T) + center_point[0],radius*np.sin(T) + center_point[1]] for T in t])
	elif traj_name == "circle2":
		# circle
		# num_points = 50
		ppm = 20
		center_point = np.array([0.9,0.0])
		radius = 0.9
		num_points = ceil(np.pi*radius*ppm*5/6)
		# t = np.linspace(0,-7*np.pi/4,num_points)
		t = np.linspace(-13*np.pi/12,-23*np.pi/12,num_points)
		trajectory = np.array([[radius*np.cos(T) + center_point[0],radius*np.sin(T) + center_point[1]] for T in t])
		# ppm = 20
		# center_point = np.array([0.85,1.1])
		# radius = 0.9
		# num_points = ceil(np.pi*radius*ppm)
		# # t = np.linspace(0,-7*np.pi/4,num_points)
		# t = np.linspace(2*np.pi,np.pi,num_points)
		# trajectory = np.array([[radius*np.cos(T) + center_point[0],radius*np.sin(T) + center_point[1]] for T in t])
	elif traj_name == "ellipse":
		# ellipse
		num_points = 50
		center_point = np.array([0.9,0.6])
		a = 0.7
		b = 0.5
		# t = np.linspace(-7*np.pi/8,7*np.pi/8,num_points)
		t = np.linspace(7*np.pi/8,-3*np.pi/8,num_points)
		trajectory = np.array([[a*np.cos(T) + center_point[0],b*np.sin(T) + center_point[1]] for T in t])
	elif traj_name == "track":
		# track
		ppm = 20 # points per meter
		center_point = np.array([0.9,0.6])
		straightaway_length = 0.5 # meter
		curve_radius = 0.7 # meter
		starting_point = np.array([center_point[0]+straightaway_length/2,center_point[1]-curve_radius])
		first_pivot = np.array([center_point[0]-straightaway_length/2,center_point[1]-curve_radius])
		second_pivot = np.array([center_point[0]-straightaway_length/2,center_point[1]+curve_radius])
		third_pivot = np.array([center_point[0]+straightaway_length/2,center_point[1]+curve_radius])
		first_leg = np.linspace(starting_point,first_pivot,int(ppm*straightaway_length))
		second_leg = make_circle((first_pivot+second_pivot)/2,curve_radius,num_points=ppm*curve_radius*np.pi,starting_angle=-np.pi/2,swept_angle=-np.pi)
		third_leg = np.linspace(second_pivot,third_pivot,int(ppm*straightaway_length))
		fourth_leg = make_circle((third_pivot+starting_point)/2,curve_radius,num_points=ppm*curve_radius*np.pi,starting_angle=np.pi/2,swept_angle=-np.pi)
		trajectory = np.vstack((first_leg,second_leg,third_leg,fourth_leg))
	elif traj_name == "rounded-rectangle":
		# rounded rectangle
		ppm = 30 # points per meter
		center_point = np.array([0.9,0.6])
		width = 1.8 # meter
		height = 0.9 # meter
		curve_radius = 0.4 # meter
		# first leg
		starting_point = np.array([center_point[0]+width/2-curve_radius,center_point[1]-height/2])
		ending_point = np.array([center_point[0]-width/2+curve_radius,center_point[1]-height/2])
		leg_length = starting_point[0] - ending_point[0]
		first_leg = np.linspace(starting_point,ending_point,ceil(ppm*leg_length))
		# next corner
		arc_center = [center_point[0]-width/2+curve_radius,center_point[1]-height/2+curve_radius]
		next_corner = make_circle(arc_center,curve_radius,num_points=ppm*curve_radius*np.pi/2,starting_angle=-np.pi/2,swept_angle=-np.pi/2)
		trajectory = np.vstack((first_leg,next_corner[1:,:]))
		# next leg
		starting_point = np.array([center_point[0]-width/2,center_point[1]-height/2+curve_radius])
		ending_point = np.array([center_point[0]-width/2,center_point[1]+height/2-curve_radius])
		leg_length = ending_point[1] - starting_point[1]
		next_leg = np.linspace(starting_point,ending_point,ceil(ppm*leg_length))
		trajectory = np.vstack((trajectory,next_leg[1:,:]))
		# next corner
		arc_center = [center_point[0]-width/2+curve_radius,center_point[1]+height/2-curve_radius]
		next_corner = make_circle(arc_center,curve_radius,num_points=ppm*curve_radius*np.pi/2,starting_angle=-np.pi,swept_angle=-np.pi/2)
		trajectory = np.vstack((trajectory,next_corner[1:,:]))
		# next leg
		starting_point = np.array([center_point[0]-width/2+curve_radius,center_point[1]+height/2])
		ending_point = np.array([center_point[0]+width/2-curve_radius,center_point[1]+height/2])
		leg_length = ending_point[0] - starting_point[0]
		next_leg = np.linspace(starting_point,ending_point,ceil(ppm*leg_length))
		trajectory = np.vstack((trajectory,next_leg[1:,:]))
		# next corner
		arc_center = [center_point[0]+width/2-curve_radius,center_point[1]+height/2-curve_radius]
		next_corner = make_circle(arc_center,curve_radius,num_points=ppm*curve_radius*np.pi/2,starting_angle=-3*np.pi/2,swept_angle=-np.pi/2)
		trajectory = np.vstack((trajectory,next_corner[1:,:]))
		# next leg
		starting_point = np.array([center_point[0]+width/2,center_point[1]+height/2-curve_radius])
		ending_point = np.array([center_point[0]+width/2,center_point[1]-height/2+curve_radius])
		leg_length = starting_point[1] - ending_point[1]
		next_leg = np.linspace(starting_point,ending_point,ceil(ppm*leg_length))
		trajectory = np.vstack((trajectory,next_leg[1:,:]))
		# next corner
		arc_center = [center_point[0]+width/2-curve_radius,center_point[1]-height/2+curve_radius]
		next_corner = make_circle(arc_center,curve_radius,num_points=ppm*curve_radius*np.pi/2,starting_angle=0,swept_angle=-np.pi/2)
		trajectory = np.vstack((trajectory,next_corner[1:,:]))
	elif traj_name == "Z":
		# Z shape
		points_per_segment = 20
		starting_point = [0,0.1]
		first_pivot = [0,1.1]
		second_pivot = [1.8,0.1]
		ending_point = [1.8,1.1]
		trajectory = np.vstack((np.linspace(starting_point,first_pivot,points_per_segment),np.linspace(first_pivot,second_pivot,2*points_per_segment)[1:,:],np.linspace(second_pivot,ending_point,points_per_segment)[1:,:]))
	elif traj_name == "rectangle":
		# rectangle
		ppm = 30 # points per meter
		starting_point = [0.1,0.2]
		third_pivot = [0.1,1.0]
		second_pivot = [1.7,1.0]
		first_pivot = [1.7,0.2]
		ending_point = [0.1,0.2]
		short_dist = first_pivot[0] - starting_point[0]
		long_dist = second_pivot[1] - first_pivot[1]
		trajectory = np.vstack((np.linspace(starting_point,first_pivot,int(ppm*short_dist)),np.linspace(first_pivot,second_pivot,int(ppm*long_dist))[1:,:],np.linspace(second_pivot,third_pivot,int(ppm*short_dist)),np.linspace(third_pivot,ending_point,int(ppm*long_dist))))
	elif traj_name == "S":
		# S shape
		num_points = 40
		amplitude = 0.55
		starting_point = [0,0.6]
		ending_point = [1.8,0.6]
		t = np.linspace(0,2*np.pi,num_points)
		x = np.linspace(starting_point[0],ending_point[0],num_points)
		y = amplitude*np.sin(t) + starting_point[1]
		trajectory = np.array([[X,amplitude*np.sin(T) + starting_point[1]] for X,T in zip(x,t)])
	elif traj_name == "triangle":
		# triangle
		ppm = 20 # points per meter
		side_length = 1 # meters
		points_per_segment = ceil(ppm*side_length)
		height = side_length/2 * sqrt(3)
		starting_point = [0.41,0.25]
		apex = [side_length/2 + starting_point[0],starting_point[1] + height]
		second_pivot = [starting_point[0] + side_length,starting_point[1]]
		trajectory_sequence = [np.linspace(starting_point,apex,points_per_segment),np.linspace(apex,second_pivot,points_per_segment),np.linspace(second_pivot,starting_point,points_per_segment)]
		trajectory = np.vstack((np.linspace(starting_point,apex,points_per_segment),np.linspace(apex,second_pivot,points_per_segment)[1:,:],np.linspace(second_pivot,starting_point,points_per_segment)[1:,:]))
	elif traj_name == "right-triangle":
		# right triangle
		ppm = 20 # points per meter
		leg_length = 1.1 # meters
		hyp_length = sqrt(2)*leg_length
		points_per_leg = ceil(ppm*leg_length)
		points_per_hyp = ceil(ppm*hyp_length)
		starting_point = [0.9,0.05]
		first_pivot = [starting_point[0] + leg_length/sqrt(2),starting_point[1] + leg_length/sqrt(2)]
		second_pivot = [first_pivot[0]-hyp_length,first_pivot[1]]
		trajectory_sequence = [np.linspace(starting_point,first_pivot,points_per_leg),np.linspace(first_pivot,second_pivot,points_per_hyp),np.linspace(second_pivot,starting_point,points_per_leg)]
		trajectory = np.vstack((np.linspace(starting_point,first_pivot,points_per_leg),np.linspace(first_pivot,second_pivot,points_per_hyp)[1:,:],np.linspace(second_pivot,starting_point,points_per_leg)[1:,:]))
	elif traj_name == "zigzag":
		# zigzag
		ppm = 20 # points per meter
		leg_length = 1.5 # meters
		points_per_segment = ceil(ppm*leg_length)
		alpha = 15 # degrees
		beta = np.pi/180 * (90-alpha/2) # radians
		Nzags = 2
		starting_point = np.array([0.15,0.25])
		trajectory_sequence = []
		for n in range(Nzags):
			trajectory_sequence.extend([np.linspace(starting_point,starting_point + leg_length*np.array([np.sin(beta),np.cos(beta)]),points_per_segment),np.linspace(starting_point + leg_length*np.array([np.sin(beta),np.cos(beta)]),starting_point + 2*leg_length*np.array([0,np.cos(beta)]),points_per_segment)])
			if n == 0:
				trajectory = np.vstack((np.linspace(starting_point,starting_point + leg_length*np.array([np.sin(beta),np.cos(beta)]),points_per_segment),np.linspace(starting_point + leg_length*np.array([np.sin(beta),np.cos(beta)]),starting_point + 2*leg_length*np.array([0,np.cos(beta)]),points_per_segment)[1:,:]))
			else:
				trajectory = np.vstack((trajectory,np.linspace(starting_point,starting_point + leg_length*np.array([np.sin(beta),np.cos(beta)]),points_per_segment)[1:,:],np.linspace(starting_point + leg_length*np.array([np.sin(beta),np.cos(beta)]),starting_point + 2*leg_length*np.array([0,np.cos(beta)]),points_per_segment)[1:,:]))
			starting_point = starting_point + 2*leg_length*np.array([0,np.cos(beta)])
		# print(len(trajectory_sequence))
	elif traj_name == "zigzagzig":
		# zigzagzig
		ppm = 20 # points per meter
		leg_length = 1.6 # meters
		points_per_segment = ceil(ppm*leg_length)
		alpha = 15 # degrees
		beta = np.pi/180 * (90-alpha/2) # radians
		Nzags = 3
		starting_point = np.array([0.12,0.3])
		trajectory_sequence = [np.linspace(starting_point + leg_length*np.array([(n%2)*np.sin(beta),n*np.cos(beta)]),starting_point + leg_length*np.array([((n+1)%2)*np.sin(beta),(n+1)*np.cos(beta)]),points_per_segment) for n in range(Nzags)]
		trajectory = np.vstack([segment for segment in trajectory_sequence])
		# for n in range(Nzags):
		# 	# trajectory_sequence.extend([np.linspace(starting_point,starting_point + leg_length*np.array([np.sin(beta),np.cos(beta)]),points_per_segment),np.linspace(starting_point + leg_length*np.array([np.sin(beta),np.cos(beta)]),starting_point + 2*leg_length*np.array([0,np.cos(beta)]),points_per_segment)])
		# 	if n == 0:
		# 		trajectory = np.vstack((np.linspace(starting_point,starting_point + leg_length*np.array([np.sin(beta),np.cos(beta)]),points_per_segment),np.linspace(starting_point + leg_length*np.array([np.sin(beta),np.cos(beta)]),starting_point + 2*leg_length*np.array([0,np.cos(beta)]),points_per_segment)[1:,:]))
		# 	else:
		# 		trajectory = np.vstack((trajectory,np.linspace(starting_point,starting_point + leg_length*np.array([np.sin(beta),np.cos(beta)]),points_per_segment)[1:,:],np.linspace(starting_point + leg_length*np.array([np.sin(beta),np.cos(beta)]),starting_point + 2*leg_length*np.array([0,np.cos(beta)]),points_per_segment)[1:,:]))
		# 	starting_point = starting_point + 2*leg_length*np.array([0,np.cos(beta)])
		# print(len(trajectory_sequence))
	elif traj_name == "circle_obstacles":
		starting_point = [-0.15,1.1]
		ending_point = [2,0.2]
		robot_length = 0.30
		radius = 0.1
		# obstacle_point_1 = [0.68,0.1]
		# obstacle_point_2 = [1.48,1.0]
		obstacles = [[0.5,0.2],[0.5,0.6],[1.5,1.0],[1.5,0.6]]
		t = np.linspace(0,2*np.pi,20)
		robot_start = np.linspace(np.array(starting_point) - np.array([0,robot_length/2]),np.array(starting_point) + np.array([0,robot_length/2]),8)
		robot_end = np.linspace(np.array(ending_point) - np.array([0,robot_length/2]),np.array(ending_point) + np.array([0,robot_length/2]),8)
		# obstacle_1 = np.array([[radius*np.cos(T) + obstacle_point_1[0],radius*np.sin(T) + obstacle_point_1[1]] for T in t])
		# obstacle_2 = np.array([[radius*np.cos(T) + obstacle_point_2[0],radius*np.sin(T) + obstacle_point_2[1]] for T in t])
		trajectory_sequence = [robot_start]
		trajectory_sequence.extend([np.array([[radius*np.cos(T) + obs[0],radius*np.sin(T) + obs[1]] for T in t]) for obs in obstacles])
		trajectory_sequence.append(robot_end)
		trajectory = np.vstack([segment for segment in trajectory_sequence])
	elif traj_name == "obstacles":
		starting_point = [-0.15,1.1]
		ending_point = [1.6,0.2]
		robot_length = 0.30
		edge_length = 0.2
		# obstacle_point_1 = [0.68,0.1]
		# obstacle_point_2 = [1.48,1.0]
		obstacles = [[0.3,0.2],[0.3,0.6],[1.5,1.0],[1.5,0.6]]
		obstacles = [[0.5,0.0],[0.5,0.4],[1.5,1.0],[1.5,0.6]]
		# t = np.linspace(0,2*np.pi,20)
		robot_start = np.linspace(np.array(starting_point) - np.array([0,robot_length/2]),np.array(starting_point) + np.array([0,robot_length/2]),8)
		robot_end = np.linspace(np.array(ending_point) - np.array([0,robot_length/2]),np.array(ending_point) + np.array([0,robot_length/2]),8)
		# obstacle_1 = np.array([[radius*np.cos(T) + obstacle_point_1[0],radius*np.sin(T) + obstacle_point_1[1]] for T in t])
		# obstacle_2 = np.array([[radius*np.cos(T) + obstacle_point_2[0],radius*np.sin(T) + obstacle_point_2[1]] for T in t])
		trajectory_sequence = [robot_start]
		trajectory_sequence.extend([make_square(obs,edge_length,20) for obs in obstacles])
		trajectory_sequence.append(robot_end)
		trajectory = np.vstack([segment for segment in trajectory_sequence])
	elif traj_name == "meteoroid":
		starting_point = [-0.1,0.8]
		ending_point = [2,0.8]
		robot_length = 0.30
		edge_length = 0.2
		obstacles = [[0.4,0.2],[0.9,0.1],[0.6,1.2],[1.4,0.6]]
		robot_start = np.linspace(np.array(starting_point) - np.array([0,robot_length/2]),np.array(starting_point) + np.array([0,robot_length/2]),8)
		robot_end = np.linspace(np.array(ending_point) - np.array([0,robot_length/2]),np.array(ending_point) + np.array([0,robot_length/2]),8)
		trajectory_sequence = [robot_start]
		trajectory_sequence.extend([make_square(obs,edge_length,20) for obs in obstacles])
		trajectory_sequence.append(robot_end)
		trajectory = np.vstack([segment for segment in trajectory_sequence])
	elif traj_name == "funnel":
		starting_point = [-0.1,0.6]
		ending_point = [2,0.6]
		robot_length = 0.30
		edge_length = 0.2
		obstacles = [[0.9,0],[1.4,0.3],[0.9,1.2],[1.4,0.9]]
		robot_start = np.linspace(np.array(starting_point) - np.array([0,robot_length/2]),np.array(starting_point) + np.array([0,robot_length/2]),8)
		robot_end = np.linspace(np.array(ending_point) - np.array([0,robot_length/2]),np.array(ending_point) + np.array([0,robot_length/2]),8)
		trajectory_sequence = [robot_start]
		trajectory_sequence.extend([make_square(obs,edge_length,20) for obs in obstacles])
		trajectory_sequence.append(robot_end)
		trajectory = np.vstack([segment for segment in trajectory_sequence])
	elif traj_name == "deadend":
		starting_point = [-0.1,0.8]
		ending_point = [1.8,0.5]
		robot_length = 0.30
		edge_length = 0.2
		obstacles = [[0.9,0.5],[1.2,0.5],[0.9,1.1],[1.2,1.1],[1.3,0.8]]
		robot_start = np.linspace(np.array(starting_point) - np.array([0,robot_length/2]),np.array(starting_point) + np.array([0,robot_length/2]),8)
		robot_end = np.linspace(np.array(ending_point) - np.array([0,robot_length/2]),np.array(ending_point) + np.array([0,robot_length/2]),8)
		trajectory_sequence = [robot_start]
		trajectory_sequence.extend([make_square(obs,edge_length,20) for obs in obstacles])
		trajectory_sequence.append(robot_end)
		trajectory = np.vstack([segment for segment in trajectory_sequence])
	elif traj_name == "outside-rebuttal":
		starting_point = [0.5,1.2]
		ending_point = [1.7,0.4]
		robot_length = 0.30
		edge_length = 0.2
		obstacles = [[1,0.5],[1,0.3],[1,0.1],[1,-0.1],[1,-0.3]]
		robot_start = np.linspace(np.array(starting_point) - np.array([0,robot_length/2]),np.array(starting_point) + np.array([0,robot_length/2]),8)
		robot_end = np.linspace(np.array(ending_point) - np.array([robot_length/2,0]),np.array(ending_point) + np.array([robot_length/2,0]),8)
		trajectory_sequence = [robot_start]
		trajectory_sequence.extend([make_square(obs,edge_length,20) for obs in obstacles])
		trajectory_sequence.append(robot_end)
		trajectory = np.vstack([segment for segment in trajectory_sequence])
	elif traj_name == "outside":
		starting_point = [0.15,1.0]
		ending_point = [1.7,0.2]
		robot_length = 0.30
		edge_length = 0.2
		# obstacle_point_1 = [0.68,0.1]
		# obstacle_point_2 = [1.48,1.0]
		# obstacles = [[0.3,0.2],[0.3,0.6],[1.5,1.0],[1.5,0.6]]
		obstacles = [[0.5,0.3],[0.5,0.5],[1.5,1.05],[1.5,0.95]]
		# t = np.linspace(0,2*np.pi,20)
		robot_start = np.linspace(np.array(starting_point) - np.array([0,robot_length/2]),np.array(starting_point) + np.array([0,robot_length/2]),8)
		robot_end = np.linspace(np.array(ending_point) - np.array([0,robot_length/2]),np.array(ending_point) + np.array([0,robot_length/2]),8)
		# obstacle_1 = np.array([[radius*np.cos(T) + obstacle_point_1[0],radius*np.sin(T) + obstacle_point_1[1]] for T in t])
		# obstacle_2 = np.array([[radius*np.cos(T) + obstacle_point_2[0],radius*np.sin(T) + obstacle_point_2[1]] for T in t])
		trajectory_sequence = [robot_start]
		trajectory_sequence.extend([make_square(obs,edge_length,20) for obs in obstacles])
		trajectory_sequence.append(robot_end)
		trajectory = np.vstack([segment for segment in trajectory_sequence])
	elif traj_name == "outside2":
		starting_point = [0.15,1.1]
		ending_point = [1.7,0.2]
		robot_length = 0.30
		edge_length = 0.2
		# obstacle_point_1 = [0.68,0.1]
		# obstacle_point_2 = [1.48,1.0]
		# obstacles = [[0.3,0.2],[0.3,0.6],[1.5,1.0],[1.5,0.6]]
		obstacles = [[0.5,0.3],[0.5,0.5],[1.1,0.5],[1.1,0.4]]
		# t = np.linspace(0,2*np.pi,20)
		robot_start = np.linspace(np.array(starting_point) - np.array([0,robot_length/2]),np.array(starting_point) + np.array([0,robot_length/2]),8)
		robot_end = np.linspace(np.array(ending_point) - np.array([robot_length/2,0]),np.array(ending_point) + np.array([robot_length/2,0]),8)
		# obstacle_1 = np.array([[radius*np.cos(T) + obstacle_point_1[0],radius*np.sin(T) + obstacle_point_1[1]] for T in t])
		# obstacle_2 = np.array([[radius*np.cos(T) + obstacle_point_2[0],radius*np.sin(T) + obstacle_point_2[1]] for T in t])
		trajectory_sequence = [robot_start]
		trajectory_sequence.extend([make_square(obs,edge_length,20) for obs in obstacles])
		trajectory_sequence.append(robot_end)
		trajectory = np.vstack([segment for segment in trajectory_sequence])
	else:
		error("You must choose a predefined trajectory.")

	# get camera parameters
	config_path = os.path.join(rospkg.RosPack().get_path('tensegrity_perception'),'configs/data_cfg.json')
	data_cfg = json.load(open(config_path,'r'))
	cam_intr = data_cfg.get('cam_intr')
	cam_extr = data_cfg.get('cam_extr')

	vis = Visualizer(trajectory,cam_intr,cam_extr)
	rospy.spin()
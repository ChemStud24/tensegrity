import numpy as np
import json
import sys
import os
from math import sqrt
from scipy.spatial.transform import Rotation as R

# ROS
import rospy
import rosnode
import message_filters
from tensegrity.msg import TensegrityStamped, SensorsStamped, ImuStamped, Node, NodesStamped
# from phasespace.msg import Markers

# home built
from calc_Node_Pos_cable_len_errors import *
from Tensegrity_model_inputs import inPairs_3, number_of_rods
from align_frames import align
from kabsch import kabsch_transformed, rigid_transform_3D

def to_centroid(points):
    centroid = np.mean(points,axis=0)
    return points - centroid

class ReconstructionListener:

    def __init__(self,kabsch_file=None,outdir=None):
        self.count = 0
        self.outdir = outdir
        self.reconstruction_topic = 'reconstruction_msg'
        self.reconstruction_pub = rospy.Publisher(self.reconstruction_topic,NodesStamped,queue_size=10)

        self.strain_topic = "/control_msg"
        strain_sub = message_filters.Subscriber(self.strain_topic, TensegrityStamped)
        
        # self.imu_topic = "/imu_msg"
        # imu_sub = message_filters.Subscriber(self.imu_topic, ImuStamped)
        self.inNodes_3 = inNodes_3
        self.angle0 = [0,0,0]

        if '/phasespace_node' in rosnode.get_node_names():
            self.mocap_topic = "/phasespace_markers"
            mocap_sub = message_filters.Subscriber(self.mocap_topic, Markers)
            self.kabsch_file = kabsch_file

            # data synchronizer with mocap
            self.time_synchornizer = message_filters.ApproximateTimeSynchronizer([strain_sub,imu_sub,mocap_sub], queue_size=10, slop=0.1)
            self.time_synchornizer.registerCallback(self.mocap_callback)
        else:
            # data synchronizer without mocap
            self.time_synchornizer = message_filters.ApproximateTimeSynchronizer([strain_sub], queue_size=10, slop=0.1)
            self.time_synchornizer.registerCallback(self.no_mocap_callback)

        self.first_message_arrived_yet = False

    def no_mocap_callback(self,strain_msg):
        # if not self.first_message_arrived_yet:
        #     # rotate inNodes_3 based on imu info
        #     imus = np.array([[imu.x,imu.y,imu.z] for imu in imu_msg.imus])
        #     imus = np.array([imu/np.linalg.norm(imu) for imu in imus])
        #     node5 = -L/2*imus[0,:]
        #     node4 = L/2*imus[0,:]
        #     node1 = -L/2*imus[1,:]
        #     node0 = L/2*imus[1,:]
        #     imu_nodes = np.vstack((node0,node1,node4,node5))
        #     inNodes = np.vstack((self.inNodes_3[0:2,:],self.inNodes_3[4:,:]))
        #     print(imu_nodes.shape)
        #     print(inNodes.shape)

        #     R, t = rigid_transform_3D(imu_nodes,inNodes)
        #     self.inNodes_3 = kabsch_transformed(self.inNodes_3,R,t)
        #     print(self.inNodes_3.shape)
        #     self.first_message_arrived_yet = True

        # calculate state reconstruction (lengths in mm) 
        values = np.array([sensor.length for sensor in strain_msg.sensors])
        if not any(values == 0):
            imus = np.array([[imu.x,imu.y,imu.z] for imu in strain_msg.imus])
            array = calculateNodePositions(self.inNodes_3,inPairs_3, values, imus, angle0=self.angle0)
            array = np.array(array)
            self.inNodes_3 = array

            # publish
            reconstruction_msg = NodesStamped()
            # reconstruction_msg.header.stamp = rospy.Time.now()
            reconstruction_msg.header = strain_msg.header
            for node_id in range(array.shape[0]):
            	node = Node()
            	node.id = node_id
            	node.x = array[node_id,0]
            	node.y = array[node_id,1]
            	node.z = array[node_id,2]
            	reconstruction_msg.reconstructed_nodes.append(node)
            # for imu in imu_msg.imus:
            #     reconstruction_msg.imus.append(imu)
            self.reconstruction_pub.publish(reconstruction_msg)

            if not outdir == None:
                # save to file
                data = {'state reconstruction': {node_id:{'x':array[node_id,0],'y':array[node_id,1],'z':array[node_id,2]} for node_id in range(array.shape[0])}}
                # json.dump(data,open('../data/live_reconstruction_data/' + str(self.count).zfill(4) + '.json','w'))
                json.dump(data,open(os.path.join(str(self.count).zfill(4) + '.json'),'w'))
                self.count += 1
                print("writing data file " + str(self.count))

    def mocap_callback(self,strain_msg,imu_msg,mocap_msg):
        # calculate state reconstruction (lengths in mm) 
        values = np.array([sensor.length for sensor in strain_msg.sensors])
        imus = np.array([[imu.x,imu.y,imu.z] for imu in imu_msg.imus])
        array = calculateNodePositions(self.inNodes_3,inPairs_3, values, imus, angle0=self.angle0)
        array = to_centroid(np.array(array))
        self.inNodes_3 = np.array(array)

        # extract the motion capture data
        mocap_coordinates = []
        missing = []
        for marker in mocap_msg.markers:
            if marker.cond == -1:
                missing.append(marker.id)
            else:
                mocap_coordinates.append([marker.x,marker.y,marker.z])
        mocap_coordinates = np.array(mocap_coordinates)
        
        # transform mocap coordinates to world frame
        if self.kabsch_file == None:
        	R = np.eye(3)
        	t = np.array([0]*3)
        else:
	        trans = json.load(open(self.kabsch_file))
	        R = np.array(trans["R"])
	        t = np.array(trans["t"])
        mocap_coordinates = to_centroid(kabsch_transformed(mocap_coordinates,R,t))

        # publish
        reconstruction_msg = NodesStamped()
        # reconstruction_msg.header.stamp = rospy.Time.now()
        reconstruction_msg.header = strain_msg.header
        for node_id in range(array.shape[0]):
        	node = Node()
        	node.id = node_id
        	node.x = array[node_id,0]
        	node.y = array[node_id,1]
        	node.z = array[node_id,2]
        	reconstruction_msg.reconstructed_nodes.append(node)
        for node_id in range(mocap_coordinates.shape[0]):
        	node = Node()
        	node.id = node_id
        	node.x = mocap_coordinates[node_id,0]
        	node.y = mocap_coordinates[node_id,1]
        	node.z = mocap_coordinates[node_id,2]
        	reconstruction_msg.mocap_nodes.append(node)
        self.reconstruction_pub.publish(reconstruction_msg)

        if not outdir == None:
            # save to file
            data = {'state reconstruction': {node_id:{'x':array[node_id,0],'y':array[node_id,1],'z':array[node_id,2]} for node_id in range(array.shape[0])},
            'mocap': {node_id:{'x':mocap_coordinates[node_id,0],'y':mocap_coordinates[node_id,1],'z':mocap_coordinates[node_id,2]}  for node_id in range(mocap_coordinates.shape[0])}}
            # json.dump(data,open('../data/live_reconstruction_data/' + str(self.count).zfill(4) + '.json','w'))
            json.dump(data,open(os.path.join(str(self.count).zfill(4) + '.json'),'w'))
            self.count += 1
            print("writing data file " + str(self.count))

    def run(self, rate):
        print("ReconstructionListener started! Waiting for messages.")
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('reconstruction_listener')

    if len(sys.argv) > 1:
        kabsch_file = sys.argv[1]
    else:
        kabsch_file = "../calibration/identity.json"

    if len(sys.argv) > 2:
        outdir = sys.argv[2]
    else:
        outdir = None

	# 	listener = ReconstructionListener(kabsch_file=sys.argv[1],outdir=sys.argv[2])
	# else:
	# 	listener = ReconstructionListener()

    listener = ReconstructionListener(kabsch_file=kabsch_file,outdir=outdir)
    rate = rospy.Rate(30)
    listener.run(rate)
import numpy as np
from vpython import *
# from matplotlib.figure import Figure

import rospy
from tensegrity.msg import NodesStamped
from Tensegrity_model_inputs import inPairs_3, number_of_rods, L, inNodes_3

class Visualizer:
    def __init__(self):
        scene = canvas(width=1904,height=1066)
        scene.autoscale = False
        scene.center = vec(0,0,0)
        scene.camera.pos = vector(0,5,0)
        scene.camera.axis = vector(0,-3,0)

        x_sr,y_sr,z_sr = self.nodes2sr(inNodes_3)

        colors = [[255,0,0],[0,255,0],[0,0,255],[0,158,115],[240,228,66],[0,114,178],[213,94,0],[204,121,167],[200,200,200]]
        node_radius = 0.2
        bar_radius = 0.05
        cab_radius = 0.05

        self.node0_sr = sphere(pos = vec(x_sr["0"]/100, y_sr["0"]/100, z_sr["0"]/100), radius = node_radius, color = vec(colors[0][0]/255, colors[0][1]/255, colors[0][2]/255))
        self.node1_sr = sphere(pos = vec(x_sr["1"]/100, y_sr["1"]/100, z_sr["1"]/100), radius = node_radius, color = self.node0_sr.color)
        self.node2_sr = sphere(pos = vec(x_sr["2"]/100, y_sr["2"]/100, z_sr["2"]/100), radius = node_radius, color = vec(colors[1][0]/255, colors[1][1]/255, colors[1][2]/255))
        self.node3_sr = sphere(pos = vec(x_sr["3"]/100, y_sr["3"]/100, z_sr["3"]/100), radius = node_radius, color = self.node2_sr.color)
        self.node4_sr = sphere(pos = vec(x_sr["4"]/100, y_sr["4"]/100, z_sr["4"]/100), radius = node_radius, color = vec(colors[2][0]/255, colors[2][1]/255, colors[2][2]/255))
        self.node5_sr = sphere(pos = vec(x_sr["5"]/100, y_sr["5"]/100, z_sr["5"]/100), radius = node_radius, color = self.node4_sr.color)
        com_sr = (self.node0_sr.pos + self.node1_sr.pos + self.node2_sr.pos + self.node3_sr.pos + self.node4_sr.pos + self.node5_sr.pos) / 6 + vec(-1,5,0)
        self.node0_sr.pos = self.node0_sr.pos - com_sr
        self.node1_sr.pos = self.node1_sr.pos - com_sr
        self.node2_sr.pos = self.node2_sr.pos - com_sr
        self.node3_sr.pos = self.node3_sr.pos - com_sr
        self.node4_sr.pos = self.node4_sr.pos - com_sr
        self.node5_sr.pos = self.node5_sr.pos - com_sr
        self.bar0_sr = cylinder(pos = self.node0_sr.pos, axis = self.node1_sr.pos-self.node0_sr.pos, radius = bar_radius, color = vec(1, 1, 1))
        self.bar1_sr = cylinder(pos = self.node2_sr.pos, axis = self.node3_sr.pos-self.node2_sr.pos, radius = bar_radius, color = self.bar0_sr.color)
        self.bar2_sr = cylinder(pos = self.node4_sr.pos, axis = self.node5_sr.pos-self.node4_sr.pos, radius = bar_radius, color = self.bar0_sr.color)
        self.cab0_sr = cylinder(pos = self.node3_sr.pos, axis = self.node5_sr.pos-self.node3_sr.pos, radius = cab_radius, color = vec(0, 1, 1))
        self.cab1_sr = cylinder(pos = self.node1_sr.pos, axis = self.node3_sr.pos-self.node1_sr.pos, radius = cab_radius, color = self.cab0_sr.color)
        self.cab2_sr = cylinder(pos = self.node1_sr.pos, axis = self.node5_sr.pos-self.node1_sr.pos, radius = cab_radius, color = self.cab0_sr.color)
        self.cab3_sr = cylinder(pos = self.node0_sr.pos, axis = self.node2_sr.pos-self.node0_sr.pos, radius = cab_radius, color = self.cab0_sr.color)
        self.cab4_sr = cylinder(pos = self.node0_sr.pos, axis = self.node4_sr.pos-self.node0_sr.pos, radius = cab_radius, color = self.cab0_sr.color)
        self.cab5_sr = cylinder(pos = self.node2_sr.pos, axis = self.node4_sr.pos-self.node2_sr.pos, radius = cab_radius, color = self.cab0_sr.color)
        self.cab6_sr = cylinder(pos = self.node2_sr.pos, axis = self.node5_sr.pos-self.node2_sr.pos, radius = cab_radius, color = self.cab0_sr.color)
        self.cab7_sr = cylinder(pos = self.node0_sr.pos, axis = self.node3_sr.pos-self.node0_sr.pos, radius = cab_radius, color = self.cab0_sr.color)
        self.cab8_sr = cylinder(pos = self.node1_sr.pos, axis = self.node4_sr.pos-self.node1_sr.pos, radius = cab_radius, color = self.cab0_sr.color)

        reconstruction_sub = rospy.Subscriber('/reconstruction_msg', NodesStamped, self.callback)

    def callback(self,msg):
        nodes = np.array([[node.x,node.y,node.z] for node in msg.reconstructed_nodes])
        x_sr,y_sr,z_sr = self.nodes2sr(nodes)

        self.node0_sr.pos = vec(x_sr["0"]/100, y_sr["0"]/100, z_sr["0"]/100)
        self.node1_sr.pos = vec(x_sr["1"]/100, y_sr["1"]/100, z_sr["1"]/100)
        self.node2_sr.pos = vec(x_sr["2"]/100, y_sr["2"]/100, z_sr["2"]/100)
        self.node3_sr.pos = vec(x_sr["3"]/100, y_sr["3"]/100, z_sr["3"]/100)
        self.node4_sr.pos = vec(x_sr["4"]/100, y_sr["4"]/100, z_sr["4"]/100)
        self.node5_sr.pos = vec(x_sr["5"]/100, y_sr["5"]/100, z_sr["5"]/100)
        com_sr = (self.node0_sr.pos + self.node1_sr.pos + self.node2_sr.pos + self.node3_sr.pos + self.node4_sr.pos + self.node5_sr.pos) / 6 + vec(-1,5,0)
        self.node0_sr.pos = self.node0_sr.pos - com_sr
        self.node1_sr.pos = self.node1_sr.pos - com_sr
        self.node2_sr.pos = self.node2_sr.pos - com_sr
        self.node3_sr.pos = self.node3_sr.pos - com_sr
        self.node4_sr.pos = self.node4_sr.pos - com_sr
        self.node5_sr.pos = self.node5_sr.pos - com_sr
        self.bar0_sr.pos = self.node0_sr.pos
        self.bar0_sr.axis = self.node1_sr.pos-self.node0_sr.pos
        self.bar1_sr.pos = self.node2_sr.pos
        self.bar1_sr.axis = self.node3_sr.pos-self.node2_sr.pos
        self.bar2_sr.pos = self.node4_sr.pos
        self.bar2_sr.axis = self.node5_sr.pos-self.node4_sr.pos
        self.cab0_sr.pos = self.node3_sr.pos
        self.cab0_sr.axis = self.node5_sr.pos-self.node3_sr.pos
        self.cab1_sr.pos = self.node1_sr.pos
        self.cab1_sr.axis = self.node3_sr.pos-self.node1_sr.pos
        self.cab2_sr.pos = self.node1_sr.pos
        self.cab2_sr.axis = self.node5_sr.pos-self.node1_sr.pos
        self.cab3_sr.pos = self.node0_sr.pos
        self.cab3_sr.axis = self.node2_sr.pos-self.node0_sr.pos
        self.cab4_sr.pos = self.node0_sr.pos
        self.cab4_sr.axis = self.node4_sr.pos-self.node0_sr.pos
        self.cab5_sr.pos = self.node2_sr.pos
        self.cab5_sr.axis = self.node4_sr.pos-self.node2_sr.pos
        self.cab6_sr.pos = self.node2_sr.pos
        self.cab6_sr.axis = self.node5_sr.pos-self.node2_sr.pos
        self.cab7_sr.pos = self.node0_sr.pos
        self.cab7_sr.axis = self.node3_sr.pos-self.node0_sr.pos
        self.cab8_sr.pos = self.node1_sr.pos
        self.cab8_sr.axis = self.node4_sr.pos-self.node1_sr.pos
        # rate(10)

    def nodes2sr(self,nodes):
        x_sr = {str(key):nodes[key,0] for key in range(number_of_rods*2)}
        y_sr = {str(key):nodes[key,1] for key in range(number_of_rods*2)}
        z_sr = {str(key):nodes[key,2] for key in range(number_of_rods*2)}
        return x_sr,y_sr,z_sr

if __name__ == '__main__':
    rospy.init_node('reconstruction_plotter')
    vis = Visualizer()
    rospy.spin()
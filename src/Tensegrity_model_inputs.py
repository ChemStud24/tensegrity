

import numpy as np
# from scipy import optimize
# from numpy import linalg
# import math
# import time
# import matplotlib.pyplot as plt
# # from vpython import *
# from mpl_toolkits.mplot3d import Axes3D

#define constants

number_of_rods = 3
# number_of_faces = 8
# massRod = 140 #g
# massSkin = 700/20 #g
# massEndCap = 25 #g
L = 330#360 #Initial bar length, in mm
# initialSensorLength = 15

# node_position = [] #an array initialized with just 0s where the node position can be inserted

# for counter in range(0, number_of_rods * 3):
#     node_position.append(0)

#definitions for the 3 bar tensegrity
# positions_3 = [1, 2, 3, 4, 5, 6 , 7, 8, 9]
# inSensorLength_3 = [14.0, 14.0, 14.0, 14.0, 14.0, 14.0, 25.0, 25.0, 25.0]

#3bar tensegrity; 1 and 2 are green, 3 and 4 are blue, 5 and 6 are red
# inPairs_3 = [[1, 2], [3, 4], [5, 6], 
#  [3, 6], [2, 5], [1,4], [2,4], [2,6], [4,6], [3,5], [1, 3], [1, 5]] 
# inPairs_3 = [[5,6],[1,4],[3,2],[2,4],[4,6],[2,6],[1,5],[3,5],[1,3],[1,2],[4,5],[3,6]]
inPairs_3 = [[0,1],[2,3],[4,5],
             [3,5],[1,3],[1,5],[0,2],[0,4],[2,4],[2,5],[0,3],[1,4]]

# inPairs_3 = [[p+1 for p in pair] for pair in inPairs_3]

# 3 bar tensegrity node positions
# inNodes_3 = np.array([[  0.0,   0.0,   0.0], 
#                       [268.0, 142.0,  92.0],
#                       [  0.0, 100.0, 120.0],
#                       [265.0,   0.0,   0.0],
#                       [  0.0, 175.0,   0.0], 
#                       [245.0,   0.0, 145.0]])

inNodes_3 = np.array([[  0.0,   0.0,   0.0], 
                      [120.0, 165.0,-300.0],
                      [160.0,  90.0,   0.0],
                      [  0.0,   0.0,-300.0],
                      [ 20.0, 180.0,   0.0], 
                      [180.0,   0.0,-300.0]])


# #for the 6 bar tensegrity

# inSensorLength_6 = [5.000, 5.000, 5.000, 5.000, 5.000, 5.000, 5.000, 5.000, 5.000, 5.000, 5.000, 5.000, 
# 5.000, 5.000, 5.000, 5.000, 5.000, 5.000, 5.000, 5.000, 5.000, 5.000, 5.000, 5.000]

# #initialize the nodes. 
# #Generic starting point

# #XYZ = [0 0 0; 0 0 L; 2*L/3 0 0; 2*L/3 0 L;
# #         -L/6 -L/3 L/2; 5*L/6 -L/3 L/2; -L/6 L/3 L/2; 5*L/6 L/3 L/2;
# #L/3 -L/6 L/6; L/3 5*L/5 L/6; L/3 -L/6 5*L/6; L/3 5*L/6 5*L/6]


# inNodes_6 =  np.array([[-0.0000, -0.0000, 0.0000], [7.6280, 39.4160, 347.6898],
# [198.4694, 12.9685, -5.8247], [206.0987, 52.3844, 341.8650], [-64.7346, -83.8566, 187.0893], [284.3709, -61.0456, 176.8439],
# [-78.2734, 113.4306, 165.0211], [270.8321, 136.2414, 154.7753],
# [112.7877, -158.5246, 91.5090],  [88.9734, 188.4999, 52.6908],
# [117.1244, -136.1160, 289.1745], [93.3102, 210.9086, 250.3563]]) 

# inPairs_6 = [[1, 2], [3, 4], [5, 6], [7 ,8], [9, 10], [11, 12], 
# [1, 5], [1, 7], [1, 9], [1, 10], [2, 5], [2, 7], [2, 11], 
# [2, 12], [3, 6], [3, 8], [3, 9], [3, 10], [4, 6], [4, 8], 
# [4, 11], [4, 12], [5, 9], [5, 11], [6, 9], [6, 11], [7, 10], 
# [7, 12], [8, 10], [8, 12]]

# # 6 bar tensegrity The nodes that comprise each face.    
# Faces_6 = [[1,5,7], [1,5,9],[1,9,3],[1,7,10], [1,3,10],
#         [3,9,6], [3,6,8], [3,8,10],
#         [2,5,7], [2,5,11], [2,7,12], [2,4,11], [2,4,12],
#         [4,11,6] ,[4,8,12], [4,6,8],
#         [5,11,9], [11,6,9] ,[7,12,10], [8,12,10]]

# desiredFace = 14
 



# for counter in range(0, number_of_rods):

#   node_position[counter] = np.array([inNodes_3[counter][0], inNodes_3[counter][1], inNodes_3[counter][2]])



# # RodColors=['r','g','b']
# RodColors=['g','b','r']
# NodesColors=['red','magenta','darkred','orange',
# 			'green','lightgreen','lime','olive',
# 			'blue','cyan','darkblue','darkmagenta']

# styles=['xkcd:red','xkcd:green','xkcd:blue','xkcd:pink','xkcd:purple','xkcd:brown','xkcd:light blue','xkcd:teal','xkcd:orange','xkcd:light green','xkcd:magenta','xkcd:yellow','xkcd:grey','xkcd:lime green','xkcd:violet','xkcd:dark green','xkcd:turquoise','xkcd:lavender','xkcd:dark blue','xkcd:tan','xkcd:cyan','xkcd:mauve','xkcd:olive','xkcd:salmon','xkcd:black','xkcd:hot pink','xkcd:light pink','xkcd:indigo','xkcd:mustard','xkcd:peach','xkcd:sea green']


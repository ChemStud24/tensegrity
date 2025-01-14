import numpy as np
from numpy import linalg, number

from scipy import optimize
from Tensegrity_model_inputs import number_of_rods, L, inNodes_3, inPairs_3 #, inNodes_6, inSensorLength_6, inPairs_6
import time
from scipy.spatial.transform import Rotation as R


def to_centroid(points):
    centroid = np.mean(points,axis=0)
    return points - centroid

def CableLengthError(inNodes,inPairs,inSensorLength):


	epsilon = np.zeros((len(inPairs)-number_of_rods,1))


	for i in range(number_of_rods,len(inPairs)):

		epsilon[i - number_of_rods] = linalg.norm(inNodes[inPairs[i][0]]-inNodes[inPairs[i][1]]) - inSensorLength[i-number_of_rods]

	error = linalg.norm(epsilon)

	return error


def oneDto3D(nodes1D):
		
    inNodes = np.reshape(nodes1D,(2 * number_of_rods,3))
	
    return inNodes

def CableLengthError1D(inNodes, inPairs, inSensorLength, nodes1D):

    inNodes = oneDto3D(nodes1D)


    return CableLengthError(inNodes,inPairs,inSensorLength)

def vectorError(angle, nodes, vectors):
 	rotation = R.from_euler('xyz',np.array(angle),degrees=True)
 	centroid = np.mean(nodes,axis=0)
 	nodes = nodes - centroid
 	newNodes = rotation.apply(np.array(nodes))

 	# node_ids specifies the order of the IMUs and their direction on the bars
 	# for example, if the first list in node_ids is [3,2],
 	# that means the first IMU is on the bar with nodes 3 and 2,
 	# the y-axis of that IMU points from node 2 to node 3.
 	node_ids = [[5,4],[1,0],[2,3]]
 	# node_ids = [[4,5],[0,1]]
 	node_ids = node_ids[:len(vectors)] #only take as many there are IMUs
 	errorVector = linalg.norm(np.array([linalg.norm((newNodes[ids[0]] - newNodes[ids[1]]) / linalg.norm(newNodes[ids[0]] - newNodes[ids[1]]) - vec) for vec,ids in zip(vectors,node_ids)]))
 	# print(errorVector)
 	return errorVector

# OLD ERROR FUNCTION
# def vectorError(angle, nodes, vec23, vec01):
#  	rotation = R.from_euler('xyz',np.array(angle),degrees=True)
#  	# rotation = rotation.inv()
#  	# print('nodes')
#  	# print(nodes)

#  	centroid = np.mean(nodes,axis=0)
#  	nodes = nodes - centroid

#  	#print('newNodes')
#  	#print(newNodes)
#  	newNodes = rotation.apply(np.array(nodes))
#  	# print('newNodes')
#  	# print(newNodes)
#  	errorVector = linalg.norm(np.array([linalg.norm((newNodes[0] - newNodes[1]) / linalg.norm(newNodes[0] - newNodes[1]) - vec23),linalg.norm((newNodes[4] - newNodes[5]) / linalg.norm(newNodes[4] - newNodes[5]) - vec01)]))
#  	return errorVector
 	

def CombinedError(inNodes,inPairs,inSensorLength,vectors,nodes1D):
	# length error
	length_err = CableLengthError1D(inNodes,inPairs,inSensorLength,nodes1D)

	# rotation error
	node_ids = [[5,4],[1,0],[2,3]]
	node_ids = node_ids[:len(vectors)] #only take as many there are IMUs
	rotation_err = linalg.norm(np.array([linalg.norm((inNodes[ids[0]] - inNodes[ids[1]]) / linalg.norm(inNodes[ids[0]] - inNodes[ids[1]]) - vec) for vec,ids in zip(vectors,node_ids)]))
	print("Length: ",length_err)
	print("Rotation: ",rotation_err)
	# combine
	return length_err + 800*rotation_err

def calculateNodePositions(inNodes,inPairs,inSensorLength,imu,angle0=[0,0,0]):
	
	if np.shape(inNodes) == (2 * number_of_rods, 3):

		inNodes0 = np.ndarray.flatten(inNodes)


		
	costFunction = lambda nodes1D: CableLengthError1D(inNodes, inPairs, inSensorLength, nodes1D)
	ctr=0
	for i in imu:
		print(ctr,linalg.norm(np.array(i)))
		ctr+=1
	imu = [np.array(i)/linalg.norm(np.array(i)) for i in imu] # normalize

	# costFunction = lambda nodes1D: CombinedError(inNodes, inPairs, inSensorLength,imu, nodes1D)
	
	# nonlcon = (
	# {'type': 'ineq', 'fun': lambda nodes: linalg.norm((oneDto3D(nodes))[inPairs[0][0]-1]-(oneDto3D(nodes))[inPairs[0][1]-1]) - L},
	# {'type': 'ineq', 'fun': lambda nodes: linalg.norm((oneDto3D(nodes))[inPairs[1][0]-1]-(oneDto3D(nodes))[inPairs[1][1]-1]) - L},
	# {'type': 'ineq', 'fun': lambda nodes: linalg.norm((oneDto3D(nodes))[inPairs[2][0]-1]-(oneDto3D(nodes))[inPairs[2][1]-1]) - L})

	nonlcon = (
	{'type': 'eq', 'fun': lambda nodes: linalg.norm((oneDto3D(nodes))[inPairs[0][0]]-(oneDto3D(nodes))[inPairs[0][1]]) - L},
	{'type': 'eq', 'fun': lambda nodes: linalg.norm((oneDto3D(nodes))[inPairs[1][0]]-(oneDto3D(nodes))[inPairs[1][1]]) - L},
	{'type': 'eq', 'fun': lambda nodes: linalg.norm((oneDto3D(nodes))[inPairs[2][0]]-(oneDto3D(nodes))[inPairs[2][1]]) - L})
	# {'type': 'eq', 'fun': lambda nodes: linalg.norm(((oneDto3D(nodes))[4]-(oneDto3D(nodes))[5])/linalg.norm((oneDto3D(nodes))[4]-(oneDto3D(nodes))[5]) - imu[0])},
	# {'type': 'eq', 'fun': lambda nodes: linalg.norm(((oneDto3D(nodes))[2]-(oneDto3D(nodes))[3])/linalg.norm((oneDto3D(nodes))[2]-(oneDto3D(nodes))[3]) - imu[1])})
	

	# Lower/Upper bounds set by assuming that the final node positions will never exceed twice the bar length.
	#MinMaxBounds = (-L*2*np.ones((12,3)), L*2*np.ones((12,3)))

	#for a 3 bar tensegrity we use 18
	#for a 6 bar tensegrity, we use 36
	MinMaxBounds =[(-L*2,L*2) for i in range(0, 18)]	# Expressed as 1D-list; bounds = ((min,max), (min,max)... )

	
	result = optimize.minimize(costFunction, inNodes0 , method='SLSQP' , bounds=MinMaxBounds , constraints=nonlcon , options={'ftol': 1e-2})

	answer = result.x
	#print(result.success)

	#print(((oneDto3D(answer))[inPairs[1][0]]-(oneDto3D(answer))[inPairs[1][1]]))
	#print(linalg.norm(((oneDto3D(answer))[inPairs[1][0]]-(oneDto3D(answer))[inPairs[1][1]])/linalg.norm((oneDto3D(answer))[inPairs[1][0]]-(oneDto3D(answer))[inPairs[1][1]]) - vec01))

	coordinates_1 = []
	coordinates_2 = []
	coordinates_3 = []
	coordinates_4 = []
	coordinates_5 = []
	coordinates_6 = []

	for counter in range(0, 3):
		coordinates_1.append(answer[counter])
		
	for counter in range(3, 6):
		coordinates_2.append(answer[counter])

	for counter in range(6, 9):
		coordinates_3.append(answer[counter])

	for counter in range(9, 12):
		coordinates_4.append(answer[counter])

	for counter in range(12, 15):
		coordinates_5.append(answer[counter])

	for counter in range(15, 18):
		coordinates_6.append(answer[counter])



	coordinates = []

	coordinates.append(coordinates_1)
	coordinates.append(coordinates_2)
	coordinates.append(coordinates_3)
	coordinates.append(coordinates_4)
	coordinates.append(coordinates_5)
	coordinates.append(coordinates_6)

	# for pair in inPairs:
	# 	print(pair)
	# 	print(np.linalg.norm(np.array(coordinates[pair[0]]) - np.array(coordinates[pair[1]])))

	# t = time.time()
	# # get the direction of each bar in the global frame as measured by the IMU
	# y = np.array([0,1,0])
	# # on the bar with node 0 and node 1, the IMU's +y-axis points toward node 0
	# r01 = R.from_euler('zxy',imu[0,:],degrees=True)
	# # r01 = R.from_quat(imu[0,:])
	# # r01 = r01.inv()
	# vec01 = r01.apply(y)
	# # vec01 = np.array([np.sqrt(2)/2,np.sqrt(2)/2,0])
	# #print(linalg.norm(vec01))
	# vec01 = np.array([-240.0,110.0,-160.0])
	# vec01 = vec01/linalg.norm(vec01)

	# # on the bar with node 2 and node 3, the IMU's +y-axis points toward node 2
	# r23 = R.from_euler('zxy',imu[1,:],degrees=True)
	# # r23 = R.from_quat(imu[1,:])
	# # r23 = r23.inv()
	# vec23 = r23.apply(y)
	# # vec23 = np.array([1,0,0])
	# #print(vec23)
	# vec23 = np.array([275.0,85.0,80.0])
	# vec23 = vec23/linalg.norm(vec23)

	#print(coordinates)

	# return coordinates

	if len(imu) > 0:
		costFunctionRotation = lambda angle: vectorError(angle, coordinates, imu)
		
		
		rotationBounds = [(-360,360) for i in range(0, 3)]
		# print('rotationBounds')
		# print(rotationBounds)
		# costFunctionRotation = lambda angle0: vectorError(angle0, coordinates, vec23, vec01)
		# result_rotation = optimize.minimize(costFunctionRotation, angle0 , method='SLSQP', bounds=rotationBounds , options={'ftol': 1e-9})
		result_rotation = optimize.minimize(costFunctionRotation, angle0 , method='l-BFGS-B', bounds=rotationBounds, options={'ftol': 1e-6})
		
		answer = result_rotation.x

	else:
		answer = [0,0,0]



	# for counter in range(0, 3):
	# 	coordinates_rotation_1.append(answer[counter])
		
	# for counter in range(3, 6):
	# 	coordinates_rotation_2.append(answer[counter])

	# for counter in range(6, 9):
	# 	coordinates_rotation_3.append(answer[counter])

	# for counter in range(9, 12):
	# 	coordinates_rotation_4.append(answer[counter])

	# for counter in range(12, 15):
	# 	coordinates_rotation_5.append(answer[counter])

	# for counter in range(15, 18):
	# 	coordinates_rotation_6.append(answer[counter])



	# coordinates_rotation = []

	# coordinates_rotation.append(coordinates_rotation_1)
	# coordinates_rotation.append(coordinates_rotation_2)
	# coordinates_rotation.append(coordinates_rotation_3)
	# coordinates_rotation.append(coordinates_rotation_4)
	# coordinates_rotation.append(coordinates_rotation_5)
	# coordinates_rotation.append(coordinates_rotation_6)
	#return coordinates_rotation
	rotation_opt = R.from_euler('xyz',answer,degrees=True)
	# rotation_opt = rotation_opt.inv()
	newNodes_opt = rotation_opt.apply(np.array(coordinates))
	# print(newNodes_opt)

	# print(newNodes_opt)

	# for i in imu:
	# 	print('before',i)
	# for ids in [[2,3],[4,5],[1,0]]:
	# 	print('after',(newNodes_opt[ids[0]] - newNodes_opt[ids[1]]) / linalg.norm(newNodes_opt[ids[0]] - newNodes_opt[ids[1]]))
	# print('vec23',vec23)
	# print('before',np.array(imu[0])/linalg.norm(np.array(imu[0])))
	# print('before',vecs[0])
	# print('after',(newNodes_opt[0] - newNodes_opt[1]) / linalg.norm(newNodes_opt[0] - newNodes_opt[1]))

	# print('vec01',vec01)
	# print('before',np.array(imu[1])/linalg.norm(np.array(imu[1])))
	# print('before',vecs[1])
	# print('after',(newNodes_opt[4] - newNodes_opt[5]) / linalg.norm(newNodes_opt[4] - newNodes_opt[5]))
	# print(time.time()-t)
	# print(answer)

	#lengths
	# print("Lengths:")
	# for pair in inPairs:
	# 	print(linalg.norm(np.array(coordinates[pair[0]]) - np.array(coordinates[pair[1]])))
	return to_centroid(newNodes_opt)
	#return coordinates













    



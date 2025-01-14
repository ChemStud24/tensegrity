import numpy as np

symmetry_mapping = {(0,2,5):[0,1,2,3,4,5],(0,3,5):[0,1,2,3,4,5],
		   (1,2,4):[1,2,0,4,5,3],(1,2,5):[1,2,0,4,5,3],
		   (0,3,4):[2,0,1,5,3,4],(1,3,4):[2,0,1,5,3,4]}

reverse_mapping = {(0,2,5):[3,5,4,0,2,1],(0,3,5):[3,5,4,0,2,1],
			(1,2,4):[4,3,5,1,0,2],(1,2,5):[4,3,5,1,0,2],
			(0,3,4):[5,4,3,2,1,0],(1,3,4):[5,4,3,2,1,0]}

next_nodes = {(0,2,5):(1,2,4),(0,3,5):(1,2,4),
			(1,2,4):(0,3,4),(1,2,5):(0,3,4),
			(0,3,4):(0,2,5),(1,3,4):(0,2,5)}

prev_nodes = {(0,2,5):(1,3,4),(0,3,5):(1,3,4),
			  (0,3,4):(1,2,5),(1,3,4):(1,2,5),
			  (1,2,4):(0,3,5),(1,2,5):(0,3,5)}

def transform_gait(gait,bottom_nodes):
	# gait is an N x 6 numpy array where N is the number of steps in the gait
	# bottom_nodes is a 3-tuple of the lowest nodes to the ground
	mapping = symmetry_mapping.get(bottom_nodes)
	if mapping == None:
		return None
	new_gait = np.array([[step[m] for m in mapping] for step in gait])
	return new_gait

def reverse_gait(gait,bottom_nodes):
	# gait is an N x 6 numpy array where N is the number of steps in the gait
	mapping = reverse_mapping.get(bottom_nodes)
	new_gait = np.array([[step[m] for m in mapping] for step in gait])
	return new_gait

if __name__ == '__main__':
	# test cases

	# this is the base unit of the quasistatic rolling gait when 0,2,5 are the bottom nodes
	states = np.array([[0.0, 1.0, 1.0, 0.0, 1.0, 0.1],[1.0, 0.1, 1.0, 1.0, 0.1, 1.0]])
	# if 0,2,5 are the bottom nodes
	bottom_nodes = (0,2,5)
	# it should print out the exact same gait: [[0.0, 1.0, 1.0, 0.0, 1.0, 0.1],[1.0, 0.1, 1.0, 1.0, 0.1, 1.0]]
	print(transform_gait(states,bottom_nodes))

	# this is the base unit of the quasistatic rolling gait when 0,2,5 are the bottom nodes
	states = np.array([[0.0, 1.0, 1.0, 0.0, 1.0, 0.1],[1.0, 0.1, 1.0, 1.0, 0.1, 1.0]])
	# after one step 1,2,4 are the bottom nodes
	bottom_nodes = (1,2,4)
	# it should print out the next step in the gait: [[1.0, 1.0, 0.0, 1.0, 0.1, 0.0],[0.1, 1.0, 1.0, 0.1, 1.0, 1.0]]
	print(reverse_gait(transform_gait(states,bottom_nodes),bottom_nodes))

	# this is the base unit of the quasistatic rolling gait when 0,2,5 are the bottom nodes
	states = np.array([[0.0, 1.0, 1.0, 0.0, 1.0, 0.1],[1.0, 0.1, 1.0, 1.0, 0.1, 1.0]])
	# after two steps 0,3,4 are the bottom nodes
	bottom_nodes = (0,3,4)
	# it should print out the last step in the gait: [[1.0, 0.0, 1.0, 0.1, 0.0, 1.0],[1.0, 1.0, 0.1, 1.0, 1.0, 0.1]]
	print(reverse_gait(transform_gait(states,bottom_nodes),bottom_nodes))

	# this is the base unit of the quasistatic rolling gait when 0,2,5 are the bottom nodes
	states = np.array([[0.0, 1.0, 1.0, 0.0, 1.0, 0.1],[1.0, 0.1, 1.0, 1.0, 0.1, 1.0]])
	# I don't know what the reverse gait is supposed to look like since we've never done it :)
	print(reverse_gait(states,bottom_nodes))
	# I hope it's reasonable
	# update: it seems reasonable
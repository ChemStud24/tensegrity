
import numpy as np
from math import sqrt
import sys
import os
import json
# import matplotlib.pyplot as plt
# from matplotlib.figure import Figure

def dist(points):
	point1,point2 = points
	x1,y1,z1 = point1
	x2,y2,z2 = point2
	return sqrt((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)

def get_points(dic,pair):
	# gets the points from a dictionary of indexed points given the pair of points that is needed
	return dic[pair[0]],dic[pair[1]]

def get_all_keys(parent_key,data):
	all_keys = []
	keys = sorted(data.keys())
	for key in keys:
		if type(data[key]) is dict:
			if parent_key == '':
				all_keys.extend(get_all_keys(str(key),data[key]))
			else:
				all_keys.extend(get_all_keys(parent_key + '_' + str(key),data[key]))
		else:
			if parent_key == '':
				all_keys.append(str(key))
			else:
				all_keys.append(parent_key + '_' + str(key))
	return all_keys

def get_all_data(data):
	all_data = []
	keys = sorted(data.keys())
	for key in keys:
		if type(data[key]) is dict:
			all_data.extend(get_all_data(data[key]))
		else:
			all_data.append(data[key])
	return all_data

if __name__ == '__main__':
	trial_dir = sys.argv[1]
	data_dir = os.path.join(trial_dir,'data')
	if len(sys.argv) > 2:
		output_filename = sys.argv[2]
	else:
		output_filename = "data.csv"
	data_files = sorted(os.listdir(data_dir))

	# initialize the first line of the file with all the keys
	data = json.load(open(os.path.join(data_dir,data_files[0])))
	open(os.path.join(trial_dir,output_filename),'w').write(','.join(key for key in get_all_keys('',data)) + '\n')
	# all_keys = get_all_keys('',data)

	# extract the data and write to a line
	for df in data_files:
		print('Recording data ' + os.path.join(data_dir,df))
		data = json.load(open(os.path.join(data_dir,df)))
		open(os.path.join(trial_dir,output_filename),'a').write(','.join(str(datum) for datum in get_all_data(data)) + '\n')
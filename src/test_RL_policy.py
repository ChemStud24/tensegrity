import os
import sys
import json
import numpy as np
from policy import ctrl_policy

dataset = "C:/Users/willj/Documents/tensegrity-dataset/RL_test_data"

if len(sys.argv) > 1:
	trial = sys.argv[1]
else:
	print("Specify trial name")

data_dir = os.path.join(dataset,trial,'data')
datafiles = sorted(os.listdir(data_dir))

CTRL = ctrl_policy(7) # set frame rate

for df in datafiles:
	data = json.load(open(os.path.join(data_dir,df),'r'))
	targets = [data.get('motors').get(str(i)).get('target') for i in range(6)]
	endcaps = np.array([[data.get('endcaps').get(key).get(k) for k in sorted(data.get('endcaps').get(key).keys())] for key in sorted(data.get('endcaps').keys())])
	endcaps = endcaps.flatten()
	# print('End Caps: ',endcaps)
	action = CTRL.get_action(endcaps)
	print('===============')
	print(df)
	print('Real Action: ',targets)
	print('RL Action: ',action)
	print('\n')
import os
import sys
import json
import numpy as np
import matplotlib.pyplot as plt

dataset = "/home/willjohnson/tensegrity_ws/data"

if len(sys.argv) > 1:
	trial = sys.argv[1]
else:
	print("Specify trial name")

data_dir = os.path.join(dataset,trial,'data')
datafiles = sorted(os.listdir(data_dir))

real_actions = []
real_commands = []
timestamps = []

plot_replay_command = False

if plot_replay_command:
    from policy import ctrl_policy
    policy_pos = ctrl_policy(8) # set frame rate
    rl_commands = []

for df in datafiles:
	data = json.load(open(os.path.join(data_dir,df),'r'))
	targets = [data.get('motors').get(str(i)).get('target') for i in range(6)]
	endcaps = np.array([[data.get('endcaps').get(key).get(k) for k in sorted(data.get('endcaps').get(key).keys())] for key in sorted(data.get('endcaps').keys())])
	action_state = [data.get('motors').get(str(i)).get('position') for i in range(6)]
	action_state = np.array(action_state)
	real_actions.append(action_state)
	real_commands.append(targets)
	timestamps.append(data.get('header').get('secs'))
	if plot_replay_command:
		if policy_pos.target_pt is None:
			policy_pos.reset_target_point(endcaps)
		action = policy_pos.get_action(endcaps, action_state)
		rl_commands.append(action)
		
real_actions = np.array(real_actions)
real_commands = np.array(real_commands)
timestamps = np.array(timestamps)
timestamps = timestamps - timestamps[0]

if plot_replay_command == False:
    plt.figure(figsize=(12, 8))
    plt.subplot(6, 1, 1)
    plt.plot(timestamps,real_actions[:, 0], marker='.', linestyle='-',label='Tendon Length')
    plt.plot(timestamps,real_commands[:, 0], linestyle='-',label='Commanded Length')
    plt.ylim([-0.1,1.1])
    plt.legend()
    plt.grid()
    plt.subplot(6, 1, 2)
    plt.plot(timestamps,real_actions[:, 1], marker='.', linestyle='-')
    plt.plot(timestamps,real_commands[:, 1], linestyle='-')
    plt.grid()
    plt.ylim([-0.1,1.1])
    plt.subplot(6, 1, 3)
    plt.plot(timestamps,real_actions[:, 2], marker='.', linestyle='-')
    plt.plot(timestamps,real_commands[:, 2], linestyle='-')
    plt.grid()
    plt.ylim([-0.1,1.1])
    plt.subplot(6, 1, 4)
    plt.plot(timestamps,real_actions[:, 3], marker='.', linestyle='-')
    plt.plot(timestamps,real_commands[:, 3], linestyle='-')
    plt.grid()
    plt.ylim([-0.1,1.1])
    plt.subplot(6, 1, 5)
    plt.plot(timestamps,real_actions[:, 4], marker='.', linestyle='-')
    plt.plot(timestamps,real_commands[:, 4], linestyle='-')
    plt.grid()
    plt.ylim([-0.1,1.1])
    plt.subplot(6, 1, 6)
    plt.plot(timestamps,real_actions[:, 5], marker='.', linestyle='-')
    plt.plot(timestamps,real_commands[:, 5], linestyle='-')
    plt.grid()
    plt.ylim([-0.1,1.1])
    plt.tight_layout()
    plt.xlabel('Time (s)')
    plt.show()
else:
    rl_commands = np.array(rl_commands)
    plt.figure(figsize=(12, 8))
    plt.subplot(6, 1, 1)
    plt.plot(real_actions[:, 0], marker='.', linestyle='-')
    plt.plot(real_commands[:, 0], linestyle='-')
    plt.plot(rl_commands[:, 0], linestyle='-')
    plt.grid()
    plt.subplot(6, 1, 2)
    plt.plot(real_actions[:, 1], marker='.', linestyle='-')
    plt.plot(real_commands[:, 1], linestyle='-')
    plt.plot(rl_commands[:, 1], linestyle='-')
    plt.grid()
    plt.subplot(6, 1, 3)
    plt.plot(real_actions[:, 2], marker='.', linestyle='-')
    plt.plot(real_commands[:, 2], linestyle='-')
    plt.plot(rl_commands[:, 2], linestyle='-')
    plt.grid()
    plt.subplot(6, 1, 4)
    plt.plot(real_actions[:, 3], marker='.', linestyle='-')
    plt.plot(real_commands[:, 3], linestyle='-')
    plt.plot(rl_commands[:, 3], linestyle='-')
    plt.grid()
    plt.subplot(6, 1, 5)
    plt.plot(real_actions[:, 4], marker='.', linestyle='-')
    plt.plot(real_commands[:, 4], linestyle='-')
    plt.plot(rl_commands[:, 4], linestyle='-')
    plt.grid()
    plt.subplot(6, 1, 6)
    plt.plot(real_actions[:, 5], marker='.', linestyle='-')
    plt.plot(real_commands[:, 5], linestyle='-')
    plt.plot(rl_commands[:, 5], linestyle='-')
    plt.grid()
    plt.tight_layout()
    plt.show()
import numpy as np
import pickle

def principal_axis(nodes):
	point024 = (nodes[0,:] + nodes[2,:] + nodes[4,:])/3
	point135 = (nodes[1,:] + nodes[3,:] + nodes[5,:])/3
	axis = point024 - point135
	axis = axis/(np.linalg.norm(axis))
	return axis

with open('displacement_dict_rolling.pkl','rb') as f:
	data = pickle.load(f)
"""
turning_actions = ['135cw','135ccw','024cw','024ccw']
for turning_action in turning_actions:
    filename = 'displacement_dict' + '_' + turning_action + '.pkl'
    with open(filename,'rb') as f:
        this_data = pickle.load(f)
        roll2turn = {key + '__' + turning_action:[value[0],value[2]] for key,value in this_data.items()}
        data.update(roll2turn)
        turn2turn = {turning_action + '__' + turning_action:[value[1],value[2]] for key,value in this_data.items()}
        data.update(turn2turn)
        # data.update({key + '__' + turning_action:value for key,value in pickle.load(f).items()})

# turning to rolling
turning_sides = ['135','024']
for turning_side in turning_sides:
    filename = 'displacement_dict_after_turn' + turning_side + '.pkl'
    with open(filename,'rb') as f:
        this_data = pickle.load(f)
        for turning_action in turning_actions:
            if turning_side in turning_action:
                turn2roll = {turning_action + '__' + key:value for key,value in this_data.items()}
                data.update(turn2roll)
"""
turning_actions = ['cw','ccw']
for turning_action in turning_actions:
    with open('positions_flip_' + turning_action + '.pkl','rb') as f:
        this_data = pickle.load(f)
        data.update({'rest__' + turning_action:this_data})
        for ta in turning_actions:
            data.update({ta + '__' + turning_action:this_data})
        data.update({turning_action + '__rest':[this_data[-1],this_data[-1]]})

with open('displacement_dict_transfer.pkl','rb') as f:
    this_data = pickle.load(f)
    rest2transition = {'rest__' + key:[value[0],value[1]] for key,value in this_data.items()}
    data.update(rest2transition)
    transition2rest = {key + '__rest':[value[1],value[2]] for key,value in this_data.items()}
    data.update(transition2rest)

printkey = '90_90__90_90'

# print('90_90__90_90' in data.keys())
for key in data.keys():
    before_points = data.get(key)[0]
    after_points = data.get(key)[1]
    if key == '90_90__90_90':
        # print(np.mean(before_points,axis=0))
        # print(np.mean(after_points,axis=0))
        t = np.mean(after_points,axis=0) - np.mean(before_points,axis=0)
        # print(t/10)
        before_axis = principal_axis(before_points)
        print('Before Axis: ',before_axis)
        after_axis = principal_axis(after_points)
        print('After Axis: ',after_axis)
        # print(before_axis)
        # print(after_axis)

    # 2D
    # before_points = np.hstack((before_points[:,0:1],before_points[:,2:]))
    # after_points = np.hstack((after_points[:,0:1],after_points[:,2:]))
    before_points = before_points[:,:2]
    after_points = after_points[:,:2]

    # change in COM and heading
    t = np.mean(after_points,axis=0) - np.mean(before_points,axis=0)
    if key == printkey:
        print(before_points)
        print('t: ',t)
        print(np.linalg.norm(t))
    before_axis = principal_axis(before_points)
    after_axis = principal_axis(after_points)
    # print('Before Axis: ',before_axis)
    # print('After Axis: ',after_axis)
    # R = np.matmul(np.reshape(before_axis,(2,1)),np.reshape(after_axis,(1,2)))
    # delta_axis = after_axis - before_axis
    # delta_angle = np.arctan2(delta_axis[1],delta_axis[0])
    # R = np.array([[np.cos(delta_angle),np.sin(delta_angle)],[-np.sin(delta_angle),np.cos(delta_angle)]])
    # data.update({key:[R[:2,:2],t[:2]/10]})

    # local frame before
    local_t = np.mean(before_points)
    local_x = before_axis
    theta = np.arctan2(local_x[1],local_x[0])
    local_R = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
    t = np.matmul(local_R.T,np.reshape(t,(2,1)))
    # after
    theta_after = np.arctan2(after_axis[1],after_axis[0])
    R_after = np.array([[np.cos(theta_after),-np.sin(theta_after)],[np.sin(theta_after),np.cos(theta_after)]])
    R = np.matmul(local_R.T,R_after)
    
    # print('R: ',R)
    # R = np.matmul(local_R.T,R)
    # R = local_R
    if key == printkey:
        print('transformed_t',t)
        print(np.linalg.norm(t))
    data.update({key:[R,t/10]})

# eliminate all "rest" intermediates
new_data = {key:value for key,value in data.items() if not 'rest' in key}
for key1 in data.keys():
    # print('Key One: ' + key1)
    state1,action1 = key1.split('__')
    if action1 == 'rest':
        # print('From state ' + state1 + '...')
        R1,t1 = data.get(key1)
        for key2 in data.keys():
            state2,action2 = key2.split('__')
            if state2 == 'rest':
                # if 'cw' in state1 or 'cw' in action2:
                if 'cw' in action2:
                    # print('Exploring action ' + action2 + '.')
                    # make a new key in new_data that is
                    # state1__action2
                    R2,t2 = data.get(key2)
                    t2 = np.matmul(R2.T,t2) # transform t2 into the world frame first
                    t = np.matmul(R1.T,t2) + t1
                    R = np.matmul(R1,R2)
                    new_data.update({state1 + '__' + action2:[R,t]})
                    print('Adding ' + state1 + '__' + action2 + ' to the dictionary.')
                elif 'cw' in state1:
                    R2,t2 = data.get(key2)
                    t2 = np.matmul(R2.T,t2) # transform t2 into the world frame first
                    t1 = np.matmul(R1.T,t2) + t1
                    R1 = np.matmul(R1,R2)
                    # after going into the transition state, do a full roll with those lenghts
                    R2,t2 = data.get(action2 + '__' + action2)
                    t2 = np.matmul(R2.T,t2) # transform t2 into the world frame first
                    t = np.matmul(R1.T,t2) + t1
                    R = np.matmul(R1,R2)
                    new_data.update({state1 + '__' + action2:[R,t]})
                    print('Adding ' + state1 + '__' + action2 + ' to the dictionary.')

pickle.dump(new_data,open('transformation_table.pkl','wb'))

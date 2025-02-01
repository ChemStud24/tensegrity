import numpy as np
import pickle

def principal_axis(nodes):
	point024 = (nodes[0,:] + nodes[2,:] + nodes[4,:])/3
	point135 = (nodes[1,:] + nodes[3,:] + nodes[5,:])/3
	axis = point024 - point135
	axis = axis/(np.linalg.norm(axis))
	return axis

with open('../calibration/new_platform_primitive_dict.pkl','rb') as f:
	data = pickle.load(f)

# turning_actions = ['cw','ccw']
# for turning_action in turning_actions:
#     with open('positions_flip_' + turning_action + '.pkl','rb') as f:
#         this_data = pickle.load(f)
#         data.update({'rest__' + turning_action:this_data})
#         for ta in turning_actions:
#             data.update({ta + '__' + turning_action:this_data})
#         data.update({turning_action + '__rest':[this_data[-1],this_data[-1]]})

# with open('displacement_dict_transfer.pkl','rb') as f:
#     this_data = pickle.load(f)
#     rest2transition = {'rest__' + key:[value[0],value[1]] for key,value in this_data.items()}
#     data.update(rest2transition)
#     transition2rest = {key + '__rest':[value[1],value[2]] for key,value in this_data.items()}
#     data.update(transition2rest)

printkey = '90_90__90_90'

# print('90_90__90_90' in data.keys())
for key in data.keys():
    before_points = data.get(key)[0]
    after_points = data.get(key)[1]

    # for planning around vertical obstacles
    max_height = max(max(before_points[:,2]),max(after_points[:,2]))
    print(key,max_height)

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
    
    if key == printkey:
        print('transformed_t',t)
        print(np.linalg.norm(t))
    data.update({key:[R,t/10,max_height/10]})

pickle.dump(data,open('../calibration/new_platform_transformation_table.pkl','wb'))

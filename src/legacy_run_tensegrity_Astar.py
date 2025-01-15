#!/usr/bin/env python3
import os
import sys
import serial
import time
import math
from math import cos,sin,tan
import re
import json
import xlrd
import numpy as np
from pynput import keyboard
from scipy.spatial.transform import Rotation as R
import scipy.spatial
from scipy import spatial

# ROS messages
import rospy
# from tensegrity.msg import Motor, Info, MotorsStamped, Sensor, SensorsStamped, Imu, ImuStamped, Node, NodesStamped
from tensegrity.msg import Motor, Info, Sensor, Imu, Node, Trajectory, TensegrityStamped, State, Action
import rospkg

# tracking
import rosgraph
import cv_bridge
from std_msgs.msg import Float64MultiArray
from tensegrity_perception.srv import InitTracker, InitTrackerRequest, InitTrackerResponse
from tensegrity_perception.srv import GetPose, GetPoseRequest, GetPoseResponse
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

# trajectory following
import pickle
from plotting_utils import plot_MPC_prediction
from symmetry_reduction_utils import *
from Tensegrity_model_inputs import *
from points_superimposed import make_circle
from math import ceil, sqrt

def best_k_actions(state,action_dict,COM,principal_axis,trajectory,k=1):
    global reverse_the_gait
    # COM = np.reshape(COM[0:2],(2,1))
    # principal_axis = principal_axis[:,0:2]
    # principal_axis = np.reshape(principal_axis,(2,1))
    # search dictionary for all possible actions
    min_cost = float('inf')

    for action in action_dict.keys():
        # no rolling/turning
        # if '_' in action.split('__')[-1]:
        #     continue
        if '80' in action.split('__')[-1] or '160' in action.split('__')[-1]:
            continue
        # if state in action[:8]:
        if state == action.split('__')[0]:
            transition_R,transition_t = action_dict.get(action)
            local_y = principal_axis
            theta = np.arctan2(local_y[0,:],local_y[1,:])[0]
            local_R = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
            t = np.matmul(local_R.T,transition_t)
            R = np.matmul(local_R,transition_R)
            if reverse_the_gait and not 'cw' in action.split('__')[1]:
                new_COM = COM - t
            else:
                new_COM = COM + t
            new_PA = np.matmul(transition_R,principal_axis)

            if k == 1:
                C = cost(new_COM,new_PA,trajectory)
                if C < min_cost:
                    min_cost = C
                    best_action_sequence = [action]
                    best_COMs = [new_COM]
                    best_PAs = [new_PA]
            else:
                # C,action_sequence,COMs,PAs = best_k_actions(action.split('__')[1],action_dict,new_COM,new_PA,trajectory,k-1)
                next_state = action.split('__')[1]
                if 'cw' in next_state:
                    next_state = '100_100'
                C,action_sequence,COMs,PAs = best_k_actions(next_state,action_dict,new_COM,new_PA,trajectory,k-1)
                if C < min_cost:
                    min_cost = C
                    best_action_sequence = [action] + action_sequence
                    best_COMs = [new_COM] + COMs
                    best_PAs = [new_PA] + PAs

    return min_cost,best_action_sequence,best_COMs,best_PAs

def bottom3(nodes):
    x_sr,y_sr,z_sr = nodes2sr(nodes)
    z_values = np.array([item[1] for item in sorted(z_sr.items())])
    bottom_nodes = tuple(sorted(np.argpartition(z_values, 3)[:3]))
    return bottom_nodes

def nodes2sr(nodes):
        x_sr = {str(key):nodes[key,0] for key in range(number_of_rods*2)}
        y_sr = {str(key):nodes[key,1] for key in range(number_of_rods*2)}
        z_sr = {str(key):nodes[key,2] for key in range(number_of_rods*2)}
        return x_sr,y_sr,z_sr

def best_action(state,action_dict,COM,principal_axis,trajectory):
    # print('Current COM: ',COM)
    COM = np.reshape(COM[0:2],(2,1))
    # print('COM: ',COM)
    principal_axis = principal_axis[:,0:2]
    # print(principal_axis)
    # search dictionary for all possible actions
    min_cost = float('inf')
    for action in action_dict.keys():
        if state in action[:8]:
            print("Action: ",action)
            transition_R,transition_t = action_dict.get(action)
            local_y = principal_axis
            print('Local Y: ',local_y)
            theta = np.arctan2(local_y[:,0],local_y[:,1])[0]
            print('Theta: ',theta)
            local_R = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
            t = np.matmul(local_R.T,transition_t)
            print('t: ',t)
            R = np.matmul(local_R,transition_R)
            new_COM = COM + t
            print('New COM: ',new_COM)
            # new_COM = new_COM.tolist()
            print(principal_axis.T)
            print(local_y.T)
            print(R)
            C = cost(new_COM,np.matmul(transition_R,principal_axis.T),trajectory)
            print("Cost: ",C)
            if C < min_cost:
                min_cost = C
                best_action = action
    # print("Cost",C)
    return best_action

def norm(vec):
    return vec/np.linalg.norm(vec)

def cost(COM,principal_axis,trajectory):
    R_90 = np.array([[0,1],[-1,0]])
    heading = np.reshape(np.matmul(R_90,principal_axis),(2,))
    # print('Heading: ',heading)

    dist_weight = 400#100#400
    ang_weight = 40#60#80#100#80
    prog_weight = 300#400
    # print('COM: ',COM)
    # print('Trajectory: ',trajectory)
    dists = spatial.distance.cdist(trajectory,COM.T)
    # print('Distances: ',dists)
    dist_cost = dist_weight*np.min(dists)
    mindex = np.argmin(dists)
    # print('Waypoint: ',mindex)

    progression_cost = prog_weight*(1-mindex/trajectory.shape[0])
    # print("Progression",(1-mindex/trajectory.shape[0]))

    if mindex < trajectory.shape[0]-1:
        closest_point = trajectory[mindex,:]
        next_point = trajectory[mindex+1,:]
        tangent = (next_point - closest_point)/np.linalg.norm(next_point - closest_point)
        # print("Tangent: ",tangent)
        dot = np.dot(heading,tangent)
        # cross = np.cross(np.reshape(heading,(2,)),tangent)
        # # print("Dot: ",dot)
        # ang_cost = ang_weight*np.arctan2(np.linalg.norm(cross),np.linalg.norm(dot))

        mag = np.linalg.norm(heading) * np.linalg.norm(tangent)
        # print('MAG:',mag) #mag is always pretty much 1 (>0.98)
        ang_cost = ang_weight*np.abs(np.arccos(dot/mag))
    else:
        # no angular cost if we have reached the last waypoint
        ang_cost = 0

    # print('distance cost: ',dist_cost)
    # print('progression cost: ',progression_cost)
    # print('angular cost: ',ang_cost)
    return dist_cost + progression_cost + ang_cost

def flush(serial_port):
    serial_port.reset_input_buffer()
    serial_port.reset_output_buffer()
    time.sleep(0.001)

def send_command(serial_port, input_string, delay_time):
    # to_microcontroller_msg = f'{input_string}\n'
    to_microcontroller_msg = '          ' + '{}\n'.format(input_string)
    serial_port.write(to_microcontroller_msg.encode('UTF-8'))
    if delay_time < 0:
        delay_time = 0
    time.sleep(delay_time/1000)

def get_imu_calibration(mag,grav):
    """Calculate the IMU's individual calibration using the direction of the magnetic and gravitational fields.
    mag is an iterable of length 3 that represents the magnetic field strength in the IMU's x-, y-, and z-directions
    grav is an iterable of length 3 that represents the gravitational field strength in the IMU's x-, y-, and z-directions
    Returns R, the rotation matrix you can use to pre-multiply all vectors in the IMU frame to transform them into the world frame
    """
    x = np.array(mag)
    x = x/np.linalg.norm(x)
    x = np.reshape(x,(3,1))
    z = -1*np.array(grav)
    z = z/np.linalg.norm(z)
    z = np.reshape(z,(3,1))
    y = np.cross(z,x)
    R = np.hstack(x,y,z)
    return R

def quat2vec(q):
    q0 = float(q[0])
    q1 = float(q[1])
    q2 = float(q[2])
    q3 = float(q[3])
    roll = -math.atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2))#convert quarternion to Euler angle for roll angle
    #convert quarternion to Euler angle for pitch angle
    sinp = 2*(q0*q2-q3*q1)
    #deal with gimlock
    if abs(sinp) >= 1:
        pitch = math.copysign(np.pi/2, sinp)
    else:
        pitch = math.asin(sinp)
    
    #yaw = -math.atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3))-np.pi/2
    #convert quarternion to Euler angle for pitch angle
    yaw = -math.atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3))

    k=np.array([cos(yaw)*cos(pitch), sin(pitch),sin(yaw)*cos(pitch)])
    r = R.from_rotvec(-np.pi/2 * np.array([0, 1, 0]))
    k = r.apply(k)
    y=np.array([0,1,0])
    s=np.cross(k,y)
    v=np.cross(s,k)
    vrot=v*cos(roll)+np.cross(k,v)*sin(roll)
    return np.cross(k,vrot)

def read_calibration_file(filename):
    if filename[-4:] == '.xls':
        # hand calibration excel file
        workbook = xlrd.open_workbook(filename)
        shortsheet = workbook.sheet_by_name('Short Sensors')
        longsheet = workbook.sheet_by_name('Long Sensors')
        m = np.array([float(shortsheet.cell_value(9,col)) for col in range(0,12,2)] + [float(longsheet.cell_value(10,col)) for col in range(0,6,2)])
        b = np.array([float(shortsheet.cell_value(9,col)) for col in range(1,13,2)] + [float(longsheet.cell_value(10,col)) for col in range(1,7,2)])
    elif filename[-5:] == '.json':
        # autocalibration JSON file
        data = json.load(open(filename))
        m = np.array(data.get('m'))
        b = np.array(data.get('b'))
    else:
        error('Invalid calibration file')
    return m,b

def read(serial_port):
    flush(serial_port)
    global state
    global count
    global capacitance
    global acceleration
    global orientation
    global imu
    global P
    global max_speed
    global RANGE
    global MAX_RANGE
    global MIN_RANGE
    global RANGE135
    global RANGE024
    global tol
    global low_tol
    global heading_error
    global cum_head_error
    global prev_head_error
    global d_head_error
    global action_dict
    global trajectory
    global trajectory_segment
    global states
    global prev_gait
    global endcaps
    global COMs
    global PAs
    global reverse_the_gait
    global action_sequence
    global done
    global bar_height_changed
    global dist_weight
    global ang_weight
    global prog_weight
    line = serial_port.read_until(b'\n')# read data string sent from central
    line = str(line)#.encode('utf-8')# convert to string
    line = line.split('*')[-1] # remove padding
    # print('Whole Line: ',line)
    s = re.findall(r"0x[0-9a-f]+", line)# extract strain data (hex number) from the data string
    q = re.findall(r"[-+]?\d*\.\d+|\d+", line)# extract imu data (decimal number) from the data string
    if len(line) >=2 :# valid data string will be longer than 9 characters    
        # print('line')
        # print(line)
        if line[0] == "S":# read strain data
            # now_time = time.time()
            # meas_time = now_time - start_time
            if len(s) >= num_sensors:
                for i in range(num_sensors):
                    cap[i] = s[i]
            # print(cap)
                #add control code here
            if not 0 in cap:
                for i, cap_hex in enumerate(cap):
                    adc_counts = int(str(cap_hex[2:]), 16)
                    if adc_counts != 0:
                        # capacitance[i] = 16.0 * 0.5 / adc_counts / 3.3 * 1024
                        capacitance[i] = 42.0 / adc_counts / 3.3 * 1024
                    length[i] = (capacitance[i] - b[i]) / m[i] #mm #/ 10
                # print(capacitance)
                #check if motor reached the target
                for i in range(num_motors):
                    if i < 3:
                        pos[i] = (length[i] - min_length) / RANGE135# calculate the current position of the motor
                    else:
                        pos[i] = (length[i] - min_length) / RANGE024# calculate the current position of the motor

                    # two tolerances for shorter and longer commands
                    if states[state, i] < 0.5:
                        tolerance = low_tol
                    else:
                        tolerance = tol
                    #check if motor reached the target
                    if pos[i] + tolerance > states[state, i] and pos[i] - tolerance < states[state, i]:
                        send_command(serial_port, "d "+str(i+1)+" 0", 0)#stop the motor
                        done[i] = True
                        command[i] = 0
                    # else:
                    #     done[i] = False
                    # update directions
                    # print(done[i])
                    if not done[i]:
                        error[i] = pos[i] - states[state, i]
                        d_error[i] = error[i] - prev_error[i]
                        cum_error[i] = cum_error[i] + error[i]
                        prev_error[i] = error[i]
                        #update speed
                        command[i] = max([min([P*error[i] + I*cum_error[i] + D*d_error[i], 1]), -1])
                        speed = command[i] * max_speed * flip[i] 
                        send_command(serial_port, "d "+str(i+1)+" "+str(speed), 0)#run the motor at new speed                            
                # print('State: ',state)
                # # print(state)
                # print("Position: ",pos)
                # print("Target: ",states[state])
                # # print(pos)
                # # print(states[state])
                # print("Done: ",done)
                # print("Length: ",length)
                # print("Capacitance: ",capacitance)
                count = count + 1     
                if all(done):
                    state += 1
                    state %= len(states)

                    for i in range(num_motors):
                        done[i] = False
                        prev_error[i] = 0
                        cum_error[i] = 0
                    
                    # # update ranges based on heading error
                    # d_head_error = heading_error - prev_head_error
                    # cum_head_error = cum_head_error + heading_error
                    # prev_head_error = heading_error
                    if 'planning' in action_sequence[0]:
                        state = 1
                    else:
                        # if it's a transition step
                        # update the ranges
                        if state % len(states) == 1:
                        # if state % 2 == 1:
                            # COM,principal_axis,endcaps = get_pose()

                            # print('COM: ',COM)
                            
                            
                            # print('Axis: ',principal_axis)
                            # print('Endcaps: ',endcaps)
                            # action = best_action(str(RANGE024) + "_" + str(RANGE135),action_dict,COM,principal_axis,trajectory)

                            # if prev_gait == 'roll':
                            #     prev_action = str(RANGE135) + "_" + str(RANGE024)
                            # else:
                            #     prev_action = prev_gait

                            prev_action = str(RANGE135) + "_" + str(RANGE024)

                            state_msg = State()
                            for x,y in trajectory:
                                point = Point()
                                point.x = x
                                point.y = y
                                state_msg.trajectory.append(point)
                            state_msg.prev_action = prev_action
                            state_msg.reverse_the_gait = reverse_the_gait
                            state_msg.bar_height_changed = bar_height_changed
                            state_pub.publish(state_msg)
                            bar_height_changed = False

                            action_sequence = ['planning__planning' for act in action_sequence]

                        # cost,action_sequence,COMs,PAs = best_k_actions(prev_action,action_dict,COM,principal_axis,trajectory,k = 2)

                        # # add current COM and PA
                        # COMs = [COM] + COMs
                        # PAs = [principal_axis] + PAs
                        """
                        print('Action Sequence: ',action_sequence)
                        # plot_MPC_prediction(trajectory,COMs,PAs)

                        if 'planning' in action_sequence[0]:
                            state = 1
                        else:

                            prev_state,action = action_sequence[0].split('__')
                            
                            if action == 'cw':
                                states = all_gaits.get('cw')
                                bottom_nodes = bottom3(endcaps)
                                print('Bottom Nodes: ',bottom_nodes)
                                states = transform_gait(states,bottom_nodes)
                                state = 1
                                prev_gait = 'cw'

                                RANGE024 = 100
                                RANGE135 = 100
                            elif action == 'ccw':
                                states = all_gaits.get('ccw')
                                bottom_nodes = bottom3(endcaps)
                                print('Bottom Nodes: ','Bottom Nodes: ',bottom_nodes)
                                states = transform_gait(states,bottom_nodes)
                                state = 1
                                prev_gait = 'ccw'

                                RANGE024 = 100
                                RANGE135 = 100
                            # elif prev_state == 'rest':
                            #     states = all_gaits.get('transition')
                            #     bottom_nodes = bottom3(endcaps)
                            #     print('Bottom Nodes: ',bottom_nodes)
                            #     states = transform_gait(states,bottom_nodes)
                            #     state = 1
                            #     # prev_gait = 'rest'

                            #     ranges = action.split('_')
                            #     RANGE024 = int(ranges[-1])
                            #     RANGE135 = int(ranges[-2])
                            else:
                                # if we have successive rolling steps,
                                # change the ranges but skip the transition
                                if not prev_gait in ['cw','ccw']:
                                    for i in range(num_motors):
                                        done[i] = True

                                states = all_gaits.get('roll')
                                bottom_nodes = bottom3(endcaps)
                                print('Bottom Nodes: ',bottom_nodes)
                                states = transform_gait(states,bottom_nodes)
                                if reverse_the_gait:
                                    states = reverse_gait(states,bottom_nodes)
                                state = 1
                                # prev_gait = action
                                prev_gait = 'roll'

                                ranges = action.split('_')
                                RANGE024 = int(ranges[-1])
                                RANGE135 = int(ranges[-2])
                            # else:
                            #     states = all_gaits.get(action)
                            #     bottom_nodes = bottom3(endcaps)
                            #     print(bottom_nodes)
                            #     states = transform_gait(states,bottom_nodes)
                            #     state = 1
                            #     prev_gait = action

                            #     RANGE135 = 120
                            #     RANGE024 = 120
                            print('Action: ',action)
                            # plot_MPC_prediction(trajectory,COMs,PAs)
                            """
                    # heading_command = heading_P*heading_error + heading_I*cum_head_error + heading_D*d_head_error
                    # if heading_command > 0:
                    #     RANGE024 = RANGE
                    #     RANGE135 = min(RANGE + abs(heading_command), MAX_RANGE)
                    # else:
                    #     RANGE024 = min(RANGE + abs(heading_command), MAX_RANGE)
                    #     RANGE135 = RANGE

                    # open('../data/output.csv','a').write(str(heading_error) + ',' + str(RANGE024) + ',' + str(RANGE135) + '\n')

                    # for i in range(num_motors):
                    #     done[i] = False
                    #     prev_error[i] = 0
                    #     cum_error[i] = 0

                    # # confined space demo
                    # if state > 2:
                    #     RANGE = 70
                    #     LEFT_RANGE = 70
                    #     # tol = 0.07
                    #     # low_tol = 0.07
                    #     tol = 0.06
                    #     low_tol = 0.06
                    # if state > 9:
                    #     RANGE = 160
                    #     LEFT_RANGE = 160
                    #     RANGE = 170
                    #     LEFT_RANGE = 170
                    #     tol = 0.2
                    #     low_tol = 0.2
                    # narrow door demo
                    # if state > 2:
                    #     RANGE = 160
                    #     LEFT_RANGE = 160
                    #     tol = 0.2
                    #     low_tol = 0.2
                    # if state > 7:
                    #     RANGE = 70
                    #     LEFT_RANGE = 70
                    #     tol = 0.07
                    #     low_tol = 0.07
            else:
                print('+++++')

        #read imu data    
        # elif line[0] == 'q' and len(q) == 8: # and abs(float(q[0])) <= 1 and abs(float(q[1])) <= 1 and abs(float(q[2])) <= 1 and abs(float(q[3])) <= 1:
        elif line[0] == 'q' and len(q) == 5:
            #print(q[0])
            #print(q[1])
            #print(q[2])
            #print(q[3])
            # for i,q in enumerate([q[0:4],q[4:]]):
            # q0 = float(q[0])
            # q1 = float(q[1])
            # q2 = float(q[2])
            # q3 = float(q[3])
            q0 = float(q[1])
            q1 = float(q[2])
            q2 = float(q[3])
            q3 = float(q[4])
            roll = -math.atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2))#convert quarternion to Euler angle for roll angle
            #convert quarternion to Euler angle for pitch angle
            sinp = 2*(q0*q2-q3*q1)
            #deal with gimlock
            if abs(sinp) >= 1:
                pitch = math.copysign(np.pi/2, sinp)
            else:
                pitch = math.asin(sinp)
            
            #yaw = -math.atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3))-np.pi/2
            #convert quarternion to Euler angle for pitch angle
            yaw = -math.atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3))

            k=np.array([cos(yaw)*cos(pitch), sin(pitch),sin(yaw)*cos(pitch)])
            r = R.from_rotvec(np.pi/2 * np.array([0, 1, 0]))
            k = r.apply(k)
            y=np.array([0,1,0])
            s=np.cross(k,y)
            v=np.cross(s,k)
            vrot=v*cos(roll)+np.cross(k,v)*sin(roll)
            imu[int(q[0])-1] = np.cross(k,vrot)
        elif line[0] == 'q' and len(q) == 8:
            imu = [quat2vec(q[:4]),quat2vec(q[4:])]

    else:
        print('+')
        send_command(serial_port, "s", 0)

    # COM,_,_ = get_pose()
    # print('COM: ',COM)

    # send ROS messages
    control_msg = TensegrityStamped()
    # strain_msg = SensorsStamped()
    # endcap_msg = NodesStamped()
    # imu_msg = ImuStamped()
    # get timestamp
    timestamp = rospy.Time.now()
    control_msg.header.stamp = timestamp
    # strain_msg.header.stamp = timestamp
    # imu_msg.header.stamp = timestamp
    # endcap_msg.header.stamp = timestamp
    # gait info
    info = Info()
    info.min_length = min_length
    info.RANGE = RANGE
    info.MAX_RANGE = MAX_RANGE
    info.MIN_RANGE = MIN_RANGE
    info.RANGE024 = RANGE024
    info.RANGE135 = RANGE135
    # info.LEFT_RANGE = LEFT_RANGE
    info.max_speed = max_speed
    info.tol = tol
    info.low_tol = low_tol
    info.P = P
    info.I = I
    info.D = D
    info.dist_weight = dist_weight
    info.ang_weight = ang_weight
    info.prog_weight = prog_weight
    control_msg.info = info
    # motors
    for motor_id in range(num_motors):
       motor = Motor()
       motor.id = motor_id
       motor.position = pos[motor_id]
       motor.target = states[state,motor_id]
       motor.speed = command[motor_id] * max_speed #abs(command[motor_id]) * max_speed
       # motor.direction = command[motor_id] > 0
       motor.done = done[motor_id]
       control_msg.motors.append(motor)
    # sensors
    for sensor_id in range(num_sensors):
       sensor = Sensor()
       sensor.id = sensor_id
       sensor.length = length[sensor_id]
       sensor.capacitance = capacitance[sensor_id]
       control_msg.sensors.append(sensor)
    # imu
    # for imu_id in range(num_imus):
    #    IMU = Imu()
    #    IMU.id = imu_id
    #    if any(imu[imu_id]) == None:
    #        IMU.x = None
    #        IMU.y = None
    #        IMU.z = None
    #    else:
    #        IMU.x = imu[imu_id][0]
    #        IMU.y = imu[imu_id][1]
    #        IMU.z = imu[imu_id][2]
    #    imu_msg.imus.append(IMU)
    # endcaps from tracking service
    for node_id in range(endcaps.shape[0]):
        node = Node()
        node.id = node_id
        node.x = endcaps[node_id,0]
        node.y = endcaps[node_id,1]
        node.z = endcaps[node_id,2]
        control_msg.nodes.append(node)
    # for imu in imu_msg.imus:
    #     reconstruction_msg.imus.append(imu)
    # endcap_pub.publish(endcap_msg)

    # trajectory information
    trajectory_msg = Trajectory()
    for x,y in trajectory:
        point = Point()
        point.x = x
        point.y = y
        trajectory_msg.trajectory.append(point)
    for x,y in COMs:
        point = Point()
        point.x = x
        point.y = y
        trajectory_msg.COMs.append(point)
    for x,y in PAs:
        point = Point()
        point.x = x
        point.y = y
        trajectory_msg.PAs.append(point)
    trajectory_msg.trajectory_segment = trajectory_segment
    control_msg.trajectory = trajectory_msg
    # # selected actions
    # if 'action_sequence' in locals():
    for act in action_sequence:
        control_msg.actions.append(act)

    # publish
    control_pub.publish(control_msg)
    # strain_pub.publish(strain_msg)
    # imu_pub.publish(imu_msg)
    # endcap_pub.publish(endcap_msg)

    # print(action_sequence)
    # print(states[state])


def tensegrity_run(device_name):
    global keep_going
    global is_tracker_initialized
    global max_speed
    # A welcome message
    print("Running serial_tx_cmdline node with device: " + device_name)
    # create the serial port object, non-exclusive (so others can use it too)
    serial_port = serial.Serial(port=device_name, baudrate=115200, timeout=1) # flush out any old data
    flush(serial_port)
    # finishing setup.
    print("Opened port. Ctrl-C to stop.")
    
    # If not using ROS, we'll do an infinite loop:
    #while keep_going and not rospy.is_shutdown():
    while keep_going:
        # request something to send
        try:
            # read(serial_port, strain_plot_list)
            read(serial_port)
            if not is_tracker_initialized:
                print('i got here once')
                all_topics = [t[0] for t in rospy.get_published_topics()]
                print(all_topics)
                if '/control_msg' in all_topics and '/rgb_images' in all_topics:
                    print('im about to init the tracker')
                    global length
                    rgb_msg = rospy.wait_for_message('/rgb_images',Image,None)
                    depth_msg = rospy.wait_for_message('/depth_images',Image,None)
                    if "trajectory_sequence" in globals():
                        full_trajectory = np.vstack((arr for arr in trajectory_sequence))
                    else:
                        full_trajectory = trajectory
                    init_tracker(rgb_msg,depth_msg,length,full_trajectory)
                    is_tracker_initialized = True
                    max_speed = 99#0
            else:
                # COM,principal_axis,endcaps = get_pose()
                # print("COM: ",COM)
                # print("Axis: ",principal_axis)
                # print("Endcaps: ",endcaps)
                pass
            
        except KeyboardInterrupt:
            # Nicely shut down this script.
            print("\nShutting down serial_tx_cmdline...")
            #set duty cycle as 0 to turn off the motors
            send_command(serial_port, "s", 0)
            sys.exit()

    # Nicely shut down this script.
    print("\nShutting down serial_tx_cmdline...")
    send_command(serial_port, "s", 0)
    sys.exit()

def onpress(key):
    global keep_going
    global states
    global tol
    global done
    global P
    global max_speed
    global RANGE
    global LEFT_RANGE
    if key == keyboard.KeyCode.from_char('s'):
        keep_going = False
    elif key == keyboard.KeyCode.from_char('r'):
        states = np.array([[1.0]*num_motors]*num_steps)
        done = np.array([False] * num_motors)
        tol = 0.03
        P = 5.0
        # max_speed = 80
        # RANGE = 90
        # LEFT_RANGE = RANGE
    elif key == keyboard.KeyCode.from_char('n'):
        states = np.array([[1.0]*num_motors]*num_steps)
        done = np.array([False] * num_motors)
        tol = 0.03
        P = 5.0
        # max_speed = 60
        RANGE = 90
        LEFT_RANGE = RANGE
    elif key == keyboard.KeyCode.from_char('p'):
        states = np.array([[-0.1]*num_motors]*num_steps)
        done = np.array([False] * num_motors)
        tol = 0.3
        P = 5.0
        # max_speed = 60
        # RANGE = 80
    elif key == keyboard.KeyCode.from_char('t'):
        states = np.array([[1.0, 1.0, 0.1, 1.0, 1.0, 0.1]])
        done = np.array([False] * num_motors)
        tol = 0.3

# def state_recon_callback(reconstruction_msg):
#     global first_principal_projection
#     global heading_error
#     global first_callback

#     nodes = np.array([[float(node.x),float(node.y),float(node.z)] for node in reconstruction_msg.reconstructed_nodes])
#     point024 = (nodes[0,:] + nodes[2,:] + nodes[4,:])/3
#     point135 = (nodes[1,:] + nodes[3,:] + nodes[5,:])/3
#     axis = point024 - point135
#     principal_axis = axis/(np.linalg.norm(axis))
#     y_proj = np.dot(principal_axis,np.array([0,1,0]))*np.array([0,1,0])
#     xz_proj = principal_axis - y_proj
#     unit_xz_proj = xz_proj/np.linalg.norm(xz_proj)

#     if first_callback:
#         first_principal_projection = unit_xz_proj
#         first_callback = False
#     else:
#         print('First Principal Projection: ',first_principal_projection)
#         print('Unit Principal Projection: ',unit_xz_proj)
#         heading_error = 1-np.dot(unit_xz_proj,first_principal_projection)
#         # print('Heading Error: ',heading_error)
#         error_vector = np.cross(first_principal_projection,unit_xz_proj)
#         heading_error = error_vector[1]
#         print('Heading Error: ',heading_error)
#             # the main function: just call the helper, while parsing the serial port path.

# def init_tracker(rgb_im: np.ndarray, depth_im: np.ndarray, cable_lengths: np.ndarray):
def init_tracker(rgb_msg, depth_msg, cable_lengths,trajectory):
    print('i got to init_tracker')
    # bridge = cv_bridge.CvBridge()
    # rgb_msg = bridge.cv2_to_imgmsg(rgb_im, encoding="rgb8")
    # depth_msg = bridge.cv2_to_imgmsg(depth_im, encoding="mono16")
    cable_length_msg = Float64MultiArray()
    # cable_length_msg.data = cable_lengths.tolist()
    cable_length_msg.data = cable_lengths

    trajectory_x = Float64MultiArray()
    trajectory_y = Float64MultiArray()
    trajectory_x.data = trajectory[:,0].tolist()
    trajectory_y.data = trajectory[:,1].tolist()

    request = InitTrackerRequest()
    request.rgb_im = rgb_msg
    request.depth_im = depth_msg
    request.cable_lengths = cable_length_msg
    request.trajectory_x = trajectory_x
    request.trajectory_y = trajectory_y

    service_name = "init_tracker"
    rospy.loginfo(f"Waiting for {service_name} service...")
    rospy.wait_for_service(service_name)
    rospy.loginfo(f"Found {service_name} service.")
    try:
        init_tracker_srv = rospy.ServiceProxy(service_name, InitTracker)
        rospy.loginfo("Request sent. Waiting for response...")
        response: InitTrackerResponse = init_tracker_srv(request)
        rospy.loginfo(f"Got response. Request success: {response.success}")
        return response.success
    except rospy.ServiceException as e:
        rospy.loginfo(f"Service call failed: {e}")
    return False

def get_pose():
    service_name = "get_pose"
    # rospy.loginfo(f"Waiting for {service_name} service...")
    # rospy.wait_for_service(service_name)
    # rospy.loginfo(f"Found {service_name} service.")
    # poses = []
    vectors = np.array([[0.0,0.0,0.0]])
    centers = []
    endcaps = []
    try:
        request = GetPoseRequest()
        get_pose_srv = rospy.ServiceProxy(service_name, GetPose)
        # rospy.loginfo("Request sent. Waiting for response...")
        response: GetPoseResponse = get_pose_srv(request)
        # rospy.loginfo(f"Got response. Request success: {response.success}")
        if response.success:
            for pose in response.poses:
                # vector = quat2vec([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])
                rotation_matrix = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z,pose.orientation.w]).as_matrix()
                # unit_vector = vector/np.linalg.norm(vector)
                unit_vector = rotation_matrix[:,2]
                center = [pose.position.x,pose.position.y,pose.position.z]
                endcaps.append(np.array(center) + L/2*unit_vector)
                endcaps.append(np.array(center) - L/2*unit_vector)
                # T = np.eye(4)
                # T[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
                # T[:3, :3] = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z,
                #                                 pose.orientation.w]).as_matrix()
                # poses.append(T)
                centers.append(center)
                vectors += unit_vector
        COM = np.mean(np.array(centers),axis=0)
        principal_axis = vectors/np.linalg.norm(vectors)
        endcaps = np.array(endcaps)/100 # convert to meters
        # reformat COM and PA
        COM = np.reshape(COM[0:2],(2,1))
        principal_axis = principal_axis[:,0:2]
        principal_axis = np.reshape(principal_axis,(2,1))
    except rospy.ServiceException as e:
        rospy.loginfo(f"Service call failed: {e}")
    return COM, principal_axis, endcaps

def mpc_callback(msg):
    print('MPC results are in!')
    global action_sequence
    global COMs
    global PAs
    global endcaps
    global state
    global states
    global reverse_the_gait
    global prev_bottom_nodes
    global RANGE024
    global RANGE135
    global prev_gait
    global num_motors
    global done
    global prev_error
    global cum_error
    global dist_weight
    global ang_weight
    global prog_weight

    # record motion planning results
    action_sequence = [act for act in msg.actions]
    COMs = np.array([[com.x,com.y] for com in msg.COMs])
    PAs = np.array([[pa.x,pa.y] for pa in msg.PAs])
    endcaps = np.array([[end.x,end.y,end.z] for end in msg.endcaps])

    # record the MPC weights
    dist_weight = msg.dist_weight
    ang_weight = msg.ang_weight
    prog_weight = msg.prog_weight

    # # ensure we incorporate the results in the next loop iteration
    # global done
    # global state
    # for i in range(len(done)):
    #     done[i] = True
    # state = 0

    # restart the step
    for i in range(num_motors):
        done[i] = False
        prev_error[i] = 0
        cum_error[i] = 0

    # addjust the ranges and gait based on MPC results
    action = action_sequence[0]

    print('Action Sequence: ',action_sequence)
                            
    if action == 'cw':
        states = all_gaits.get('cw')
        # bottom_nodes = prev_nodes.get(prev_bottom_nodes)
        bottom_nodes = prev_bottom_nodes
        # prev_bottom_nodes = bottom_nodes
        prev_bottom_nodes = prev_nodes.get(bottom_nodes)
        print('Bottom Nodes: ',bottom_nodes)
        states = transform_gait(states,bottom_nodes)
        state = 1
        prev_gait = 'cw'

        RANGE024 = 100
        RANGE135 = 100
    elif action == 'ccw':
        states = all_gaits.get('ccw')
        bottom_nodes = prev_bottom_nodes
        print('Bottom Nodes: ','Bottom Nodes: ',bottom_nodes)
        states = transform_gait(states,bottom_nodes)
        state = 1
        prev_gait = 'ccw'

        RANGE024 = 100
        RANGE135 = 100
    else:
        # if we have successive rolling steps,
        # change the ranges but skip the transition
        if not prev_gait in ['cw','ccw']:
            for i in range(num_motors):
                done[i] = True
        states = all_gaits.get('roll')
        # bottom_nodes = next_nodes.get(prev_bottom_nodes)
        bottom_nodes = prev_bottom_nodes
        # prev_bottom_nodes = bottom_nodes
        prev_bottom_nodes = next_nodes.get(bottom_nodes)
        print('Bottom Nodes: ',bottom_nodes)
        states = transform_gait(states,bottom_nodes)
        if reverse_the_gait:
            states = reverse_gait(states,bottom_nodes)
        state = 1
        prev_gait = 'roll'
        ranges = action.split('_')
        RANGE024 = int(ranges[-1])
        RANGE135 = int(ranges[-2])
    print('Action: ',action)

    # catch perception error
    if states is None:
        states = np.array([[1]*num_motors]*num_steps) # recover

if __name__ == '__main__':
    # init ROS stuff
    rospy.init_node('tensegrity')

    control_pub = rospy.Publisher('control_msg',TensegrityStamped,queue_size=100)
    state_pub = rospy.Publisher('/state_msg',State,queue_size=10)
    action_sub = rospy.Subscriber('/action_msg',Action,mpc_callback)

    # strain_pub = rospy.Publisher('strain_msg',SensorsStamped,queue_size=100)
    # imu_pub = rospy.Publisher('imu_msg',ImuStamped,queue_size=10)
    # endcap_pub = rospy.Publisher('reconstruction_msg',NodesStamped,queue_size=100)

    # rospy.Subscriber('reconstruction_msg',NodesStamped,state_recon_callback)

    ## keyboard listener for quitting
    keep_going = True
    my_listener = keyboard.Listener(on_press=onpress)
    my_listener.start()

    # tracker service
    is_tracker_initialized = True

    # find the directory on the local machine
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('tensegrity')

    with open(os.path.join(package_path,'calibration/motion_primitives.pkl'),'rb') as f:
        action_dict = pickle.load(f)

    calibration_file = os.path.join(package_path,'calibration/calibration.json')

    m,b = read_calibration_file(calibration_file)

    # heading/imu stuff
    first_callback = True
    heading_error = 0
    cum_head_error = 0
    prev_head_error = 0
    d_head_error = 0
    first_principal_projection = np.array([0,0,0])

    max_speed = 99# set duty cycle as 99 for the max speed, resolution can be improved by changing the bits in C++ code 
    num_sensors = 9# set number of strain sensors
    num_motors = 6# set number of motors
    num_imus = 2#set number of inertial measurement units
    min_length = 100#95#100 #mm #7.2#6.8get_pose#6.5 #set minimum length of sensors
    count = 0
    pos = [0] * num_motors
    cap = [0] * num_sensors
    capacitance = [0] * num_sensors
    length = [0] * num_sensors #mm
    imu = [[0,0,0]] * num_imus
    error = [0] * num_motors
    prev_error = [0] * num_motors
    cum_error = [0] * num_motors
    d_error = [0] * num_motors
    command = [0] * num_motors
    # beta flip
    flip = [1,1,1,1,-1,1]
    acceleration = [0]*3
    orientation = [0]*3
    endcaps = np.zeros((6,3))
    MIN_RANGE = 80
    RANGE = 90
    MAX_RANGE = 160
    RANGE024 = 100
    RANGE135 = 100
    tol = 0.2
    low_tol = 0.2
    #define PID
    # P = 10.0
    P = 6.0
    I = 0.01
    D = 0.5

    heading_P = 200
    heading_I = 0
    heading_D = 0

    dist_weight = 0
    ang_weight = 0
    prog_weight = 0

    # # BEST GAIT
    # quasi-static rolling
    states = np.array([[0.0, 1.0, 0.1, 0.0, 1.0, 1.0], [0.8, 0.1, 1.0, 1.0, 0.1, 1.0], [0.8, 0.1, 0.0, 1.0, 1.0, 0.0], [0.1, 1.0, 1.0, 0.1, 1.0, 1.0], [0.1, 0.0, 1.0, 1.0, 0.0, 1.0],[0.8, 1.0, 0.1, 1.0, 1.0, 0.1]])#6 steps gait
    states = np.array([[0.0, 1.0, 0.1, 0.0, 1.0, 1.0],[1.0, 1.0, 1.0, 1.0, 1.0, 1.0]])#steps gait
    states = np.array([[0.0, 1.0, 1.0, 0.0, 1.0, 0.1],[1.0, 1.0, 1.0, 1.0, 1.0, 1.0]]) # one step and recover
    states = np.array([[1.0, 1.0, 1.0, 1.0, 1.0, 1.0],[0.0, 1.0, 1.0, 0.0, 1.0, 0.1],
                       [1.0, 0.1, 1.0, 1.0, 0.1, 1.0],[1.0, 1.0, 0.0, 1.0, 0.1, 0.0],
                       [0.1, 1.0, 1.0, 0.1, 1.0, 1.0],[1.0, 0.0, 1.0, 0.1, 0.0, 1.0]]) # quasi-static rolling

    # dictionary of all gaits
    prev_gait = 'roll'
    reverse_the_gait = False
    bar_height_changed = False
    # prev_bottom_nodes = prev_nodes.get((0,2,5))
    prev_bottom_nodes = (0,2,5)
    # roll
    roll = np.array([[1.0, 0.1, 1.0, 1.0, 0.1, 1.0],[1.0, 1.0, 0.1, 1.0, 1.0, 0.1],[0.0, 1.0, 1.0, 0.0, 1.0, 0.1]])

    cw = np.array([[1, 1, 1, 1, 1, 1], [1, 1, 1, 1, 1, 1], [0, 0, 0, 1, 0, 1], [0, 0, 0, 0, 0, 0.7], [0, 0, 0.7, 0, 1, 1]])

    ccw = np.array([[1, 1, 1, 1, 1, 1], [1, 1, 1, 1, 1, 1], [1, 1, 1, 0, 1, 1], [1, 0, 1, 0, 1, 1], [0, 0, 0, 0, 0, 0]])

    transition = np.array([[1.0, 1.0, 0.1, 1.0, 1.0, 0.1],[1.0, 1.0, 0.1, 1.0, 1.0, 0.1]])

    #024 cw
    cw024 = np.array([[0, 0, 0, 0.1, 0.1, 0.1],
                      [0, 0, 0, 0.1, 0.1, 0.1],
                      [0, 0, 0, 1, 1, 0.1],
                      [0, 0, 0, 1, 0.1, 1],
                      [0, 0, 0, 0.1, 0.1, 0.1]])

    #024 ccw
    ccw024 = np.array([[0, 0, 0, 0.1, 0.1, 0.1],
                      [0, 0, 0, 0.1, 0.1, 0.1],
                      [0, 0, 0, 1, 0.1, 1],
                      [0, 0, 0, 1, 1, 0.1],
                      [0, 0, 0, 0.1, 0.1, 0.1]])

    #135 cw
    cw135 = np.array([[0.1, 0.1, 0.1, 0, 0, 0],
                      [0.1, 0.1, 0.1, 0, 0, 0],
                      [1, 0.1, 1, 0, 0, 0],
                      [1, 1, 0.1, 0, 0, 0],
                      [0.1, 0.1, 0.1, 0, 0, 0]])

    #135 ccw
    ccw135 = np.array([[0.1, 0.1, 0.1, 0, 0, 0],
                      [0.1, 0.1, 0.1, 0, 0, 0],
                      [1, 1, 0.1, 0, 0, 0],
                      [1, 0.1, 1, 0, 0, 0],
                      [0.1, 0.1, 0.1, 0, 0, 0]])

    # all_gaits = {'roll':roll,'135ccw':ccw135,'024ccw':ccw024,'135cw':cw135,'024cw':cw024}
    all_gaits = {'roll':roll,'transition':transition,'ccw':ccw,'cw':cw}

    num_steps = len(states)
    state = 0
    done = np.array([False] * num_motors)

    # # trajectory
    # num_points = 40
    # starting_point = [0,0.61]
    # turning_point = [1.83,0.61]
    # ending_point = [1.83,1.22]
    # # trajectory = np.vstack((np.linspace(starting_point,turning_point,num_points),np.linspace(turning_point,ending_point,int(num_points/2))[1:,:]))
    # trajectory = np.linspace(starting_point,turning_point,num_points)

    # # circle
    # num_points = 40
    # center_point = np.array([0.9,0.6])
    # radius = 0.6
    # t = np.linspace(0,-7*np.pi/4,num_points)
    # trajectory = np.array([[radius*np.cos(T) + center_point[0],radius*np.sin(T) + center_point[1]] for T in t])

    traj_name = "straight"

    if traj_name == "straight":
        # straight line
        ppm = 20 # points per meter
        starting_point = [0.1,0.6]
        ending_point = [1.7,0.6]
        num_points = ceil(ppm*(ending_point[0] - starting_point[0]))
        trajectory_sequence = [np.linspace(starting_point,ending_point,num_points)]
        trajectory_segment = 0
        trajectory = trajectory_sequence[trajectory_segment]
    elif traj_name == "back-and-forth":
        # straight line
        ppm = 20 # points per meter
        starting_point = [0.1,0.6]
        ending_point = [1.7,0.6]
        num_points = ceil(ppm*(ending_point[0] - starting_point[0]))
        trajectory_sequence = [np.linspace(starting_point,ending_point,num_points),np.linspace(ending_point,starting_point,num_points)]
        trajectory_segment = 0
        trajectory = trajectory_sequence[trajectory_segment]
    elif traj_name == "frame":
        # world frame
        trajectory = [[0,0],[0.1,0],[0.2,0],[0.3,0],[0,0.1],[0,0.2],[0,0.3]]
    elif traj_name == "circle":
        # circle
        # num_points = 50
        ppm = 20
        center_point = np.array([0.9,0.1])
        radius = 0.9
        num_points = ceil(np.pi*radius*ppm*5/6)
        # t = np.linspace(0,-7*np.pi/4,num_points)
        t = np.linspace(-13*np.pi/12,-23*np.pi/12,num_points)
        trajectory = np.array([[radius*np.cos(T) + center_point[0],radius*np.sin(T) + center_point[1]] for T in t])
        trajectory_sequence = [trajectory]
        trajectory_segment = 0
        # ppm = 20
        # center_point = np.array([0.85,1.1])
        # radius = 0.9
        # num_points = ceil(np.pi*radius*ppm)
        # # t = np.linspace(0,-7*np.pi/4,num_points)
        # t = np.linspace(2*np.pi,np.pi,num_points)
        # trajectory = np.array([[radius*np.cos(T) + center_point[0],radius*np.sin(T) + center_point[1]] for T in t])
    elif traj_name == "ellipse":
        # ellipse
        num_points = 50
        center_point = np.array([0.9,0.6])
        major_axis = 0.7
        minor_axis = 0.5
        t = np.linspace(-7*np.pi/8,np.pi/8,num_points)
        trajectory = np.array([[major_axis*np.cos(T) + center_point[0],minor_axis*np.sin(T) + center_point[1]] for T in t])
    elif traj_name == "Z":
        # Z shape
        points_per_segment = 20
        starting_point = [0,0.1]
        first_pivot = [0,1.1]
        second_pivot = [1.8,0.1]
        ending_point = [1.8,1.1]
        trajectory = np.vstack((np.linspace(starting_point,first_pivot,points_per_segment),np.linspace(first_pivot,second_pivot,2*points_per_segment)[1:,:],np.linspace(second_pivot,ending_point,points_per_segment)[1:,:]))
    elif traj_name == "rectangle":
        # rectangle
        ppm = 30 # points per meter
        starting_point = [0.1,0.2]
        third_pivot = [0.1,1.0]
        second_pivot = [1.7,1.0]
        first_pivot = [1.7,0.2]
        ending_point = [0.1,0.2]
        short_dist = first_pivot[0] - starting_point[0]
        long_dist = second_pivot[1] - first_pivot[1]
        trajectory_sequence = [np.linspace(starting_point,first_pivot,int(ppm*short_dist)),np.linspace(first_pivot,second_pivot,int(ppm*long_dist)),np.linspace(second_pivot,third_pivot,int(ppm*short_dist)),np.linspace(third_pivot,ending_point,int(ppm*long_dist))]
        trajectory_segment = 0
        trajectory = trajectory_sequence[trajectory_segment]
        # trajectory = np.vstack((np.linspace(starting_point,first_pivot,ppm*short_dist),np.linspace(first_pivot,second_pivot,ppm*long_dist)[1:,:],np.linspace(second_pivot,third_pivot,points_per_segment)[1:,:],np.linspace(third_pivot,ending_point,points_per_segment)[1:,:]))
    elif traj_name == "rounded-rectangle":
        # rounded rectangle
        ppm = 30 # points per meter
        center_point = np.array([0.9,0.6])
        width = 1.8 # meter
        height = 0.9 # meter
        curve_radius = 0.4 # meter
        # first leg
        starting_point = np.array([center_point[0]+width/2-curve_radius,center_point[1]-height/2])
        ending_point = np.array([center_point[0]-width/2+curve_radius,center_point[1]-height/2])
        leg_length = starting_point[0] - ending_point[0]
        first_leg = np.linspace(starting_point,ending_point,ceil(ppm*leg_length))
        # next corner
        arc_center = [center_point[0]-width/2+curve_radius,center_point[1]-height/2+curve_radius]
        next_corner = make_circle(arc_center,curve_radius,num_points=ppm*curve_radius*np.pi/2,starting_angle=-np.pi/2,swept_angle=-np.pi/2)
        trajectory = np.vstack((first_leg,next_corner[1:,:]))
        # next leg
        starting_point = np.array([center_point[0]-width/2,center_point[1]-height/2+curve_radius])
        ending_point = np.array([center_point[0]-width/2,center_point[1]+height/2-curve_radius])
        leg_length = ending_point[1] - starting_point[1]
        next_leg = np.linspace(starting_point,ending_point,ceil(ppm*leg_length))
        trajectory = np.vstack((trajectory,next_leg[1:,:]))
        trajectory_sequence = [trajectory]
        trajectory_segment = 0
    elif traj_name == "S":
        # S shape
        num_points = 40
        amplitude = 0.55
        starting_point = [0,0.6]
        ending_point = [1.8,0.6]
        t = np.linspace(0,2*np.pi,num_points)
        x = np.linspace(starting_point[0],ending_point[0],num_points)
        y = amplitude*np.sin(t) + starting_point[1]
        trajectory = np.array([[X,amplitude*np.sin(T) + starting_point[1]] for X,T in zip(x,t)])
    elif traj_name == "triangle":
        # triangle
        ppm = 20 # points per meter
        side_length = 1 # meters
        points_per_segment = ceil(ppm*side_length)
        height = side_length/2 * sqrt(3)
        starting_point = [0.41,0.25]
        apex = [side_length/2 + starting_point[0],starting_point[1] + height]
        second_pivot = [starting_point[0] + side_length,starting_point[1]]
        trajectory_sequence = [np.linspace(starting_point,second_pivot,points_per_segment),np.linspace(second_pivot,apex,points_per_segment),np.linspace(apex,starting_point,points_per_segment)]
        trajectory_segment = 0
        trajectory = trajectory_sequence[trajectory_segment]
    elif traj_name == "right-triangle":
        # right triangle
        ppm = 20 # points per meter
        leg_length = 1.1 # meters
        hyp_length = sqrt(2)*leg_length
        points_per_leg = ceil(ppm*leg_length)
        points_per_hyp = ceil(ppm*hyp_length)
        starting_point = [0.9,0.05]
        first_pivot = [starting_point[0] + leg_length/sqrt(2),starting_point[1] + leg_length/sqrt(2)]
        second_pivot = [first_pivot[0]-hyp_length,first_pivot[1]]
        trajectory_sequence = [np.linspace(starting_point,first_pivot,points_per_leg),np.linspace(first_pivot,second_pivot,points_per_hyp),np.linspace(second_pivot,starting_point,points_per_leg)]
        trajectory = np.vstack((np.linspace(starting_point,first_pivot,points_per_leg),np.linspace(first_pivot,second_pivot,points_per_hyp)[1:,:],np.linspace(second_pivot,starting_point,points_per_leg)[1:,:]))
        trajectory_segment = 0
        trajectory = trajectory_sequence[trajectory_segment]
    elif traj_name == "zigzag":
        # zigzag
        ppm = 20 # points per meter
        leg_length = 1.5 # meters
        points_per_segment = ceil(ppm*leg_length)
        alpha = 15 # degrees
        beta = np.pi/180 * (90-alpha/2) # radians
        Nzags = 2
        starting_point = np.array([0.15,0.25])
        trajectory_sequence = []
        for n in range(Nzags):
            trajectory_sequence.extend([np.linspace(starting_point,starting_point + leg_length*np.array([np.sin(beta),np.cos(beta)]),points_per_segment),np.linspace(starting_point + leg_length*np.array([np.sin(beta),np.cos(beta)]),starting_point + 2*leg_length*np.array([0,np.cos(beta)]),points_per_segment)])
            if n == 0:
                trajectory = np.vstack((np.linspace(starting_point,starting_point + leg_length*np.array([np.sin(beta),np.cos(beta)]),points_per_segment),np.linspace(starting_point + leg_length*np.array([np.sin(beta),np.cos(beta)]),starting_point + 2*leg_length*np.array([0,np.cos(beta)]),points_per_segment)[1:,:]))
            else:
                trajectory = np.vstack((trajectory,np.linspace(starting_point,starting_point + leg_length*np.array([np.sin(beta),np.cos(beta)]),points_per_segment)[1:,:],np.linspace(starting_point + leg_length*np.array([np.sin(beta),np.cos(beta)]),starting_point + 2*leg_length*np.array([0,np.cos(beta)]),points_per_segment)[1:,:]))
            starting_point = starting_point + 2*leg_length*np.array([0,np.cos(beta)])
        trajectory_segment = 0
        trajectory = trajectory_sequence[trajectory_segment]
    elif traj_name == "zigzagzig":
        # zigzagzig
        ppm = 20 # points per meter
        leg_length = 1.6 # meters
        points_per_segment = ceil(ppm*leg_length)
        alpha = 15 # degrees
        beta = np.pi/180 * (90-alpha/2) # radians
        Nzags = 3
        starting_point = np.array([0.12,0.3])
        trajectory_sequence = [np.linspace(starting_point + leg_length*np.array([(n%2)*np.sin(beta),n*np.cos(beta)]),starting_point + leg_length*np.array([((n+1)%2)*np.sin(beta),(n+1)*np.cos(beta)]),points_per_segment) for n in range(Nzags)]
        trajectory_segment = 0
        trajectory = trajectory_sequence[trajectory_segment]
    else:
        error("You must choose a predefined trajectory.")

    COMs = [[-10,-10],[-10,-10],[-10,-10]]
    PAs = [[0,0],[0,0],[0,0]]
    action_sequence = [' __ ',' __ ']

    # states = np.array([[1.0, 1.0, 0.1, 1.0, 1.0, 0.1],
    #     [0.0, 1.0, 1.0, 0.0, 1.0, 0.1],
    #     [1, 0.1, 1, 1, 0.1, 1],
    #     [1, 1, 0, 1, 0.1, 0],
    #     [0.1, 1, 1, 0.1, 1, 1]]) 

    try:
        # the 0-th arg is the name of the file itself, so we want the 1st.
        # tensegrity_run(sys.argv[1])
        tensegrity_run('/dev/ttyACM0')
    except KeyboardInterrupt:
        # why is this here?
        pass

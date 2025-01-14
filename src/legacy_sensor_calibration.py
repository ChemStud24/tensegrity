#!/usr/bin/env python3
import os
import sys
import serial
import time
import math
from math import cos,sin,tan
import re
# import csv
import json
import xlrd
import numpy as np
import pandas as pd
# import matplotlib.pyplot as plt
# from vpython import *
from pynput import keyboard
from scipy.spatial.transform import Rotation as R

# # save data
# import os

# ROS messages
import rospy
# from tensegrity.msg import Motor, MotorsStamped, Sensor, SensorsStamped, ImuStamped, Imu
from tensegrity.msg import Motor, Sensor, Imu, TensegrityStamped
# from geometry_msgs.msg import QuaternionStamped
import rospkg

def flush(serial_port):
    serial_port.reset_input_buffer()
    serial_port.reset_output_buffer()
    time.sleep(0.001)

def send_command(serial_port, input_string, delay_time):
    # to_microcontroller_msg = f'{input_string}\n'
    to_microcontroller_msg = '{}\n'.format(input_string)
    serial_port.write(to_microcontroller_msg.encode('UTF-8'))
    if delay_time < 0:
        delay_time = 0
    time.sleep(delay_time/1000)

def twos_comp(val, bits):
    """compute the 2's complement of int value val"""
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return val 

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
    yaw = -math.atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3))+np.pi/2

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
    global pos_prev 
    global acceleration
    global orientation
    global imu
    global acc
    line = serial_port.read_until(b'\n')# read data string sent from central
    line = str(line)#.encode('utf-8')# convert to string
    line = line.split('*')[-1] # remove padding
    print(line)
    s = re.findall(r"0x[0-9a-f]+", line)# extract strain data (hex number) from the data string
    q = re.findall(r"[-+]?\d*\.\d+|\d+", line)# extract imu data (decimal number) from the data string
    if len(line) >= 2 :# valid data string will be longer than 9 characters  

        # read strain data
        if line[0] == "S":
            now_time = time.time()
            meas_time = now_time - start_time
            for i in range(num_sensors):
                cap[i] = s[i]
            if not 0 in cap:
                for i, cap_hex in enumerate(cap):
                    adc_counts = int(str(cap_hex[2:]),16)
                    print("Sensor " + str(i))
                    if adc_counts != 0:
                        # capacitance[i] = 16.0 * 0.5 / adc_counts / 3.3 * 1024
                        capacitance[i] = 42.0 / adc_counts / 3.3 * 1024
                    length[i] = (capacitance[i] - b[i]) / m[i] #/ 10
                    print("Capacitance: " + str(capacitance[i]))
                    print("Length: " + str(length[i]))
                # count = count + 1 

        # read acceleration data    
        elif line[0] == "A":
            q = re.findall(r"[-+]?\d*\.\d+|\d+", line)# extract imu data (decimal number) from the data string
            acceleration = np.array([acc for acc in q])
            # print('Acceleration: ',np.linalg.norm(acceleration))
            print('Acceleration: ',np.linalg.norm(acceleration))
            open('../data/acceleration.csv','a').write(str(rospy.Time.now().to_sec()) + ',' + str(np.linalg.norm(acceleration)) + '\n')

        # read orientation data
        elif line[0] == "O":
            q = re.findall(r"[-+]?\d*\.\d+|\d+", line)# extract imu data (decimal number) from the data string
            orientation = [orio for orio in q]
            # print('Orientation: ',orientation)

            # else:
            #     print(line)
        #read imu data    
        elif line[0] == 'q' and len(q) == 5:
        # elif line[0] == 'q':#len(q) == 8: # and abs(float(q[0])) <= 1 and abs(float(q[1])) <= 1 and abs(float(q[2])) <= 1 and abs(float(q[3])) <= 1:
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

            # quat[i] = [q0,q1,q2,q3]
            # print(quat[i])

            # print(q)
            #trying something
            r = R.from_quat([q0,q1,q2,q3])
            x = np.array([1,0,0])
            vec = r.apply(x)
            
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
            # k = rotate(k, angle=np.pi/2, axis=vector(0,1,0))
            y=np.array([0,1,0])
            s=np.cross(k,y)
            v=np.cross(s,k)
            vrot=v*cos(roll)+np.cross(k,v)*sin(roll)
            imu[int(q[0])-1] = np.cross(k,vrot)
            # imu[i] = np.cross(k,vrot)
            print(imu)

            # imu[int(q[0])-1] = np.cross(k,vrot)

            # Roll = roll/np.pi*180
            # print('roll')
            # print(roll/np.pi*180)
            #print('pitch')
            #print(pitch/np.pi*180)
            # print('yaw')
            #print(yaw/np.pi*180)
            #modify this part if we want to integrate imu data to determine the actuation order of three long cables
            #125
            # if Roll>72.85 and Roll<=137.55:
            #     send_command(serial_port, "y 0 1", 0)
            #     send_command(serial_port, "n 2 3 4 5", 0)
            # #145
            # elif Roll>15.95 and Roll<=72.85:
            #     send_command(serial_port, "n 0 1 4 5", 0)
            #     send_command(serial_port, "y 2 3", 0)
            # #256
            # elif (Roll>137.55 and Roll<=180) or (Roll>=-180 and Roll<=-163.5):
            #     send_command(serial_port, "n 0 1 2 3", 0)
            #     send_command(serial_port, "y 4 5", 0)
            # #236    
            # elif Roll>-163.5 and Roll<=-106.05:
            #     send_command(serial_port, "n 0 1 4 5", 0)
            #     send_command(serial_port, "y 2 3", 0)
            # #346  
            # elif Roll>-106.05 and Roll<=-41.9:
            #     send_command(serial_port, "n 2 3 4 5", 0)
            #     send_command(serial_port, "y 0 1", 0)
            # #134
            # elif Roll>-41.9 and Roll<15.95:
            #     send_command(serial_port, "n 0 1 2 3", 0)
            #     send_command(serial_port, "y 4 5", 0)
            # else:
            #     print('wrong orientation')            
        elif line[0] == 'q' and len(q) == 8:
            imu = [quat2vec(q[:4]),quat2vec(q[4:])]
        elif line[0] == 'q' and len(q) == 14:
            imu = [quat2vec(q[:4]),quat2vec(q[4:8])]
            acc = [q[8:11],q[11:]]
            # acc1 = q[8:11]
            # acc2 = q[11:]
            print(np.linalg.norm(acc[0]),np.linalg.norm(acc[1]))
            # print("Acc1: ", np.linalg.norm(acc1))
            # print("Acc2: ", np.linalg.norm(acc2))
    else:
        print('+') 

    # send ROS messages
    # control_msg = MotorsStamped()
    # strain_msg = SensorsStamped()
    # imu_msg = ImuStamped()
    control_msg = TensegrityStamped()
    # get timestamp
    timestamp = rospy.Time.now()
    control_msg.header.stamp = timestamp
    # strain_msg.header.stamp = timestamp
    # imu_msg.header.stamp = timestamp
    # sensors
    for sensor_id in range(num_sensors):
        sensor = Sensor()
        sensor.id = sensor_id
        sensor.length = length[sensor_id]
        sensor.capacitance = capacitance[sensor_id]
        # # HARD-CODING THIS ERASE LATER
        # if sensor_id == 3:
        #     #     sensor.length = 16.0
        #     # else:
        #     # sensor.length = 308.0
        #     sensor.length = 125.0
        # elif sensor_id == 4:
        #     sensor.length = 90.0
        # # HARD-CODING THIS ERASE LATER
        control_msg.sensors.append(sensor)
    # IMU
    # imu_msg.ax = float(acceleration[0])
    # imu_msg.ay = float(acceleration[1])
    # imu_msg.az = float(acceleration[2])
    # imu_msg.yaw = float(orientation[0])
    # imu_msg.pitch = float(orientation[1])
    # imu_msg.roll = float(orientation[2])
    for imu_id in range(num_imus):
        IMU = Imu()
        IMU.id = imu_id
        if any(imu[imu_id]) == None:
            IMU.x = None
            IMU.y = None
            IMU.z = None
        else:
            IMU.x = imu[imu_id][0]
            IMU.y = imu[imu_id][1]
            IMU.z = imu[imu_id][2]
            # IMU.q1 = quat[imu_id][0]
            # IMU.q2 = quat[imu_id][1]
            # IMU.q3 = quat[imu_id][2]
            # IMU.q4 = quat[imu_id][3]
            # IMU.ax = np.linalg.norm(np.array(acc[imu_id]))
        control_msg.imus.append(IMU)
    # publish
    control_pub.publish(control_msg)
    # strain_pub.publish(strain_msg)
    # imu_pub.publish(imu_msg)
    
    # # save data
    # data = {}
    # data['header'] = {'seq':control_msg.header.seq,'secs':control_msg.header.stamp.to_sec()}
    # data['sensors'] = {}
    # for sensor in control_msg.sensors:
    #     data['sensors'][sensor.id] = {'length':sensor.length,'capacitance':sensor.capacitance}
    # json.dump(data,open(os.path.join('../data/no_locomotion/data', str(count).zfill(4) + ".json"),'w'))
    # count += 1
    # print(count)
    
def tensegrity_run(device_name):
    global keep_going
    # A welcome message
    print("Running serial_tx_cmdline node with device: " + device_name)
    # create the serial port object, non-exclusive (so others can use it too)
    serial_port = serial.Serial(port=device_name, baudrate=115200, timeout=1)    # flush out any old data
    flush(serial_port)
    # finishing setup.
    print("Opened port. Press q to stop.")
    global start_time
    start_time = time.time()# set starting time
    
    # If not using ROS, use the line below instead
    while keep_going and not rospy.is_shutdown():
    # while keep_going:
        
        # request something to send
        try:
            read(serial_port)
            
        except KeyboardInterrupt:
            # Nicely shut down this script.
            print("\nShutting down serial_tx_cmdline...")
            sys.exit()

    # Nicely shut down this script.
    print("\nShutting down serial_tx_cmdline...")
    sys.exit()

def onpress(key):
    global keep_going
    if key == keyboard.KeyCode.from_char('q'):
        keep_going = False
    elif key == keyboard.KeyCode.from_char('s'):
        keep_going = False


            # the main function: just call the helper, while parsing the serial port path.
if __name__ == '__main__':
    # init ROS stuff
    rospy.init_node('tensegrity')

    control_pub = rospy.Publisher('control_msg',TensegrityStamped,queue_size=100)
    # strain_pub = rospy.Publisher('strain_msg',SensorsStamped,queue_size=10)
    # imu_pub = rospy.Publisher('imu_msg',ImuStamped,queue_size=10)

    # keyboard listener for quitting
    keep_going = True
    my_listener = keyboard.Listener(on_press=onpress)
    my_listener.start()

    rospack = rospkg.RosPack()
    calibration_file = os.path.join(rospack.get_path('tensegrity'),'calibration/calibration.json')

    num_sensors = 9# set number of strain sensors
    num_imus = 2#set number of inertial measurement units
    count = 0
    cap = [0]*num_sensors
    capacitance = [0]*num_sensors
    acceleration = [0]*3
    orientation = [0]*3
    length = [0] * num_sensors #mm
    imu = [[0,0,0]] * num_imus
    acc = [[0,0,0]] * num_imus
    # quat = [[0,0,0,0]] * num_imus

    # m = np.array([0.05015,0.08364,0.08381,0.06574,0.08528,0.07355,0.12693,0.11281,0.11803])
    # b = np.array([12.839,12.546,12.650,13.365,11.526,13.358,13.230,11.761,12.884])

    # m = np.array([0.04686,0.04140,0.04133,0.03620,0.04841,0.04468,0.04296,0.03685,0.03965])
    # b = np.array([15.831,16.588,15.931,14.534,13.733,15.246,10.301,13.761,12.006])

    # m = np.array([0.05061,0.05455,0.04455,0.04701,0.05200,0.04135,0.03161,0.03196,0.03672])
    # b = np.array([14.256,15.232,15.264,16.159,14.024,15.343,15.422,14.297,14.304])

    # m = np.array([0.03738,0.04733,0.03054,0.03657,0.04805,0.04152,0.03161,0.03233,0.03360])
    # b = np.array([17.394,16.841,17.881,18.236,15.622,16.197,15.422,14.327,15.851])

    # m = np.array([0.04309,0.05103,0.03923,0.04461,0.04159,0.03893,0.03795,0.02317,0.04196])
    # b = np.array([19.672,18.631,19.306,17.803,19.178,17.478,16.029,20.233,15.409])

    m = np.array([0.04088,0.03714,0.04448,0.03179,0.04833,0.03890,0.03437,0.02701,0.02905])
    b = np.array([15.623,14.650,15.023,10.605,15.213,16.303,14.494,14.508,15.190])

    m = np.array([0.04437,0.06207,0.02356,0.04440,0.04681,0.05381,0.02841,0.03599,0.03844])
    b = np.array([15.763,13.524,15.708,10.084,15.628,15.208,16.356,12.575,13.506])
    # beta calibration
    # # FROM AUTOCALIBRATION
    # m = np.array([19.70001,12.25614,13.98790,13.96344,12.76481,11.82479,8.00394,10.48498,9.62916])
    # b = np.array([-232.92862,-128.42923,-166.21920,-148.34993,-148.39009,-114.26119,40.03870,-23.04798,-14.34742])
    # m2 = 1.0/m
    # b2 = -b/m
    # m = m2
    # b = b2

    m,b = read_calibration_file(calibration_file)

    # # load autocalibration file
    # data = json.load(open(calibration_file))
    # m = data.get('m')
    # b = data.get('b')



    try:
        # the 0-th arg is the name of the file itself, so we want the 1st.
        tensegrity_run(sys.argv[1])
    except KeyboardInterrupt:
        # why is this here?
        pass

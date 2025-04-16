#!/usr/bin/env python3
import os
import time
import math
from math import cos, sin
import json
import xlrd
import numpy as np
from pynput import keyboard
from scipy.spatial.transform import Rotation as R
import rospy
import rosnode
import rospkg
import socket
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from tensegrity_perception.srv import InitTracker, InitTrackerRequest, InitTrackerResponse
from tensegrity_perception.srv import GetPose, GetPoseRequest, GetPoseResponse
from tensegrity.msg import Motor, Info, Sensor, Imu, TensegrityStamped, State, Action
from geometry_msgs.msg import Point
from Tensegrity_model_inputs import *
from policy_vel import ctrl_policy_vel


class FileError(Exception):
    pass
class S_Q_Pressed(Exception):
    pass

class TensegrityRobot:
    def __init__(self):
        self.num_sensors = 9
        self.num_motors = 6
        self.num_imus = 2
        self.num_arduino = 3
        self.min_length = 100
        self.pos = [0] * self.num_motors
        self.cap = [0] * self.num_sensors
        self.length = [0] * self.num_sensors
        self.imu = [[0, 0, 0]] * self.num_imus
        self.error = [0] * self.num_motors
        self.prev_error = [0] * self.num_motors
        self.cum_error = [0] * self.num_motors
        self.d_error = [0] * self.num_motors
        self.command = [0] * self.num_motors
        self.speed = [0] * self.num_motors
        self.flip = [1, -1, 1, -1, -1, -1] # flip direction of motors
        self.accelerometer = [[0]*3]*3
        self.gyroscope = [[0]*3]*3
        self.encoder_counts = [0]*self.num_motors
        self.encoder_length = [0]*self.num_motors
        self.RANGE024 = 130
        self.RANGE135 = 130
        self.max_speed = 40
        self.tol = 0.15
        self.low_tol = 0.15
        self.P = 10.0
        self.I = 0.01
        self.D = 0.5
        self.gear_ratio = 150
        self.winch_diameter = 6.35
        self.encoder_resolution = 12

        # planning and control
        self.prev_bottom_nodes = (0,2,5)
        self.prev_gait = 'roll'
        self.reverse_the_gait = False
        self.action_sequence = [' _ ', ' _ ']

        # RL
        fps = 10 # maybe lower because sometimes there are bigger gaps
        self.policy = ctrl_policy_vel(fps)
        
        self.num_steps = None
        self.state = None
        self.states = None
        self.control_pub = None
        self.my_listener = None
        self.keep_going = True
        self.quitting = False
        self.calibration = False
        self.done = None
        self.m = None
        self.b = None
        self.stop_msg = None
        self.init_speed = None
        self.which_Arduino = None
        
        # UDP variables
        self.UDP_IP = "0.0.0.0"  # Listen to all incoming interfaces
        self.UDP_PORT = 2390     # Same port used in the Arduino sketch
        self.sock_receive = None
        self.sock_send = None
        self.addresses = [None] * self.num_arduino
        self.offset = None # Nb of leading end ending 0 preventing errors 

        print("Initializing")
        self.initialize()
        print("Running UDP connection with Arduino's: ")

        # Create a UDP socket for receiving data
        self.sock_receive = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Bind the socket to the address and port
        self.sock_receive.bind((self.UDP_IP, self.UDP_PORT))
        
        # finishing setup.
        print("Opened connection press s to stop motor and q to quit")

        # init tracker
        if '/tracking_service' in rosnode.get_node_names():
            # self.trajectory = obstacle_trajectory
            self.init_tracker()
        
        # communicating with the planner
        # self.action_sub = rospy.Subscriber('/action_msg',Action,self.mpc_callback)
        # self.state_pub = rospy.Publisher('/state_msg',State,queue_size=10)
        # self.COMs = []
        # self.PAs = []
        # self.next_states = None

    def initialize(self):

        self.my_listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.my_listener.start()
        
        rospy.init_node('tensegrity')
        self.control_pub = rospy.Publisher('control_msg', TensegrityStamped, queue_size=10) ## correct ??

        package_path = rospkg.RosPack().get_path('tensegrity')
        calibration_file = os.path.join(package_path,'calibration/new_calibration.json')
        
        self.m, self.b = self.read_calibration_file(calibration_file)
        
        """
        # # BEST GAIT
        # quasi-static rolling
        states = np.array([[0.0, 1.0, 0.1, 0.0, 1.0, 1.0], [0.8, 0.1, 1.0, 1.0, 0.1, 1.0], [0.8, 0.1, 0.0, 1.0, 1.0, 0.0], [0.1, 1.0, 1.0, 0.1, 1.0, 1.0], [0.1, 0.0, 1.0, 1.0, 0.0, 1.0],[0.8, 1.0, 0.1, 1.0, 1.0, 0.1]])#6 steps gait
        states = np.array([[0.0, 1.0, 0.1, 0.0, 1.0, 1.0],[1.0, 1.0, 1.0, 1.0, 1.0, 1.0]])#steps gait
        states = np.array([[0.0, 1.0, 1.0, 0.0, 1.0, 0.1],[1.0, 1.0, 1.0, 1.0, 1.0, 1.0]]) # one step and recover
        """
        self.states = np.array([[1.0, 1.0, 0.1, 1.0, 1.0, 0.1],[0.0, 1.0, 1.0, 0.0, 0.8, 0.1],[1.0, 0.1, 1.0, 1.0, 0.1, 1.0],[1.0, 1.0, 0.0, 0.8, 0.1, 0.0],[0.1, 1.0, 1.0, 0.1, 1.0, 1.0],[1.0, 0.0, 1.0, 0.1, 0.0, 0.8]]) # quasi-static rolling
        #self.states = np.array([[1, 1, 1, 0, 1, 1], [1, 0, 1, 0, 1, 1], [0, 0, 0, 0, 0, 0], [1, 1, 1, 1, 1, 1]]) # counterclockwise
        #self.states = np.array([[0, 0, 0, 1, 0, 1], [0, 0, 0, 0, 0, 0.7], [0, 0, 0.7, 0, 1, 1], [1, 1, 1, 1, 1, 1]]) # clockwise 
        #self.states = np.array([[1, 1, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], [0, 1, 1, 0, 0.8, 0], [1, 1, 1, 1, 1, 1], [1, 0, 1, 0, 0, 0], [1, 0, 0, 0, 0, 0], [1, 1, 0, 0.8, 0, 0], [1, 1, 1, 1, 1, 1], [0, 1, 1, 0, 0, 0], [0, 0, 1, 0, 0, 0], [1, 0, 1, 0, 0, 0.8], [1, 1, 1, 1, 1, 1]]) #clockwise
        #self.states = np.array([[0, 1, 1, 1, 1, 1], [0, 1, 1, 1, 1, 0], [0, 0, 0, 0, 0, 0], [1, 1, 1, 1, 1, 1]]) # counterclockwise
        # self.states = np.array([[1,1,1,1,1,1],[0.2,1,1,1,1,1],[1,1,1,1,1,1],[1,0.2,1,1,1,1],[1,1,1,1,1,1],[1,1,0.2,1,1,1],[1,1,1,1,1,1],[1,1,1,0.2,1,1],[1,1,1,1,1,1],[1,1,1,1,0.2,1],[1,1,1,1,1,1],[1,1,1,1,1,0.2]])
        self.num_steps = len(self.states)
        self.state = 0
        self.offset = 3
        self.done = np.array([False] * self.num_motors)
        self.stop_msg = ' '.join(['0'] * (self.num_motors+2*self.offset))
        self.init_speed = 70

        # gaits
        # roll = np.array([[1,1,1,1,1,1],[1,1,1,1,1,1],[1.0, 0.1, 1.0, 1.0, 0.1, 1.0],[1.0, 1.0, 0.1, 1.0, 1.0, 0.1],[0.0, 1.0, 1.0, 0.0, 1.0, 0.1]])
        roll = np.array([[1,1,1,1,1,1],[1,1,1,1,1,1],[1.0, 1.0, 0.1, 1.0, 1.0, 0.1],[0.0, 1.0, 1.0, 0.0, 1.0, 0.1]])
        cw = np.array([[1, 1, 1, 1, 1, 1], [1, 1, 1, 1, 1, 1], [0, 0, 0, 1, 0, 1], [0, 0, 0, 0, 0, 0.7], [0, 0, 0.7, 0, 1, 1]])
        ccw = np.array([[1, 1, 1, 1, 1, 1], [1, 1, 1, 1, 1, 1], [1, 1, 1, 0, 1, 1], [1, 0, 1, 0, 1, 1], [0, 0, 0, 0, 0, 0]])
        self.all_gaits = {'roll':roll,'ccw':ccw,'cw':cw}

    def read_calibration_file(self, filename):
        try : 
            if filename[-4:] == '.xls':
                # hand calibration excel file
                workbook = xlrd.open_workbook(filename)
                shortsheet = workbook.sheet_by_name('Short Sensors')
                longsheet = workbook.sheet_by_name('Long Sensors')
                m = np.array([float(shortsheet.cell_value(9, col)) for col in range(0, 12, 2)] +
                            [float(longsheet.cell_value(10, col)) for col in range(0, 6, 2)])
                b = np.array([float(shortsheet.cell_value(9, col)) for col in range(1, 13, 2)] +
                            [float(longsheet.cell_value(10, col)) for col in range(1, 7, 2)])
            elif filename[-5:] == '.json':
                # autocalibration JSON file
                data = json.load(open(filename))
                m = np.array(data.get('m'))
                b = np.array(data.get('b'))
            else:
                raise FileError('Invalid calibration file')
            return m, b
        except FileError as ce:
            print("Error occurred:", ce)

    def quat2vec(self, q):
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

    def send_command(self, input_string, addr, delay_time):
        self.sock_send.sendto(input_string.encode('utf-8'), addr)
        if delay_time < 0:
            delay_time = 0
        time.sleep(delay_time/1000)
        
    def sendRosMSG(self):
        # send ROS messages
        control_msg = TensegrityStamped()
        # get timestamp
        timestamp = rospy.Time.now()
        control_msg.header.stamp = timestamp
        # gait info
        info = Info()
        info.min_length = self.min_length
        info.RANGE024 = self.RANGE024
        info.RANGE135 = self.RANGE135
        info.max_speed = self.max_speed
        info.tol = self.tol
        info.low_tol = self.low_tol
        info.P = self.P
        info.I = self.I
        info.D = self.D
        control_msg.info = info
        # motors
        for motor_id in range(self.num_motors):
           motor = Motor()
           motor.id = motor_id
           motor.position = self.pos[motor_id]
           motor.target = self.states[self.state,motor_id]
           motor.speed = self.command[motor_id] * self.max_speed #abs(command[motor_id]) * max_speed
           # motor.direction = command[motor_id] > 0
           motor.done = self.done[motor_id]
           motor.encoder_counts = int(self.encoder_counts[motor_id])
           motor.encoder_length = self.encoder_length[motor_id]
           control_msg.motors.append(motor)
        # sensors
        for sensor_id in range(self.num_sensors):
           sensor = Sensor()
           sensor.id = sensor_id
           sensor.length = self.length[sensor_id]
           sensor.capacitance = self.cap[sensor_id]
           control_msg.sensors.append(sensor)
        # imu
        for rod in range(3):
            IMU = Imu()
            IMU.ax = self.accelerometer[rod][0]
            IMU.ay = self.accelerometer[rod][1]
            IMU.az = self.accelerometer[rod][2]
            IMU.gx = self.gyroscope[rod][0]
            IMU.gy = self.gyroscope[rod][1]
            IMU.gz = self.gyroscope[rod][2]
            control_msg.imus.append(IMU)
        # trajectory_msg = Trajectory()
        # for x,y in self.COMs:
        #     point = Point()
        #     point.x = x
        #     point.y = y
        #     trajectory_msg.COMs.append(point)
        # for x,y in self.PAs:
        #     point = Point()
        #     point.x = x
        #     point.y = y
        #     trajectory_msg.PAs.append(point)
        # control_msg.trajectory = trajectory_msg

        # publish
        self.control_pub.publish(control_msg)
        
    def read(self):
        data, addr = self.sock_receive.recvfrom(255)  # Receive data (up to 255 bytes)
        # Decode the data (assuming it's sent as a string)
        received_data = data.decode('utf-8')
        # print('The data I received: ',received_data)
        # try :
        # Received data in the form "N_Arduino q0 q1 q2 q3 C0 C1 C2" where N_Arduino indicates the number of the Arduino of the received data
        sensor_values = received_data.split()
        # Convert the string values to actual float values and store them in an array
        sensor_array = [float(value) for value in sensor_values]
        # print('The formattted data: ',sensor_array)
        if(addr not in self.addresses):
            self.addresses[int(sensor_array[0])] = addr
        #print(sensor_array)
        """
        Following code of function read(self) configurated for a 3 bar tensegrity with following sensors
        Rod 0 (red) has sensors C, E, and I (2, 4, and 8) and motors 2 and 4
        Rod 1 (green) has sensors B, D, and H (1, 3, and 7) and motors 1 and 3
        Rod 2 (blue) has sensors A, F, and G (0, 5, and 6) and motors 0 and 5
        
        The first IMU is on the blue bar and points from node 5 to node 4
        The second IMU is on the red bar and points from node 1 to node 0
        """
        if(len(sensor_array) == 13) : #Number of data send space
            self.which_Arduino = int(sensor_array[0])
            if(sensor_array[1] == 0.2 or sensor_array[2] == 0.2 or sensor_array[3] == 0.2 ) :
                print('MPR121 or I2C of Arduino '+str(self.which_Arduino)+' wrongly initialized, please reboot Arduino')

            if(int(sensor_array[0]) == 0) :
                self.cap[4] = sensor_array[1]
                self.cap[2] = sensor_array[2]
                self.cap[8] = sensor_array[3]
                self.encoder_counts[4] = sensor_array[6]
                self.encoder_counts[2] = sensor_array[5]
            if(int(sensor_array[0]) == 1) :
                self.cap[3] = sensor_array[1]
                self.cap[1] = sensor_array[2] 
                self.cap[7] = sensor_array[3]
                self.encoder_counts[3] = sensor_array[6]
                self.encoder_counts[1] = sensor_array[5]
            if(int(sensor_array[0]) == 2) :
                self.cap[5] = sensor_array[1]
                self.cap[0] = sensor_array[2] 
                self.cap[6] = sensor_array[3]
                self.encoder_counts[5] = sensor_array[6]
                self.encoder_counts[0] = sensor_array[5]

            self.encoder_length = [counts/self.encoder_resolution/self.gear_ratio*np.pi*self.winch_diameter for counts in self.encoder_counts]
            
            #add control code here
            if not 0.2 in self.cap: #Default capacitance value of MPR121
                for i in range(len(self.cap)) :
                    self.length[i] = (self.cap[i] - self.b[i]) / self.m[i] #mm 
                #check if motor reached the target
                for i in range(self.num_motors):
                    if i < 3:
                        self.pos[i] = (self.length[i] - self.min_length) / self.RANGE135# calculate the current position of the motor
                    else:
                        self.pos[i] = (self.length[i] - self.min_length) / self.RANGE024# calculate the current position of the motor   
            # #read imu data
            # if(sensor_array[0] == 0) :
            #     self.imu[1] = self.quat2vec(sensor_array[1:5])

            # if(sensor_array[0] == 2) :
            #     self.imu[0] = self.quat2vec(sensor_array[1:5])

            #if(sensor_array[0] == 3) : If 3 IMU's used
            #   self.imu[3] = self.quat2vec(sensor_array[1:5])

            self.accelerometer[self.which_Arduino][0] = sensor_array[7] # ax
            self.accelerometer[self.which_Arduino][1] = sensor_array[8] # ay
            self.accelerometer[self.which_Arduino][2] = sensor_array[9] # az
            self.gyroscope[self.which_Arduino][0] = sensor_array[10]    # gx
            self.gyroscope[self.which_Arduino][1] = sensor_array[11]    # gy
            self.gyroscope[self.which_Arduino][2] = sensor_array[12]    # gz

        else:
            if (None in self.addresses) :
                for i in range(len(self.addresses)):
                    if(self.addresses[i] == None) : 
                        print('Arduino '+str(i)+' wrongly initialized, please reboot Arduino')
                    else:     
                        self.send_command(self.stop_msg, self.addresses[i],0)

            else :
                print('+')
                for i in range(len(self.addresses)) :
                    self.send_command(self.stop_msg, self.addresses[i],0)
            
        # except :
        #     print('There has been an error')
        #     print('Received data:', received_data)

    def compute_command(self) :
        # update STATES based on RL policy
        _,_,endcaps = self.get_pose()
        self.states = np.array([self.policy.get_action(endcaps,self.pos)])
        self.state = 0


        command_msg = self.stop_msg.split()
        for i in range(self.num_motors):
        #     # two tolerances for shorter and longer commands
        #     if self.states[self.state, i] < 0.5:
        #         tolerance = self.low_tol
        #     else:
        #         tolerance = self.tol

            # NO REST FOR THE WEARY
            #check if motor reached the target
            # if self.pos[i] + tolerance > self.states[self.state, i] and self.pos[i] - tolerance < self.states[self.state, i]:
            #     self.done[i] = True
            #     self.command[i] = 0
            # if not self.done[i]:

            self.error[i] = self.pos[i] - self.states[self.state, i]
            # self.d_error[i] = self.error[i] - self.prev_error[i]
            # self.cum_error[i] = self.cum_error[i] + self.error[i]
            # self.prev_error[i] = self.error[i]
            #update speed
            self.command[i] = max([min([self.P*self.error[i], 1]), -1])
            self.speed[i] = self.command[i] * self.max_speed * self.flip[i]
            command_msg[i+self.offset] = str(self.speed[i])

                
        # if all(self.done):
        #     self.state += 1
        #     self.state %= len(self.states)#self.num_steps
        #     for i in range(self.num_motors):
        #         self.done[i] = False
        #         self.prev_error[i] = 0
        #         self.cum_error[i] = 0
        #     if self.next_states is not None:
        #         # print('starting next primitive')
        #         print('Next:',self.next_states)
        #         self.states = self.next_states
        #         # print(self.states)
        #         self.next_states = None
        #         self.state = 1
        #     elif 'planning' in self.action_sequence[0]:
        #         print('planning')
        #         self.state = 1
        #         self.states = np.array([[1,1,1,1,1,1],[1,1,1,1,1,1],[1,1,1,1,1,1],[1,1,1,1,1,1]])
        #         self.RANGE024 = 100
        #         self.RANGE135 = 100

        #     else:
        #         # if it's a transition step
        #             # update the ranges
        #             if self.state % len(self.states) == 1:
        #             # if state % 2 == 1:
        #                 # COM,principal_axis,endcaps = get_pose()

        #                 # print('COM: ',COM)
                        
                        
        #                 # print('Axis: ',principal_axis)
        #                 # print('Endcaps: ',endcaps)
        #                 # action = best_action(str(RANGE024) + "_" + str(RANGE135),action_dict,COM,principal_axis,trajectory)

        #                 # if prev_gait == 'roll':
        #                 #     prev_action = str(RANGE135) + "_" + str(RANGE024)
        #                 # else:
        #                 #     prev_action = prev_gait

        #                 prev_action = str(self.RANGE135) + "_" + str(self.RANGE024)

        #                 state_msg = State()
        #                 state_msg.prev_action = prev_action
        #                 state_msg.reverse_the_gait = self.reverse_the_gait
        #                 self.state_pub.publish(state_msg)

        #                 self.action_sequence = ['planning__planning' for act in self.action_sequence]

        #                 self.next_states = None
        #                 self.state = 1
        #                 self.states = np.array([[1,1,1,1,1,1],[1,1,1,1,1,1],[1,1,1,1,1,1],[1,1,1,1,1,1]])
        #                 self.RANGE024 = 100
        #                 self.RANGE135 = 100

        print('State: ',self.state)
        # print(state)
        print("Position: ",self.pos)
        print("Target: ",self.states[self.state])
        # print(pos)
        # print(states[state])
        # print("Done: ",self.done)
        print("Length: ",self.length)
        print("Capacitance: ",self.cap)
        print(' '.join(command_msg))
        self.send_command(' '.join(command_msg), self.addresses[self.which_Arduino],0)
        #self.send_command(self.stop_msg, self.addresses[self.which_Arduino],0)
        # print('+++++')

    # def mpc_callback(self,msg):
    #     print('Planning results are in!')

    #     # record motion planning results
    #     self.action_sequence = [act for act in msg.actions]
    #     self.COMs = np.array([[com.x,com.y] for com in msg.COMs])
    #     self.PAs = np.array([[pa.x,pa.y] for pa in msg.PAs])
    #     endcaps = np.array([[end.x,end.y,end.z] for end in msg.endcaps])

    #     # # ensure we incorporate the results in the next loop iteration
    #     # global done
    #     # global state
    #     # for i in range(len(done)):
    #     #     done[i] = True
    #     # state = 0

    #     # restart the step
    #     for i in range(self.num_motors):
    #         self.done[i] = False
    #         self.prev_error[i] = 0
    #         self.cum_error[i] = 0

    #     # addjust the ranges and gait based on MPC results
    #     action = self.action_sequence[0]

    #     print('Action Sequence: ',self.action_sequence)
                                
    #     if action == 'cw':
    #         self.next_states = self.all_gaits.get('cw')
    #         # bottom_nodes = prev_nodes.get(prev_bottom_nodes)
    #         bottom_nodes = self.bottom3(endcaps)
    #         if not bottom_nodes in prev_nodes.keys():
    #             bottom_nodes = self.prev_bottom_nodes
    #         # prev_bottom_nodes = bottom_nodes
    #         self.prev_bottom_nodes = prev_nodes.get(bottom_nodes)
    #         print('Bottom Nodes: ',bottom_nodes)
    #         self.next_states = transform_gait(self.next_states,bottom_nodes)
    #         self.state = 1
    #         self.prev_gait = 'cw'

    #         self.RANGE024 = 100
    #         self.RANGE135 = 100
    #     elif action == 'ccw':
    #         self.next_states = self.all_gaits.get('ccw')
    #         bottom_nodes = self.bottom3(endcaps)
    #         if not bottom_nodes in prev_nodes.keys():
    #             bottom_nodes = self.prev_bottom_nodes
    #         print('Bottom Nodes: ','Bottom Nodes: ',bottom_nodes)
    #         self.next_states = transform_gait(self.next_states,bottom_nodes)
    #         self.state = 1
    #         self.prev_gait = 'ccw'

    #         self.RANGE024 = 100
    #         self.RANGE135 = 100
    #     else:
    #         # if we have successive rolling steps,
    #         # change the ranges but skip the transition
    #         if not self.prev_gait in ['cw','ccw']:
    #             for i in range(self.num_motors):
    #                 self.done[i] = True
    #         self.next_states = self.all_gaits.get('roll')
    #         # bottom_nodes = next_nodes.get(prev_bottom_nodes)
    #         bottom_nodes = self.bottom3(endcaps)
    #         if not bottom_nodes in prev_nodes.keys():
    #             bottom_nodes = self.prev_bottom_nodes
    #         # prev_bottom_nodes = bottom_nodes
    #         self.prev_bottom_nodes = next_nodes.get(bottom_nodes)
    #         print('Bottom Nodes: ',bottom_nodes)
    #         self.next_states = transform_gait(self.next_states,bottom_nodes)
    #         if self.reverse_the_gait:
    #             self.next_states = reverse_gait(self.next_states,bottom_nodes)
    #         self.state = 1
    #         self.prev_gait = 'roll'
    #         ranges = action.split('_')
    #         self.RANGE024 = int(ranges[-1])
    #         self.RANGE135 = int(ranges[-2])
    #         if self.RANGE135 >= 130 or self.RANGE024 >= 130:
    #             self.tol = 0.35
    #         else:
    #             self.tol = 0.15
    #     print('Action: ',action)

    #     # # catch perception error
    #     # if self.next_states is None:
    #     #     self.next_states = np.array([[1]*self.num_motors]*self.num_steps) # recover

    def bottom3(self,nodes):
        try:
            x_sr,y_sr,z_sr = self.nodes2sr(nodes)
            z_values = np.array([item[1] for item in sorted(z_sr.items())])
            bottom_nodes = tuple(sorted(np.argpartition(z_values, 3)[:3]))
            return bottom_nodes
        except:
            return None

    def nodes2sr(self,nodes):
        x_sr = {str(key):nodes[key,0] for key in range(number_of_rods*2)}
        y_sr = {str(key):nodes[key,1] for key in range(number_of_rods*2)}
        z_sr = {str(key):nodes[key,2] for key in range(number_of_rods*2)}
        return x_sr,y_sr,z_sr

    def on_press(self, key):
        # print('press')
        try : 
            # if key == keyboard.KeyCode.from_char('q'):
            #     # self.quitting = True
            #     raise S_Q_Pressed()
            if key == keyboard.KeyCode.from_char('s'):
                self.quitting = True
                raise S_Q_Pressed()
            elif key == keyboard.KeyCode.from_char('r'):
                self.states = np.array([[1.0]*self.num_motors]*self.num_steps)
                self.done = np.array([False] * self.num_motors)
                self.tol = 0.2
                self.P = 5.0
                self.max_speed = 70

            elif key == keyboard.KeyCode.from_char('n'):
                self.states = np.array([[1.0]*self.num_motors]*self.num_steps)
                self.done = np.array([False] * self.num_motors)
                self.tol = 0.03
                self.P = 5.0
                # max_speed = 80
                self.RANGE = 90
                self.LEFT_RANGE = self.RANGE   
            # elif key == keyboard.KeyCode.from_char('f'):
            #     self.keep_going = False
            #     msg = self.stop_msg.split()
            #     if self.zero_pressed:
            #         msg[0+self.offset] = str(self.init_speed)
            #         for i in range(len(self.addresses)) :
            #             self.send_command(' '.join(msg), self.addresses[i],0)
            #     elif self.one_pressed:
            #         msg[1+self.offset] = str(self.init_speed)
            #         for i in range(len(self.addresses)) :
            #             self.send_command(' '.join(msg), self.addresses[i],0)
            #     elif self.two_pressed:
            #         msg[2+self.offset] = str(self.init_speed)
            #         for i in range(len(self.addresses)) :
            #             self.send_command(' '.join(msg), self.addresses[i],0)
            #     elif self.three_pressed:
            #         msg[3+self.offset] = str(self.init_speed)
            #         for i in range(len(self.addresses)) :
            #             self.send_command(' '.join(msg), self.addresses[i],0)
            #     elif self.four_pressed:
            #         msg[4+self.offset] = str(self.init_speed)
            #         for i in range(len(self.addresses)) :
            #             self.send_command(' '.join(msg), self.addresses[i],0)
            #     elif self.five_pressed:
            #         msg[5+self.offset] = str(self.init_speed)
            #         for i in range(len(self.addresses)) :
            #             self.send_command(' '.join(msg), self.addresses[i],0)
            # elif key == keyboard.KeyCode.from_char('b'):
            #     self.keep_going = False
            #     msg = self.stop_msg.split()
            #     if self.zero_pressed:
            #         msg[0+self.offset] = str(-self.init_speed)
            #         for i in range(len(self.addresses)) :
            #             self.send_command(' '.join(msg), self.addresses[i],0)
            #     elif self.one_pressed:
            #         msg[1+self.offset] = str(-self.init_speed)
            #         for i in range(len(self.addresses)) :
            #             self.send_command(' '.join(msg), self.addresses[i],0)
            #     elif self.two_pressed:
            #         msg[2+self.offset] = str(-self.init_speed)
            #         for i in range(len(self.addresses)) :
            #             self.send_command(' '.join(msg), self.addresses[i],0)
            #     elif self.three_pressed:
            #         msg[3+self.offset] = str(-self.init_speed)
            #         for i in range(len(self.addresses)) :
            #             self.send_command(' '.join(msg), self.addresses[i],0)
            #     elif self.four_pressed:
            #         msg[4+self.offset] = str(-self.init_speed)
            #         for i in range(len(self.addresses)) :
            #             self.send_command(' '.join(msg), self.addresses[i],0)
            #     elif self.five_pressed:
            #         msg[5+self.offset] = str(-self.init_speed)
            #         for i in range(len(self.addresses)) :
            #             self.send_command(' '.join(msg), self.addresses[i],0)
            # elif key == keyboard.KeyCode.from_char('0'):
            #     self.zero_pressed = True
            # elif key == keyboard.KeyCode.from_char('1'):
            #     self.one_pressed = True
            # elif key == keyboard.KeyCode.from_char('2'):
            #     self.two_pressed = True
            # elif key == keyboard.KeyCode.from_char('3'):
            #     self.three_pressed = True
            # elif key == keyboard.KeyCode.from_char('4'):
            #     self.four_pressed = True
            # elif key == keyboard.KeyCode.from_char('5'):
            #     self.five_pressed = True
            # elif key == keyboard.KeyCode.from_char('c'):
            #     self.keep_going = False
            #     self.calibration = True

        except AttributeError:
            pass

        except S_Q_Pressed :
            print("\nStopping motors")
            self.keep_going = False
            # set duty cycle as 0 to turn off the motors
            for i in range(len(self.addresses)):
                self.send_command(self.stop_msg, self.addresses[i], 0)
                
    def on_release(self,key):
        # print('release')
        if  key == keyboard.KeyCode.from_char('0'):
            self.zero_pressed = False
            for i in range(len(self.addresses)) :
                    self.send_command(self.stop_msg, self.addresses[i],0)
        elif key == keyboard.KeyCode.from_char('1'):
            self.one_pressed = False
            for i in range(len(self.addresses)) :
                    self.send_command(self.stop_msg, self.addresses[i],0)
        elif key == keyboard.KeyCode.from_char('2'):
            self.two_pressed = False
            for i in range(len(self.addresses)) :
                    self.send_command(self.stop_msg, self.addresses[i],0)
        elif key == keyboard.KeyCode.from_char('3'):
            self.three_pressed = False
            for i in range(len(self.addresses)) :
                    self.send_command(self.stop_msg, self.addresses[i],0)
        elif key == keyboard.KeyCode.from_char('4'):
            self.four_pressed = False
            for i in range(len(self.addresses)) :
                    self.send_command(self.stop_msg, self.addresses[i],0)
        elif key == keyboard.KeyCode.from_char('5'):
            self.five_pressed = False
            for i in range(len(self.addresses)) :
                    self.send_command(self.stop_msg, self.addresses[i],0)
        elif key == keyboard.KeyCode.from_char('f'):
            for i in range(len(self.addresses)) :
                    self.send_command(self.stop_msg, self.addresses[i],0)
        elif key == keyboard.KeyCode.from_char('b'):
            for i in range(len(self.addresses)) :
                    self.send_command(self.stop_msg, self.addresses[i],0)

    def init_tracker(self):
        print('i got to init_tracker')

        # get cable lengths
        while None in self.addresses:
            self.read()
            print('getting cable lengths...')

        cable_length_msg = Float64MultiArray()
        cable_length_msg.data = self.length

        # get RGBD data
        rgb_msg = rospy.wait_for_message('/rgb_images',Image,None)
        depth_msg = rospy.wait_for_message('/depth_images',Image,None)

        # send trajectory or other points to be superimposed
        # trajectory_x = Float64MultiArray()
        # trajectory_y = Float64MultiArray()
        # trajectory_x.data = self.trajectory[:,0].tolist()
        # trajectory_y.data = self.trajectory[:,1].tolist()

        request = InitTrackerRequest()
        request.rgb_im = rgb_msg
        request.depth_im = depth_msg
        request.cable_lengths = cable_length_msg
        # request.trajectory_x = trajectory_x
        # request.trajectory_y = trajectory_y

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

    def get_pose(self):
        service_name = "get_pose"
        
        vectors = np.array([[0.0,0.0,0.0]])
        centers = []
        endcaps = []
        try:
            request = GetPoseRequest()
            get_pose_srv = rospy.ServiceProxy(service_name, GetPose)
            rospy.loginfo("Request sent. Waiting for response...")
            response: GetPoseResponse = get_pose_srv(request)
            rospy.loginfo(f"Got response. Request success: {response.success}")
            if response.success:
                for pose in response.poses:
                    
                    rotation_matrix = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z,pose.orientation.w]).as_matrix()
                    
                    unit_vector = rotation_matrix[:,2]
                    center = [pose.position.x,pose.position.y,pose.position.z]
                    endcaps.append(np.array(center) + L/2*unit_vector)
                    endcaps.append(np.array(center) - L/2*unit_vector)
                    
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
            
    def run(self):
        while not self.quitting :
            # try : 
                self.read()
                # self.sendRosMSG()
                if(self.keep_going and None not in self.addresses) :
                    self.compute_command()
                    self.sendRosMSG()
                # else:
                    # set duty cycle as 0 to turn off the motors
                    # for i in qend_command(self.stop_msg, self.addresses[i], 0)
                # if(self.calibration) :
                #     self.sendRosMSG()
                #     for i in range(self.num_sensors) :
                #         print(f"Capacitance {chr(i + 97)}: {self.cap[i]:.2f} \t Length: {self.length[i]:.2f} \n")
            # except :
            #     print("BIG ERROR")
            
        
if __name__ == '__main__':
    tensegrity_robot = TensegrityRobot()
    tensegrity_robot.run()
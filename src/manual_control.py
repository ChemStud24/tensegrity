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
import rospkg
import socket
#from tensegrity.msg import Motor, Info, MotorsStamped, Sensor, SensorsStamped, Imu, ImuStamped
from tensegrity.msg import Motor, Info, Sensor, Imu, TensegrityStamped
#from geometry_msgs.msg import QuaternionStamped


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
        self.flip = [1, 1, -1, 1, 1, 1] # flip direction of motors
        self.acceleration = [0] * 3
        self.RANGE = 90
        self.LEFT_RANGE = 90
        self.max_speed = 80
        self.tol = 0.15
        self.low_tol = 0.15
        self.P = 6.0
        self.I = 0.01
        self.D = 0.5
        
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

        #keyboard variables
        self.pressed_key = None
        self.awaiting_command = None
        

    def initialize(self):

        self.my_listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.my_listener.start()
        
        rospy.init_node('tensegrity')
        self.control_pub = rospy.Publisher('control_msg', TensegrityStamped, queue_size=10) ## correct ??

        package_path = rospkg.RosPack().get_path('tensegrity')
        calibration_file = os.path.join(package_path,'calibration/calibration.json')
        
        #self.m = np.array([0.04437, 0.06207, 0.02356, 0.04440, 0.04681, 0.05381, 0.02841, 0.03599, 0.03844])
        #self.b = np.array([15.763, 13.524, 15.708, 10.084, 15.628, 15.208, 16.356, 12.575, 13.506])
        
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

    def send_command(self, input_string, addr, delay_time):
        self.sock_send.sendto(input_string.encode('utf-8'), addr)
        if delay_time < 0:
            delay_time = 0
        time.sleep(delay_time/1000)
        
    def sendRosMSG(self):
        # send ROS messages
        control_msg = TensegrityStamped()
        # strain_msg = SensorsStamped()
        # imu_msg = ImuStamped()
        # get timestamp
        timestamp = rospy.Time.now()
        control_msg.header.stamp = timestamp
        # strain_msg.header.stamp = timestamp
        # imu_msg.header.stamp = timestamp
        # gait info
        info = Info()
        info.min_length = self.min_length
        info.RANGE = self.RANGE
        # info.LEFT_RANGE = self.LEFT_RANGE
        info.max_speed = self.max_speed
        info.tol = self.tol
        # info.low_tol = self.low_tol
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
           control_msg.motors.append(motor)
        # sensors
        for sensor_id in range(self.num_sensors):
           sensor = Sensor()
           sensor.id = sensor_id
           sensor.length = self.length[sensor_id]
           sensor.capacitance = self.cap[sensor_id]
           control_msg.sensors.append(sensor)
        # imu
        # for imu_id in range(self.num_imus):
        #    IMU = Imu()
        #    IMU.id = imu_id
        #    if any(self.imu[imu_id]) == None:
        #        IMU.x = None
        #        IMU.y = None
        #        IMU.z = None
        #    else:
        #        IMU.x = self.imu[imu_id][0]
        #        IMU.y = self.imu[imu_id][1]
        #        IMU.z = self.imu[imu_id][2]
        #    imu_msg.imus.append(IMU)
        # publish
        self.control_pub.publish(control_msg)
        # strain_pub.publish(strain_msg)
        # imu_pub.publish(imu_msg)
        
    def read(self):
        data, addr = self.sock_receive.recvfrom(255)  # Receive data (up to 255 bytes)
        # Decode the data (assuming it's sent as a string)
        received_data = data.decode('utf-8')
        print(received_data)
        try :
            # Received data in the form "N_Arduino q0 q1 q2 q3 C0 C1 C2" where N_Arduino indicates the number of the Arduino of the received data
            sensor_values = received_data.split()
            # Convert the string values to actual float values and store them in an array
            sensor_array = [float(value) for value in sensor_values]
            print(sensor_array)
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
            if(len(sensor_array) == 8) : #Number of data send space
                self.which_Arduino = int(sensor_array[0])
                if(sensor_array[5] == 0.2 or sensor_array[6] == 0.2 or sensor_array[7] == 0.2 ) :
                    print('MPR121 or I2C of Arduino '+str(self.which_Arduino)+' wrongly initialized, please reboot Arduino')

                if(int(sensor_array[0]) == 0) :
                    self.cap[4] = sensor_array[5]
                    self.cap[2] = sensor_array[6] 
                    self.cap[8] = sensor_array[7]
                if(int(sensor_array[0]) == 1) :
                    self.cap[3] = sensor_array[5]
                    self.cap[1] = sensor_array[6] 
                    self.cap[7] = sensor_array[7]
                if(int(sensor_array[0]) == 2) :
                    self.cap[5] = sensor_array[5]
                    self.cap[0] = sensor_array[6] 
                    self.cap[6] = sensor_array[7]
                
                #add control code here
                if not 0.2 in self.cap: #Default capacitance value of MPR121
                    for i in range(len(self.cap)) :
                        self.length[i] = (self.cap[i] - self.b[i]) / self.m[i] #mm 
                    #check if motor reached the target
                    for i in range(self.num_motors):
                        if i < 3:
                            self.pos[i] = (self.length[i] - self.min_length) / self.LEFT_RANGE# calculate the current position of the motor
                        else:
                            self.pos[i] = (self.length[i] - self.min_length) / self.RANGE# calculate the current position of the motor   
                #read imu data
                if(sensor_array[0] == 0) :
                    self.imu[1] = self.quat2vec(sensor_array[1:5])

                if(sensor_array[0] == 2) :
                    self.imu[0] = self.quat2vec(sensor_array[1:5])

                #if(sensor_array[0] == 3) : If 3 IMU's used
                #   self.imu[3] = self.quat2vec(sensor_array[1:5])

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
            
        except :
            print('There has been an error')
            print('Received data:', received_data)

    def on_press(self, key):
        if key == keyboard.KeyCode.from_char('q'):
            self.keep_going = False
        elif key == keyboard.KeyCode.from_char('s'):
            self.keep_going = False
        elif key == keyboard.KeyCode.from_char('f'):
            if self.pressed_key in range(num_motors):
                if self.awaiting_command:
                    command_msg = self.stop_msg.split()
                    command_msg[self.pressed_key + self.offset] = str(self.max_speed)
                    self.send_command(' '.join(command_msg), self.addresses[self.which_Arduino],0)
                    self.awaiting_command = False
        elif key == keyboard.KeyCode.from_char('b'):
            if self.pressed_key in range(self.num_motors):
                if self.awaiting_command:
                    command_msg = self.stop_msg.split()
                    command_msg[self.pressed_key + self.offset] = str(-self.max_speed)
                    self.send_command(' '.join(command_msg), self.addresses[self.which_Arduino],0)
                    self.awaiting_command = False
        elif key == keyboard.KeyCode.from_char('0'):
            self.pressed_key = 0
        elif key == keyboard.KeyCode.from_char('1'):
            self.pressed_key = 1
        elif key == keyboard.KeyCode.from_char('2'):
            self.pressed_key = 2
        elif key == keyboard.KeyCode.from_char('3'):
            self.pressed_key = 3
        elif key == keyboard.KeyCode.from_char('4'):
            self.pressed_key = 4
        elif key == keyboard.KeyCode.from_char('5'):
            self.pressed_key = 5
                
    def on_release(self,key):
        if key == keyboard.KeyCode.from_char('f'):
            for i in range(len(self.addresses)):
                self.send_command(self.stop_msg, self.addresses[i], 0)
            self.awaiting_command = True
        elif key == keyboard.KeyCode.from_char('b'):
            for i in range(len(self.addresses)):
                self.send_command(self.stop_msg, self.addresses[i], 0)
            self.awaiting_command = True
            
    def run(self):
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
        while not self.quitting :
            try : 
                self.read()
                # self.sendRosMSG()
                if(self.keep_going and None not in self.addresses) :
                    self.sendRosMSG()
                    self.compute_command()
                # else:
                    # set duty cycle as 0 to turn off the motors
                    # for i in qend_command(self.stop_msg, self.addresses[i], 0)
                if(self.calibration) :
                    self.sendRosMSG()
                    for i in range(self.num_sensors) :
                        print(f"Capacitance {chr(i + 97)}: {self.cap[i]:.2f} \t Length: {self.length[i]:.2f} \n")
            except :
                pass
            
        
if __name__ == '__main__':
    tensegrity_robot = TensegrityRobot()
    tensegrity_robot.run()
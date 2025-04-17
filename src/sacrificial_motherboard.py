#!/usr/bin/env python3
import os
import sys
import time
import json
import numpy as np
import rospy
import rospkg
import socket
import datetime

class TensegrityRobot:
    def __init__(self):
        # self.imu = [[0, 0, 0]] * self.num_imus
        # self.accelerometer = [[0]*3]*3
        # self.gyroscope = [[0]*3]*3
        self.data_labels = ['ax','ay','az','gx','gy','gz']
        self.package_path = rospkg.RosPack().get_path('tensegrity')
        self.output_dir = package_path + '../../data/' + datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        os.mkdir(self.output_dir)
        self.count = 0
        
        # UDP variables
        self.UDP_IP = "0.0.0.0"  # Listen to all incoming interfaces
        self.UDP_PORT = 2390     # Same port used in the Arduino sketch
        self.sock_receive = None
        self.sock_send = None
        
    def read(self):
        print("Processing data " + str(self.count))
        data, addr = self.sock_receive.recvfrom(255)  # Receive data (up to 255 bytes)

        # Decode the data (assuming it's sent as a string)
        received_data = data.decode('utf-8')
        # print('Received Data: ',received_data)

        # Received data in the form "ax ay az gx gy gz"
        sensor_values = received_data.split()
        data = {key:value for key,value in zip(self.data_labels,sensor_values)}
        data['time'] = rospy.get_time()
        json.dump(data,open(os.path.join(self.output_dir, str(self.count).zfill(4) + ".json"),'w'))
        self.count += 1

        # # Convert the string values to actual float values and store them in an array
        # sensor_array = [float(value) for value in sensor_values]

        # print(sensor_array)
        # if(addr not in self.addresses) :
        #     self.addresses[int(sensor_array[0])] = addr
        # print('Sensor Array: ',sensor_array)
            
        # self.accelerometer[self.which_Arduino][0] = sensor_array[7] # ax
        # self.accelerometer[self.which_Arduino][1] = sensor_array[8] # ay
        # self.accelerometer[self.which_Arduino][2] = sensor_array[9] # az
        # self.gyroscope[self.which_Arduino][0] = sensor_array[10]    # gx
        # self.gyroscope[self.which_Arduino][1] = sensor_array[11]    # gy
        # self.gyroscope[self.which_Arduino][2] = sensor_array[12]    # gz
            
    def run(self):
        print("Initializing")
        print("Running UDP connection with one Arduino: ")

        # Create a UDP socket for receiving data
        self.sock_receive = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Bind the socket to the address and port
        self.sock_receive.bind((self.UDP_IP, self.UDP_PORT))
        
        while True:
            self.read()
        
if __name__ == '__main__':
    tensegrity_robot = TensegrityRobot()
    tensegrity_robot.run()
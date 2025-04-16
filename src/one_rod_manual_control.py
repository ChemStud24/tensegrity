#!/usr/bin/env python3
import os
import time
import math
import numpy as np
from pynput import keyboard
import socket


class FileError(Exception):
    pass
class S_Q_Pressed(Exception):
    pass

class TensegrityRobot:
    def __init__(self):
        # self.num_sensors = 9
        self.num_motors = 6
        # self.num_imus = 2
        self.num_arduino = 3
        self.command = [0] * self.num_motors
        self.speed = [0] * self.num_motors
        self.flip = [1, 1, -1, 1, 1, 1] # flip direction of motors
        self.max_speed = 60
        
        self.my_listener = None
        self.keep_going = True
        self.quitting = False
        self.calibration = False
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
        self.motor2arduino = {0:2, 1:1, 2:0, 3:1, 4:0, 5:2}
        

    def initialize(self):

        self.my_listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.my_listener.start()
        
        self.offset = 3
        self.stop_msg = ' '.join(['0'] * (self.num_motors+2*self.offset))
        self.go_msg = ' '.join([str(self.max_speed)] * (self.num_motors+2*self.offset))
        self.back_msg = ' '.join([str(-self.max_speed)] * (self.num_motors+2*self.offset))
        # self.init_speed = 70

    def send_command(self, input_string, addr, delay_time):
        self.sock_send.sendto(input_string.encode('utf-8'), addr)
        if delay_time < 0:
            delay_time = 0
        time.sleep(delay_time/1000)
        
    def read(self):
        data, addr = self.sock_receive.recvfrom(255)  # Receive data (up to 255 bytes)
        # Decode the data (assuming it's sent as a string)
        received_data = data.decode('utf-8')
        print(received_data)
        # try :
        # Received data in the form "N_Arduino q0 q1 q2 q3 C0 C1 C2" where N_Arduino indicates the number of the Arduino of the received data
        sensor_values = received_data.split()
        # Convert the string values to actual float values and store them in an array
        sensor_array = [float(value) for value in sensor_values]
        # print(sensor_array)
        # print(self.addresses)
        if(addr not in self.addresses):
            self.addresses[int(sensor_array[0])] = addr
        #print(sensor_array)
        #     """
        #     Following code of function read(self) configurated for a 3 bar tensegrity with following sensors
        #     Rod 0 (red) has sensors C, E, and I (2, 4, and 8) and motors 2 and 4
        #     Rod 1 (green) has sensors B, D, and H (1, 3, and 7) and motors 1 and 3
        #     Rod 2 (blue) has sensors A, F, and G (0, 5, and 6) and motors 0 and 5
            
        #     The first IMU is on the blue bar and points from node 5 to node 4
        #     The second IMU is on the red bar and points from node 1 to node 0
        #     """
        #     if(len(sensor_array) == 13) : #Number of data send space
        #         self.which_Arduino = int(sensor_array[0])
        #         if(sensor_array[1] == 0.2 or sensor_array[2] == 0.2 or sensor_array[3] == 0.2 ) :
        #             print('MPR121 or I2C of Arduino '+str(self.which_Arduino)+' wrongly initialized, please reboot Arduino')

        #     else:
        #         if (None in self.addresses) :
        #             for i in range(len(self.addresses)):
        #                 if(self.addresses[i] == None) : 
        #                     print('Arduino '+str(i)+' wrongly initialized, please reboot Arduino')
        #                 else:     
        #                     self.send_command(self.stop_msg, self.addresses[i],0)

        #         else :
        #             print('+')
        #             for i in range(len(self.addresses)) :
        #                 self.send_command(self.stop_msg, self.addresses[i],0)
            
        # except :
        #     print('There has been an error')
        #     print('Received data:', received_data)

    def on_press(self, key):
        # if None in self.addresses:
        #     print('at least one rod is not connected')
        #     print(self.addresses)
        if key == keyboard.KeyCode.from_char('q'):
            self.keep_going = False
            self.quitting = True
        elif key == keyboard.KeyCode.from_char('s'):
            self.keep_going = False
            self.quitting = True
        elif key == keyboard.KeyCode.from_char('f'):
            if self.awaiting_command:
                for address in self.addresses:
                    if address is not None:
                        self.send_command(self.go_msg, address,0)
                self.awaiting_command = False
        elif key == keyboard.KeyCode.from_char('b'):
            if self.awaiting_command:
                for address in self.addresses:
                    if address is not None:
                        self.send_command(self.back_msg, address,0)
                self.awaiting_command = False
        # elif key == keyboard.KeyCode.from_char('0'):
        #     self.pressed_key = 0
        # elif key == keyboard.KeyCode.from_char('1'):
        #     self.pressed_key = 1
        # elif key == keyboard.KeyCode.from_char('2'):
        #     self.pressed_key = 2
        # elif key == keyboard.KeyCode.from_char('3'):
        #     self.pressed_key = 3
        # elif key == keyboard.KeyCode.from_char('4'):
        #     self.pressed_key = 4
        # elif key == keyboard.KeyCode.from_char('5'):
        #     self.pressed_key = 5
                
    def on_release(self,key):
        if key == keyboard.KeyCode.from_char('f'):
            for i in range(len(self.addresses)):
                if self.addresses[i] is not None:
                    self.send_command(self.stop_msg, self.addresses[i], 0)
            self.awaiting_command = True
        elif key == keyboard.KeyCode.from_char('b'):
            for i in range(len(self.addresses)):
                if self.addresses[i] is not None:
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
            # try : 
                self.read()
                # self.sendRosMSG()
                if(self.keep_going and None not in self.addresses) :
                    # self.compute_command()
                    pass
                # else:
                    # set duty cycle as 0 to turn off the motors
                    # for i in qend_command(self.stop_msg, self.addresses[i], 0)
            # except :
                # print('Big error')
            
        
if __name__ == '__main__':
    tensegrity_robot = TensegrityRobot()
    tensegrity_robot.run()
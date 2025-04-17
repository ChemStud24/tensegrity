import os
import sys
import time
import math
import tkinter as tk
import json
import xlrd
import numpy as np
from pynput import keyboard
import socket


class FileError(Exception):
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
        self.flip = [1, 1, -1, 1, -1, -1] # flip direction of motors
        self.accelerometer = [[0]*3]*3
        self.gyroscope = [[0]*3]*3
        self.encoder_counts = [0]*self.num_motors
        self.encoder_length = [0]*self.num_motors
        self.RANGE = 100
        self.LEFT_RANGE = 100
        self.max_speed = 80
        self.tol = 0.15
        self.low_tol = 0.15
        self.P = 6.0
        self.I = 0.01
        self.D = 0.5
        self.gear_ratio = 150
        self.winch_diameter = 6.35
        self.encoder_resolution = 12
        
        self.num_steps = None
        self.state = None
        self.states = None
        self.control_pub = None
        self.my_listener = None
        self.quitting = False
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

        # GUI setup
        self.root = tk.Tk()
        self.cap_labels = []
        for i in range(self.num_sensors):
            label = tk.Label(self.root, text=f"Capacitance {chr(i + 97)}: 0.00")
            label.pack()
            self.cap_labels.append(label)
        
        #keyboard variables
        self.zero_pressed = False
        self.one_pressed = False
        self.two_pressed = False
        self.three_pressed = False
        self.four_pressed = False
        self.five_pressed = False
        
    def initialize(self):
        
        self.my_listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.my_listener.start()
        
        package_path = "./"
        calibration_file = os.path.join(package_path,'calibration/new_calibration.json')
        
        self.m, self.b = self.read_calibration_file(calibration_file)
        
    def read_calibration_file(self, filename):
        try: 
            if filename[-4:] == '.xls':
                workbook = xlrd.open_workbook(filename)
                shortsheet = workbook.sheet_by_name('Short Sensors')
                longsheet = workbook.sheet_by_name('Long Sensors')
                m = np.array([float(shortsheet.cell_value(9, col)) for col in range(0, 12, 2)] +
                             [float(longsheet.cell_value(10, col)) for col in range(0, 6, 2)])
                b = np.array([float(shortsheet.cell_value(9, col)) for col in range(1, 13, 2)] +
                             [float(longsheet.cell_value(10, col)) for col in range(1, 7, 2)])
            elif filename[-5:] == '.json':
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
        
    def read(self):
        data, addr = self.sock_receive.recvfrom(255)
        received_data = data.decode('utf-8')
        print('Received Data: ', received_data)
        try:
            sensor_values = received_data.split()
            sensor_array = [float(value) for value in sensor_values]
            if addr not in self.addresses:
                self.addresses[int(sensor_array[0])] = addr
            print('Sensor Array: ', sensor_array)
            if len(sensor_array) == 13:
                self.which_Arduino = int(sensor_array[0])
                if sensor_array[1] == 0.2 or sensor_array[2] == 0.2 or sensor_array[3] == 0.2:
                    print('MPR121 or I2C of Arduino ' + str(int(sensor_array[0])) + ' wrongly initialized, please reboot Arduino')
                if int(sensor_array[0]) == 0:
                    self.cap[4] = sensor_array[1]
                    self.cap[2] = sensor_array[2]
                    self.cap[8] = sensor_array[3]
                    self.encoder_counts[4] = sensor_array[6]
                    self.encoder_counts[2] = sensor_array[5]
                if int(sensor_array[0]) == 1:
                    self.cap[3] = sensor_array[1]
                    self.cap[1] = sensor_array[2]
                    self.cap[7] = sensor_array[3]
                    self.encoder_counts[3] = sensor_array[6]
                    self.encoder_counts[1] = sensor_array[5]
                if int(sensor_array[0]) == 2:
                    self.cap[5] = sensor_array[1]
                    self.cap[0] = sensor_array[2]
                    self.cap[6] = sensor_array[3]
                    self.encoder_counts[5] = sensor_array[6]
                    self.encoder_counts[0] = sensor_array[5]
                self.encoder_length = [counts / self.encoder_resolution / self.gear_ratio * np.pi * self.winch_diameter for counts in self.encoder_counts]
                if not 0.2 in self.cap:
                    for i in range(len(self.cap)):
                        self.length[i] = (self.cap[i] - self.b[i]) / self.m[i]
                    for i in range(self.num_motors):
                        if i < 3:
                            self.pos[i] = (self.length[i] - self.min_length) / self.LEFT_RANGE
                        else:
                            self.pos[i] = (self.length[i] - self.min_length) / self.RANGE
                self.accelerometer[self.which_Arduino][0] = sensor_array[7]
                self.accelerometer[self.which_Arduino][1] = sensor_array[8]
                self.accelerometer[self.which_Arduino][2] = sensor_array[9]
                self.gyroscope[self.which_Arduino][0] = sensor_array[10]
                self.gyroscope[self.which_Arduino][1] = sensor_array[11]
                self.gyroscope[self.which_Arduino][2] = sensor_array[12]
            else:
                if None in self.addresses:
                    for i in range(len(self.addresses)):
                        if self.addresses[i] == None:
                            print('Arduino ' + str(i) + ' wrongly initialized, please reboot Arduino')
                        else:
                            self.send_command(self.stop_msg, self.addresses[i], 0)
                else:
                    print('+')
                    for i in range(len(self.addresses)):
                        self.send_command(self.stop_msg, self.addresses[i], 0)
        except Exception as this_error:
            exc_type, exc_value, exc_traceback = sys.exc_info()
            print('There has been an error: ', this_error)
            print('Line number: ', exc_traceback.tb_lineno)
            print('Received data:', received_data)

    def update_gui(self):
        for i in range(self.num_sensors):
            self.cap_labels[i].config(text=f"Capacitance {chr(i + 97)}: {self.cap[i]:.2f}")
        self.root.update_idletasks()
        self.root.after(100, self.update_gui) # Schedule the update_gui method to be called again after 100 ms

    def run(self):
        print("Initializing")
        self.initialize()
        print("Running UDP connection with Arduino's: ")
        self.sock_receive = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_receive.bind((self.UDP_IP, self.UDP_PORT))
        print("Opened connection press q to quit")
        
        self.root.after(100, self.update_gui) # Schedule the first call to update_gui
        self.root.mainloop()
        
        while not self.quitting:
            try:
                self.read()
                
            except Exception as this_error:
                exc_type, exc_value, exc_traceback = sys.exc_info()
                print('There has been an error: ', this_error)
                print('Line number: ', exc_traceback.tb_lineno)

if __name__ == '__main__':
    tensegrity_robot = TensegrityRobot()
    tensegrity_robot.run()
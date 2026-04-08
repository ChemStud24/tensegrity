import time
import math
from math import cos, sin
import json
import xlrd
import numpy as np
from pynput import keyboard
from scipy.spatial.transform import Rotation as R
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
        self.encoder_counts = [0] * self.num_motors

        # ✅ NEW distance sensors
        self.distance = [0] * self.num_motors

        self.imu = [[0, 0, 0]] * self.num_imus

        self.command = [0] * self.num_motors
        self.speed = [0] * self.num_motors

        self.flip = [1] * self.num_motors

        self.RANGE = 100
        self.LEFT_RANGE = 100
        self.max_speed = 60

        self.tol = 0.15

        self.P = 6.0
        self.I = 0.01
        self.D = 0.5

        self.states = None
        self.state = 0

        self.m = None
        self.b = None

        self.stop_msg = None
        self.init_speed = None

        self.which_Arduino = None

        # UDP
        self.UDP_IP = "0.0.0.0"
        self.UDP_PORT = 2390

        self.sock_receive = None
        self.sock_send = None

        self.addresses = [
            ('172.16.71.78', 2390),
            ('172.16.71.79', 2390),
            ('172.16.71.80', 2390)
        ]

        self.offset = 3

        self.quitting = False

        # keyboard states
        self.zero_pressed = False
        self.one_pressed = False
        self.two_pressed = False
        self.three_pressed = False
        self.four_pressed = False
        self.five_pressed = False

    def initialize(self):
        calibration_file = '../calibration/calibration_charles.xls'
        self.m, self.b = self.read_calibration_file(calibration_file)

        self.states = np.array([[0.0, 1.0, 1.0, 0.0, 1.0, 0.1]])

        self.stop_msg = ' '.join(['0'] * (self.num_motors + 2 * self.offset))
        self.init_speed = 70

        listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        listener.start()

    def read_calibration_file(self, filename):
        workbook = xlrd.open_workbook(filename)
        shortsheet = workbook.sheet_by_name('Short Sensors')
        longsheet = workbook.sheet_by_name('Long Sensors')

        m = np.array([float(shortsheet.cell_value(9, col)) for col in range(0, 12, 2)] +
                     [float(longsheet.cell_value(10, col)) for col in range(0, 6, 2)])

        b = np.array([float(shortsheet.cell_value(9, col)) for col in range(1, 13, 2)] +
                     [float(longsheet.cell_value(10, col)) for col in range(1, 7, 2)])

        return m, b

    def send_command(self, msg, addr, delay_time):
        self.sock_send.sendto(msg.encode(), addr)
        time.sleep(max(delay_time / 1000, 0))

    # ===========================
    # 🔥 UPDATED READ (WITH DISTANCE)
    # ===========================
    def read(self):
        data, addr = self.sock_receive.recvfrom(255)
        received_data = data.decode('utf-8')

        try:
            sensor_array = [float(v) for v in received_data.split()]

            aid = int(sensor_array[0])

            # distances
            d1 = sensor_array[13] 
            d2 = sensor_array[14]

            if aid == 0:
                self.cap[4], self.cap[2], self.cap[8] = sensor_array[1:4]
                self.encoder_counts[4] = sensor_array[6]
                self.encoder_counts[2] = sensor_array[5]
                self.distance[4], self.distance[2] = d1, d2

            elif aid == 1:
                self.cap[3], self.cap[1], self.cap[7] = sensor_array[1:4]
                self.encoder_counts[3] = sensor_array[6]
                self.encoder_counts[1] = sensor_array[5]
                self.distance[3], self.distance[1] = d1, d2

            elif aid == 2:
                self.cap[5], self.cap[0], self.cap[6] = sensor_array[1:4]
                self.encoder_counts[5] = sensor_array[6]
                self.encoder_counts[0] = sensor_array[5]
                self.distance[5], self.distance[0] = d1, d2

            print("Distance:", self.distance)

        except:
            print("Error:", received_data)

    # ===========================
    # 🎮 KEYBOARD CONTROL (RESTORED)
    # ===========================
    def on_press(self, key):
        try:
            if key.char == 'q':
                self.quitting = True

            elif key.char == 'f':
                msg = self.stop_msg.split()

                if self.zero_pressed: msg[0+self.offset] = str(self.init_speed)
                if self.one_pressed: msg[1+self.offset] = str(self.init_speed)
                if self.two_pressed: msg[2+self.offset] = str(self.init_speed)
                if self.three_pressed: msg[3+self.offset] = str(self.init_speed)
                if self.four_pressed: msg[4+self.offset] = str(self.init_speed)
                if self.five_pressed: msg[5+self.offset] = str(self.init_speed)

                for addr in self.addresses:
                    self.send_command(' '.join(msg), addr, 0)

            elif key.char == 'b':
                msg = self.stop_msg.split()

                if self.zero_pressed: msg[0+self.offset] = str(-self.init_speed)
                if self.one_pressed: msg[1+self.offset] = str(-self.init_speed)
                if self.two_pressed: msg[2+self.offset] = str(-self.init_speed)
                if self.three_pressed: msg[3+self.offset] = str(-self.init_speed)
                if self.four_pressed: msg[4+self.offset] = str(-self.init_speed)
                if self.five_pressed: msg[5+self.offset] = str(-self.init_speed)

                for addr in self.addresses:
                    self.send_command(' '.join(msg), addr, 0)

            elif key.char == '0': self.zero_pressed = True
            elif key.char == '1': self.one_pressed = True
            elif key.char == '2': self.two_pressed = True
            elif key.char == '3': self.three_pressed = True
            elif key.char == '4': self.four_pressed = True
            elif key.char == '5': self.five_pressed = True

        except:
            pass

    def on_release(self, key):
        try:
            if key.char in ['0','1','2','3','4','5','f','b']:
                self.zero_pressed = False
                self.one_pressed = False
                self.two_pressed = False
                self.three_pressed = False
                self.four_pressed = False
                self.five_pressed = False

                for addr in self.addresses:
                    self.send_command(self.stop_msg, addr, 0)
        except:
            pass

    def run(self):
        print("Initializing")
        self.initialize()

        self.sock_receive = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.sock_receive.bind((self.UDP_IP, self.UDP_PORT))

        print("Running... press q to quit")

        while not self.quitting:
            try:
                self.read()
            except:
                pass


if __name__ == '__main__':
    robot = TensegrityRobot()
    robot.run()

#!/usr/bin/env python3
"""
run_tensegrity_json_gait.py

Runs a tensegrity robot gait loaded from a JSON file.

Usage:
    python run_tensegrity_json_gait.py gait.json              # run once (default)
    python run_tensegrity_json_gait.py gait.json -n 5         # run 5 times
    python run_tensegrity_json_gait.py gait.json -n inf       # repeat indefinitely
    Press 'q' at any time to stop.

JSON format expected:
    {
        "actions": [
            [0.1, 1.0, 1.0, 1.0, 1.0, 1.0],
            ...
        ]
    }
    Values: 1.0 = extended (baseline), 0.1 = fully contracted.
"""

import os
import argparse
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
from tensegrity.msg import Motor, Info, Sensor, Imu, TensegrityStamped


class FileError(Exception):
    pass

class S_Q_Pressed(Exception):
    pass


def load_gait_json(filepath):
    """Load gait actions from a JSON file. Returns np.ndarray of shape (N, 6)."""
    with open(filepath, 'r') as f:
        data = json.load(f)
    if 'actions' not in data:
        raise FileError(f"JSON file '{filepath}' must contain an 'actions' key.")
    actions = np.array(data['actions'], dtype=float)
    if actions.ndim != 2 or actions.shape[1] != 6:
        raise FileError(f"Actions must be shape (N, 6), got {actions.shape}.")
    return actions


class TensegrityRobot:
    def __init__(self, gait_actions, repeat):
        """
        Args:
            gait_actions: np.ndarray of shape (N, 6) — one gait cycle.
            repeat: int or float('inf') — how many times to run the gait.
        """
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
        self.flip = [-1, 1, -1, 1, -1, 1]
        self.accelerometer = [[0]*3 for _ in range(3)]
        self.gyroscope = [[0]*3 for _ in range(3)]
        self.encoder_counts = [0]*self.num_motors
        self.encoder_length = [0]*self.num_motors
        self.RANGE = 100
        self.LEFT_RANGE = 100
        self.max_speed = 70
        self.tol = 0.15
        self.low_tol = 0.15
        self.P = 10.0
        self.I = 0.01
        self.D = 0.5
        self.gear_ratio = 150
        self.winch_diameter = 6.35
        self.encoder_resolution = 12

        # Gait state
        self._gait_actions = gait_actions       # single cycle, shape (N, 6)
        self._repeat = repeat                    # int or float('inf')
        self._repeat_count = 0                   # completed cycles so far
        self.states = gait_actions.copy()        # active states array
        self.num_steps = len(self.states)
        self.state = 0

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

        # UDP
        self.UDP_IP = "0.0.0.0"
        self.UDP_PORT = 2390
        self.sock_receive = None
        self.sock_send = None
        self.addresses = [None] * self.num_arduino
        self.offset = None

        # Keyboard digit flags
        self.zero_pressed = False
        self.one_pressed = False
        self.two_pressed = False
        self.three_pressed = False
        self.four_pressed = False
        self.five_pressed = False

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------

    def initialize(self):
        self.my_listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.my_listener.daemon = True
        self.my_listener.start()

        rospy.init_node('tensegrity')
        self.control_pub = rospy.Publisher('control_msg', TensegrityStamped, queue_size=10)

        package_path = rospkg.RosPack().get_path('tensegrity')
        calibration_file = os.path.join(package_path, 'calibration/new_calibration.json')
        self.m, self.b = self.read_calibration_file(calibration_file)

        self.offset = 3
        self.done = np.array([False] * self.num_motors)
        self.stop_msg = ' '.join(['0'] * (self.num_motors + 2 * self.offset))
        self.init_speed = 70

        repeat_str = 'inf' if self._repeat == float('inf') else str(int(self._repeat))
        print(f"Gait loaded: {self.num_steps} steps | repeat: {repeat_str}x | press 'q' to stop")

    # ------------------------------------------------------------------
    # Calibration
    # ------------------------------------------------------------------

    def read_calibration_file(self, filename):
        try:
            if filename.endswith('.xls'):
                workbook = xlrd.open_workbook(filename)
                shortsheet = workbook.sheet_by_name('Short Sensors')
                longsheet = workbook.sheet_by_name('Long Sensors')
                m = np.array([float(shortsheet.cell_value(9, col)) for col in range(0, 12, 2)] +
                             [float(longsheet.cell_value(10, col)) for col in range(0, 6, 2)])
                b = np.array([float(shortsheet.cell_value(9, col)) for col in range(1, 13, 2)] +
                             [float(longsheet.cell_value(10, col)) for col in range(1, 7, 2)])
            elif filename.endswith('.json'):
                data = json.load(open(filename))
                m = np.array(data.get('m'))
                b = np.array(data.get('b'))
            else:
                raise FileError('Invalid calibration file')
            return m, b
        except FileError as ce:
            print("Error occurred:", ce)

    # ------------------------------------------------------------------
    # Math helpers
    # ------------------------------------------------------------------

    def quat2vec(self, q):
        q0, q1, q2, q3 = float(q[0]), float(q[1]), float(q[2]), float(q[3])
        roll = -math.atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2))
        sinp = 2*(q0*q2-q3*q1)
        pitch = math.copysign(np.pi/2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
        yaw = -math.atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3)) + np.pi/2
        k = np.array([cos(yaw)*cos(pitch), sin(pitch), sin(yaw)*cos(pitch)])
        r = R.from_rotvec(-np.pi/2 * np.array([0, 1, 0]))
        k = r.apply(k)
        y = np.array([0, 1, 0])
        s = np.cross(k, y)
        v = np.cross(s, k)
        vrot = v*cos(roll) + np.cross(k, v)*sin(roll)
        return np.cross(k, vrot)

    # ------------------------------------------------------------------
    # Communication
    # ------------------------------------------------------------------

    def send_command(self, input_string, addr, delay_time):
        self.sock_send.sendto(input_string.encode('utf-8'), addr)
        time.sleep(max(delay_time, 0) / 1000)

    def sendRosMSG(self):
        control_msg = TensegrityStamped()
        timestamp = rospy.Time.now()
        control_msg.header.stamp = timestamp
        info = Info()
        info.min_length = self.min_length
        info.RANGE = self.RANGE
        info.max_speed = self.max_speed
        info.tol = self.tol
        info.P = self.P
        info.I = self.I
        info.D = self.D
        control_msg.info = info
        for motor_id in range(self.num_motors):
            motor = Motor()
            motor.id = motor_id
            motor.position = self.pos[motor_id]
            motor.target = self.states[self.state, motor_id]
            motor.speed = self.command[motor_id] * self.max_speed
            motor.done = self.done[motor_id]
            motor.encoder_counts = int(self.encoder_counts[motor_id])
            motor.encoder_length = self.encoder_length[motor_id]
            control_msg.motors.append(motor)
        for sensor_id in range(self.num_sensors):
            sensor = Sensor()
            sensor.id = sensor_id
            sensor.length = self.length[sensor_id]
            sensor.capacitance = self.cap[sensor_id]
            control_msg.sensors.append(sensor)
        for rod in range(3):
            IMU = Imu()
            IMU.id = rod
            IMU.ax = self.accelerometer[rod][0]
            IMU.ay = self.accelerometer[rod][1]
            IMU.az = self.accelerometer[rod][2]
            IMU.gx = self.gyroscope[rod][0]
            IMU.gy = self.gyroscope[rod][1]
            IMU.gz = self.gyroscope[rod][2]
            control_msg.imus.append(IMU)
        self.control_pub.publish(control_msg)

    def read(self):
        data, addr = self.sock_receive.recvfrom(255)
        received_data = data.decode('utf-8')
        print(received_data)
        try:
            sensor_values = received_data.split()
            sensor_array = [float(v) for v in sensor_values]
            print(sensor_array)
            if addr not in self.addresses:
                self.addresses[int(sensor_array[0])] = addr

            if len(sensor_array) == 13:
                self.which_Arduino = int(sensor_array[0])
                if sensor_array[1] == 0.2 or sensor_array[2] == 0.2 or sensor_array[3] == 0.2:
                    print(f'MPR121 or I2C of Arduino {self.which_Arduino} wrongly initialized, please reboot Arduino')

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

                self.encoder_length = [
                    counts / self.encoder_resolution / self.gear_ratio * np.pi * self.winch_diameter
                    for counts in self.encoder_counts
                ]

                if 0.2 not in self.cap:
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
                        if self.addresses[i] is None:
                            print(f'Arduino {i} wrongly initialized, please reboot Arduino')
                        else:
                            self.send_command(self.stop_msg, self.addresses[i], 0)
                else:
                    print('+')
                    for i in range(len(self.addresses)):
                        self.send_command(self.stop_msg, self.addresses[i], 0)

        except Exception as e:
            print('There has been an error')
            print('Received data:', received_data)
            print(f"Error type: {type(e).__name__}")
            print(f"Error message: {e}")
            print("\nStopping motors")
            self.keep_going = False
            self.quitting = True
            for i in range(len(self.addresses)):
                self.send_command(self.stop_msg, self.addresses[i], 0)

    # ------------------------------------------------------------------
    # Control
    # ------------------------------------------------------------------

    def compute_command(self):
        command_msg = self.stop_msg.split()
        for i in range(self.num_motors):
            tolerance = self.low_tol if self.states[self.state, i] < 0.5 else self.tol
            if (self.pos[i] + tolerance > self.states[self.state, i] and
                    self.pos[i] - tolerance < self.states[self.state, i]):
                self.done[i] = True
                self.command[i] = 0
            if not self.done[i]:
                self.error[i] = self.pos[i] - self.states[self.state, i]
                self.d_error[i] = self.error[i] - self.prev_error[i]
                self.cum_error[i] += self.error[i]
                self.prev_error[i] = self.error[i]
                self.command[i] = max(min(
                    self.P * self.error[i] + self.I * self.cum_error[i] + self.D * self.d_error[i],
                    1), -1)
                self.speed[i] = self.command[i] * self.max_speed * self.flip[i]
                command_msg[i + self.offset] = str(self.speed[i])

        if all(self.done):
            next_state = self.state + 1

            # Check if we've just finished the last step of a cycle
            if next_state >= self.num_steps:
                self._repeat_count += 1
                repeat_str = 'inf' if self._repeat == float('inf') else str(int(self._repeat))
                print(f"Completed gait cycle {self._repeat_count} / {repeat_str}")

                if self._repeat != float('inf') and self._repeat_count >= self._repeat:
                    # All requested cycles done — stop cleanly
                    print("Gait complete. Stopping.")
                    self.keep_going = False
                    self.quitting = True
                    for i in range(len(self.addresses)):
                        self.send_command(self.stop_msg, self.addresses[i], 0)
                    return

                # Start next cycle from step 0
                next_state = 0

            self.state = next_state
            self.done = np.array([False] * self.num_motors)
            self.prev_error = [0] * self.num_motors
            self.cum_error = [0] * self.num_motors

        print(f"State: {self.state} | Cycle: {self._repeat_count+1}")
        print("Position: ", self.pos)
        print("Target:   ", self.states[self.state])
        print("Done:     ", self.done)
        print("Length:   ", self.length)
        print("Cap:      ", self.cap)
        print(' '.join(command_msg))
        self.send_command(' '.join(command_msg), self.addresses[self.which_Arduino], 0)
        print('+++++')

    # ------------------------------------------------------------------
    # Keyboard
    # ------------------------------------------------------------------

    def on_press(self, key):
        print('press')
        if key == keyboard.KeyCode.from_char('q'):
            print('I hear you Q — stopping.')
            self.keep_going = False
            self.quitting = True
            for i in range(len(self.addresses)):
                self.send_command(self.stop_msg, self.addresses[i], 0)
        elif key == keyboard.KeyCode.from_char('s'):
            print('I hear you S — stopping.')
            self.keep_going = False
            self.quitting = True
            for i in range(len(self.addresses)):
                self.send_command(self.stop_msg, self.addresses[i], 0)
        elif key == keyboard.KeyCode.from_char('r'):
            self.states = np.array([[1.0] * self.num_motors] * self.num_steps)
            self.done = np.array([False] * self.num_motors)
            self.tol = 0.2
            self.P = 5.0
            self.max_speed = 90
        elif key == keyboard.KeyCode.from_char('n'):
            self.states = np.array([[1.0] * self.num_motors] * self.num_steps)
            self.done = np.array([False] * self.num_motors)
            self.tol = 0.03
            self.P = 5.0
            self.RANGE = 90
            self.LEFT_RANGE = self.RANGE

    def on_release(self, key):
        print('release')
        digit_map = {
            '0': (0, 'zero_pressed'),
            '1': (1, 'one_pressed'),
            '2': (2, 'two_pressed'),
            '3': (3, 'three_pressed'),
            '4': (4, 'four_pressed'),
            '5': (5, 'five_pressed'),
        }
        for char, (_, attr) in digit_map.items():
            if key == keyboard.KeyCode.from_char(char):
                setattr(self, attr, False)
                for i in range(len(self.addresses)):
                    self.send_command(self.stop_msg, self.addresses[i], 0)
                return
        if key in (keyboard.KeyCode.from_char('f'), keyboard.KeyCode.from_char('b')):
            for i in range(len(self.addresses)):
                self.send_command(self.stop_msg, self.addresses[i], 0)

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def run(self):
        print("Initializing")
        self.initialize()
        print("Running UDP connection with Arduinos")

        self.sock_receive = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_receive.bind((self.UDP_IP, self.UDP_PORT))

        print("Connection open. Press 'q' to stop at any time.")
        while not self.quitting:
            try:
                self.read()
                if self.keep_going and None not in self.addresses:
                    self.sendRosMSG()
                    self.compute_command()
                if self.calibration:
                    self.sendRosMSG()
                    for i in range(self.num_sensors):
                        print(f"Cap {chr(i+97)}: {self.cap[i]:.2f}\tLength: {self.length[i]:.2f}")
            except Exception as e:
                print("\nStopping motors")
                self.keep_going = False
                self.quitting = True
                for i in range(len(self.addresses)):
                    self.send_command(self.stop_msg, self.addresses[i], 0)
                print(f"Error type: {type(e).__name__}")
                print(f"Error message: {e}")


# ----------------------------------------------------------------------
# Entry point
# ----------------------------------------------------------------------

def parse_args():
    parser = argparse.ArgumentParser(
        description="Run tensegrity robot with a JSON gait file.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument(
        'gait_file',
        help="Path to gait JSON file (must contain an 'actions' key)."
    )
    parser.add_argument(
        '-n', '--repeat',
        default='1',
        metavar='N',
        help="Number of times to repeat the gait. Use 'inf' to repeat indefinitely. Default: 1."
    )
    return parser.parse_args()


def main():
    args = parse_args()

    # Parse repeat value
    repeat_raw = args.repeat.strip().lower()
    if repeat_raw == 'inf':
        repeat = float('inf')
    else:
        try:
            repeat = int(repeat_raw)
            if repeat < 1:
                raise ValueError
        except ValueError:
            print(f"Error: --repeat must be a positive integer or 'inf', got '{args.repeat}'")
            exit(1)

    # Load gait
    try:
        gait_actions = load_gait_json(args.gait_file)
    except (FileNotFoundError, FileError, json.JSONDecodeError) as e:
        print(f"Error loading gait file: {e}")
        exit(1)

    print(f"Loaded '{args.gait_file}': {len(gait_actions)} steps")

    robot = TensegrityRobot(gait_actions=gait_actions, repeat=repeat)
    robot.run()


if __name__ == '__main__':
    main()
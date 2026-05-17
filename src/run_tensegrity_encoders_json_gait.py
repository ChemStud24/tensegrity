#!/usr/bin/env python3
"""
run_tensegrity_encoders_json_gait.py

Runs a tensegrity robot gait loaded from a JSON file using encoder-based
closed-loop position feedback. Includes flip direction and starting length
calibration routines.

Usage:
    rosrun tensegrity run_tensegrity_encoders_json_gait.py <robot_name> <gait.json>
    rosrun tensegrity run_tensegrity_encoders_json_gait.py diglett gaits/cw.json
    rosrun tensegrity run_tensegrity_encoders_json_gait.py diglett gaits/cw.json -n 5
    rosrun tensegrity run_tensegrity_encoders_json_gait.py diglett gaits/cw.json -n inf
    rosrun tensegrity run_tensegrity_encoders_json_gait.py diglett gaits/cw.json --calibrate-flip

run() flow:
    1. Parse args, load gait JSON, load calibration (m, b, flip) from <robot_name>.json
    2. Open UDP sockets, wait for all 3 Arduinos to connect
    3. If --calibrate-flip: run calibrate_flip() — f/b to move, c to confirm, y/n contraction prompt, saves flip to JSON
    4. Run calibrate_starting_length() — f/b to position each motor, c to confirm, records encoder_offset
    5. Prompt "Press Enter to start gait"
    6. Main loop: read() -> sendRosMSG() -> compute_command()
    7. Stop after -n cycles complete, or on q/s at any time

Gait JSON format:
    {
        "actions": [
            [0.1, 1.0, 1.0, 1.0, 1.0, 1.0],
            ...
        ]
    }
    Values: 1.0 = fully extended, 0.0 = fully contracted.

Keyboard controls during gait:
    q / s : stop immediately
    r     : override to all-extended, loose tolerance
    n     : override to all-extended, tight tolerance
"""

import os
import sys
import time
import math
import json
import argparse
import numpy as np
from math import cos, sin
from pynput import keyboard
from scipy.spatial.transform import Rotation as R
import rospy
import rospkg
import socket
from tensegrity.msg import Motor, Info, Sensor, Imu, TensegrityStamped


class FileError(Exception):
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
    def __init__(self, calibration_filename, gait_actions, repeat):
        self.num_sensors = 9
        self.num_motors = 6
        self.num_imus = 2
        self.num_arduino = 3
        self.min_length = 100
        self.pos = [0] * self.num_motors
        self.cap = [0] * self.num_sensors
        self.length = [0] * self.num_sensors
        self.error = [0] * self.num_motors
        self.prev_error = [0] * self.num_motors
        self.cum_error = [0] * self.num_motors
        self.d_error = [0] * self.num_motors
        self.command = [0] * self.num_motors
        self.speed = [0] * self.num_motors
        self.accelerometer = [[0]*3 for _ in range(3)]
        self.gyroscope = [[0]*3 for _ in range(3)]
        self.encoder_counts = [0] * self.num_motors
        self.encoder_offset = [0] * self.num_motors
        self.encoder_length = [0] * self.num_motors
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
        self.starting_length = 200

        # Gait
        self.states = gait_actions.copy()
        self.num_steps = len(self.states)
        self.state = 0
        self._gait_actions = gait_actions
        self._repeat = repeat
        self._repeat_count = 0

        self.control_pub = None
        self.my_listener = None
        self.keep_going = True
        self.quitting = False
        self.done = None
        self.m = None
        self.b = None
        self.flip = None
        self.stop_msg = None
        self.which_Arduino = None

        # UDP
        self.UDP_IP = "0.0.0.0"
        self.UDP_PORT = 2390
        self.sock_receive = None
        self.sock_send = None
        self.addresses = [None] * self.num_arduino
        self.offset = None

        # Keyboard / calibration state
        self.pressed_key = None
        self.awaiting_command = False
        self.motor2arduino = {0: 2, 1: 1, 2: 0, 3: 1, 4: 0, 5: 2}
        self.calibrating_length = False
        self.advance_motor = False
        self.yn_response = None
        self.waiting_yn = False

        package_path = rospkg.RosPack().get_path('tensegrity')
        self.calibration_file = os.path.join(package_path, 'calibration', calibration_filename)

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------

    def initialize(self):
        self.my_listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.my_listener.daemon = True
        self.my_listener.start()

        rospy.init_node('tensegrity')
        self.control_pub = rospy.Publisher('control_msg', TensegrityStamped, queue_size=10)

        self.m, self.b, self.flip = self.read_calibration_file(self.calibration_file)

        self.offset = 3
        self.done = np.array([False] * self.num_motors)
        self.stop_msg = ' '.join(['0'] * (self.num_motors + 2 * self.offset))

        repeat_str = 'inf' if self._repeat == float('inf') else str(int(self._repeat))
        print(f"Gait loaded: {self.num_steps} steps | repeat: {repeat_str}x")

    # ------------------------------------------------------------------
    # Calibration file
    # ------------------------------------------------------------------

    def read_calibration_file(self, filename):
        try:
            if filename.endswith('.json'):
                data = json.load(open(filename))
                m = np.array(data.get('m'))
                b = np.array(data.get('b'))
                flip = data.get('flip')
            else:
                raise FileError('Invalid calibration file — must be .json')
            return m, b, flip
        except FileError as e:
            print("Error:", e)
            sys.exit(1)

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
        try:
            sensor_values = received_data.split()
            sensor_array = [float(v) for v in sensor_values]
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
                    self.starting_length - flip * (counts - offset) / self.encoder_resolution / self.gear_ratio * np.pi * self.winch_diameter
                    for counts, flip, offset in zip(self.encoder_counts, self.flip, self.encoder_offset)
                ]

                if 0.2 not in self.cap:
                    for i in range(len(self.cap)):
                        self.length[i] = (self.cap[i] - self.b[i]) / self.m[i]
                    for i in range(self.num_motors):
                        if i < 3:
                            self.pos[i] = (self.encoder_length[i] - self.min_length) / self.LEFT_RANGE
                        else:
                            self.pos[i] = (self.encoder_length[i] - self.min_length) / self.RANGE

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
            if next_state >= self.num_steps:
                self._repeat_count += 1
                repeat_str = 'inf' if self._repeat == float('inf') else str(int(self._repeat))
                print(f"Completed gait cycle {self._repeat_count} / {repeat_str}")
                if self._repeat != float('inf') and self._repeat_count >= self._repeat:
                    print("Gait complete. Stopping.")
                    self.keep_going = False
                    self.quitting = True
                    for i in range(len(self.addresses)):
                        self.send_command(self.stop_msg, self.addresses[i], 0)
                    return
                next_state = 0
            self.state = next_state
            self.done = np.array([False] * self.num_motors)
            self.prev_error = [0] * self.num_motors
            self.cum_error = [0] * self.num_motors

        print(f"State: {self.state} | Cycle: {self._repeat_count + 1}")
        print("Position:", self.pos)
        print("Target:  ", self.states[self.state])
        print("Done:    ", self.done)
        print(' '.join(command_msg))
        self.send_command(' '.join(command_msg), self.addresses[self.which_Arduino], 0)
        print('+++++')

    # ------------------------------------------------------------------
    # Keyboard
    # ------------------------------------------------------------------

    def on_press(self, key):
        # q/s always quit regardless of mode
        if key == keyboard.KeyCode.from_char('q') or key == keyboard.KeyCode.from_char('s'):
            self.quitting = True
            self.keep_going = False
            for i in range(len(self.addresses)):
                if self.addresses[i] is not None:
                    self.send_command(self.stop_msg, self.addresses[i], 0)
        elif self.waiting_yn:
            if key == keyboard.KeyCode.from_char('y'):
                self.yn_response = 'y'
            elif key == keyboard.KeyCode.from_char('n'):
                self.yn_response = 'n'
        else:
            if key == keyboard.KeyCode.from_char('r'):
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
            elif key == keyboard.KeyCode.from_char('c'):
                if self.calibrating_length:
                    for i in range(len(self.addresses)):
                        if self.addresses[i] is not None:
                            self.send_command(self.stop_msg, self.addresses[i], 0)
                    self.advance_motor = True
            elif key == keyboard.KeyCode.from_char('f'):
                if self.pressed_key in range(self.num_motors) and self.awaiting_command:
                    command_msg = self.stop_msg.split()
                    command_msg[self.pressed_key + self.offset] = str(self.max_speed)
                    self.send_command(' '.join(command_msg), self.addresses[self.motor2arduino[self.pressed_key]], 0)
                    self.awaiting_command = False
            elif key == keyboard.KeyCode.from_char('b'):
                if self.pressed_key in range(self.num_motors) and self.awaiting_command:
                    command_msg = self.stop_msg.split()
                    command_msg[self.pressed_key + self.offset] = str(-self.max_speed)
                    self.send_command(' '.join(command_msg), self.addresses[self.motor2arduino[self.pressed_key]], 0)
                    self.awaiting_command = False

    def on_release(self, key):
        if key == keyboard.KeyCode.from_char('f') or key == keyboard.KeyCode.from_char('b'):
            for i in range(len(self.addresses)):
                if self.addresses[i] is not None:
                    self.send_command(self.stop_msg, self.addresses[i], 0)
            self.awaiting_command = True

    # ------------------------------------------------------------------
    # Calibration helpers
    # ------------------------------------------------------------------

    def _wait_yn(self):
        self.awaiting_command = False
        self.yn_response = None
        self.waiting_yn = True
        while self.yn_response not in ('y', 'n') and not self.quitting:
            time.sleep(0.05)
        self.waiting_yn = False
        return self.yn_response

    def calibrate_flip(self):
        print("\n=== Flip Direction Calibration ===")
        print("f/b: move motor  |  c: confirm and show counts  |  q: quit\n")
        original_flip = list(self.flip)
        self.calibrating_length = True
        for i in range(self.num_motors):
            if self.quitting:
                break
            self.pressed_key = i
            self.awaiting_command = True
            self.advance_motor = False
            self.read()
            start_counts = self.encoder_counts[i]
            print(f"Motor {i}: use f/b to move, press c when done.")
            while not self.advance_motor and not self.quitting:
                self.read()
            end_counts = self.encoder_counts[i]
            diff = end_counts - start_counts
            print(f"  counts: start={start_counts:.0f}  end={end_counts:.0f}  diff={diff:+.0f}")

            print("  Did tendon CONTRACT? (y/n)")
            contracted = self._wait_yn()
            if self.quitting:
                break

            if contracted == 'n':
                self.flip[i] *= -1
                print(f"  flip[{i}] -> {self.flip[i]}")
            else:
                print(f"  flip[{i}] = {self.flip[i]} (no change)")

            self.awaiting_command = False
            self.pressed_key = None

        self.calibrating_length = False

        changes = [i for i in range(self.num_motors) if self.flip[i] != original_flip[i]]
        if changes:
            print(f"\nRecommended flip for {self.calibration_file}:")
            print(f"  \"flip\": {self.flip}")
        with open(self.calibration_file, 'r') as f:
            data = json.load(f)
        data['flip'] = self.flip
        with open(self.calibration_file, 'w') as f:
            json.dump(data, f, indent='\t')
        print("Flip saved.\n")

    def calibrate_starting_length(self):
        print("\n=== Starting Length Calibration ===")
        print(f"Set each motor to starting length ({self.starting_length}mm) — may require f or b depending on current position.")
        print("f/b: move  |  c: confirm  |  q: quit\n")
        self.calibrating_length = True
        for i in range(self.num_motors):
            if self.quitting:
                break
            self.pressed_key = i
            self.awaiting_command = True
            self.advance_motor = False
            self.read()
            start_counts = self.encoder_counts[i]
            print(f"Motor {i}: f/b to position, c to confirm.")
            while not self.advance_motor and not self.quitting:
                self.read()
            end_counts = self.encoder_counts[i]
            diff = end_counts - start_counts
            print(f"  counts: start={start_counts:.0f}  end={end_counts:.0f}  diff={diff:+.0f}")

            print("  Did encoder counts INCREASE? (y/n)")
            self._wait_yn()

            self.awaiting_command = False
            self.pressed_key = None

        self.calibrating_length = False
        if not self.quitting:
            self.encoder_offset = list(self.encoder_counts)
            print("Starting length set.\n")

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def run(self, calibrate_flip):
        print("Initializing")
        self.initialize()
        print("Running UDP connection with Arduinos")

        self.sock_receive = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_receive.bind((self.UDP_IP, self.UDP_PORT))

        print("Waiting for all Arduinos to connect...")
        while None in self.addresses:
            self.read()

        if calibrate_flip:
            self.calibrate_flip()

        if not self.quitting:
            self.calibrate_starting_length()

        if not self.quitting:
            input("Press Enter to start the gait (q/s to stop at any time)...")
            print("Running.")

        while not self.quitting:
            try:
                self.read()
                if self.keep_going and None not in self.addresses:
                    self.sendRosMSG()
                    self.compute_command()
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
        description="Run tensegrity robot with a JSON gait file using encoder feedback.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument('robot_name', help="Robot name — loads calibration/<robot_name>.json")
    parser.add_argument('gait_file', help="Path to gait JSON file (must contain an 'actions' key)")
    parser.add_argument('-n', '--repeat', default='1', metavar='N',
                        help="Number of gait cycles. Use 'inf' to repeat indefinitely. Default: 1.")
    parser.add_argument('--calibrate-flip', action='store_true',
                        help="Run flip direction calibration before starting.")
    return parser.parse_args()


def main():
    args = parse_args()

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
            sys.exit(1)

    try:
        gait_actions = load_gait_json(args.gait_file)
    except (FileNotFoundError, FileError, json.JSONDecodeError) as e:
        print(f"Error loading gait file: {e}")
        sys.exit(1)

    print(f"Loaded '{args.gait_file}': {len(gait_actions)} steps")

    calibration_filename = args.robot_name + '.json'
    robot = TensegrityRobot(
        calibration_filename=calibration_filename,
        gait_actions=gait_actions,
        repeat=repeat,
    )
    robot.run(calibrate_flip=args.calibrate_flip)


if __name__ == '__main__':
    main()

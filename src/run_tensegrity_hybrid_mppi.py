#!/usr/bin/env python3
"""
Hybrid runner for the physical tensegrity robot hardware.

This version is configured for use with physical Arduino hardware.
For simulator use, see run_tensegrity_hybrid_mppi_simulator.py

This mirrors `run_tensegrity_Astar.py`'s UDP + PID control loop, but listens to
`tensegrity/ActionHybridMPPI` on `/action_mppi_msg` instead of `tensegrity/Action`.

- If `control_type == 'astar'`: consumes `primitive_actions` (string gait primitives)
  and executes them using the same gait library / symmetry transforms as the A* runner.
- If `control_type == 'mppi'`: consumes `mppi_actions` (T x 6 matrix of normalized controls)
  and directly streams motor speed commands for `control_interval` seconds per row.
"""

import os
import time
import math
from math import cos, sin
import json
import xlrd
import numpy as np
import signal
import sys
# from pynput import keyboard
from scipy.spatial.transform import Rotation as R

import rospy
import rosnode
import rospkg
import socket

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

# Try to import perception services, create mocks if not available
try:
    from tensegrity.srv import InitTracker, InitTrackerRequest, InitTrackerResponse
    from tensegrity.srv import GetPose, GetPoseRequest, GetPoseResponse
except ImportError:
    print("Warning: tensegrity services not available in run_tensegrity_hybrid_mppi, using mocks")
    from unittest.mock import MagicMock
    InitTracker = InitTrackerRequest = InitTrackerResponse = MagicMock
    GetPose = GetPoseRequest = GetPoseResponse = MagicMock

from tensegrity.msg import (
    Motor,
    Info,
    Sensor,
    Imu,
    TensegrityStamped,
    State,
    ActionHybridMPPI,
    Trajectory,
)

from symmetry_reduction_utils import *
from points_superimposed import obstacle_trajectory
from Tensegrity_model_inputs import *


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
        self.flip = [1, 1, 1, 1, 1, 1]  # flip direction of motors
        self.accelerometer = [[0] * 3 for _ in range(3)]
        self.gyroscope = [[0] * 3 for _ in range(3)]
        self.encoder_counts = [0] * self.num_motors
        self.encoder_length = [0] * self.num_motors
        self.RANGE024 = 100
        self.RANGE135 = 100
        self.max_cable_length = 0.23 # meters
        self.min_cable_length = 0.05 # meters
        self.max_speed = 70
        self.tol = 0.15
        self.low_tol = 0.15
        self.P = 10.0
        self.I = 0.01
        self.D = 0.5
        self.gear_ratio = 150
        self.winch_diameter = 6.35
        self.encoder_resolution = 12

        # planning and control
        self.prev_bottom_nodes = (0, 2, 5)
        self.prev_gait = "roll"
        self.reverse_the_gait = False
        self.action_sequence = [" _ ", " _ "]
        self.control_mode = "astar"  # 'astar' or 'mppi'

        # MPPI streaming state
        self.mppi_actions = None  # np.ndarray shape (T, 6)
        self.mppi_interval = None  # seconds per action row
        self.mppi_start_time = None

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

        # UDP variables (configured for hardware)
        self.UDP_IP = "0.0.0.0"  # Listen to all incoming interfaces
        self.UDP_PORT = 2390  # Same port used in the Arduino sketch
        self.sock_receive = None
        self.sock_send = None
        self.addresses = [None] * self.num_arduino
        self.offset = None  # Nb of leading end ending 0 preventing errors

        print("Initializing")
        self.initialize()
        print("Running UDP connection with Arduino's: ")

        # Create UDP sockets
        self.sock_receive = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_receive.bind((self.UDP_IP, self.UDP_PORT))

        print("Opened connection press s to stop motor and q to quit")

        # init tracker
        if "/tracking_service" in rosnode.get_node_names():
            self.trajectory = obstacle_trajectory
            self.init_tracker()

        # communicating with the planner
        self.action_sub = rospy.Subscriber("/action_mppi_msg", ActionHybridMPPI, self.hybrid_callback)
        self.state_pub = rospy.Publisher("/state_msg", State, queue_size=10)
        self.COMs = []
        self.PAs = []
        self.next_states = None
        self.prim_start_time = None
        self.max_prim_time = 2

    def initialize(self):
        # self.my_listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        # self.my_listener.start()

        rospy.init_node("tensegrity_hybrid_mppi")
        self.control_pub = rospy.Publisher("/control_msg", TensegrityStamped, queue_size=10)

        package_path = rospkg.RosPack().get_path("tensegrity")
        calibration_file = '../calibration/calibration_patrick.xls'
        self.m, self.b = self.read_calibration_file(calibration_file)

        # Default gait library (same as A* runner)
        self.states = np.array(
            [
                [1.0, 1.0, 0.1, 1.0, 1.0, 0.1],
                [0.0, 1.0, 1.0, 0.0, 0.8, 0.1],
                [1.0, 0.1, 1.0, 1.0, 0.1, 1.0],
                [1.0, 1.0, 0.0, 0.8, 0.1, 0.0],
                [0.1, 1.0, 1.0, 0.1, 1.0, 1.0],
                [1.0, 0.0, 1.0, 0.1, 0.0, 0.8],
            ]
        )
        self.num_steps = len(self.states)
        self.state = 0
        self.offset = 3
        self.done = np.array([False] * self.num_motors)
        self.stop_msg = " ".join(["0"] * (self.num_motors + 2 * self.offset))
        self.init_speed = 70

        # gaits
        '''
        roll = np.array(
            [
                [1, 1, 1, 1, 1, 1],
                [1., 1., 0.1, 1., 1., 0.1], 
                [0., 1., 1., 0., 1., 0.1],
                [1, 1, 1, 1, 1, 1],
            ]
        )
        cw = np.array(
            [
                [1, 1, 1, 1, 1, 1],
                [1., 1., 0., 0., 0., 0.], 
                [0., 1., 0., 0., 0., 0.], 
                [0., 1., 1., 0., 0.8, 0.],
                [1, 1, 1, 1, 1, 1],
            ]
        )
        ccw = np.array(
            [
                [1, 1, 1, 1, 1, 1],
                [1., 1., 1., 0., 1., 1.], 
                [1., 0., 1., 0., 1., 1.], 
                [0., 0., 0., 0., 0., 0.],
                [1, 1, 1, 1, 1, 1],
            ]
        )
        rest = np.array(
            [
                [1, 1, 1, 1, 1, 1],
                [1, 1, 1, 1, 1, 1],
                [1, 1, 1, 1, 1, 1],
                [1, 1, 1, 1, 1, 1],
            ]
        )
        '''
        roll = np.array([[1, 1, 1, 1, 1, 1], [1, 1, 0.1, 1, 1, 0.1], [0, 1, 1, 0, 1, 0.1], [1, 1, 1, 1, 1, 1]]) #new tensegrity
        #roll = np.array([[1, 1, 1, 1, 1, 1], [1, 1, 0.1, 1, 1, 0.1], [0, 1, 1, 0, 1, 0.1], [1, 1, 1, 1, 1, 1]]) #based off observed video
        #cw = np.array([[1, 1, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], [0, 1, 1, 0, 0.8, 0], [1, 1, 1, 1, 1, 1]]) #new tensegrity
        cw = np.array([[1, 1, 1, 1, 1, 1],[0, 0, 1, 0, 1, 0], [0, 0, 0, 0, 1, 0], [1, 0.8, 0, 0, 1, 0], [1, 1, 1, 1, 1, 1]]) #based off observed video
        #ccw = np.array([[1, 1, 1, 0, 1, 1], [1, 0, 1, 0, 1, 1], [0, 0, 0, 0, 0, 0], [1, 1, 1, 1, 1, 1]])#new tensegrity
        ccw = np.array([[1, 1, 1, 1, 1, 1],[1, 1, 0, 1, 1, 1], [1, 1, 0, 0, 1, 1], [0, 0, 0, 0, 0, 0], [1, 1, 1, 1, 1, 1]]) #based off observed video
        self.all_gaits = {"roll": roll, "ccw": ccw, "cw": cw, "rest": rest}
        # self.states = np.vstack([roll])
        # self.states = transform_gait(self.states, self.prev_bottom_nodes)

    def read_calibration_file(self, filename):
        try:
            if filename[-4:] == ".xls":
                workbook = xlrd.open_workbook(filename)
                shortsheet = workbook.sheet_by_name("Short Sensors")
                longsheet = workbook.sheet_by_name("Long Sensors")
                m = np.array(
                    [float(shortsheet.cell_value(9, col)) for col in range(0, 12, 2)]
                    + [float(longsheet.cell_value(10, col)) for col in range(0, 6, 2)]
                )
                b = np.array(
                    [float(shortsheet.cell_value(9, col)) for col in range(1, 13, 2)]
                    + [float(longsheet.cell_value(10, col)) for col in range(1, 7, 2)]
                )
            elif filename[-5:] == ".json":
                data = json.load(open(filename))
                m = np.array(data.get("m"))
                b = np.array(data.get("b"))
            else:
                raise FileError("Invalid calibration file")
            return m, b
        except FileError as ce:
            print("Error occurred:", ce)

    def quat2vec(self, q):
        q0 = float(q[0])
        q1 = float(q[1])
        q2 = float(q[2])
        q3 = float(q[3])
        roll = -math.atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2))
        sinp = 2 * (q0 * q2 - q3 * q1)
        if abs(sinp) >= 1:
            pitch = math.copysign(np.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        yaw = -math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)) + np.pi / 2

        k = np.array([cos(yaw) * cos(pitch), sin(pitch), sin(yaw) * cos(pitch)])
        r = R.from_rotvec(-np.pi / 2 * np.array([0, 1, 0]))
        k = r.apply(k)
        y = np.array([0, 1, 0])
        s = np.cross(k, y)
        v = np.cross(s, k)
        vrot = v * cos(roll) + np.cross(k, v) * sin(roll)
        return np.cross(k, vrot)

    def send_command(self, input_string, addr, delay_time):
        self.sock_send.sendto(input_string.encode("utf-8"), addr)
        if delay_time < 0:
            delay_time = 0
        time.sleep(delay_time / 1000)

    def send_ros_msg(self):
        control_msg = TensegrityStamped()
        timestamp = rospy.Time.now()
        control_msg.header.stamp = timestamp

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

        for motor_id in range(self.num_motors):
            motor = Motor()
            motor.id = motor_id
            motor.position = self.pos[motor_id]
            motor.target = float(self.states[self.state, motor_id]) if self.states is not None else 0.0
            motor.speed = float(self.speed[motor_id])
            motor.done = bool(self.done[motor_id]) if self.done is not None else False
            motor.encoder_counts = int(self.encoder_counts[motor_id])
            motor.encoder_counts = int(self.encoder_counts[motor_id])
            if(motor.id % 2 == 1):
                motor.encoder_length = 180 + self.encoder_length[motor_id]# NEW
            else:
                motor.encoder_length = 180 - self.encoder_length[motor_id]
            motor.encoder_length = float(self.encoder_length[motor_id])
            control_msg.motors.append(motor)

        for sensor_id in range(self.num_sensors):
            sensor = Sensor()
            sensor.id = sensor_id
            sensor.length = float(self.length[sensor_id])
            sensor.capacitance = float(self.cap[sensor_id])
            control_msg.sensors.append(sensor)

        for rod in range(3):
            IMU = Imu()
            IMU.ax = self.accelerometer[rod][0]
            IMU.ay = self.accelerometer[rod][1]
            IMU.az = self.accelerometer[rod][2]
            IMU.gx = self.gyroscope[rod][0]
            IMU.gy = self.gyroscope[rod][1]
            IMU.gz = self.gyroscope[rod][2]
            control_msg.imus.append(IMU)

        # Optional trajectory field (kept for compatibility)
        trajectory_msg = Trajectory()
        for x, y in self.COMs:
            p = Point()
            p.x = float(x)
            p.y = float(y)
            trajectory_msg.COMs.append(p)
        for x, y in self.PAs:
            p = Point()
            p.x = float(x)
            p.y = float(y)
            trajectory_msg.PAs.append(p)
        control_msg.trajectory = trajectory_msg

        self.control_pub.publish(control_msg)

    def read(self):
        arduino_info_in = [False] * self.num_arduino
        while not all(arduino_info_in):
            try:
                data, addr = self.sock_receive.recvfrom(255)
            except socket.timeout:
                # Timeout - no data received, continue
                return

            received_data = data.decode("utf-8")
            sensor_values = received_data.split()
            sensor_array = [float(value) for value in sensor_values]
            arduino_id = int(sensor_array[0])
            if self.addresses[arduino_id] is None:
                self.addresses[arduino_id] = addr

            if len(sensor_array) == 13:
                self.which_Arduino = int(sensor_array[0])
                arduino_info_in[self.which_Arduino] = True
                if sensor_array[1] == 0.2 or sensor_array[2] == 0.2 or sensor_array[3] == 0.2:
                    print("MPR121 or I2C of Arduino " + str(self.which_Arduino) + " wrongly initialized, please reboot Arduino")

            # if int(sensor_array[0]) == 0:
            #     self.cap[4] = sensor_array[1]
            #     self.cap[2] = sensor_array[2]
            #     self.cap[8] = sensor_array[3]
            #     self.encoder_counts[4] = sensor_array[6]
            #     self.encoder_counts[2] = sensor_array[5]
            # if int(sensor_array[0]) == 1:
            #     self.cap[3] = sensor_array[1]
            #     self.cap[1] = sensor_array[2]
            #     self.cap[7] = sensor_array[3]
            #     self.encoder_counts[3] = sensor_array[6]
            #     self.encoder_counts[1] = sensor_array[5]
            # if int(sensor_array[0]) == 2:
            #     self.cap[5] = sensor_array[1]
            #     self.cap[0] = sensor_array[2]
            #     self.cap[6] = sensor_array[3]
            #     self.encoder_counts[5] = sensor_array[6]
            #     self.encoder_counts[0] = sensor_array[5]
            if(int(sensor_array[0]) == 0) :
                self.cap[0] = sensor_array[1]
                self.cap[1] = sensor_array[2]
                self.cap[8] = sensor_array[4]
                self.encoder_counts[1] = sensor_array[5]
                self.encoder_counts[0] = sensor_array[6]
            if(int(sensor_array[0]) == 1) :
                self.cap[2] = sensor_array[1]
                self.cap[3] = sensor_array[2] 
                self.cap[7] = sensor_array[4]
                self.encoder_counts[3] = sensor_array[5]
                self.encoder_counts[2] = sensor_array[6]
            if(int(sensor_array[0]) == 2) :
                self.cap[4] = sensor_array[1]
                self.cap[5] = sensor_array[2] 
                self.cap[6] = sensor_array[4]
                self.encoder_counts[5] = sensor_array[5]
                self.encoder_counts[4] = sensor_array[6]

            self.encoder_length = [
                counts / self.encoder_resolution / self.gear_ratio * np.pi * self.winch_diameter
                for counts in self.encoder_counts
            ]

            if 0.2 not in self.cap:
                for i in range(len(self.cap)):
                    self.length[i] = (self.cap[i] - self.b[i]) / self.m[i]
                for i in range(self.num_motors):
                    if i < 3:
                        self.pos[i] = (self.length[i] - self.min_length) / self.RANGE135
                    else:
                        self.pos[i] = (self.length[i] - self.min_length) / self.RANGE024

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
                        print("Arduino " + str(i) + " wrongly initialized, please reboot Arduino")
                    else:
                        self.send_command(self.stop_msg, self.addresses[i], 0)
            else:
                for i in range(len(self.addresses)):
                    self.send_command(self.stop_msg, self.addresses[i], 0)

    def compute_command(self):
        # MPPI mode: stream motor speeds directly from the received plan.
        if self.control_mode == "mppi":
            if self.mppi_actions is not None and self.mppi_interval is not None:
                if self.mppi_start_time is None:
                    self.mppi_start_time = time.time()

                elapsed = time.time() - self.mppi_start_time
                idx = int(elapsed / max(self.mppi_interval, 1e-6))

                if idx >= self.mppi_actions.shape[0]:
                    # Plan finished -> stop and request next action.
                    self.mppi_actions = None
                    self.mppi_interval = None
                    self.mppi_start_time = None
                    self._publish_ready_for_next_action("mppi")
                    u = np.array([0.0] * self.num_motors)
                else:
                    u = np.array(self.mppi_actions[idx], dtype=float).flatten()
                    if u.shape[0] != self.num_motors:
                        u = u[: self.num_motors] if u.shape[0] > self.num_motors else np.pad(u, (0, self.num_motors - u.shape[0]))

                    cable_lengths = np.array(self.length[:self.num_motors]).reshape(u.shape)
                    upper_bound = 1.0 * (cable_lengths >= self.min_cable_length)
                    lower_bound = -1.0 * (cable_lengths <= self.max_cable_length)
                    u = np.clip(u, lower_bound, upper_bound)
            else:
                u = np.array([0.0] * self.num_motors)

            command_msg = self.stop_msg.split()
            for i in range(self.num_motors):
                self.speed[i] = float(u[i] * self.max_speed * self.flip[i])
                command_msg[i + self.offset] = str(self.speed[i])
        else:
            # A* / primitive mode: PID towards current gait targets (same as A* runner).
            if self.prim_start_time is None:
                self.prim_start_time = time.time()
            else:
                elapsed = time.time() - self.prim_start_time
                if elapsed > self.max_prim_time:
                    print("elapsed", elapsed, "max_prim_time", self.max_prim_time)
                    self.prim_start_time = None
                    self.done = [True] * self.num_motors
                        
            command_msg = self.stop_msg.split()
            for i in range(self.num_motors):
                tolerance = self.low_tol if self.states[self.state, i] < 0.5 else self.tol

                if self.pos[i] + tolerance > self.states[self.state, i] and self.pos[i] - tolerance < self.states[self.state, i]:
                    self.done[i] = True
                    self.command[i] = 0
                if not self.done[i]:
                    self.error[i] = self.pos[i] - self.states[self.state, i]
                    self.d_error[i] = self.error[i] - self.prev_error[i]
                    self.cum_error[i] = self.cum_error[i] + self.error[i]
                    self.prev_error[i] = self.error[i]
                    
                    self.command[i] = max([min([self.P * self.error[i] + self.I * self.cum_error[i] + self.D * self.d_error[i], 1]), -1])
                    self.speed[i] = self.command[i] * self.max_speed * self.flip[i]
                    command_msg[i + self.offset] = str(self.speed[i])
            # print([round(p, 4) for p in self.pos], [round(e, 4) for e in self.error], [round(s, 4) for s in self.states[self.state]])

            if all(self.done):
                self.state += 1
                self.state %= len(self.states)
                for i in range(self.num_motors):
                    self.done[i] = False
                    self.prev_error[i] = 0
                    self.cum_error[i] = 0

                if self.next_states is not None:
                    print("Next:", self.next_states)
                    self.states = self.next_states
                    self.next_states = None
                    self.state = 1
                elif "planning" in self.action_sequence[0]:
                    print("planning")
                    self.state = 1
                    self.states = np.array([[1, 1, 1, 1, 1, 1]] * 4)
                    self.RANGE024 = 100
                    self.RANGE135 = 100
                elif self.state % len(self.states) == 1:
                    prev_action = str(self.RANGE135) + "_" + str(self.RANGE024)
                    self._publish_ready_for_next_action(prev_action)

                    self.action_sequence = ["planning__planning" for _ in self.action_sequence]
                    self.next_states = None
                    self.state = 1
                    self.states = np.array([[1, 1, 1, 1, 1, 1]] * 4)
                    self.RANGE024 = 100
                    self.RANGE135 = 100

        if self.which_Arduino is not None and self.addresses[self.which_Arduino] is not None:
            self.send_command(" ".join(command_msg), self.addresses[self.which_Arduino], 0)

    def _publish_ready_for_next_action(self, prev_action_str):
        """Publish state message indicating readiness for next action."""
        state_msg = State()
        state_msg.prev_action = prev_action_str
        state_msg.reverse_the_gait = self.reverse_the_gait
        state_msg.bar_height_changed = False
        # Trajectory is empty for now - can be populated if needed
        state_msg.trajectory = []
        self.state_pub.publish(state_msg)
        rospy.loginfo(f"Published ready state with prev_action: {prev_action_str}")

    def get_pose_endcaps(self):
        """Fetch endcaps via perception service, returning (COM, principal_axis, endcaps)."""
        service_name = "get_pose"
        try:
            request = GetPoseRequest()
            get_pose_srv = rospy.ServiceProxy(service_name, GetPose)
            response: GetPoseResponse = get_pose_srv(request)
            if not response.success:
                return None

            centers = []
            endcaps = []
            vectors = np.array([[0.0, 0.0, 0.0]])
            for pose in response.poses:
                rotation_matrix = R.from_quat(
                    [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
                ).as_matrix()
                unit_vector = rotation_matrix[:, 2]
                center = [pose.position.x, pose.position.y, pose.position.z]
                endcaps.append(np.array(center) - L / 2 / 1000 * unit_vector)
                endcaps.append(np.array(center) + L / 2 / 1000 * unit_vector)
                centers.append(center)
                vectors += unit_vector

            COM = np.mean(np.array(centers), axis=0)
            principal_axis = vectors / np.linalg.norm(vectors)
            endcaps = np.array(endcaps)
            return COM, principal_axis, endcaps
        except rospy.ServiceException:
            return None

    def hybrid_callback(self, msg: ActionHybridMPPI):
        print("Planning results are in! control_type=", msg.control_type)

        if msg.control_type == "mppi":
            # Store and start streaming MPPI controls
            # mppi_actions comes as a flattened 1D array, reshape to (T, num_motors)
            actions_flat = np.array(msg.mppi_actions, dtype=float)
            if actions_flat.size == 0:
                rospy.logwarn("Received empty mppi_actions; ignoring.")
                return
            if actions_flat.size % self.num_motors != 0:
                rospy.logwarn(f"mppi_actions size {actions_flat.size} not divisible by {self.num_motors}; ignoring.")
                return
            # Reshape from flat to (T, num_motors)
            actions = actions_flat.reshape(-1, self.num_motors)
            self.mppi_actions = actions
            self.mppi_interval = float(msg.control_interval) if msg.control_interval > 0 else 0.5
            self.mppi_start_time = None
            self.control_mode = "mppi"
            return

        # Otherwise treat as primitive/A*.
        self.control_mode = "astar"
        self.action_sequence = list(msg.primitive_actions) if msg.primitive_actions else ["planning__planning"]

        # restart the step
        for i in range(self.num_motors):
            self.done[i] = False
            self.prev_error[i] = 0
            self.cum_error[i] = 0

        # Try to fetch endcaps so we can do the same symmetry-reduction transforms.
        pose_data = self.get_pose_endcaps()
        endcaps = None
        if pose_data is not None:
            _, _, endcaps = pose_data

        action = self.action_sequence[0]
        print("Primitive action sequence:", self.action_sequence)

        if action == "cw":
            self.next_states = self.all_gaits.get("cw")
            bottom_nodes = self.bottom3(endcaps) if endcaps is not None else None
            if bottom_nodes is None or bottom_nodes not in prev_nodes.keys():
                bottom_nodes = self.prev_bottom_nodes
            self.prev_bottom_nodes = prev_nodes.get(bottom_nodes)
            self.next_states = transform_gait(self.next_states, bottom_nodes)
            self.state = 1
            self.prev_gait = "cw"
            self.RANGE024 = 100
            self.RANGE135 = 100
        elif action == "ccw":
            self.next_states = self.all_gaits.get("ccw")
            bottom_nodes = self.bottom3(endcaps) if endcaps is not None else None
            if bottom_nodes is None or bottom_nodes not in prev_nodes.keys():
                bottom_nodes = self.prev_bottom_nodes
            self.next_states = transform_gait(self.next_states, bottom_nodes)
            self.state = 1
            self.prev_gait = "ccw"
            self.RANGE024 = 100
            self.RANGE135 = 100
        elif action == "rest":
            self.next_states = self.all_gaits.get("rest")
            self.state = 1
            self.prev_gait = "rest"
            self.RANGE024 = 100
            self.RANGE135 = 100
        else:
            if self.prev_gait not in ["cw", "ccw"]:
                for i in range(self.num_motors):
                    self.done[i] = True
            self.next_states = self.all_gaits.get("roll")
            bottom_nodes = self.bottom3(endcaps) if endcaps is not None else None
            if bottom_nodes is None or bottom_nodes not in prev_nodes.keys():
                bottom_nodes = self.prev_bottom_nodes
            self.prev_bottom_nodes = next_nodes.get(bottom_nodes)
            self.next_states = transform_gait(self.next_states, bottom_nodes)
            if self.reverse_the_gait:
                self.next_states = reverse_gait(self.next_states, bottom_nodes)
            self.state = 1
            self.prev_gait = "roll"
            try:
                ranges = action.split("_")
                self.RANGE024 = int(ranges[-1])
                self.RANGE135 = int(ranges[-2])
            except Exception:
                self.RANGE024 = 100
                self.RANGE135 = 100
            self.tol = 0.35 if (self.RANGE135 >= 130 or self.RANGE024 >= 130) else 0.15

        print("Selected primitive:", action)

    def bottom3(self, nodes):
        try:
            x_sr, y_sr, z_sr = self.nodes2sr(nodes)
            z_values = np.array([item[1] for item in sorted(z_sr.items())])
            bottom_nodes = tuple(sorted(np.argpartition(z_values, 3)[:3]))
            return bottom_nodes
        except Exception:
            return None

    def nodes2sr(self, nodes):
        x_sr = {str(key): nodes[key, 0] for key in range(number_of_rods * 2)}
        y_sr = {str(key): nodes[key, 1] for key in range(number_of_rods * 2)}
        z_sr = {str(key): nodes[key, 2] for key in range(number_of_rods * 2)}
        return x_sr, y_sr, z_sr

    def get_bottom_nodes(self):
        pose_data = self.get_pose_endcaps()
        endcaps = None
        if pose_data is not None:
            _, _, endcaps = pose_data
        bottom_nodes = self.bottom3(endcaps) if endcaps is not None else None
        return bottom_nodes

    def on_press(self, key):
        try:
            if key == keyboard.KeyCode.from_char("s"):
                self.quitting = True
                raise S_Q_Pressed()
            elif key == keyboard.KeyCode.from_char("r"):
                self.states = np.array([[1.0] * self.num_motors] * self.num_steps)
                self.done = np.array([False] * self.num_motors)
                self.tol = 0.2
                self.P = 5.0
                self.max_speed = 70
        except AttributeError:
            pass
        except S_Q_Pressed:
            print("\nStopping motors")
            self.keep_going = False
            for i in range(len(self.addresses)):
                if self.addresses[i] is not None:
                    self.send_command(self.stop_msg, self.addresses[i], 0)

    def on_release(self, key):
        # Keep same stop behavior
        if key in [
            keyboard.KeyCode.from_char(str(d)) for d in range(6)
        ] or key in [keyboard.KeyCode.from_char("f"), keyboard.KeyCode.from_char("b")]:
            for i in range(len(self.addresses)):
                if self.addresses[i] is not None:
                    self.send_command(self.stop_msg, self.addresses[i], 0)

    def init_tracker(self):
        print("i got to init_tracker")

        while None in self.addresses:
            self.read()
            print("getting cable lengths...")

        cable_length_msg = Float64MultiArray()
        cable_length_msg.data = self.length

        rgb_msg = rospy.wait_for_message("/rgb_images", Image, None)
        depth_msg = rospy.wait_for_message("/depth_images", Image, None)

        trajectory_x = Float64MultiArray()
        trajectory_y = Float64MultiArray()
        trajectory_x.data = self.trajectory[:, 0].tolist()
        trajectory_y.data = self.trajectory[:, 1].tolist()

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

    def run(self):
        self._publish_ready_for_next_action("start")
        try:
            while not self.quitting:
                self.read()
                if self.keep_going and None not in self.addresses:
                    self.send_ros_msg()
                    self.compute_command()
                if self.calibration:
                    self.send_ros_msg()
                    for i in range(self.num_sensors):
                        print(f"Capacitance {chr(i + 97)}: {self.cap[i]:.2f} \t Length: {self.length[i]:.2f} \n")
        except KeyboardInterrupt:
            print("\n\nShutting down controller...")
            self.quitting = True
            # Send stop command to all Arduinos
            for i in range(len(self.addresses)):
                if self.addresses[i] is not None:
                    try:
                        self.send_command(self.stop_msg, self.addresses[i], 0)
                    except:
                        pass
            # Close socket
            try:
                self.sock.close()
            except:
                pass
            print("Controller stopped")
        finally:
            # Ensure cleanup happens
            try:
                self.sock.close()
            except:
                pass


def signal_handler(sig, frame):
    """Handle Ctrl+C by forcefully exiting."""
    print('\n\nReceived interrupt signal, forcing shutdown...')
    try:
        rospy.signal_shutdown('Interrupted')
    except:
        pass
    sys.exit(0)


if __name__ == "__main__":
    # Register signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)

    tensegrity_robot = TensegrityRobot()
    tensegrity_robot.run()

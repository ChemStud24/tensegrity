#!/usr/bin/env python3
"""
UDP-based MuJoCo simulator with continuous real-time physics.

This simulator acts as a drop-in replacement for the physical tensegrity robot
by emulating the Arduino UDP communication protocol. The TensegrityRobot class
in run_tensegrity_hybrid_mppi.py remains unchanged and communicates with this
simulator via UDP, just as it would with real hardware.

Architecture:
    TensegrityRobot (unchanged) <--UDP--> TensegrityUDPSimulator <--> MuJoCo

Multi-threaded real-time design:
    - Physics Thread: Runs MuJoCo continuously at 1000 Hz, synced to wall clock
    - UDP Receive Thread: Asynchronously receives motor commands
    - Sensor Broadcast Thread: Periodically sends sensor data at 100 Hz

The simulator:
- Listens on UDP port 2390 for motor commands from TensegrityRobot
- Applies commands to MuJoCo simulation running continuously in real-time
- Sends back simulated sensor data (capacitance, encoders, IMU) in Arduino format
- Simulates 3 Arduino boards (IDs: 0, 1, 2)
- Optional realistic sensor noise (Gaussian noise, bias drift, counting errors)

Usage:
    python tensegrity_udp_simulator.py [xml_path] [--physics-rate 1000] [--sensor-noise] [--viewer]

    Then run the normal: python run_tensegrity_hybrid_mppi.py
"""

import socket
import time
import json
import threading
import numpy as np
from pathlib import Path
from collections import deque
import sys
import os
import cv2
import signal

# When --viewer is passed, use GLFW OpenGL backend instead of EGL so that
# the interactive viewer window and mujoco.Renderer can coexist.
# Must be set before importing mujoco (gl_context is resolved at import time).
if '--viewer' in sys.argv:
    os.environ['MUJOCO_GL'] = 'glfw'

import mujoco

# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from mujoco_simulator.tensegrity_mjc_simulation import ThreeBarTensegrityMuJoCoSimulator
from mujoco_simulator.mujoco_simulation import RemoteViewer, PassiveViewer

# ROS imports for camera publishing
try:
    import rospy
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("Warning: ROS not available. Camera publishing will be disabled.")


class SharedSimulatorState:
    """Thread-safe shared state between simulation threads."""

    def __init__(self, num_motors=6, num_sensors=9, num_rods=3):
        self.num_rods = num_rods
        self.lock = threading.Lock()

        # Motor commands (updated by UDP thread, read by physics thread)
        self.motor_speeds = [0.0] * num_motors  # -max_speed to max_speed

        # Sensor data (updated by physics thread, read by sensor thread)
        self.sensor_data = {
            'capacitance': [0.0] * num_sensors,
            'length': [100.0] * num_sensors,
            'encoder_counts': [0] * num_motors,
            'accelerometer': [[0.0] * 3 for _ in range(num_rods)],
            'gyroscope': [[0.0] * 3 for _ in range(num_rods)],
        }

        # Overlay data for remote viewer HUD (updated by physics thread)
        self.overlay_data = {
            'com': [0.0, 0.0],
            'heading': 0.0,
            'cable_lengths': [0.0] * num_sensors,
            'controls': [0.0] * num_motors,
        }

        # Network state
        self.client_address = None

        # Control flags
        self.running = True
        self.last_command_time = 0.0

        # Timing statistics
        self.sim_time = 0.0  # Simulation time in seconds
        self.sim_steps = 0
        self.timing_errors = deque(maxlen=1000)  # Track timing drift


class TensegrityUDPSimulator:
    """
    MuJoCo simulator that emulates Arduino UDP communication protocol.

    This class replaces the physical hardware by:
    1. Running MuJoCo physics continuously in real-time (separate thread)
    2. Receiving motor commands via UDP asynchronously (separate thread)
    3. Sending sensor data periodically (separate thread)
    """

    def __init__(self,
                 xml_path: str,
                 start_se2: np.ndarray = None,
                 visualize: bool = True,
                 attach_type: str = 'real_attach',
                 udp_port: int = 2390,
                 pose_query_port: int = 2391,
                 physics_rate: float = 100.0,
                 sensor_rate: float = 20.0,
                 timing_stats: bool = False,
                 sensor_noise: bool = False,
                 noise_scale: float = 1.0,
                 camera_rate: float = 30.0,
                 camera_size: tuple = (640, 480),
                 enable_ros: bool = True,
                 use_viewer: bool = False,
                 viewer_fps: float = 30.0,
                 use_remote_viewer: bool = False,
                 remote_port: int = 8765):
        """
        Initialize UDP simulator with multi-threaded real-time physics.

        Args:
            xml_path: Path to MuJoCo XML model file
            start_se2: Initial SE(2) pose of the robot
            visualize: Whether to render visualization
            attach_type: Cable attachment type
            udp_port: UDP port for motor commands (must match TensegrityRobot.UDP_PORT)
            pose_query_port: UDP port for pose queries from mock tracker (default: 2391)
            physics_rate: Physics simulation rate in Hz (default: 1000)
            sensor_rate: Sensor broadcast rate in Hz (default: 100)
            timing_stats: Whether to print timing diagnostics
            sensor_noise: Whether to add realistic sensor noise (default: False)
            noise_scale: Scale factor for noise magnitude (default: 1.0)
            camera_rate: Camera image publishing rate in Hz (default: 30.0)
            camera_size: Camera image size as (width, height) tuple (default: 640x480)
            enable_ros: Whether to enable ROS integration for camera publishing (default: True)
            use_viewer: Whether to open an interactive MuJoCo viewer window (default: False)
            viewer_fps: Viewer rendering frame rate in Hz (default: 30.0)
            use_remote_viewer: Whether to enable remote WebSocket viewer (default: False)
            remote_port: Port for remote viewer server (default: 8765)
        """
        # Configuration
        self.physics_rate = physics_rate
        self.sensor_rate = sensor_rate
        self.timing_stats = timing_stats
        self.visualize = visualize
        self.sensor_noise = sensor_noise
        self.noise_scale = noise_scale
        self.camera_rate = camera_rate
        self.camera_width = camera_size[0]
        self.camera_height = camera_size[1]
        self.enable_ros = enable_ros
        self.use_viewer = use_viewer
        self.viewer_fps = viewer_fps
        self.viewer = None
        self.use_remote_viewer = use_remote_viewer
        self.remote_port = remote_port
        self.remote_viewer = None

        # UDP communication
        self.UDP_IP = "0.0.0.0"
        self.UDP_PORT = udp_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.UDP_IP, self.UDP_PORT))
        self.sock.settimeout(0.01)  # Short timeout for non-blocking

        # Separate UDP socket for pose queries (so it doesn't interfere with motor commands)
        self.POSE_QUERY_PORT = pose_query_port
        self.pose_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.pose_sock.bind((self.UDP_IP, self.POSE_QUERY_PORT))
        self.pose_sock.settimeout(0.01)

        # Robot configuration
        self.num_motors = 6
        self.num_sensors = 9
        self.min_length = 100  # mm

        # Calibration parameters (for simulating capacitance sensors)
        self.load_calibration()

        # Initialize MuJoCo simulator
        xml_path = Path(xml_path)
        if not xml_path.exists():
            raise FileNotFoundError(f"XML model not found: {xml_path}")

        print(f"Initializing MuJoCo simulator with: {xml_path}")
        self.sim = ThreeBarTensegrityMuJoCoSimulator(
            xml_path,
            visualize=visualize,
            attach_type=attach_type
        )
        self.sim.mjc_model.opt.timestep = 1.0 / physics_rate
        self.sim.dt = 1.0 / physics_rate
        self.num_rods = self.sim.num_rods
        
        if start_se2 is not None:
            new_prin = np.array([[np.cos(start_se2[2]), np.sin(start_se2[2]), 0.0]])
            new_com = np.array([[start_se2[0] * 10, start_se2[1] * 10]])
            self.sim.align_prin(new_prin, new_com)
            print("Start SE2: ", self.sim.get_unscaled_se2())

        # p = '/home/nelsonchen/research/tensegrity/data_sets/tensegrity_real_datasets/mjc_6d_new_platform_v6/dataset_8/roll_0/'
        # data = json.load(Path(p, 'processed_data.json').open('r'))
        # pos = np.array(data[0]['pos'], dtype=np.float64).reshape(-1, 3)
        # quat = np.array(data[0]['quat'], dtype=np.float64).reshape(-1, 4)
        # qpos = np.hstack([pos, quat]).flatten()
        # self.sim.mjc_data.qpos = qpos
        # self.sim.mjc_data.qvel = np.zeros_like(self.sim.mjc_data.qvel)

        # extra = json.load(Path(p, 'extra_state_data.json').open('r'))
        # rest_lengths = np.array(extra[0]['rest_lengths'], dtype=np.float64)
        # self.sim.mjc_model.tendon_lengthspring[:6, 0] = rest_lengths
        # self.sim.mjc_model.tendon_lengthspring[:6, 1] = rest_lengths
        # self.controls = np.array([d['controls'] for d in extra]).repeat(10, axis=0)

        # qpos = self.sim.mjc_data.qpos.copy().reshape(-1, 7)
        # qpos[:, 2] += 12
        # self.sim.mjc_data.qpos = qpos.flatten()
        self.sim.mjc_data.qvel[:] = 0

        # Encoder simulation
        self.gear_ratio = 150
        self.winch_diameter = 6.35  # mm
        self.encoder_resolution = 12
        self.prev_rest_lengths = None

        # Sensor noise parameters (standard deviations scaled by noise_scale)
        self.noise_params = {
            'capacitance_std': 0.5 * noise_scale * 0,  # Capacitance noise (units)
            'capacitance_drift_rate': 0.02 * noise_scale * 0,  # Slow drift rate
            'encoder_error_prob': 0.001 * noise_scale * 0,  # Probability of count error
            'accel_std': 0.05 * noise_scale * 0,  # Accelerometer noise (m/s^2)
            'accel_bias_std': 0.01 * noise_scale * 0,  # Accelerometer bias drift
            'gyro_std': 0.01 * noise_scale * 0,  # Gyroscope noise (rad/s)
            'gyro_bias_std': 0.005 * noise_scale * 0,  # Gyroscope bias drift
        }

        # Persistent noise states (for drift/bias)
        self.capacitance_bias = np.zeros(self.num_sensors)
        self.accel_bias = np.zeros((self.num_rods, 3))  # [3 rods, 3 axes]
        self.gyro_bias = np.zeros((self.num_rods, 3))   # [3 rods, 3 axes]

        # Shared state for thread coordination
        self.shared_state = SharedSimulatorState(self.num_motors, self.num_sensors, self.num_rods)

        # Camera setup (if ROS is available and enabled)
        if ROS_AVAILABLE and self.enable_ros:
            # Initialize ROS node if not already initialized
            try:
                rospy.init_node('tensegrity_udp_simulator', anonymous=True)
            except rospy.exceptions.ROSException:
                # Node already initialized
                pass

            self.bridge = CvBridge()
            self.color_im_pub = rospy.Publisher('rgb_images', Image, queue_size=6)
            self.depth_im_pub = rospy.Publisher('depth_images', Image, queue_size=6)
        else:
            self.bridge = None
            self.color_im_pub = None
            self.depth_im_pub = None

        # Threads
        self.physics_thread = None
        self.udp_thread = None
        self.sensor_thread = None
        self.pose_query_thread = None
        self.camera_thread = None

        # Timing diagnostics
        self.start_wall_time = None
        self.last_stats_print = 0.0

        print(f"UDP Simulator initialized:")
        print(f"  - Physics rate: {self.physics_rate} Hz ({1000.0/self.physics_rate:.3f}ms timestep)")
        print(f"  - Sensor rate: {self.sensor_rate} Hz")
        print(f"  - Visualization: {'enabled' if self.visualize else 'disabled'}")
        print(f"  - Sensor noise: {'enabled' if self.sensor_noise else 'disabled'}" +
              (f" (scale={self.noise_scale})" if self.sensor_noise else ""))
        print(f"  - Mock camera: {'enabled' if (ROS_AVAILABLE and self.enable_ros) else 'disabled'}" +
              (f" ({self.camera_rate}Hz, {self.camera_width}x{self.camera_height})" if (ROS_AVAILABLE and self.enable_ros) else ""))
        print(f"  - Interactive viewer: {'enabled' if self.use_viewer else 'disabled'}")
        print(f"  - Remote viewer: {'enabled (port ' + str(self.remote_port) + ')' if self.use_remote_viewer else 'disabled'}")
        print(f"  - Motor commands port: {self.UDP_IP}:{self.UDP_PORT}")
        print(f"  - Pose query port: {self.UDP_IP}:{self.POSE_QUERY_PORT}")
        print("  - Waiting for TensegrityRobot to connect...")

    def get_overlay_data(self):
        """Get current robot state for HUD overlay display.

        Returns cached overlay data from shared state (updated by physics thread).
        This avoids calling MuJoCo from the main thread which would cause race conditions.
        """
        with self.shared_state.lock:
            return self.shared_state.overlay_data.copy()

    def load_calibration(self):
        """Load calibration for capacitance sensor simulation."""
        try:
            import rospkg
            package_path = rospkg.RosPack().get_path("tensegrity")
            calibration_file = os.path.join(package_path, "calibration/new_calibration.json")
            if os.path.exists(calibration_file):
                data = json.load(open(calibration_file))
                self.m = np.array(data.get("m"))
                self.b = np.array(data.get("b"))
                print(f"  - Loaded calibration from: {calibration_file}")
            else:
                self.set_default_calibration()
        except Exception as e:
            print(f"  - Could not load calibration: {e}")
            self.set_default_calibration()

    def set_default_calibration(self):
        """Set default calibration values."""
        # Approximate linear relationship: capacitance = m * length + b
        self.m = np.array([1.0] * self.num_sensors)
        self.b = np.array([50.0] * self.num_sensors)
        print("  - Using default calibration")

    def update_sensor_data(self):
        """
        Update simulated sensor readings from MuJoCo state.
        Called by physics thread, writes to shared state.
        """
        # Get cable rest lengths from simulation (in meters)
        rest_lengths = self.sim.get_unscaled_rest_lengths()  # simulation scaled 10x
        cable_lengths = self.sim.get_unscaled_cable_lengths().tolist()
        sensor_data = {}

        # Capacitance and length sensors
        cap = [0.0] * self.num_sensors
        length = [0.0] * self.num_sensors

        for i in range(len(cable_lengths)):
            cable_length_mm = cable_lengths[i] * 1000.0
            length[i] = cable_length_mm
            cap[i] = self.m[i] * cable_length_mm + self.b[i]

        # Add noise to capacitance sensors if enabled
        if self.sensor_noise:
            # Gaussian white noise
            cap_noise = np.random.normal(0, self.noise_params['capacitance_std'], self.num_sensors)

            # Slowly drifting bias (random walk)
            drift = np.random.normal(0, self.noise_params['capacitance_drift_rate'], self.num_sensors)
            self.capacitance_bias += drift

            # Apply noise
            cap = [cap[i] + cap_noise[i] + self.capacitance_bias[i] for i in range(self.num_sensors)]

        sensor_data['capacitance'] = cap
        sensor_data['length'] = length

        # Update encoder counts (based on cable length changes)
        if self.prev_rest_lengths is None:
            self.prev_rest_lengths = rest_lengths[:self.num_motors].copy()
            encoder_counts = [0] * self.num_motors
        else:
            # Get previous counts from shared state
            with self.shared_state.lock:
                encoder_counts = self.shared_state.sensor_data['encoder_counts'].copy()

            for i in range(self.num_motors):
                # Calculate length change in mm
                delta_length_m = rest_lengths[i] - self.prev_rest_lengths[i]
                delta_length_mm = delta_length_m * 1000.0

                # Convert to encoder counts
                delta_counts = (delta_length_mm / (np.pi * self.winch_diameter) *
                              self.gear_ratio * self.encoder_resolution)

                encoder_counts[i] += int(delta_counts)

            self.prev_rest_lengths = rest_lengths[:self.num_motors].copy()

        # Add noise to encoder counts if enabled
        if self.sensor_noise:
            # Random counting errors (occasional missed/extra counts)
            for i in range(self.num_motors):
                if np.random.random() < self.noise_params['encoder_error_prob']:
                    encoder_counts[i] += np.random.choice([-1, 1])

        sensor_data['encoder_counts'] = encoder_counts

        # Simulate IMU data from rigid body state
        qvel = self.sim.get_unscaled_vels()  # [num_rods, 6]
        qacc = self.sim.get_unscaled_accels()  # [num_rods, 6]

        accelerometer = []
        gyroscope = []
        for rod in range(self.num_rods):
            # Accelerometer: use linear acceleration (simplified)
            accel = [
                qacc[rod, 0],  # scaled for realism
                qacc[rod, 1],
                qacc[rod, 2]  
            ]

            # Add accelerometer noise if enabled
            if self.sensor_noise:
                # Gaussian white noise
                accel_noise = np.random.normal(0, self.noise_params['accel_std'], 3)

                # Slowly drifting bias (random walk)
                bias_drift = np.random.normal(0, self.noise_params['accel_bias_std'], 3)
                self.accel_bias[rod] += bias_drift

                # Apply noise and bias
                accel = [accel[i] + accel_noise[i] + self.accel_bias[rod, i] for i in range(3)]

            accelerometer.append(accel)

            # Gyroscope: angular velocity in rad/s
            gyro = qvel[rod, 3:].tolist()

            # Add gyroscope noise if enabled
            if self.sensor_noise:
                # Gaussian white noise
                gyro_noise = np.random.normal(0, self.noise_params['gyro_std'], 3)

                # Slowly drifting bias (random walk)
                bias_drift = np.random.normal(0, self.noise_params['gyro_bias_std'], 3)
                self.gyro_bias[rod] += bias_drift

                # Apply noise and bias
                gyro = [gyro[i] + gyro_noise[i] + self.gyro_bias[rod, i] for i in range(3)]

            gyroscope.append(gyro)

        # Pad to 3 rods
        while len(accelerometer) < self.num_rods:
            accelerometer.append([0.0, 0.0, 9.81])
            gyroscope.append([0.0, 0.0, 0.0])

        sensor_data['accelerometer'] = accelerometer
        sensor_data['gyroscope'] = gyroscope

        # Write to shared state
        with self.shared_state.lock:
            self.shared_state.sensor_data = sensor_data

    def update_overlay_data(self, controls):
        """
        Update overlay data for remote viewer HUD.
        Called by physics thread after each simulation step.

        Args:
            controls: Current control values (already normalized to [-1, 1])
        """
        try:
            # Get SE2 pose (center of mass and heading)
            com, heading = self.sim.get_unscaled_se2()
            com = com.flatten()[:2].tolist()
            heading = float(heading.flatten()[0])

            # Get cable lengths in mm
            cable_lengths = (self.sim.get_unscaled_cable_lengths() * 1000.0).tolist()

            # Get rest lengths in mm (first 6 cables)
            rest_lengths = (self.sim.get_unscaled_rest_lengths()[:6] * 1000.0).tolist()

            # Write to shared state
            with self.shared_state.lock:
                self.shared_state.overlay_data = {
                    'sim_time': self.shared_state.sim_time,
                    'com': com,
                    'heading': heading,
                    'cable_lengths': cable_lengths,
                    'rest_lengths': rest_lengths,
                    'controls': controls.flatten().tolist(),
                }
        except Exception:
            pass  # Silently ignore errors

    def get_robot_pose(self):
        """
        Get current robot pose from MuJoCo simulation.

        Returns:
            dict: Robot pose data in format:
                {
                    'status': 'ok' or 'error',
                    'rods': [
                        {
                            'position': [x, y, z],  # rod center position in meters
                            'orientation': [qx, qy, qz, qw]  # quaternion
                        },
                        ...
                    ]
                }
        """
        try:
            # Get rigid body positions and orientations from MuJoCo
            # Assuming the first 3 bodies are the rods
            num_rods = self.sim.num_rods
            pose = self.sim.get_unscaled_pose()

            rods = []
            for i in range(num_rods):
                # MuJoCo stores pose as [pos(3), quat(4)] for each body
                pos = pose[i, :3].tolist()
                quat = pose[i, 3:].tolist()  # [w, x, y, z] in MuJoCo

                # Convert MuJoCo quaternion [w, x, y, z] to standard [x, y, z, w]
                quat_standard = [quat[1], quat[2], quat[3], quat[0]]

                rods.append({
                    'position': pos,
                    'orientation': quat_standard
                })

            return {
                'status': 'ok',
                'rods': rods
            }

        except Exception as e:
            return {
                'status': 'error',
                'error': str(e)
            }

    def send_sensor_data(self, arduino_id: int):
        """
        Send sensor data packet for one Arduino.

        Arduino packet format (13 values):
        [arduino_id, cap1, cap2, cap3, 0, encoder1, encoder2, ax, ay, az, gx, gy, gz]

        Args:
            arduino_id: Arduino ID (0, 1, or 2)
        """
        # Read from shared state
        with self.shared_state.lock:
            if self.shared_state.client_address is None:
                return
            client_addr = self.shared_state.client_address
            cap = self.shared_state.sensor_data['capacitance']
            enc = self.shared_state.sensor_data['encoder_counts']
            accel = self.shared_state.sensor_data['accelerometer']
            gyro = self.shared_state.sensor_data['gyroscope']

        # Map Arduino ID to sensors (from TensegrityRobot.read())
        if arduino_id == 0:
            cap_vals = [cap[4], cap[2], cap[8]]
            enc_vals = [enc[0], enc[1]]
        elif arduino_id == 1:
            cap_vals = [cap[3], cap[1], cap[7]]
            enc_vals = [enc[2], enc[3]]
        elif arduino_id == 2:
            cap_vals = [cap[5], cap[0], cap[6]]
            enc_vals = [enc[4], enc[5]]
        else:
            return

        # Build packet
        packet = [
            float(arduino_id),
            cap_vals[0], cap_vals[1], cap_vals[2],
            0.0,  # unused field
            enc_vals[0], enc_vals[1],
            accel[arduino_id][0], accel[arduino_id][1], accel[arduino_id][2],
            gyro[arduino_id][0], gyro[arduino_id][1], gyro[arduino_id][2]
        ]

        # Format as space-separated string
        message = " ".join([f"{v:.6f}" for v in packet])

        try:
            self.sock.sendto(message.encode('utf-8'), client_addr)
        except Exception as e:
            if "Broken pipe" not in str(e):
                print(f"Error sending to {client_addr}: {e}")

    def _physics_thread_func(self):
        """
        Physics thread: Run MuJoCo simulation continuously at real-time rate.

        This thread:
        - Steps physics at target rate (default 1000 Hz)
        - Synchronizes with wall clock to maintain real-time
        - Reads motor commands from shared state
        - Updates sensor data in shared state
        - Handles visualization (if enabled)
        """
        target_dt = 1.0 / self.physics_rate
        next_step_time = time.time()

        # Timing stats
        step_count = 0
        late_count = 0
        max_lateness = 0.0

        print(f"[Physics Thread] Started (target dt={target_dt*1000:.3f}ms)")

        while self.shared_state.running:
            current_time = time.time()

            if current_time >= next_step_time:
                # Get current motor commands
                with self.shared_state.lock:
                    motor_speeds = self.shared_state.motor_speeds.copy()
                # Convert to controls
                max_speeds = self.sim.get_motor_max_speeds()
                controls = motor_speeds / max_speeds
                controls = np.clip(controls, -1.0, 1.0)

                # # Account for motor direction flips
                flip = [1, -1, 1, 1, -1, -1]
                controls = controls * np.array(flip)

                # controls = self.controls[step_count: step_count + 1]

                # cable_lengths = self.sim.get_unscaled_cable_lengths().tolist()
                # print("cable_lengths", [round(c, 4) for c in cable_lengths])
                # print("controls", [round(c, 4) for c in controls])

                # Apply to simulation
                self.sim.sim_step(controls.reshape(1, -1))
                # print(self.shared_state.sim_time, self.sim.get_vels().reshape(-1, 6)[:, 2])

                # Update sensor data
                self.update_sensor_data()

                # Update overlay data for remote viewer HUD
                self.update_overlay_data(controls)

                # Sync viewers
                if self.viewer is not None and self.viewer.is_running():
                    self.viewer.sync()
                if self.remote_viewer is not None and self.remote_viewer.is_running():
                    self.remote_viewer.sync()

                # Update timing stats
                with self.shared_state.lock:
                    self.shared_state.sim_time += target_dt
                    self.shared_state.sim_steps += 1

                step_count += 1
                # if step_count % 100 == 0:
                #     com, heading = self.sim.get_se2()
                #     print(com, np.rad2deg(heading), controls)

                # Schedule next step to maintain real-time
                next_step_time += target_dt

                # Check if we're falling behind
                lateness = current_time - next_step_time
                if lateness > 0:
                    late_count += 1
                    max_lateness = max(max_lateness, lateness)
                    self.shared_state.timing_errors.append(lateness)

                    # If very behind, skip to current time
                    if lateness > target_dt * 100:
                        print(f"[Physics] Warning: {lateness*1000:.1f}ms behind schedule, resetting")
                        next_step_time = current_time

            # Timing control: hybrid sleep/busy-wait
            remaining = next_step_time - time.time()
            if remaining > 0.001:  # If >1ms remaining
                time.sleep(remaining * 0.5)  # Sleep for half the time
            # Busy-wait for the rest for precision

            # Print stats periodically
            if self.timing_stats and step_count % (self.physics_rate * 10) == 0:  # Every 10 seconds
                wall_time = time.time() - self.start_wall_time
                with self.shared_state.lock:
                    sim_time = self.shared_state.sim_time
                drift = abs(sim_time - wall_time)
                late_pct = 100.0 * late_count / step_count if step_count > 0 else 0

                print(f"[Physics] Steps: {step_count}, "
                      f"SimTime: {sim_time:.2f}s, WallTime: {wall_time:.2f}s, "
                      f"Drift: {drift*1000:.1f}ms, Late: {late_pct:.1f}%, "
                      f"MaxLate: {max_lateness*1000:.1f}ms")

        print("[Physics Thread] Stopped")

    def _udp_receive_thread_func(self):
        """
        UDP receive thread: Asynchronously receive motor commands.

        This thread:
        - Listens for UDP packets continuously
        - Parses motor speed commands
        - Updates shared state with new commands
        - Tracks client address for responses
        """
        print("[UDP Receive Thread] Started")

        while self.shared_state.running:
            try:
                data, addr = self.sock.recvfrom(1024)

                # Store client address for sending responses
                with self.shared_state.lock:
                    if self.shared_state.client_address is None:
                        self.shared_state.client_address = addr
                        print(f"[UDP] Connected to TensegrityRobot at {addr}")

                # Parse command
                message = data.decode('utf-8')
                values = message.strip().split()

                if len(values) >= 9:  # offset=3, so expect at least 3+6 values
                    # Extract motor speeds (skip offset padding)
                    offset = 3  # matches TensegrityRobot.offset
                    motor_speeds = []
                    for i in range(self.num_motors):
                        try:
                            speed = float(values[offset + i])
                            motor_speeds.append(speed)
                        except (IndexError, ValueError):
                            print(f"[UDP] Error parsing motor speed: {values}")
                            motor_speeds.append(0.0)

                    # Update motor speeds in shared state
                    with self.shared_state.lock:
                        self.shared_state.motor_speeds = motor_speeds
                        self.shared_state.last_command_time = time.time()

            except socket.timeout:
                # No data received, continue
                pass
            except Exception as e:
                if self.shared_state.running and "timed out" not in str(e):
                    print(f"[UDP] Error receiving command: {e}")

        print("[UDP Receive Thread] Stopped")

    def _sensor_broadcast_thread_func(self):
        """
        Sensor broadcast thread: Periodically send sensor data.

        This thread:
        - Sends sensor data at target rate (default 100 Hz)
        - Reads from shared state
        - Broadcasts for all 3 simulated Arduinos
        - Mimics real Arduino sensor update rate
        """
        target_dt = 1.0 / self.sensor_rate
        print(f"[Sensor Broadcast Thread] Started (rate={self.sensor_rate}Hz)")

        while self.shared_state.running:
            start = time.time()

            # Send data for all 3 Arduinos
            for arduino_id in range(3):
                self.send_sensor_data(arduino_id)

            # Maintain rate
            elapsed = time.time() - start
            sleep_time = target_dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        print("[Sensor Broadcast Thread] Stopped")

    def _pose_query_thread_func(self):
        """
        Pose query thread: Handle GET_POSE requests from mock tracking service.

        This thread:
        - Listens for "GET_POSE" UDP requests
        - Queries current robot pose from MuJoCo
        - Responds with JSON-encoded pose data
        """
        print(f"[Pose Query Thread] Started (port={self.POSE_QUERY_PORT})")

        while self.shared_state.running:
            try:
                data, addr = self.pose_sock.recvfrom(1024)
                message = data.decode('utf-8').strip()

                if message == "GET_POSE":
                    # Get current robot pose
                    pose_data = self.get_robot_pose()

                    # Send JSON response
                    response = json.dumps(pose_data)
                    self.pose_sock.sendto(response.encode('utf-8'), addr)

            except socket.timeout:
                # No request received, continue
                pass
            except Exception as e:
                if self.shared_state.running and "timed out" not in str(e):
                    print(f"[Pose Query] Error: {e}")

        print("[Pose Query Thread] Stopped")

    def _camera_publish_thread_func(self):
        """
        Camera publishing thread: Render and publish top-down images from MuJoCo.

        This thread:
        - Renders top-down view from MuJoCo at specified rate (default 30 Hz)
        - Publishes RGB images to 'rgb_images' topic
        - Publishes zero depth images to 'depth_images' topic
        - Mimics RealSense camera interface for compatibility
        """
        import mujoco

        target_dt = 1.0 / self.camera_rate
        print(f"[Camera Thread] Started (rate={self.camera_rate}Hz, size={self.camera_width}x{self.camera_height})")

        # Create a separate renderer for this thread to avoid OpenGL context conflicts
        # Each thread needs its own mujoco.Renderer with its own EGL/OpenGL context
        camera_renderer = mujoco.Renderer(self.sim.mjc_model, self.camera_width, self.camera_height)

        while self.shared_state.running:
            start = time.time()

            try:
                # Render top-down image from MuJoCo using thread-local renderer
                camera_renderer.update_scene(self.sim.mjc_data, 'camera')
                rendered_image = camera_renderer.render().copy()

                # Resize to match expected camera output size if needed
                # if rendered_image.shape[0] != self.camera_height or rendered_image.shape[1] != self.camera_width:
                #     color_image = cv2.resize(rendered_image, (self.camera_width, self.camera_height))
                # else:
                color_image = rendered_image

                # Create zero depth image with appropriate size
                depth_image = np.zeros((self.camera_height, self.camera_width), dtype=np.uint16)

                # Get timestamp and create ROS messages
                timestamp = rospy.get_rostime()

                # MuJoCo renders in RGB format
                color_msg = self.bridge.cv2_to_imgmsg(color_image, 'rgb8')
                depth_msg = self.bridge.cv2_to_imgmsg(depth_image, 'mono16')

                color_msg.header.stamp = timestamp
                depth_msg.header.stamp = timestamp

                # Publish messages
                self.color_im_pub.publish(color_msg)
                self.depth_im_pub.publish(depth_msg)

            except Exception as e:
                if self.shared_state.running:
                    print(f"[Camera] Error publishing images: {e}")

            # Maintain rate
            elapsed = time.time() - start
            sleep_time = target_dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            elif elapsed > target_dt * 2:
                print(f"[Camera] Warning: rendering took {elapsed*1000:.1f}ms (target: {target_dt*1000:.1f}ms)")

        # Clean up the renderer when thread exits
        camera_renderer.close()
        print("[Camera Thread] Stopped")

    def run(self):
        """Run the simulator with multi-threaded architecture."""
        try:
            print("\n" + "="*70)
            print("MuJoCo UDP Simulator Running (Real-Time Multi-Threaded)")
            print("="*70)
            print(f"Physics: {self.physics_rate} Hz | Sensors: {self.sensor_rate} Hz")
            print(f"Motor Port: {self.UDP_PORT} | Pose Query Port: {self.POSE_QUERY_PORT}")
            if ROS_AVAILABLE and self.enable_ros:
                print(f"Camera: Publishing to 'rgb_images' and 'depth_images' topics at {self.camera_rate}Hz")
            print("Start run_tensegrity_hybrid_mppi.py to connect")
            print("Press Ctrl+C to stop")
            print("="*70 + "\n")

            self.start_wall_time = time.time()

            # Launch interactive viewer if requested (must be from main thread)
            if self.use_viewer:
                self.viewer = PassiveViewer(
                    self.sim.mjc_model, self.sim.mjc_data,
                    overlay_callback=self.get_overlay_data
                )
                print("[Viewer] Interactive MuJoCo viewer launched")

            # Launch remote viewer if requested
            if self.use_remote_viewer:
                self.remote_viewer = RemoteViewer(
                    self.sim.mjc_model, self.sim.mjc_data, port=self.remote_port,
                    overlay_callback=self.get_overlay_data
                )
                print(f"[RemoteViewer] Open http://localhost:{self.remote_port} in your browser")

            # Start threads
            self.physics_thread = threading.Thread(
                target=self._physics_thread_func,
                name="PhysicsThread",
                daemon=True
            )
            self.udp_thread = threading.Thread(
                target=self._udp_receive_thread_func,
                name="UDPReceiveThread",
                daemon=True
            )
            self.sensor_thread = threading.Thread(
                target=self._sensor_broadcast_thread_func,
                name="SensorBroadcastThread",
                daemon=True
            )
            self.pose_query_thread = threading.Thread(
                target=self._pose_query_thread_func,
                name="PoseQueryThread",
                daemon=True
            )

            if ROS_AVAILABLE and self.enable_ros:
                self.camera_thread = threading.Thread(
                    target=self._camera_publish_thread_func,
                    name="CameraThread",
                    daemon=True
                )
            else:
                self.camera_thread = None

            self.physics_thread.start()
            self.udp_thread.start()
            self.sensor_thread.start()
            self.pose_query_thread.start()
            if self.camera_thread:
                self.camera_thread.start()

            # Main thread: render viewer + monitor threads
            viewer_fps = getattr(self, 'viewer_fps', 30)
            health_check_interval = 0.5  # seconds between thread health checks
            last_health_check = time.time()

            has_any_viewer = self.viewer is not None or self.remote_viewer is not None

            while self.shared_state.running:
                # Render viewer frame(s) from main thread
                if self.viewer is not None:
                    self.viewer.render()
                    if not self.viewer.is_running():
                        print("Viewer window closed, shutting down...")
                        break
                if self.remote_viewer is not None:
                    self.remote_viewer.render()

                if has_any_viewer:
                    time.sleep(1 / viewer_fps)
                else:
                    time.sleep(0.1)

                # Periodic thread health checks
                now = time.time()
                if now - last_health_check >= health_check_interval:
                    last_health_check = now
                    if not self.physics_thread.is_alive():
                        print("Error: Physics thread died")
                        break
                    if not self.udp_thread.is_alive():
                        print("Error: UDP thread died")
                        break
                    if not self.sensor_thread.is_alive():
                        print("Error: Sensor thread died")
                        break
                    if not self.pose_query_thread.is_alive():
                        print("Error: Pose query thread died")
                        break
                    if self.camera_thread and not self.camera_thread.is_alive():
                        print("Error: Camera thread died")
                        break

        except KeyboardInterrupt:
            print("\n\nShutting down simulator...")
        finally:
            # Stop all threads
            self.shared_state.running = False

            # Wait for threads to finish
            if self.physics_thread:
                self.physics_thread.join(timeout=2.0)
            if self.udp_thread:
                self.udp_thread.join(timeout=2.0)
            if self.sensor_thread:
                self.sensor_thread.join(timeout=2.0)
            if self.pose_query_thread:
                self.pose_query_thread.join(timeout=2.0)
            if self.camera_thread:
                self.camera_thread.join(timeout=2.0)

            # Close viewers
            if self.viewer is not None and self.viewer.is_running():
                self.viewer.close()
            if self.remote_viewer is not None and self.remote_viewer.is_running():
                self.remote_viewer.close()

            # Close sockets
            self.sock.close()
            self.pose_sock.close()

            # Print final statistics
            if self.timing_stats:
                wall_time = time.time() - self.start_wall_time
                with self.shared_state.lock:
                    sim_time = self.shared_state.sim_time
                    sim_steps = self.shared_state.sim_steps

                print(f"\nFinal Statistics:")
                print(f"  Wall time: {wall_time:.2f}s")
                print(f"  Sim time: {sim_time:.2f}s")
                print(f"  Drift: {abs(sim_time - wall_time)*1000:.1f}ms ({abs(sim_time - wall_time)/wall_time*100:.2f}%)")
                print(f"  Total steps: {sim_steps}")
                print(f"  Average rate: {sim_steps/wall_time:.1f} Hz")

                if len(self.shared_state.timing_errors) > 0:
                    errors = list(self.shared_state.timing_errors)
                    print(f"  Timing errors: mean={np.mean(errors)*1000:.2f}ms, "
                          f"max={np.max(errors)*1000:.2f}ms, "
                          f"std={np.std(errors)*1000:.2f}ms")

            print("Simulator stopped")


def main():
    """Main entry point."""
    import argparse

    parser = argparse.ArgumentParser(
        description="MuJoCo UDP Simulator for Tensegrity Robot (Real-Time Multi-Threaded)"
    )
    parser.add_argument(
        "xml_path",
        nargs="?",
        default=None,
        help="Path to MuJoCo XML model file"
    )
    parser.add_argument(
        "--no-viz",
        action="store_true",
        help="Disable visualization (faster simulation)"
    )
    parser.add_argument(
        "--port",
        type=int,
        default=2390,
        help="UDP port for motor commands (default: 2390)"
    )
    parser.add_argument(
        "--pose-port",
        type=int,
        default=2391,
        help="UDP port for pose queries (default: 2391)"
    )
    parser.add_argument(
        "--physics-rate",
        type=float,
        default=1000.0,
        help="Physics simulation rate in Hz (default: 1000.0)"
    )
    parser.add_argument(
        "--sensor-rate",
        type=float,
        default=20.0,
        help="Sensor broadcast rate in Hz (default: 20.0)"
    )
    parser.add_argument(
        "--timing-stats",
        action="store_true",
        help="Print timing diagnostics"
    )
    parser.add_argument(
        "--sensor-noise",
        action="store_true",
        help="Enable realistic sensor noise (Gaussian noise, bias drift, etc.)"
    )
    parser.add_argument(
        "--noise-scale",
        type=float,
        default=1.0,
        help="Scale factor for noise magnitude (default: 1.0)"
    )
    parser.add_argument(
        "--camera-rate",
        type=float,
        default=30.0,
        help="Camera image publishing rate in Hz (default: 30.0)"
    )
    parser.add_argument(
        "--camera-width",
        type=int,
        default=640,
        help="Camera image width in pixels (default: 640)"
    )
    parser.add_argument(
        "--camera-height",
        type=int,
        default=480,
        help="Camera image height in pixels (default: 480)"
    )
    parser.add_argument(
        "--no-ros",
        action="store_true",
        help="Disable ROS integration (camera publishing)"
    )
    parser.add_argument(
        "--viewer",
        action="store_true",
        help="Open an interactive MuJoCo viewer window for real-time visualization"
    )
    parser.add_argument(
        "--viewer-fps",
        type=float,
        default=30.0,
        help="Viewer rendering frame rate in Hz (default: 30.0)"
    )
    parser.add_argument(
        "--remote-viewer",
        action="store_true",
        help="Enable remote WebSocket viewer (headless, accessible via browser)"
    )
    parser.add_argument(
        "--remote-port",
        type=int,
        default=8765,
        help="Port for remote viewer web server (default: 8765)"
    )

    args = parser.parse_args()

    # Determine XML path
    if args.xml_path is None:
        # Try to find default XML
        possible_paths = [
            "xml_models/3bar_new_platform_all_cables.xml",
            "../xml_models/3bar_new_platform_all_cables.xml",
            "../../xml_models/3bar_new_platform_all_cables.xml",
        ]

        xml_path = None
        for p in possible_paths:
            if os.path.exists(p):
                xml_path = p
                break

        if xml_path is None:
            print("Error: No XML model specified and default not found")
            print("Usage: python tensegrity_udp_simulator.py <xml_path>")
            sys.exit(1)
    else:
        xml_path = args.xml_path

    print(f"Using XML model: {xml_path}\n")

    # Create and run simulator
    simulator = TensegrityUDPSimulator(
        xml_path=xml_path,
        start_se2=[0.7, 0, 0],
        visualize=not args.no_viz,
        attach_type='real_attach',
        udp_port=args.port,
        pose_query_port=args.pose_port,
        physics_rate=args.physics_rate,
        sensor_rate=args.sensor_rate,
        timing_stats=args.timing_stats,
        sensor_noise=args.sensor_noise,
        noise_scale=args.noise_scale,
        camera_rate=args.camera_rate,
        camera_size=(args.camera_width, args.camera_height),
        enable_ros=not args.no_ros,
        use_viewer=args.viewer,
        viewer_fps=args.viewer_fps,
        use_remote_viewer=args.remote_viewer,
        remote_port=args.remote_port
    )

    simulator.run()


def signal_handler(sig, frame):
    """Handle Ctrl+C by forcefully exiting."""
    print('\n\nReceived interrupt signal, forcing shutdown...')
    sys.exit(0)


if __name__ == "__main__":
    # Register signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
    main()

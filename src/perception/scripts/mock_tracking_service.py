#!/usr/bin/env python3
"""
Mock tracking service that requests noisy robot poses from MuJoCo UDP simulator.

This service acts as a drop-in replacement for the real tracking service during
simulation. Instead of processing camera images, it queries the MuJoCo simulator
for ground truth poses and adds configurable noise to simulate tracking uncertainty.

Architecture:
    ROS Subscribers --> MockTracker <--UDP--> MuJoCo Simulator
                           |
                           v
                    ROS Publishers (PoseStateStamped)

Usage:
    1. Start the MuJoCo UDP simulator: python tensegrity_udp_simulator.py
    2. Start this service: rosrun tensegrity mock_tracking_service.py
    3. Run your control code normally
"""

import socket
import time
import json
import os
import datetime
import numpy as np
import cv2
from copy import deepcopy
from collections import defaultdict

# ROS library
import rospy
import rospkg
import message_filters
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation

from tensegrity.msg import SensorsStamped, TensegrityStamped, PoseStateStamped

from tensegrity_perception.srv import InitTracker, InitTrackerRequest, InitTrackerResponse
from tensegrity_perception.srv import GetPose, GetPoseRequest, GetPoseResponse
from tensegrity_perception.srv import GetBarHeight, GetBarHeightRequest, GetBarHeightResponse


class MockTracker:
    """
    Mock tracker that gets ground truth poses from MuJoCo simulator with added noise.
    """

    def __init__(self, data_cfg, simulator_host='127.0.0.1', simulator_port=2391,
                 position_noise_std=0.01, orientation_noise_std=0.05,
                 publish_delay=0.0, save_data=True):
        """
        Initialize mock tracking service.

        Args:
            data_cfg: Configuration dictionary (same format as real tracker)
            simulator_host: Host address of MuJoCo simulator
            simulator_port: UDP port for pose queries (separate from motor control port)
            position_noise_std: Standard deviation of position noise in meters
            orientation_noise_std: Standard deviation of orientation noise in radians
            publish_delay: Delay in seconds before publishing pose (simulates processing latency)
        """
        self.data_cfg = data_cfg

        # UDP connection to simulator for pose queries
        self.simulator_host = simulator_host
        self.simulator_port = simulator_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.1)  # 100ms timeout

        # Noise parameters
        self.position_noise_std = position_noise_std
        self.orientation_noise_std = orientation_noise_std
        
        # Publish delay (simulates processing latency)
        self.publish_delay = publish_delay


        # ROS subscribers
        self.rgb_topic = "/rgb_images"
        self.depth_topic = "/depth_images"
        self.strain_topic = "/control_msg"

        color_im_sub = message_filters.Subscriber(self.rgb_topic, Image)
        depth_im_sub = message_filters.Subscriber(self.depth_topic, Image)
        strain_sub = message_filters.Subscriber(self.strain_topic, TensegrityStamped)

        self.time_synchronizer = message_filters.ApproximateTimeSynchronizer(
            [color_im_sub, depth_im_sub, strain_sub], queue_size=10, slop=0.1)
        self.time_synchronizer.registerCallback(self.tracking_callback)

        self.count = 0

        # ROS publishers
        self.trajectory_topic = "/trajectory_images"
        self.trajectory_pub = rospy.Publisher(self.trajectory_topic, Image, queue_size=10)
        self.pose_topic = '/pose_msg'
        self.pose_pub = rospy.Publisher(self.pose_topic, PoseStateStamped, queue_size=10)

        self.bridge = CvBridge()
        self.initialized = True  # Auto-initialize for simulation (no manual init needed)
        self.current_poses = None

        # saving data (optional - disable if directories can't be created)
        self.save_data = save_data

        if self.save_data:
            try:
                data_path = os.path.normpath(os.path.join(rospkg.RosPack().get_path('tensegrity'),'../../data/'))
                self.output_dir = os.path.join(data_path, datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))
                self.color_dir = os.path.join(self.output_dir, 'color')
                self.depth_dir = os.path.join(self.output_dir, 'depth')
                self.track_dir = os.path.join(self.output_dir, 'tracking')
                self.data_dir = os.path.join(self.output_dir, 'data')
                os.makedirs(self.output_dir, exist_ok=True)
                os.makedirs(self.color_dir, exist_ok=True)
                os.makedirs(self.depth_dir, exist_ok=True)
                os.makedirs(self.track_dir, exist_ok=True)
                os.makedirs(self.data_dir, exist_ok=True)
                self.save_data = True
                rospy.loginfo(f"Data will be saved to: {self.output_dir}")
            except Exception as e:
                rospy.logwarn(f"Could not create data directories: {e}. Data saving disabled.")
                self.save_data = False
                self.color_dir = None
                self.depth_dir = None
                self.track_dir = None
                self.data_dir = None

        # Register ROS services (after directory setup to avoid race condition)
        rospy.Service('init_tracker', InitTracker, self.initialize_tracker)
        rospy.Service('get_pose', GetPose, self.get_current_state)
        rospy.Service('get_bar_height', GetBarHeight, self.get_bar_height)

        rospy.loginfo(f"Mock Tracker initialized:")
        rospy.loginfo(f"  - Simulator: {simulator_host}:{simulator_port}")
        rospy.loginfo(f"  - Position noise std: {position_noise_std}m")
        rospy.loginfo(f"  - Orientation noise std: {orientation_noise_std}rad")
        rospy.loginfo(f"  - Publish delay: {publish_delay}s")

    def query_simulator_pose(self):
        """
        Query the MuJoCo simulator for current robot pose via UDP.

        Returns:
            list of Pose objects (one per rod), or None if query fails
        """
        try:
            # Send pose query request
            request = "GET_POSE"
            self.sock.sendto(request.encode('utf-8'), (self.simulator_host, self.simulator_port))

            # Receive response
            data, _ = self.sock.recvfrom(4096)
            response = json.loads(data.decode('utf-8'))

            if response.get('status') != 'ok':
                rospy.logwarn(f"Simulator pose query failed: {response.get('error', 'unknown')}")
                return None

            # Parse poses
            poses = []
            for rod_data in response['rods']:
                pose = Pose()

                # Position (in meters)
                pos = rod_data['position']
                pose.position.x = pos[0]
                pose.position.y = pos[1]
                pose.position.z = pos[2]

                # Orientation (as quaternion)
                quat = rod_data['orientation']  # [x, y, z, w]
                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]

                poses.append(pose)

            return poses

        except socket.timeout:
            rospy.logwarn_throttle(5.0, "Simulator pose query timeout")
            return None
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"Error querying simulator pose: {e}")
            return None

    def add_pose_noise(self, poses):
        """
        Add Gaussian noise to poses to simulate tracking uncertainty.

        Args:
            poses: List of Pose objects

        Returns:
            List of noisy Pose objects
        """
        if not poses:
            return None

        noisy_poses = []
        for pose in poses:
            noisy_pose = deepcopy(pose)

            try:
                # Add position noise
                noise_trans_dir = 2 * (np.random.rand(3) - 0.5)
                noise_trans_dir = noise_trans_dir / np.linalg.norm(noise_trans_dir)
                noise_trans_mag = np.random.normal(0, self.position_noise_std)
                noisy_pose.position.x += noise_trans_mag * noise_trans_dir[0]
                noisy_pose.position.y += noise_trans_mag * noise_trans_dir[1]
                noisy_pose.position.z += noise_trans_mag * noise_trans_dir[2]

                # Random axis rotation
                noise_rot_axis = 2 * (np.random.rand(3) - 0.5)
                noise_rot_axis = noise_rot_axis / np.linalg.norm(noise_rot_axis)
                noise_angle = np.random.normal(0, self.orientation_noise_std)
                noise_rot = Rotation.from_rotvec(noise_angle * noise_rot_axis)

                original_rot = Rotation.from_quat([
                    noisy_pose.orientation.x,
                    noisy_pose.orientation.y,
                    noisy_pose.orientation.z,
                    noisy_pose.orientation.w
                ])

                # Compose rotations
                noisy_rot = noise_rot * original_rot
                quat = noisy_rot.as_quat()  # [x, y, z, w]

                noisy_pose.orientation.x = quat[0]
                noisy_pose.orientation.y = quat[1]
                noisy_pose.orientation.z = quat[2]
                noisy_pose.orientation.w = quat[3]
            except Exception as e:
                rospy.logwarn(f"Error adding orientation noise: {e}")
                noisy_pose.orientation.x = pose.orientation.x
                noisy_pose.orientation.y = pose.orientation.y
                noisy_pose.orientation.z = pose.orientation.z
                noisy_pose.orientation.w = pose.orientation.w

            noisy_poses.append(noisy_pose)

        return noisy_poses

    def get_current_state(self, req: GetPoseRequest):
        """ROS service to get current robot pose."""
        rospy.loginfo("Received 'get_pose' service request")
        response = GetPoseResponse()

        if self.current_poses is None:
            response.success = False
            return response

        response.poses = self.current_poses
        response.success = True
        return response

    def get_bar_height(self, req: GetBarHeightRequest):
        """ROS service to get bar height (mock implementation)."""
        rospy.loginfo("Received 'get_bar_height' service request")
        response = GetBarHeightResponse()

        # For simulation, just return a dummy value
        # In real scenario, this would query the simulator
        response.height = 0.5  # meters
        response.success = True
        return response

    def initialize_tracker(self, req: InitTrackerRequest):
        """ROS service to initialize tracker."""
        rospy.loginfo("Received 'init_tracker' service request")

        # For mock tracker, just set initialized flag
        self.initialized = True

        response = InitTrackerResponse()
        response.success = True
        return response

    def tracking_callback(self, rgb_msg, depth_msg, strain_msg):
        """
        Main tracking callback: Query simulator for pose instead of computing from images.

        Args:
            rgb_msg: RGB image message (unused, for interface compatibility)
            depth_msg: Depth image message (unused, for interface compatibility)
            strain_msg: Control/strain message with motor and sensor data
        """
        if not self.initialized:
            rospy.loginfo_throttle(5.0, 'Mock tracking service not initialized yet')
            return

        rospy.logdebug(f"Processing tracking frame {self.count}")

        # Query simulator for ground truth pose
        gt_poses = self.query_simulator_pose()

        if gt_poses is None:
            rospy.logwarn_throttle(1.0, "Failed to get pose from simulator")
            return

        # Add noise to simulate tracking uncertainty
        noisy_poses = self.add_pose_noise(gt_poses)
        self.current_poses = noisy_poses

        # Publish PoseStateStamped message
        pose_state_msg = PoseStateStamped()
        pose_state_msg.header.stamp = rgb_msg.header.stamp
        pose_state_msg.header.frame_id = rgb_msg.header.frame_id if hasattr(rgb_msg.header, 'frame_id') else ''

        # Add poses
        pose_state_msg.poses = noisy_poses

        # Extract encoder_lengths and motor_speeds from strain_msg
        encoder_lengths = []
        motor_speeds = []
        for motor in strain_msg.motors:
            encoder_lengths.append(float(motor.encoder_length))
            motor_speeds.append(float(motor.speed))

        pose_state_msg.encoder_lengths = encoder_lengths
        pose_state_msg.motor_speeds = motor_speeds

        # Simulate processing latency if configured
        if self.publish_delay > 0:
            time.sleep(self.publish_delay)

        rospy.loginfo("Publishing pose message")
        self.pose_pub.publish(pose_state_msg)

        # Compute endcaps from noisy poses for data saving
        endcaps = []
        for pose in noisy_poses:
            # Convert quaternion to rotation matrix
            R = Rotation.from_quat([
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
            ]).as_matrix()

            # Get rod center position
            t = np.array([pose.position.x, pose.position.y, pose.position.z])

            # Rod z-axis unit vector
            unit_vector = R[:, 2]

            # Compute endcap positions
            endcaps.append(t - self.data_cfg['rod_length']/2 * unit_vector)
            endcaps.append(t + self.data_cfg['rod_length']/2 * unit_vector)

        # Save data (optional - only if directories were successfully created)
        if self.save_data:
            rgb_im = self.bridge.imgmsg_to_cv2(rgb_msg, 'rgb8')
            cv2.imwrite(os.path.join(self.color_dir, str(self.count).zfill(4) + ".png"), cv2.cvtColor(rgb_im, cv2.COLOR_RGB2BGR))
            cv2.imwrite(os.path.join(self.depth_dir, str(self.count).zfill(4) + ".png"), self.bridge.imgmsg_to_cv2(depth_msg, 'mono16'))
            cv2.imwrite(os.path.join(self.track_dir, str(self.count).zfill(4) + ".png"), cv2.cvtColor(rgb_im, cv2.COLOR_RGB2BGR))
            data = {}
            data['header'] = {'seq':strain_msg.header.seq,'secs':strain_msg.header.stamp.to_sec()}
            data['info'] = {'min_length':strain_msg.info.min_length,'RANGE':strain_msg.info.RANGE,'RANGE024':strain_msg.info.RANGE024,'RANGE135':strain_msg.info.RANGE135,'MAX_RANGE':strain_msg.info.MAX_RANGE,'MIN_RANGE':strain_msg.info.MIN_RANGE,'max_speed':strain_msg.info.max_speed,'tol':strain_msg.info.tol,'low_tol':strain_msg.info.low_tol,'P':strain_msg.info.P,'I':strain_msg.info.I,'D':strain_msg.info.D,'dist_weight':strain_msg.info.dist_weight,'ang_weight':strain_msg.info.ang_weight,'prog_weight':strain_msg.info.prog_weight}
            data['motors'] = {}
            for motor in strain_msg.motors:
                data['motors'][motor.id] = {'target':motor.target,'position':motor.position,'speed':motor.speed,'done':motor.done}
            data['sensors'] = {}
            for sensor in strain_msg.sensors:
                data['sensors'][sensor.id] = {'length':sensor.length,'capacitance':sensor.capacitance}
            data['trajectory'] = {i:{'x':point.x,'y':point.y} for i,point in enumerate(strain_msg.trajectory.trajectory)}
            data['COM'] = {i:{'x':point.x,'y':point.y} for i,point in enumerate(strain_msg.trajectory.COMs)}
            data['PA'] = {i:{'x':point.x,'y':point.y} for i,point in enumerate(strain_msg.trajectory.PAs)}
            data['action'] = {str(i):act for i,act in enumerate(strain_msg.actions)}
            data['endcaps'] = {i:{'x':end[0],'y':end[1],'z':end[2]} for i,end in enumerate(endcaps)}
            data['segment'] = strain_msg.trajectory.trajectory_segment
            # data['robot-height'] = highest_node
            # data['bar-height'] = float(self.latest_bar_height)
            json.dump(data,open(os.path.join(self.data_dir, str(self.count).zfill(4) + ".json"),'w'))

        self.count += 1

    def run(self):
        """Run the mock tracker."""
        rospy.loginfo("Mock tracking service started")
        rospy.loginfo("Waiting for messages...")
        rospy.spin()


def main():
    """Main entry point."""
    rospy.init_node("mock_tracking_service")

    # Load configuration
    try:
        import rospkg
        package_path = rospkg.RosPack().get_path("tensegrity")

        # Try to load data config (same as real tracker)
        data_cfg_file = rospy.get_param("data_cfg_file", None)
        if data_cfg_file:
            with open(data_cfg_file, 'r') as fp:
                data_cfg = json.load(fp)
        else:
            # Minimal config
            data_cfg = {
                'num_rods': 3,
                'end_cap_colors': ['red', 'green', 'blue'],
                'rod_length': 0.295,  # 12 inches in meters
            }
            rospy.logwarn("No data_cfg_file parameter found, using minimal config")
    except Exception as e:
        rospy.logwarn(f"Could not load config: {e}, using defaults")
        data_cfg = {
            'num_rods': 3,
            'end_cap_colors': ['red', 'green', 'blue'],
            'rod_length': 0.295,
        }

    # Get parameters
    simulator_host = rospy.get_param("~simulator_host", "127.0.0.1")
    simulator_port = rospy.get_param("~simulator_pose_port", 2391)
    position_noise_std = rospy.get_param("~position_noise_std", 0.0)  # 3cm default
    orientation_noise_std = rospy.get_param("~orientation_noise_std", 0.0)  # ~5 degrees default
    publish_delay = rospy.get_param("~publish_delay", 0.00)  # seconds, default no delay

    # Initialize tracker
    tracker = MockTracker(
        data_cfg,
        simulator_host=simulator_host,
        simulator_port=simulator_port,
        position_noise_std=position_noise_std,
        orientation_noise_std=orientation_noise_std,
        publish_delay=publish_delay,
        save_data=False
    )

    tracker.run()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

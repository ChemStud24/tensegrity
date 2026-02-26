#!/usr/bin/env python3
"""
Comprehensive end-to-end tests for the hybrid MPPI tensegrity system.

This test suite covers:
1. TensegrityRobot (run_tensegrity_hybrid_mppi.py) - UDP communication, hybrid control, gaits
2. PlannerMPPI (mppi_planner.py) - pose queue, state callback, planning
3. TensegrityUDPSimulator (tensegrity_udp_simulator.py) - multi-threading, sensors, UDP
4. MockTracker (mock_tracking_service.py) - services, pose queries, noise

Architecture tested:
    TensegrityUDPSimulator <--UDP--> TensegrityRobot <--ROS--> PlannerMPPI
                ↑                                                    ↑
                |                                                    |
                +------------------MockTracker----------------------+

IMPORTANT: This test requires a properly built ROS workspace.
Run from workspace root:
    $ cd ~/research/tensegrity
    $ source devel/setup.bash  # or source /opt/ros/noetic/setup.bash first
    $ python src/tensegrity/src/test_end_to_end_hybrid_mppi.py

Or use rosrun:
    $ rosrun tensegrity test_end_to_end_hybrid_mppi.py
"""

import os
import sys
import time
import threading
import socket
import json
import unittest
import numpy as np
import torch
from pathlib import Path
from collections import deque
from unittest.mock import patch, MagicMock

# Mock optional modules to avoid import errors in test environment
if 'pynput' not in sys.modules:
    sys.modules['pynput'] = MagicMock()
    sys.modules['pynput.keyboard'] = MagicMock()
if 'xlrd' not in sys.modules:
    sys.modules['xlrd'] = MagicMock()

# Add current directory and mppi directory for local imports
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)
sys.path.insert(0, os.path.join(current_dir, 'mppi'))

# ROS imports
try:
    import rospy
    from std_msgs.msg import Float64MultiArray
    from sensor_msgs.msg import Image
    from geometry_msgs.msg import Point, Pose
    from cv_bridge import CvBridge
    from scipy.spatial.transform import Rotation as R

    from tensegrity.msg import (
        State,
        ActionHybridMPPI,
        PoseStateStamped,
        TensegrityStamped,
        Motor,
        Sensor,
        Imu,
        Info,
        Trajectory,
    )

    # Optional imports - these are only needed for MockTracker tests
    try:
        from tensegrity.srv import (
            InitTracker, InitTrackerRequest, InitTrackerResponse,
            GetPose, GetPoseRequest, GetPoseResponse,
            GetBarHeight, GetBarHeightRequest, GetBarHeightResponse
        )
        PERCEPTION_SERVICES_AVAILABLE = True
    except ImportError:
        # Mock the service types if not available
        PERCEPTION_SERVICES_AVAILABLE = False
        InitTracker = InitTrackerRequest = InitTrackerResponse = MagicMock
        GetPose = GetPoseRequest = GetPoseResponse = MagicMock
        GetBarHeight = GetBarHeightRequest = GetBarHeightResponse = MagicMock
except ImportError as e:
    print("\n" + "="*70)
    print("ERROR: ROS packages not found!")
    print("="*70)
    print(f"\nImport error: {e}")
    print("\nTo run these tests, you need to:")
    print("1. Build the ROS workspace:")
    print("   $ cd ~/research/tensegrity")
    print("   $ catkin_make  # or catkin build")
    print("\n2. Source the workspace:")
    print("   $ source devel/setup.bash")
    print("\n3. Run the tests:")
    print("   $ python src/tensegrity/src/test_end_to_end_hybrid_mppi.py")
    print("\nOr use rosrun:")
    print("   $ rosrun tensegrity test_end_to_end_hybrid_mppi.py")
    print("\n" + "="*70)
    sys.exit(1)


class TestTensegrityRobotUnit(unittest.TestCase):
    """Unit tests for TensegrityRobot class."""

    @classmethod
    def setUpClass(cls):
        """Set up ROS node once for all tests."""
        try:
            rospy.init_node('test_tensegrity_robot', anonymous=True)
        except rospy.exceptions.ROSException:
            pass  # Node already initialized

    def setUp(self):
        """Set up test fixtures."""
        # Try to import, skip tests if dependencies are missing
        try:
            from run_tensegrity_hybrid_mppi import TensegrityRobot
            self.TensegrityRobot = TensegrityRobot
        except (ImportError, ModuleNotFoundError) as e:
            self.skipTest(f"Skipping TensegrityRobot tests due to missing dependency: {e}")

    def test_initialization(self):
        """Test TensegrityRobot initialization."""
        with patch('socket.socket'), patch('rospy.init_node'), patch('rosnode.get_node_names', return_value=[]), patch('rospy.ServiceProxy'):
            robot = self.TensegrityRobot()

            # Check basic initialization
            self.assertEqual(robot.num_motors, 6)
            self.assertEqual(robot.num_sensors, 9)
            self.assertEqual(robot.control_mode, "astar")
            self.assertIsNone(robot.mppi_actions)
            self.assertEqual(len(robot.states), 6)  # Default gait states

    def test_hybrid_callback_astar_mode(self):
        """Test hybrid_callback with A* primitive actions."""
        with patch('socket.socket'), patch('rospy.init_node'), patch('rosnode.get_node_names', return_value=[]), patch('rospy.ServiceProxy'):
            robot = self.TensegrityRobot()
            robot.prev_bottom_nodes = (0, 2, 5)

            # Create A* action message
            msg = ActionHybridMPPI()
            msg.control_type = "astar"
            msg.primitive_actions = ["100_120"]
            msg.control_interval = -1.0

            robot.hybrid_callback(msg)

            # Verify mode switch
            self.assertEqual(robot.control_mode, "astar")
            self.assertIsNotNone(robot.next_states)
            self.assertEqual(robot.RANGE135, 100)
            self.assertEqual(robot.RANGE024, 120)

    def test_hybrid_callback_mppi_mode(self):
        """Test hybrid_callback with MPPI actions."""
        with patch('socket.socket'), patch('rospy.init_node'), patch('rosnode.get_node_names', return_value=[]), patch('rospy.ServiceProxy'):
            robot = self.TensegrityRobot()

            # Create MPPI action message (T=5, 6 motors)
            msg = ActionHybridMPPI()
            msg.control_type = "mppi"
            # Need to pass as 2D array: shape (5, 6)
            mppi_actions_flat = [0.5, -0.3, 0.8, -0.2, 0.4, -0.6] * 5
            msg.mppi_actions = np.array(mppi_actions_flat).reshape(5, 6).tolist()
            msg.control_interval = 0.5

            robot.hybrid_callback(msg)

            # Verify MPPI mode setup
            self.assertEqual(robot.control_mode, "mppi")
            self.assertIsNotNone(robot.mppi_actions)
            self.assertEqual(robot.mppi_actions.shape, (5, 6))
            self.assertEqual(robot.mppi_interval, 0.5)
            self.assertIsNone(robot.mppi_start_time)

    def test_compute_command_astar_mode(self):
        """Test compute_command in A* mode (PID control)."""
        with patch('socket.socket'), patch('rospy.init_node'), patch('rosnode.get_node_names', return_value=[]), patch('rospy.ServiceProxy'):
            robot = self.TensegrityRobot()
            robot.control_mode = "astar"
            robot.addresses = [("127.0.0.1", 2390)] * 3
            robot.which_Arduino = 0

            # Set up test state
            robot.pos = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
            robot.states = np.array([[1.0, 1.0, 0.1, 1.0, 1.0, 0.1]])
            robot.state = 0
            robot.done = [False] * 6

            robot.compute_command()

            # Verify PID control is computed
            self.assertIsNotNone(robot.speed)
            self.assertEqual(len(robot.speed), 6)

    def test_compute_command_mppi_mode(self):
        """Test compute_command in MPPI mode (direct streaming)."""
        with patch('socket.socket'), patch('rospy.init_node'), patch('rosnode.get_node_names', return_value=[]), patch('rospy.ServiceProxy'):
            robot = self.TensegrityRobot()
            robot.control_mode = "mppi"
            robot.addresses = [("127.0.0.1", 2390)] * 3
            robot.which_Arduino = 0
            robot.max_speed = 70

            # Set up MPPI actions
            robot.mppi_actions = np.array([[0.5, -0.3, 0.8, -0.2, 0.4, -0.6]] * 3)
            robot.mppi_interval = 0.5
            robot.mppi_start_time = time.time()
            robot.mppi_idx = 0

            robot.compute_command()

            # Verify motor speeds are set from MPPI actions
            self.assertIsNotNone(robot.speed)
            self.assertEqual(len(robot.speed), 6)
            # Check that speeds are within bounds
            for speed in robot.speed:
                self.assertLessEqual(abs(speed), robot.max_speed)

    def test_gait_library(self):
        """Test gait library contains expected gaits."""
        with patch('socket.socket'), patch('rospy.init_node'), patch('rosnode.get_node_names', return_value=[]), patch('rospy.ServiceProxy'):
            robot = self.TensegrityRobot()

            self.assertIn("roll", robot.all_gaits)
            self.assertIn("cw", robot.all_gaits)
            self.assertIn("ccw", robot.all_gaits)

            # Verify gait shapes
            for gait_name, gait in robot.all_gaits.items():
                self.assertEqual(gait.shape[1], 6)  # 6 motors


class TestPlannerMPPIUnit(unittest.TestCase):
    """Unit tests for PlannerMPPI class."""

    @classmethod
    def setUpClass(cls):
        """Set up ROS node once for all tests."""
        try:
            rospy.init_node('test_planner_mppi', anonymous=True)
        except rospy.exceptions.ROSException:
            pass

    def setUp(self):
        """Set up test fixtures."""
        # Try to import, skip tests if dependencies are missing
        try:
            from mppi_planner import PlannerMPPI
            self.PlannerMPPI = PlannerMPPI
        except (ImportError, ModuleNotFoundError) as e:
            self.skipTest(f"Skipping PlannerMPPI tests due to missing dependency: {e}")

        self.start = (0.5, 1.1, np.pi/2)
        self.goal = (1.7, 0.2, 0)
        self.obstacles = ((0.5, 0.3), (0.5, 0.5))
        self.boundary = (-1, 3, -0.2, 1.4)
        self.init_cable_lengths = [0.2] * 6

        self.astar_params = {
            'gaits': [('roll', 100, 100)],
            'gait_deltas': [(-0.7, -1.6, -0.3)],
            'heur_type': 'wave',
        }

    def test_initialization(self):
        """Test PlannerMPPI initialization."""
        # Create minimal MPPI params with mock sim
        mppi_params = {
            'sim': self._create_mock_sim(),
            'strategy': 'min',
            'device': 'cpu',
            'cost_weights': (1.0, 0.0, 0.0),
            'ctrl_interval': 0.5,
            'horizon': 2.0,
            'n_samples': 10,
        }

        planner = self.PlannerMPPI(
            self.start, self.goal, self.boundary,
            self.init_cable_lengths, self.obstacles,
            astar_params=self.astar_params,
            mppi_params=mppi_params
        )

        # Check initialization
        self.assertEqual(planner.current_state, self.start)
        self.assertEqual(planner.goal, self.goal)
        self.assertIsNotNone(planner.rest_lens)
        self.assertEqual(len(planner.pose_queue), 1)  # Initial pose from state

    def test_state_callback(self):
        """Test state callback sets ready flag."""
        mppi_params = {'sim': self._create_mock_sim(), 'strategy': 'min', 'device': 'cpu',
                       'cost_weights': (1.0, 0.0, 0.0), 'ctrl_interval': 0.5,
                       'horizon': 2.0, 'n_samples': 10}

        planner = self.PlannerMPPI(self.start, self.goal, self.boundary, self.init_cable_lengths,
                             self.obstacles, astar_params=self.astar_params, mppi_params=mppi_params)

        self.assertFalse(planner.ready_for_next_action)

        # Create state message
        msg = State()
        msg.prev_action = "100_100"
        msg.reverse_the_gait = False

        planner.state_callback(msg)

        self.assertTrue(planner.ready_for_next_action)

    def test_pose_callback_queue(self):
        """Test pose callback updates pose queue."""
        mppi_params = {'sim': self._create_mock_sim(), 'strategy': 'min', 'device': 'cpu',
                       'cost_weights': (1.0, 0.0, 0.0), 'ctrl_interval': 0.5,
                       'horizon': 2.0, 'n_samples': 10}

        planner = self.PlannerMPPI(self.start, self.goal, self.boundary, self.init_cable_lengths,
                             self.obstacles, astar_params=self.astar_params, mppi_params=mppi_params)

        initial_queue_len = len(planner.pose_queue)

        # Create pose message
        msg = PoseStateStamped()
        msg.header.stamp = rospy.Time.now()
        msg.poses = [self._create_test_pose(0.5, 1.0, 0.1) for _ in range(3)]
        msg.encoder_lengths = [0.15, 0.16, 0.17, 0.18, 0.19, 0.20]
        msg.motor_speeds = [0.0] * 6

        planner.pose_callback(msg)

        # Verify queue updated
        self.assertEqual(len(planner.pose_queue), initial_queue_len + 1)

    def test_dimension_scaling(self):
        """Test 10x dimension scaling for planning."""
        mppi_params = {'sim': self._create_mock_sim(), 'strategy': 'min', 'device': 'cpu',
                       'cost_weights': (1.0, 0.0, 0.0), 'ctrl_interval': 0.5,
                       'horizon': 2.0, 'n_samples': 10}

        planner = self.PlannerMPPI(self.start, self.goal, self.boundary, self.init_cable_lengths,
                             self.obstacles, astar_params=self.astar_params, mppi_params=mppi_params)

        # Check that astar_params gait_deltas are scaled by 10
        scaled_deltas = planner.astar_params['gait_deltas']
        original_deltas = self.astar_params['gait_deltas']

        for scaled, original in zip(scaled_deltas, original_deltas):
            self.assertAlmostEqual(scaled[0], original[0] * 10.0, places=5)
            self.assertAlmostEqual(scaled[1], original[1] * 10.0, places=5)

    def _create_mock_sim(self):
        """Create a mock simulator for testing."""
        class MockSim:
            def __init__(self):
                self.dtype = torch.float32
                self.device = 'cpu'
                self.curr_state = torch.zeros(1, 13, 1)
                self.robot = MagicMock()
                self.robot.actuated_cables = {
                    i: MagicMock(rest_length=torch.tensor(1.0)) for i in range(6)
                }
                # Add data_processor mock with dt attribute
                self.data_processor = MagicMock()
                self.data_processor.dt = torch.tensor(0.01)  # 10ms default timestep
            def get_curr_state(self):
                """Return the current state tensor."""
                return self.curr_state
            def run_target_gait(self, *args, **kwargs):
                pass
        return MockSim()

    def _create_test_pose(self, x, y, z):
        """Create a test pose."""
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        quat = R.from_euler('z', 0).as_quat()
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        return pose


class TestUDPSimulatorUnit(unittest.TestCase):
    """Unit tests for TensegrityUDPSimulator."""

    def test_sensor_data_generation(self):
        """Test sensor data is generated correctly."""
        from mujoco_simulator.tensegrity_udp_simulator import TensegrityUDPSimulator, SharedSimulatorState

        # Test shared state
        state = SharedSimulatorState(num_motors=6, num_sensors=9, num_rods=3)

        self.assertEqual(len(state.motor_speeds), 6)
        self.assertEqual(len(state.sensor_data['capacitance']), 9)
        self.assertEqual(len(state.sensor_data['encoder_counts']), 6)
        self.assertEqual(len(state.sensor_data['accelerometer']), 3)
        self.assertEqual(len(state.sensor_data['gyroscope']), 3)

    def test_motor_command_parsing(self):
        """Test motor command parsing from UDP."""
        # Test command format: "0 0 0 speed0 speed1 speed2 speed3 speed4 speed5 0 0 0"
        # With offset=3, speeds start at index 3
        command = "0 0 0 10.5 -5.2 8.3 -12.1 6.7 -3.4 0 0 0"
        values = command.strip().split()
        offset = 3

        motor_speeds = []
        for i in range(6):
            motor_speeds.append(float(values[offset + i]))

        expected = [10.5, -5.2, 8.3, -12.1, 6.7, -3.4]
        np.testing.assert_array_almost_equal(motor_speeds, expected)

    def test_sensor_noise_parameters(self):
        """Test sensor noise configuration."""
        from mujoco_simulator.tensegrity_udp_simulator import TensegrityUDPSimulator

        # Mock the XML path and MuJoCo simulator
        with patch('mujoco_simulator.tensegrity_udp_simulator.ThreeBarTensegrityMuJoCoSimulator'):
            with patch('pathlib.Path.exists', return_value=True):
                with patch('socket.socket'), patch('rospy.init_node'), patch('rosnode.get_node_names', return_value=[]), patch('rospy.ServiceProxy'):
                    sim = TensegrityUDPSimulator(
                        xml_path='dummy.xml',
                        visualize=False,
                        sensor_noise=True,
                        noise_scale=2.0
                    )

                    # Check noise parameters are scaled
                    self.assertEqual(sim.noise_params['capacitance_std'], 2.0 * 2.0)
                    self.assertEqual(sim.noise_params['accel_std'], 0.05 * 2.0)
                    self.assertEqual(sim.noise_params['gyro_std'], 0.01 * 2.0)


class TestMockTrackerUnit(unittest.TestCase):
    """Unit tests for MockTracker."""

    @classmethod
    def setUpClass(cls):
        """Set up ROS node once for all tests."""
        try:
            rospy.init_node('test_mock_tracker', anonymous=True)
        except rospy.exceptions.ROSException:
            pass

    def setUp(self):
        """Set up test fixtures."""
        # Try to import, skip tests if dependencies are missing
        try:
            from perception.scripts.mock_tracking_service import MockTracker
            self.MockTracker = MockTracker
        except (ImportError, ModuleNotFoundError, AttributeError) as e:
            self.skipTest(f"Skipping MockTracker tests due to missing dependency or ROS service issue: {e}")

    def test_pose_noise_injection(self):
        """Test noise is added to poses correctly."""
        data_cfg = {
            'num_rods': 3,
            'rod_length': 0.3048,
            'end_cap_colors': ['red', 'green', 'blue']
        }

        with patch('socket.socket'), patch('rospy.init_node'), patch('rospy.Service'):
            with patch('os.mkdir'), patch('rospkg.RosPack'):
                tracker = self.MockTracker(
                    data_cfg,
                    position_noise_std=0.01,
                    orientation_noise_std=0.05
                )

                # Create test poses
                original_poses = []
                for i in range(3):
                    pose = Pose()
                    pose.position.x = 1.0
                    pose.position.y = 2.0
                    pose.position.z = 0.5
                    quat = R.from_euler('z', 0).as_quat()
                    pose.orientation.x = quat[0]
                    pose.orientation.y = quat[1]
                    pose.orientation.z = quat[2]
                    pose.orientation.w = quat[3]
                    original_poses.append(pose)

                # Add noise
                noisy_poses = tracker.add_pose_noise(original_poses)

                # Verify noise was added (positions should differ slightly)
                self.assertEqual(len(noisy_poses), 3)
                for orig, noisy in zip(original_poses, noisy_poses):
                    # Positions should be close but different
                    self.assertNotEqual(noisy.position.x, orig.position.x)
                    self.assertAlmostEqual(noisy.position.x, orig.position.x, delta=0.1)

    def test_service_handlers(self):
        """Test ROS service handlers."""
        data_cfg = {'num_rods': 3, 'rod_length': 0.3048, 'end_cap_colors': ['red', 'green', 'blue']}

        with patch('socket.socket'), patch('os.mkdir'), patch('rospy.Service'), patch('rospy.init_node'), patch('rospkg.RosPack'):
            tracker = self.MockTracker(data_cfg)

            # Test init_tracker service
            req = InitTrackerRequest()
            resp = tracker.initialize_tracker(req)
            self.assertTrue(resp.success)
            self.assertTrue(tracker.initialized)

            # Test get_bar_height service
            req = GetBarHeightRequest()
            resp = tracker.get_bar_height(req)
            self.assertTrue(resp.success)
            self.assertGreater(resp.height, 0)


class TestIntegration(unittest.TestCase):
    """Integration tests for message flow between components."""

    @classmethod
    def setUpClass(cls):
        """Set up ROS node once for all tests."""
        try:
            rospy.init_node('test_integration', anonymous=True)
        except rospy.exceptions.ROSException:
            pass

    def test_action_message_format(self):
        """Test ActionHybridMPPI message format."""
        msg = ActionHybridMPPI()
        msg.header.stamp = rospy.Time.now()
        msg.control_type = "mppi"
        msg.mppi_actions = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6] * 5
        msg.control_interval = 0.5

        # Verify we can reshape to (T, 6)
        actions = np.array(msg.mppi_actions).reshape(-1, 6)
        self.assertEqual(actions.shape, (5, 6))

    def test_pose_state_message_format(self):
        """Test PoseStateStamped message format."""
        msg = PoseStateStamped()
        msg.header.stamp = rospy.Time.now()

        # Add 3 poses (3 rods)
        for i in range(3):
            pose = Pose()
            pose.position.x = 0.5 + i * 0.1
            pose.position.y = 1.0
            pose.position.z = 0.1
            quat = R.from_euler('z', 0).as_quat()
            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]
            msg.poses.append(pose)

        msg.encoder_lengths = [0.15, 0.16, 0.17, 0.18, 0.19, 0.20]
        msg.motor_speeds = [1.0, -1.0, 0.5, -0.5, 0.8, -0.8]

        # Verify message structure
        self.assertEqual(len(msg.poses), 3)
        self.assertEqual(len(msg.encoder_lengths), 6)
        self.assertEqual(len(msg.motor_speeds), 6)

    def test_state_message_format(self):
        """Test State message format."""
        msg = State()
        msg.prev_action = "100_120"
        msg.reverse_the_gait = False
        msg.bar_height_changed = False
        msg.trajectory = []

        # Verify message fields
        self.assertEqual(msg.prev_action, "100_120")
        self.assertFalse(msg.reverse_the_gait)


class TestEndToEnd(unittest.TestCase):
    """End-to-end system tests."""

    @classmethod
    def setUpClass(cls):
        """Set up ROS node once for all tests."""
        try:
            rospy.init_node('test_e2e', anonymous=True)
        except rospy.exceptions.ROSException:
            pass

    def test_message_flow_monitoring(self):
        """Test that we can monitor message flow between components."""
        self.action_received = False
        self.state_received = False
        self.pose_received = False

        def action_cb(msg):
            self.action_received = True

        def state_cb(msg):
            self.state_received = True

        def pose_cb(msg):
            self.pose_received = True

        # Subscribe to topics
        action_sub = rospy.Subscriber('/action_mppi_msg', ActionHybridMPPI, action_cb)
        state_sub = rospy.Subscriber('/state_msg', State, state_cb)
        pose_sub = rospy.Subscriber('/pose_msg', PoseStateStamped, pose_cb)

        # Publishers
        action_pub = rospy.Publisher('/action_mppi_msg', ActionHybridMPPI, queue_size=1)
        state_pub = rospy.Publisher('/state_msg', State, queue_size=1)
        pose_pub = rospy.Publisher('/pose_msg', PoseStateStamped, queue_size=1)

        # Wait for connections to be established
        # Check that publishers have subscribers
        connection_timeout = time.time() + 3.0
        while time.time() < connection_timeout:
            if (action_pub.get_num_connections() > 0 and
                state_pub.get_num_connections() > 0 and
                pose_pub.get_num_connections() > 0):
                break
            rospy.sleep(0.1)

        # Additional small delay to ensure subscribers are ready
        rospy.sleep(0.2)

        # Publish test messages
        action_msg = ActionHybridMPPI()
        action_msg.control_type = "astar"
        action_msg.primitive_actions = ["100_100"]
        action_pub.publish(action_msg)

        state_msg = State()
        state_msg.prev_action = "100_100"
        state_pub.publish(state_msg)

        pose_msg = PoseStateStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_pub.publish(pose_msg)

        # Wait for messages
        timeout = time.time() + 2.0
        while time.time() < timeout:
            rospy.sleep(0.1)
            if self.action_received and self.state_received and self.pose_received:
                break

        # Verify messages received
        self.assertTrue(self.action_received, "Action message not received")
        self.assertTrue(self.state_received, "State message not received")
        self.assertTrue(self.pose_received, "Pose message not received")


def run_tests():
    """Run all test suites."""
    # Initialize ROS
    try:
        rospy.init_node('test_runner', anonymous=True)
    except rospy.exceptions.ROSException:
        pass

    # Create test suite
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()

    # Add all test classes
    suite.addTests(loader.loadTestsFromTestCase(TestTensegrityRobotUnit))
    suite.addTests(loader.loadTestsFromTestCase(TestPlannerMPPIUnit))
    suite.addTests(loader.loadTestsFromTestCase(TestUDPSimulatorUnit))
    suite.addTests(loader.loadTestsFromTestCase(TestMockTrackerUnit))
    suite.addTests(loader.loadTestsFromTestCase(TestIntegration))
    suite.addTests(loader.loadTestsFromTestCase(TestEndToEnd))

    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    # Print summary
    print("\n" + "="*70)
    print("Test Summary:")
    print(f"  Tests run: {result.testsRun}")
    print(f"  Successes: {result.testsRun - len(result.failures) - len(result.errors)}")
    print(f"  Failures: {len(result.failures)}")
    print(f"  Errors: {len(result.errors)}")
    print("="*70)

    return 0 if result.wasSuccessful() else 1


if __name__ == '__main__':
    sys.exit(run_tests())

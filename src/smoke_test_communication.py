#!/usr/bin/env python3
"""
Simple smoke test for Hybrid MPPI communication flow.

This test verifies that the four core components can communicate:
1. MuJoCo UDP Simulator (tensegrity_udp_simulator.py)
2. Robot Controller (run_tensegrity_hybrid_mppi_simulator.py)
3. Mock Tracking Service (mock_tracking_service.py)
4. MPPI Planner (mppi_planner.py)

Communication Flow:
    Simulator --UDP--> Controller --/control_msg--> Tracker --/pose_msg--> Planner --/action_mppi_msg--> Controller

Usage:
    python src/smoke_test_communication.py

Requirements:
    All four components must be running in separate terminals before running this test.
"""

import os
import sys
import time
import socket
import json
import threading
from collections import defaultdict

import rospy
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R

try:
    from tensegrity.msg import ActionHybridMPPI, PoseStateStamped, State, TensegrityStamped
except ImportError:
    print("ERROR: Cannot import tensegrity messages.")
    print("Source the catkin workspace first:")
    print("  source ~/catkin_ws_tensegrity/devel/setup.bash")
    sys.exit(1)


class CommunicationMonitor:
    """Monitor communication between all system components."""

    def __init__(self):
        self.messages_received = defaultdict(int)
        self.latest_messages = {}
        self.lock = threading.Lock()

    def start(self):
        """Initialize ROS subscribers."""
        rospy.init_node('smoke_test_communication', anonymous=True)

        # Subscribe to all key topics
        rospy.Subscriber('/control_msg', TensegrityStamped, self._on_control_msg)
        rospy.Subscriber('/pose_msg', PoseStateStamped, self._on_pose_msg)
        rospy.Subscriber('/action_mppi_msg', ActionHybridMPPI, self._on_action_msg)
        rospy.Subscriber('/state_msg', State, self._on_state_msg)

        print("✓ ROS subscribers initialized")

    def _on_control_msg(self, msg):
        with self.lock:
            self.messages_received['control_msg'] += 1
            self.latest_messages['control_msg'] = msg

    def _on_pose_msg(self, msg):
        with self.lock:
            self.messages_received['pose_msg'] += 1
            self.latest_messages['pose_msg'] = msg

    def _on_action_msg(self, msg):
        with self.lock:
            self.messages_received['action_msg'] += 1
            self.latest_messages['action_msg'] = msg

    def _on_state_msg(self, msg):
        with self.lock:
            self.messages_received['state_msg'] += 1
            self.latest_messages['state_msg'] = msg

    def get_counts(self):
        with self.lock:
            return dict(self.messages_received)

    def get_latest(self, topic):
        with self.lock:
            return self.latest_messages.get(topic)


def test_simulator_udp(host='127.0.0.1', motor_port=2390, pose_port=2391):
    """Test UDP communication with simulator."""
    print("\n[1/4] Testing Simulator UDP Communication...")

    # Test motor command port (send-only, no response expected)
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(2.0)
        test_cmd = "0 0 0 0 0 0 0 0 0"
        sock.sendto(test_cmd.encode('utf-8'), (host, motor_port))
        sock.close()
        print(f"  ✓ Motor command port ({motor_port}): Command sent (no response expected)")
        motor_ok = True
    except Exception as e:
        print(f"  ✗ Motor command port ({motor_port}): Error - {e}")
        sock.close()
        motor_ok = False

    # Test pose query port
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(2.0)
        sock.sendto("GET_POSE".encode('utf-8'), (host, pose_port))

        try:
            data, addr = sock.recvfrom(4096)
            response = json.loads(data.decode('utf-8'))
            if response.get('status') == 'ok':
                print(f"  ✓ Pose query port ({pose_port}): OK - {len(response.get('rods', []))} rods")
                pose_ok = True
            else:
                print(f"  ✗ Pose query port ({pose_port}): Error in response")
                pose_ok = False
            sock.close()
        except socket.timeout:
            print(f"  ✗ Pose query port ({pose_port}): No response")
            sock.close()
            pose_ok = False
    except Exception as e:
        print(f"  ✗ Pose query port ({pose_port}): Error - {e}")
        pose_ok = False

    return motor_ok and pose_ok


def test_ros_topics():
    """Test that all required ROS topics exist."""
    print("\n[2/4] Checking ROS Topics...")

    # First check if roscore is running
    try:
        rospy.get_master().getPid()
        print("  ✓ ROS Master (roscore) is running")
    except Exception:
        print("  ✗ ROS Master (roscore) is NOT running!")
        print("    Start roscore in a separate terminal first")
        return False

    required_topics = [
        '/control_msg',
        '/pose_msg',
        '/action_mppi_msg',
        '/state_msg'
    ]

    all_topics = rospy.get_published_topics()
    topic_names = [name for name, _ in all_topics]

    all_ok = True
    for topic in required_topics:
        if topic in topic_names:
            print(f"  ✓ {topic}: Published")
        else:
            print(f"  ✗ {topic}: NOT published")
            all_ok = False

    return all_ok


def test_message_flow(monitor, timeout=15.0):
    """Test that messages flow through the entire system."""
    print(f"\n[3/4] Testing Message Flow (timeout: {timeout}s)...")

    required_messages = ['control_msg', 'pose_msg']
    optional_messages = ['action_msg', 'state_msg']

    start_time = time.time()
    last_report = start_time

    print("  Waiting for messages...")

    while (time.time() - start_time) < timeout:
        counts = monitor.get_counts()

        # Progress update every 3 seconds
        if time.time() - last_report > 3.0:
            print(f"    Progress: control_msg={counts.get('control_msg', 0)}, "
                  f"pose_msg={counts.get('pose_msg', 0)}, "
                  f"action_msg={counts.get('action_msg', 0)}, "
                  f"state_msg={counts.get('state_msg', 0)}")
            last_report = time.time()

        # Check if we've received required messages
        if all(counts.get(msg, 0) > 0 for msg in required_messages):
            print("\n  ✓ Required messages received!")
            break

        rospy.sleep(0.1)

    # Final report
    final_counts = monitor.get_counts()
    print("\n  Message counts:")
    for topic in required_messages + optional_messages:
        count = final_counts.get(topic, 0)
        status = "✓" if count > 0 else "✗"
        required = "(required)" if topic in required_messages else "(optional)"
        print(f"    {status} {topic}: {count} {required}")

    # Check if all required messages were received
    all_required_ok = all(final_counts.get(msg, 0) > 0 for msg in required_messages)
    return all_required_ok


def test_message_content(monitor):
    """Test that message contents are valid."""
    print("\n[4/4] Validating Message Content...")

    all_ok = True

    # Check control message
    control_msg = monitor.get_latest('control_msg')
    if control_msg:
        print(f"  ✓ Control message: {len(control_msg.motors)} motors, "
              f"{len(control_msg.sensors)} sensors, {len(control_msg.imus)} IMUs")
        if len(control_msg.motors) != 6:
            print(f"    ⚠ Expected 6 motors, got {len(control_msg.motors)}")
            all_ok = False
    else:
        print("  ✗ No control message received")
        all_ok = False

    # Check pose message
    pose_msg = monitor.get_latest('pose_msg')
    if pose_msg:
        print(f"  ✓ Pose message: {len(pose_msg.poses)} poses, "
              f"{len(pose_msg.encoder_lengths)} encoder lengths")
        if len(pose_msg.poses) != 3:
            print(f"    ⚠ Expected 3 poses, got {len(pose_msg.poses)}")
            all_ok = False
    else:
        print("  ✗ No pose message received")
        all_ok = False

    # Check action message (optional)
    action_msg = monitor.get_latest('action_msg')
    if action_msg:
        print(f"  ✓ Action message: type={action_msg.control_type}")
    else:
        print("  ℹ No action message (planner may not be running)")

    # Check state message (optional)
    state_msg = monitor.get_latest('state_msg')
    if state_msg:
        print(f"  ✓ State message: prev_action={state_msg.prev_action}")
    else:
        print("  ℹ No state message (controller may not have completed an action)")

    return all_ok


def print_header():
    """Print test header."""
    print("\n" + "="*70)
    print("SMOKE TEST: Hybrid MPPI Communication Flow")
    print("="*70)
    print("\nThis test verifies communication between:")
    print("  1. MuJoCo UDP Simulator")
    print("  2. Robot Controller")
    print("  3. Mock Tracking Service")
    print("  4. MPPI Planner (optional)")


def print_instructions():
    """Print instructions for running components."""
    print("\nBEFORE RUNNING THIS TEST, start each component in a separate terminal:\n")
    print("Terminal 1 - ROS Master (REQUIRED):")
    print("  roscore\n")
    print("Terminal 2 - Simulator:")
    print("  cd ~/research/tensegrity/tensegrity")
    print("  python src/mujoco_simulator/tensegrity_udp_simulator.py \\")
    print("    src/mujoco_simulator/xml_models/3bar_new_platform_all_cables.xml --no-viz\n")
    print("Terminal 3 - Mock Tracker:")
    print("  cd ~/research/tensegrity/tensegrity")
    print("  source ~/catkin_ws_tensegrity/devel/setup.bash")
    print("  python src/perception/scripts/mock_tracking_service.py\n")
    print("Terminal 4 - Controller:")
    print("  cd ~/research/tensegrity/tensegrity")
    print("  source ~/catkin_ws_tensegrity/devel/setup.bash")
    print("  python src/run_tensegrity_hybrid_mppi_simulator.py\n")
    print("Terminal 5 - Planner (optional, for full test):")
    print("  cd ~/research/tensegrity/tensegrity")
    print("  source ~/catkin_ws_tensegrity/devel/setup.bash")
    print("  python src/mppi_planner.py\n")
    print("Terminal 6 - This Test:")
    print("  cd ~/research/tensegrity/tensegrity")
    print("  source ~/catkin_ws_tensegrity/devel/setup.bash")
    print("  python src/smoke_test_communication.py\n")
    print("="*70)


def print_summary(results):
    """Print test summary."""
    print("\n" + "="*70)
    print("TEST SUMMARY")
    print("="*70)

    all_passed = all(results.values())

    for test_name, passed in results.items():
        status = "✓ PASS" if passed else "✗ FAIL"
        print(f"  {status}: {test_name}")

    print("\n" + "="*70)
    if all_passed:
        print("RESULT: ✓✓✓ ALL TESTS PASSED ✓✓✓")
        print("="*70)
        print("\nThe communication pipeline is working correctly!")
        return 0
    else:
        print("RESULT: ✗✗✗ SOME TESTS FAILED ✗✗✗")
        print("="*70)
        print("\nTroubleshooting:")
        if not results.get('Simulator UDP'):
            print("  - Check that the simulator is running")
            print("  - Verify XML model path is correct")
        if not results.get('ROS Topics'):
            print("  - Check that controller and tracker are running")
            print("  - Verify ROS environment is sourced")
        if not results.get('Message Flow'):
            print("  - Check component logs for errors")
            print("  - Verify all components are properly initialized")
        return 1


def main():
    """Run smoke test."""
    print_header()
    print_instructions()

    try:
        response = input("\n✓ Have you started all required components? (y/n): ")
        if response.lower() != 'y':
            print("\nPlease start the components first, then run this test again.")
            return 1
    except (KeyboardInterrupt, EOFError):
        print("\nTest cancelled.")
        return 1

    print("\nStarting smoke test...")

    results = {}

    try:
        # Initialize monitor
        monitor = CommunicationMonitor()
        monitor.start()

        # Give ROS time to initialize
        time.sleep(2.0)

        # Run tests
        results['Simulator UDP'] = test_simulator_udp()
        results['ROS Topics'] = test_ros_topics()
        results['Message Flow'] = test_message_flow(monitor, timeout=15.0)
        results['Message Content'] = test_message_content(monitor)

        # Print summary
        return print_summary(results)

    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        return 1
    except Exception as e:
        print(f"\n\nTest failed with exception: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())

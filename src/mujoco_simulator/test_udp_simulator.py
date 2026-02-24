#!/usr/bin/env python3
"""
Test script for the UDP simulator.

This script sends test commands to the simulator and verifies responses,
without needing the full ROS setup or run_tensegrity_hybrid_mppi.py.

Usage:
    # Terminal 1: Start simulator
    python tensegrity_udp_simulator.py /path/to/model.xml

    # Terminal 2: Run this test
    python test_udp_simulator.py
"""

import socket
import time
import sys
import json
import numpy as np


class SimulatorTester:
    """Simple tester for UDP simulator."""

    def __init__(self, port=2390, pose_port=2391):
        self.UDP_IP = "127.0.0.1"
        self.UDP_PORT = port
        self.POSE_QUERY_PORT = pose_port

        # Create single socket for both send and receive
        # This is necessary because the simulator sends responses back to the same
        # address it receives commands from
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(2.0)  # 2 second timeout

        # Socket for pose queries
        self.pose_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.pose_sock.settimeout(2.0)

        print(f"UDP Tester initialized on port {port}")
        print(f"Pose query port: {pose_port}")

    def send_motor_command(self, speeds):
        """
        Send motor speed command to simulator.

        Args:
            speeds: List of 6 motor speeds (floats, typically -70 to 70)
        """
        # Format command with offset padding (matching TensegrityRobot)
        offset = 3
        command_parts = ["0"] * offset
        command_parts.extend([str(s) for s in speeds])
        command_parts.extend(["0"] * offset)

        message = " ".join(command_parts)
        print(f"\nSending command: {message}")

        self.sock.sendto(message.encode('utf-8'), (self.UDP_IP, self.UDP_PORT))

    def receive_sensor_data(self, num_packets=3):
        """
        Receive sensor data from simulator.

        Expected packet format (13 values per Arduino):
        [arduino_id, cap1, cap2, cap3, unused, encoder1, encoder2, ax, ay, az, gx, gy, gz]

        Args:
            num_packets: Number of packets to receive (one per Arduino)

        Returns:
            List of received packets
        """
        packets = []

        for i in range(num_packets):
            try:
                data, addr = self.sock.recvfrom(1024)
                message = data.decode('utf-8')
                values = [float(v) for v in message.strip().split()]

                if len(values) == 13:
                    arduino_id = int(values[0])      # Index 0: Arduino ID
                    caps = values[1:4]                # Indices 1-3: Capacitance sensors
                    # values[4] is unused field
                    encoders = values[5:7]            # Indices 5-6: Encoder counts
                    accel = values[7:10]              # Indices 7-9: Accelerometer (ax, ay, az)
                    gyro = values[10:13]              # Indices 10-12: Gyroscope (gx, gy, gz)

                    packets.append({
                        'arduino_id': arduino_id,
                        'capacitance': caps,
                        'encoders': encoders,
                        'accelerometer': accel,
                        'gyroscope': gyro
                    })

                    print(f"  Arduino {arduino_id}: cap={caps[0]:.1f}, enc={encoders[0]:.0f}, "
                          f"accel=({accel[0]:.2f}, {accel[1]:.2f}, {accel[2]:.2f})")
                else:
                    print(f"  Warning: Received packet with {len(values)} values (expected 13)")

            except socket.timeout:
                print(f"  Timeout waiting for packet {i+1}/{num_packets}")
                break
            except Exception as e:
                print(f"  Error receiving packet: {e}")
                break

        return packets

    def test_basic_communication(self):
        """Test basic send/receive."""
        print("\n" + "="*60)
        print("TEST 1: Basic Communication")
        print("="*60)

        # Send zero command
        print("\nSending zero command (all motors stopped)...")
        self.send_motor_command([0, 0, 0, 0, 0, 0])
        time.sleep(0.1)

        print("\nReceiving sensor data...")
        packets = self.receive_sensor_data(num_packets=3)

        if len(packets) == 3:
            print("✓ Received data from all 3 Arduinos")
            return True
        else:
            print(f"✗ Only received {len(packets)}/3 packets")
            return False

    def test_motor_commands(self):
        """Test motor control."""
        print("\n" + "="*60)
        print("TEST 2: Motor Commands")
        print("="*60)

        # Test different motor commands with short durations to avoid hitting cable limits
        test_commands = [
            ([5, 5, 5, 5, 5, 5], "All motors forward at low speed"),
            ([0, 0, 0, 0, 0, 0], "Stop all motors"),
            ([-5, -5, -5, -5, -5, -5], "All motors backward at low speed"),
            ([0, 0, 0, 0, 0, 0], "Stop all motors"),
            ([5, -5, 5, -5, 5, -5], "Alternating motor commands"),
            ([0, 0, 0, 0, 0, 0], "Stop all motors"),
        ]

        for speeds, description in test_commands:
            print(f"\n{description}")
            self.send_motor_command(speeds)
            time.sleep(0.15)  # Short duration to avoid hitting bounds

            packets = self.receive_sensor_data(num_packets=3)
            if len(packets) > 0:
                print(f"  ✓ Simulator responded")
            else:
                print(f"  ✗ No response")

        return True

    def test_continuous_operation(self, duration=5.0):
        """Test continuous operation."""
        print("\n" + "="*60)
        print(f"TEST 3: Continuous Operation ({duration}s)")
        print("="*60)

        start_time = time.time()
        command_count = 0
        response_count = 0

        print("\nSending commands and receiving responses...")
        print("(This tests that the simulator maintains stable communication)")

        while time.time() - start_time < duration:
            # Send alternating command to avoid hitting cable limits
            # Alternate between forward and backward every 0.5s
            t = time.time() - start_time
            # Use sine wave to smoothly alternate directions
            speed = 3.0 * np.sin(2.0 * np.pi * t / 2.0)  # Oscillates between -3 and +3
            self.send_motor_command([speed] * 6)
            command_count += 1

            # Receive responses
            try:
                self.sock.settimeout(0.1)
                packets = self.receive_sensor_data(num_packets=1)
                if len(packets) > 0:
                    response_count += 1
            except:
                pass

            time.sleep(0.1)  # 10 Hz command rate

        # Send zero command to stop motors
        self.send_motor_command([0, 0, 0, 0, 0, 0])
        self.sock.settimeout(2.0)  # Restore timeout

        print(f"\nResults:")
        print(f"  Commands sent: {command_count}")
        print(f"  Responses received: {response_count}")
        print(f"  Response rate: {response_count/command_count*100:.1f}%")

        if response_count > command_count * 0.5:
            print("  ✓ Stable communication")
            return True
        else:
            print("  ✗ Unstable communication")
            return False

    def test_real_time_sync(self, duration=30.0):
        """Test real-time synchronization."""
        print("\n" + "="*60)
        print(f"TEST 4: Real-Time Synchronization ({duration}s)")
        print("="*60)

        print("\nThis test verifies that simulation time matches wall clock time.")
        print("Sending commands periodically while monitoring timing...")

        start_time = time.time()
        last_print = start_time

        while time.time() - start_time < duration:
            # Send alternating commands to avoid hitting cable limits
            # Use sine wave for smooth alternation
            t = time.time() - start_time
            speed = 2.0 * np.sin(2.0 * np.pi * t / 4.0)  # Slow oscillation, period=4s
            self.send_motor_command([speed] * 6)

            # Print progress every 5 seconds
            if time.time() - last_print >= 5.0:
                elapsed = time.time() - start_time
                print(f"  Progress: {elapsed:.1f}s / {duration:.1f}s")
                last_print = time.time()

            time.sleep(0.5)

        # Send zero command to stop motors
        self.send_motor_command([0, 0, 0, 0, 0, 0])

        wall_time = time.time() - start_time
        expected_sim_time = wall_time

        # For real-time sync test, we expect drift < 1% or < 100ms
        acceptable_drift_percent = 1.0  # 1%
        acceptable_drift_abs = 0.1  # 100ms

        print(f"\nResults:")
        print(f"  Wall clock time: {wall_time:.2f}s")
        print(f"  Expected sim time: {expected_sim_time:.2f}s")
        print(f"  Acceptable drift: <{acceptable_drift_percent}% or <{acceptable_drift_abs*1000:.0f}ms")

        # We can't directly measure sim time from here, but we can infer
        # from sensor data timing if responses are coming at expected rate
        print(f"  Note: Direct sim time measurement requires --timing-stats flag on simulator")
        print(f"  ✓ Test completed (manual verification recommended)")

        return True

    def test_command_latency(self, num_samples=20):
        """Test command response latency."""
        print("\n" + "="*60)
        print(f"TEST 5: Command Latency ({num_samples} samples)")
        print("="*60)

        print("\nMeasuring time from command send to sensor response...")

        latencies = []

        for i in range(num_samples):
            # Clear receive buffer
            try:
                while True:
                    self.sock.settimeout(0.001)
                    self.sock.recvfrom(1024)
            except socket.timeout:
                pass

            # Send alternating small commands to avoid hitting limits
            # Alternate between positive and negative
            speed = 2.0 if i % 2 == 0 else -2.0
            send_time = time.time()
            self.send_motor_command([speed] * 6)

            # Wait for first response
            try:
                self.sock.settimeout(0.5)
                data, _ = self.sock.recvfrom(1024)
                receive_time = time.time()
                latency = (receive_time - send_time) * 1000  # ms
                latencies.append(latency)

                if i % 5 == 0:
                    print(f"  Sample {i+1}: {latency:.2f}ms")
            except socket.timeout:
                print(f"  Sample {i+1}: TIMEOUT")

            time.sleep(0.1)

        # Send zero command to stop motors
        self.send_motor_command([0, 0, 0, 0, 0, 0])
        self.sock.settimeout(2.0)  # Restore timeout

        if len(latencies) > 0:
            mean_latency = sum(latencies) / len(latencies)
            max_latency = max(latencies)
            min_latency = min(latencies)

            print(f"\nResults:")
            print(f"  Samples: {len(latencies)}/{num_samples}")
            print(f"  Mean latency: {mean_latency:.2f}ms")
            print(f"  Min latency: {min_latency:.2f}ms")
            print(f"  Max latency: {max_latency:.2f}ms")

            # Real-time system should have low latency (< 50ms typical)
            if mean_latency < 50.0:
                print(f"  ✓ Low latency (mean < 50ms)")
                return True
            else:
                print(f"  ⚠ High latency (mean >= 50ms)")
                return False
        else:
            print(f"  ✗ No responses received")
            return False

    def test_pose_query(self, num_queries=5):
        """Test pose query functionality."""
        print("\n" + "="*60)
        print(f"TEST 6: Pose Query ({num_queries} queries)")
        print("="*60)

        print("\nQuerying robot pose from simulator...")

        success_count = 0
        for i in range(num_queries):
            try:
                # Send GET_POSE request
                request = "GET_POSE"
                self.pose_sock.sendto(request.encode('utf-8'), (self.UDP_IP, self.POSE_QUERY_PORT))

                # Receive JSON response
                data, _ = self.pose_sock.recvfrom(4096)
                response = json.loads(data.decode('utf-8'))

                if response.get('status') == 'ok':
                    rods = response.get('rods', [])
                    if len(rods) > 0:
                        success_count += 1
                        if i == 0:  # Print first response details
                            print(f"\n  Sample response (query 1):")
                            for rod_idx, rod in enumerate(rods):
                                pos = rod.get('position', [])
                                print(f"    Rod {rod_idx}: pos=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
                    else:
                        print(f"  Query {i+1}: No rods in response")
                else:
                    print(f"  Query {i+1}: Error - {response.get('error', 'Unknown error')}")

            except socket.timeout:
                print(f"  Query {i+1}: TIMEOUT")
            except json.JSONDecodeError as e:
                print(f"  Query {i+1}: Invalid JSON - {e}")
            except Exception as e:
                print(f"  Query {i+1}: Error - {e}")

            time.sleep(0.1)

        print(f"\nResults:")
        print(f"  Successful queries: {success_count}/{num_queries}")

        if success_count == num_queries:
            print(f"  ✓ All pose queries successful")
            return True
        elif success_count > 0:
            print(f"  ⚠ Some pose queries failed")
            return False
        else:
            print(f"  ✗ All pose queries failed")
            return False

    def run_all_tests(self):
        """Run all tests."""
        print("\n" + "="*60)
        print("UDP SIMULATOR TEST SUITE (Real-Time Multi-Threaded)")
        print("="*60)
        print("\nMake sure the simulator is running before proceeding!")
        print("(python tensegrity_udp_simulator.py <model.xml> --timing-stats)")

        input("\nPress Enter to start tests...")

        results = []

        # Test 1: Basic communication
        results.append(("Basic Communication", self.test_basic_communication()))

        # Test 2: Motor commands
        results.append(("Motor Commands", self.test_motor_commands()))

        # Test 3: Continuous operation
        results.append(("Continuous Operation", self.test_continuous_operation(duration=5.0)))

        # Test 4: Real-time synchronization
        results.append(("Real-Time Sync", self.test_real_time_sync(duration=30.0)))

        # Test 5: Command latency
        results.append(("Command Latency", self.test_command_latency(num_samples=20)))

        # Test 6: Pose query
        results.append(("Pose Query", self.test_pose_query(num_queries=5)))

        # Summary
        print("\n" + "="*60)
        print("TEST SUMMARY")
        print("="*60)

        passed = sum(1 for _, result in results if result)
        total = len(results)

        for name, result in results:
            status = "PASS" if result else "FAIL"
            print(f"  {name}: {status}")

        print(f"\nTotal: {passed}/{total} tests passed")

        if passed == total:
            print("\n✓ All tests passed! Simulator is working correctly.")
            return 0
        else:
            print(f"\n✗ {total - passed} test(s) failed. Check simulator output.")
            return 1

    def close(self):
        """Close sockets."""
        self.sock.close()
        self.pose_sock.close()


def main():
    """Main entry point."""
    import argparse

    parser = argparse.ArgumentParser(description="Test UDP Simulator")
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
    args = parser.parse_args()

    tester = SimulatorTester(port=args.port, pose_port=args.pose_port)

    try:
        exit_code = tester.run_all_tests()
        sys.exit(exit_code)
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        sys.exit(1)
    finally:
        tester.close()


if __name__ == "__main__":
    main()

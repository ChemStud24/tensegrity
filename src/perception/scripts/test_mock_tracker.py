#!/usr/bin/env python3
"""
Test script for mock tracking service and UDP simulator pose query.

This script tests the integration between the mock tracking service
and the MuJoCo UDP simulator's pose query functionality.

Usage:
    1. Start the MuJoCo simulator: python tensegrity_udp_simulator.py
    2. Run this test: python test_mock_tracker.py
"""

import socket
import json
import time
import numpy as np


def test_pose_query(host='127.0.0.1', port=2391, num_queries=10):
    """
    Test pose query functionality by sending GET_POSE requests to simulator.

    Args:
        host: Simulator host address
        port: Pose query port (default: 2391)
        num_queries: Number of queries to test
    """
    print(f"Testing pose query to {host}:{port}")
    print(f"Sending {num_queries} GET_POSE requests...\n")

    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)  # 1 second timeout

    success_count = 0
    latencies = []

    for i in range(num_queries):
        try:
            # Send GET_POSE request
            start_time = time.time()
            sock.sendto(b"GET_POSE", (host, port))

            # Receive response
            data, _ = sock.recvfrom(4096)
            latency = (time.time() - start_time) * 1000  # ms
            latencies.append(latency)

            # Parse JSON response
            response = json.loads(data.decode('utf-8'))

            if response.get('status') == 'ok':
                success_count += 1
                print(f"Query {i+1}/{num_queries}: SUCCESS (latency: {latency:.2f}ms)")

                # Print first response in detail
                if i == 0:
                    print("\nFirst response details:")
                    print(f"  Number of rods: {len(response['rods'])}")
                    for j, rod in enumerate(response['rods']):
                        pos = rod['position']
                        ori = rod['orientation']
                        print(f"  Rod {j}:")
                        print(f"    Position: [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}] m")
                        print(f"    Orientation (quat): [{ori[0]:.4f}, {ori[1]:.4f}, {ori[2]:.4f}, {ori[3]:.4f}]")
                    print()
            else:
                print(f"Query {i+1}/{num_queries}: ERROR - {response.get('error', 'unknown')}")

        except socket.timeout:
            print(f"Query {i+1}/{num_queries}: TIMEOUT")
        except Exception as e:
            print(f"Query {i+1}/{num_queries}: ERROR - {e}")

        # Small delay between queries
        time.sleep(0.1)

    sock.close()

    # Print statistics
    print("\n" + "="*60)
    print("Test Summary:")
    print("="*60)
    print(f"Total queries: {num_queries}")
    print(f"Successful: {success_count}")
    print(f"Failed: {num_queries - success_count}")
    print(f"Success rate: {100.0 * success_count / num_queries:.1f}%")

    if latencies:
        print(f"\nLatency statistics:")
        print(f"  Mean: {np.mean(latencies):.2f}ms")
        print(f"  Std: {np.std(latencies):.2f}ms")
        print(f"  Min: {np.min(latencies):.2f}ms")
        print(f"  Max: {np.max(latencies):.2f}ms")


def test_noisy_poses(host='127.0.0.1', port=2391, num_samples=100, noise_std=0.01):
    """
    Test that adding noise to poses produces reasonable variations.

    Args:
        host: Simulator host address
        port: Pose query port
        num_samples: Number of samples to collect
        noise_std: Standard deviation of noise to add
    """
    print(f"\nTesting pose noise (std={noise_std}m)...")
    print(f"Collecting {num_samples} samples...\n")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)

    # Collect ground truth pose
    sock.sendto(b"GET_POSE", (host, port))
    data, _ = sock.recvfrom(4096)
    response = json.loads(data.decode('utf-8'))

    if response.get('status') != 'ok':
        print("Failed to get ground truth pose")
        return

    gt_pos = np.array(response['rods'][0]['position'])
    print(f"Ground truth position: [{gt_pos[0]:.4f}, {gt_pos[1]:.4f}, {gt_pos[2]:.4f}]")

    # Simulate noisy measurements
    noisy_positions = []
    for _ in range(num_samples):
        noise = np.random.normal(0, noise_std, 3)
        noisy_pos = gt_pos + noise
        noisy_positions.append(noisy_pos)

    noisy_positions = np.array(noisy_positions)

    # Compute statistics
    mean_pos = np.mean(noisy_positions, axis=0)
    std_pos = np.std(noisy_positions, axis=0)
    errors = np.linalg.norm(noisy_positions - gt_pos, axis=1)

    print(f"\nNoisy measurements statistics:")
    print(f"  Mean position: [{mean_pos[0]:.4f}, {mean_pos[1]:.4f}, {mean_pos[2]:.4f}]")
    print(f"  Std per axis: [{std_pos[0]:.4f}, {std_pos[1]:.4f}, {std_pos[2]:.4f}]")
    print(f"  Mean error: {np.mean(errors):.4f}m")
    print(f"  Max error: {np.max(errors):.4f}m")
    print(f"  Expected std: {noise_std:.4f}m")

    sock.close()


def main():
    """Main test function."""
    import argparse

    parser = argparse.ArgumentParser(description="Test mock tracking service pose queries")
    parser.add_argument("--host", default="127.0.0.1", help="Simulator host")
    parser.add_argument("--port", type=int, default=2391, help="Pose query port")
    parser.add_argument("--queries", type=int, default=10, help="Number of test queries")
    parser.add_argument("--noise-test", action="store_true", help="Run noise test")

    args = parser.parse_args()

    print("="*60)
    print("Mock Tracker Pose Query Test")
    print("="*60)
    print("Make sure the MuJoCo simulator is running!")
    print("  python tensegrity_udp_simulator.py")
    print("="*60 + "\n")

    # Test basic pose queries
    test_pose_query(args.host, args.port, args.queries)

    # Optionally test noise characteristics
    if args.noise_test:
        test_noisy_poses(args.host, args.port)

    print("\nTest complete!")


if __name__ == '__main__':
    main()

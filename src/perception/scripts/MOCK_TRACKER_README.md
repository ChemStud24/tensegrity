# Mock Tracking Service

A drop-in replacement for the vision-based tracking service that uses ground truth poses from the MuJoCo simulator with configurable noise for simulation testing.

## Overview

The mock tracking service provides the same ROS interface as the real tracking service ([tracking_service.py](tracking_service.py)) but instead of processing camera images to estimate robot pose, it queries the MuJoCo UDP simulator for ground truth poses and adds configurable noise to simulate tracking uncertainty.

### Architecture

```
ROS Topics                Mock Tracker              MuJoCo Simulator
   |                           |                           |
   ├─ /rgb_images ────────────>│                           │
   ├─ /depth_images ──────────>│                           │
   └─ /control_msg ───────────>│──── GET_POSE (UDP) ─────>│
                                │<──── Pose + Noise ────────│
                                │                           │
                                ├─ /pose_msg ─────────────>
                                └─ /trajectory_images ────>
```

## Features

- **Drop-in replacement**: Same ROS interface as real tracker (services, topics)
- **Configurable noise**: Add position and orientation noise to simulate tracking uncertainty
- **Low latency**: Direct UDP queries to simulator (~1-5ms)
- **No vision processing**: Bypasses camera image processing for faster simulation
- **Thread-safe**: Asynchronous UDP communication

## Installation

No additional dependencies needed beyond the standard ROS/tensegrity packages.

Files:
- `mock_tracking_service.py` - Main mock tracker implementation
- `test_mock_tracker.py` - Test script for pose queries
- `MOCK_TRACKER_README.md` - This documentation

## Usage

### 1. Start the MuJoCo UDP Simulator

The simulator must be running with pose query support:

```bash
cd src/mujoco_simulator
python tensegrity_udp_simulator.py [xml_path] --pose-port 2391
```

The simulator runs two UDP services:
- Port 2390: Motor commands (from TensegrityRobot)
- Port 2391: Pose queries (from MockTracker)

### 2. Start the Mock Tracking Service

Using rosrun:

```bash
rosrun tensegrity_perception mock_tracking_service.py
```

Or as a standalone script:

```bash
cd src/perception/scripts
python mock_tracking_service.py
```

### 3. Run Your Control Code

Run your control code normally (e.g., `run_tensegrity_hybrid_mppi.py`). The mock tracker will:
1. Subscribe to camera and control topics
2. Query simulator for ground truth pose
3. Add configurable noise
4. Publish PoseStateStamped messages

## Configuration

### ROS Parameters

Set parameters in your launch file or via `rosparam`:

```yaml
# Simulator connection
~simulator_host: "127.0.0.1"        # Simulator host (default: localhost)
~simulator_pose_port: 2391           # Pose query port (default: 2391)

# Noise parameters
~position_noise_std: 0.01            # Position noise std dev in meters (default: 0.01m = 1cm)
~orientation_noise_std: 0.05         # Orientation noise std dev in radians (default: 0.05rad ≈ 3°)

# Data config (same as real tracker)
data_cfg_file: "/path/to/data_cfg.json"
```

### Noise Configuration

Adjust noise levels to simulate different tracking qualities:

| Scenario | Position Std (m) | Orientation Std (rad) | Description |
|----------|------------------|-----------------------|-------------|
| Perfect tracking | 0.0 | 0.0 | Ground truth (no noise) |
| Good tracking | 0.005 | 0.02 | High-quality vision (~5mm, 1°) |
| Normal tracking | 0.01 | 0.05 | Typical performance (~1cm, 3°) |
| Poor tracking | 0.03 | 0.1 | Challenging conditions (~3cm, 6°) |

### Example Launch File

```xml
<launch>
    <!-- Mock tracking service -->
    <node name="mock_tracking_service" pkg="tensegrity_perception"
          type="mock_tracking_service.py" output="screen">

        <!-- Simulator connection -->
        <param name="simulator_host" value="127.0.0.1"/>
        <param name="simulator_pose_port" value="2391"/>

        <!-- Noise parameters (1cm position, 3° orientation) -->
        <param name="position_noise_std" value="0.01"/>
        <param name="orientation_noise_std" value="0.05"/>

        <!-- Data config -->
        <param name="data_cfg_file" value="$(find tensegrity_perception)/config/data_cfg.json"/>
    </node>
</launch>
```

## Testing

### Basic Pose Query Test

Test that the pose query communication works:

```bash
# Terminal 1: Start simulator
python tensegrity_udp_simulator.py

# Terminal 2: Run test
cd src/perception/scripts
python test_mock_tracker.py --queries 10
```

Expected output:
```
Query 1/10: SUCCESS (latency: 2.34ms)
Query 2/10: SUCCESS (latency: 1.89ms)
...
Success rate: 100.0%
Latency: 2.1±0.3ms
```

### Noise Characteristics Test

Verify noise statistics:

```bash
python test_mock_tracker.py --noise-test
```

This samples the pose multiple times with noise and verifies the noise distribution matches the configured parameters.

## ROS Interface

### Published Topics

- `/pose_msg` (PoseStateStamped): Robot pose with encoder and motor data
- `/trajectory_images` (Image): Trajectory visualization (passthrough)

### Subscribed Topics

- `/rgb_images` (Image): RGB camera images (not used, for compatibility)
- `/depth_images` (Image): Depth camera images (not used, for compatibility)
- `/control_msg` (TensegrityStamped): Motor and sensor data

### Services

- `init_tracker` (InitTracker): Initialize tracker (always succeeds for mock)
- `get_pose` (GetPose): Get current robot pose
- `get_bar_height` (GetBarHeight): Get bar height (returns dummy value)

## Comparison with Real Tracker

| Feature | Real Tracker | Mock Tracker |
|---------|--------------|--------------|
| Input | RGB-D images | Simulator state |
| Processing | Vision algorithms (ICP, etc.) | UDP query |
| Latency | ~50-100ms | ~1-5ms |
| Accuracy | Depends on lighting/occlusion | Perfect + noise |
| Hardware | Camera required | Simulator only |
| Use case | Physical robot | Simulation |

## Integration with Existing Code

The mock tracker is a **drop-in replacement** - no changes to control code needed:

```python
# Control code remains unchanged
class TensegrityController:
    def __init__(self):
        # Subscribe to pose topic (works with both trackers)
        rospy.Subscriber('/pose_msg', PoseStateStamped, self.pose_callback)

    def pose_callback(self, msg):
        # Process pose (same for real or mock tracker)
        poses = msg.poses
        # ... use poses for control ...
```

Just swap the tracker node in your launch file or startup script.

## Troubleshooting

### "Simulator pose query timeout"

- Check that the MuJoCo simulator is running
- Verify the pose query port (default: 2391)
- Check firewall settings if running on different machines

### "Failed to get pose from simulator"

- Ensure simulator was started with `--pose-port 2391` (or matching port)
- Check that simulator is fully initialized (wait for "Waiting for TensegrityRobot" message)

### High latency (>10ms)

- Check network connection if using remote simulator
- Verify no heavy background processes
- Consider reducing visualization in simulator (`--no-viz`)

### Noise seems incorrect

- Run `test_mock_tracker.py --noise-test` to verify noise distribution
- Check that `position_noise_std` and `orientation_noise_std` are set correctly
- Remember: noise is in meters and radians, not mm/degrees

## Advanced Usage

### Custom Noise Models

Modify `add_pose_noise()` in `mock_tracking_service.py` to implement custom noise models:

```python
def add_pose_noise(self, poses):
    # Example: Distance-dependent noise
    for pose in poses:
        distance = np.linalg.norm([pose.position.x, pose.position.y, pose.position.z])
        noise_std = self.position_noise_std * (1 + 0.1 * distance)  # Increases with distance
        # ... add noise ...
```

### Logging Ground Truth vs Noisy Poses

For analysis, you can log both:

```python
# In tracking_callback
gt_poses = self.query_simulator_pose()  # Ground truth
noisy_poses = self.add_pose_noise(gt_poses)  # Noisy

# Log both for comparison
rospy.loginfo(f"GT: {gt_poses[0].position}, Noisy: {noisy_poses[0].position}")
```

## Performance

Typical performance on a modern laptop:

- Pose query latency: 1-5ms
- Memory usage: ~50MB (minimal, no image processing)
- CPU usage: <5% (mostly waiting for messages)

Compare to real tracker:
- Pose estimation latency: 50-100ms
- Memory usage: ~500MB (point clouds, meshes)
- CPU usage: 30-50% (vision processing)

## Future Improvements

Potential enhancements:

- [ ] Support for occlusion simulation (drop pose estimates when occluded)
- [ ] Time-correlated noise (Kalman filter simulation)
- [ ] Outlier injection (simulate tracking failures)
- [ ] Multi-camera fusion simulation
- [ ] Latency injection (simulate camera/processing delays)

## License

Same license as the tensegrity project.

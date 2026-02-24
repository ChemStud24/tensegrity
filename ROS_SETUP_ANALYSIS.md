# ROS System Analysis and Fixes

## System Overview

This is a hybrid A*/MPPI control system for a tensegrity robot with 4 main components:

1. **MuJoCo UDP Simulator** (`tensegrity_udp_simulator.py`)
   - Simulates the physical robot and Arduino communication
   - Publishes camera images to ROS
   - Responds to UDP pose queries

2. **Mock Tracking Service** (`mock_tracking_service.py`)
   - Replaces real camera tracking during simulation
   - Queries simulator for ground truth poses (with configurable noise)
   - Publishes `PoseStateStamped` messages

3. **MPPI Planner** (`mppi_planner.py`)
   - Receives pose/state information
   - Computes hybrid A*/MPPI controls
   - Publishes `ActionHybridMPPI` messages

4. **Robot Controller** (`run_tensegrity_hybrid_mppi.py`)
   - Receives action commands
   - Sends motor commands via UDP
   - Publishes control/sensor data

## Data Flow

```
┌─────────────────────────────────────────────────────────────────┐
│  MuJoCo Simulator                                               │
│  - Physics (1000 Hz)                                            │
│  - Publishes: /rgb_images, /depth_images                       │
│  - UDP Servers: :2390 (motor cmds), :2391 (pose queries)      │
└──────────────┬───────────────────────────┬──────────────────────┘
               │ Images                    │ Pose Queries (UDP)
               ▼                           ▼
┌──────────────────────────────────────────────────────────────────┐
│  Mock Tracking Service                                           │
│  - Subscribes: /rgb_images, /depth_images, /control_msg         │
│  - Publishes: /pose_msg (PoseStateStamped)                      │
│  - Services: init_tracker, get_pose, get_bar_height             │
└──────────────┬───────────────────────────────────────────────────┘
               │ /pose_msg
               ▼
┌──────────────────────────────────────────────────────────────────┐
│  MPPI Planner                                                    │
│  - Subscribes: /pose_msg                                         │
│  - Publishes: /action_mppi_msg (ActionHybridMPPI)               │
└──────────────┬───────────────────────────────────────────────────┘
               │ /action_mppi_msg
               ▼
┌──────────────────────────────────────────────────────────────────┐
│  Robot Controller                                                │
│  - Subscribes: /action_mppi_msg                                  │
│  - Publishes: /control_msg (TensegrityStamped), /state_msg     │
│  - UDP Client: sends motor commands to :2390                     │
└──────────────┬───────────────────────────────────────────────────┘
               │ /control_msg (loops back to tracker)
               └─────────────────────────────────────────────────────┘
```

## Critical Issues Found and Fixed

### 1. Missing Method: `_publish_ready_for_next_action()` ✅ FIXED
**File:** `run_tensegrity_hybrid_mppi.py`
**Lines:** Called at 407, 463 but never defined
**Impact:** Runtime error - AttributeError

**Fix:** Added method definition:
```python
def _publish_ready_for_next_action(self, prev_action_str):
    """Publish state message indicating readiness for next action."""
    state_msg = State()
    state_msg.prev_action = prev_action_str
    state_msg.reverse_the_gait = self.reverse_the_gait
    state_msg.bar_height_changed = False
    state_msg.trajectory = []
    self.state_pub.publish(state_msg)
    rospy.loginfo(f"Published ready state with prev_action: {prev_action_str}")
```

### 2. Incorrect Message Format for `primitive_actions` ✅ FIXED
**File:** `mppi_planner.py`
**Line:** 177
**Issue:** `ActionHybridMPPI.primitive_actions` is defined as `string[]` but was being set to a single string
**Impact:** Subscriber would receive incorrect data type

**Fix:** Wrapped primitive action in a list:
```python
# Before:
action_msg.primitive_actions = actions[0] if 'cw' in actions[0] else str(actions[1]) + '_' + str(actions[2])

# After:
prim_action = actions[0] if 'cw' in actions[0] else str(actions[1]) + '_' + str(actions[2])
action_msg.primitive_actions = [prim_action] if isinstance(prim_action, str) else prim_action
```

### 3. Incorrect Encoder Length Accumulation ✅ FIXED
**File:** `mppi_planner.py`
**Line:** 151
**Issue:** Was accumulating encoder_lengths causing unbounded growth: `self.rest_lens += np.vstack(encoder_lengths_list).sum(axis=0)`
**Impact:** Planner would receive incorrect cable lengths over time

**Fix:** Use latest encoder_lengths directly (they are absolute, not deltas):
```python
# Before:
self.rest_lens += np.vstack(encoder_lengths_list).sum(axis=0)

# After:
if len(latest_encoder_lengths) > 0:
    self.rest_lens = np.array(latest_encoder_lengths)
```

### 4. Missing ROS Node Initialization ✅ FIXED
**File:** `tensegrity_udp_simulator.py`
**Lines:** 205-206, 720
**Issue:** Used `rospy.Publisher()` and `rospy.get_rostime()` without calling `rospy.init_node()`
**Impact:** Runtime error when ROS is available

**Fix:** Added ROS node initialization:
```python
if ROS_AVAILABLE:
    # Initialize ROS node if not already initialized
    try:
        rospy.init_node('tensegrity_udp_simulator', anonymous=True)
    except rospy.exceptions.ROSException:
        # Node already initialized
        pass
```

## Message Definitions Verified

### ActionHybridMPPI.msg ✅
```
Header header
string control_type          # 'astar' or 'mppi'
float64[] mppi_actions       # Flattened control matrix
float64 control_interval     # Time per action (for mppi)
string[] primitive_actions   # Primitive gait names (for astar)
```

### PoseStateStamped.msg ✅
```
Header header
geometry_msgs/Pose[] poses      # 3 poses (one per rod)
float64[] encoder_lengths       # Cable lengths in meters
float64[] motor_speeds          # Motor speeds
```

### State.msg ✅
```
geometry_msgs/Point[] trajectory
string prev_action
bool reverse_the_gait
bool bar_height_changed
```

## Topic/Service Connections Verified

| Publisher | Topic | Subscriber | Message Type |
|-----------|-------|------------|--------------|
| Simulator | `/rgb_images` | Mock Tracker | `sensor_msgs/Image` |
| Simulator | `/depth_images` | Mock Tracker | `sensor_msgs/Image` |
| Mock Tracker | `/pose_msg` | MPPI Planner | `PoseStateStamped` |
| MPPI Planner | `/action_mppi_msg` | Robot Controller | `ActionHybridMPPI` |
| Robot Controller | `/control_msg` | Mock Tracker | `TensegrityStamped` |
| Robot Controller | `/state_msg` | (Future use) | `State` |

| Service Provider | Service Name | Client | Service Type |
|-----------------|--------------|--------|--------------|
| Mock Tracker | `init_tracker` | Robot Controller | `InitTracker` |
| Mock Tracker | `get_pose` | Robot Controller | `GetPose` |
| Mock Tracker | `get_bar_height` | (Future use) | `GetBarHeight` |

## UDP Communication Verified

| Component | Address | Protocol | Purpose |
|-----------|---------|----------|---------|
| Simulator | `0.0.0.0:2390` | UDP Server | Receive motor commands |
| Simulator | `0.0.0.0:2391` | UDP Server | Respond to pose queries |
| Robot Controller | `*:2390` | UDP Client | Send motor commands |
| Mock Tracker | `127.0.0.1:2391` | UDP Client | Query robot poses |

## How to Run the System

### Terminal 1: Start ROS Core
```bash
roscore
```

### Terminal 2: Start MuJoCo Simulator
```bash
cd /home/nelsonchen/research/tensegrity/tensegrity
python src/mujoco_simulator/tensegrity_udp_simulator.py xml_models/3bar_new_platform_all_cables.xml
```

### Terminal 3: Start Mock Tracking Service
```bash
cd /home/nelsonchen/research/tensegrity/tensegrity
rosrun tensegrity_perception mock_tracking_service.py
```

### Terminal 4: Start MPPI Planner
```bash
cd /home/nelsonchen/research/tensegrity/tensegrity/src
python mppi_planner.py
```

### Terminal 5: Start Robot Controller
```bash
cd /home/nelsonchen/research/tensegrity/tensegrity/src
python run_tensegrity_hybrid_mppi.py
```

## Remaining Considerations

### 1. Timing and Synchronization
- Physics runs at 1000 Hz
- Sensor broadcast at 20 Hz (configurable)
- Camera at 30 Hz (configurable)
- Mock tracker adds configurable latency
- Consider time synchronization between components

### 2. Coordinate Frames
- Simulator uses 10x scaling internally
- Planner scales dimensions by 10x when calling plan()
- Ensure consistent units (meters vs cm vs mm) across system

### 3. Quaternion Conventions
- ROS Pose: [x, y, z, w]
- MuJoCo internal: [w, x, y, z]
- Planner expects: [w, x, y, z]
- Conversions are implemented correctly

### 4. Error Handling
- Add timeout handling for UDP communication
- Add reconnection logic if components crash
- Consider watchdog for detecting stuck threads

### 5. Configuration
- Consider using ROS parameter server for configuration
- Create launch files for easier multi-component startup
- Add config validation on startup

## Testing Checklist

- [ ] Run end-to-end test: `src/test_end_to_end_hybrid_mppi.py`
- [ ] Verify all ROS topics are publishing
- [ ] Check message frequencies match expected rates
- [ ] Verify UDP communication is working
- [ ] Test both 'astar' and 'mppi' control modes
- [ ] Check that simulator visualization is working
- [ ] Verify pose tracking has reasonable accuracy
- [ ] Test graceful shutdown of all components

## Status

✅ All critical issues have been fixed
✅ Message definitions are aligned
✅ Topic/service connections are correct
✅ UDP communication is properly configured
✅ System is ready for testing

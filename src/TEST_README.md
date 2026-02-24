# Hybrid MPPI Communication Tests

This directory contains tests for verifying the communication pipeline between all components of the Hybrid MPPI tensegrity control system.

## System Architecture

The system consists of four main components that communicate via UDP and ROS:

```
┌─────────────────────┐
│  MuJoCo Simulator   │  (Physics simulation, sensor data)
│  UDP Port 2390,2391 │
└──────────┬──────────┘
           │ UDP
           ↓
┌─────────────────────┐     ROS /control_msg     ┌─────────────────────┐
│  Robot Controller   │─────────────────────────>│   Mock Tracker      │
│  (run_tensegrity_   │                          │  (pose estimation)  │
│   hybrid_mppi_      │<─────────────────────────│                     │
│   simulator.py)     │     ROS /pose_msg        └─────────────────────┘
└──────────┬──────────┘                                     ↑
           │                                                │
           │ ROS /state_msg                                 │ UDP GET_POSE
           ↓                                                │
┌─────────────────────┐                                     │
│   MPPI Planner      │                                     │
│  (mppi_planner.py)  │─────────────────────────────────────┘
└─────────────────────┘
           │
           │ ROS /action_mppi_msg
           ↓
     (back to controller)
```

## Test Files

### 1. smoke_test_communication.py - Smoke Test (Recommended)

A lightweight test that verifies basic communication between all components.

**What it tests:**
- ✓ Simulator UDP ports (motor commands & pose queries)
- ✓ ROS topics existence
- ✓ Message flow through the pipeline
- ✓ Message content validation

**Usage:**
```bash
# Quick start with helper script:
./run_tests.sh

# Or manually in separate terminals:

# Terminal 1: ROS Master (REQUIRED)
roscore

# Terminal 2: Simulator
python src/mujoco_simulator/tensegrity_udp_simulator.py src/mujoco_simulator/xml_models/3bar_new_platform_all_cables.xml --no-viz

# Terminal 3: Mock Tracker
source ~/catkin_ws_tensegrity/devel/setup.bash
python src/perception/scripts/mock_tracking_service.py

# Terminal 4: Controller
source ~/catkin_ws_tensegrity/devel/setup.bash
python src/run_tensegrity_hybrid_mppi_simulator.py

# Terminal 5: Smoke Test
source ~/catkin_ws_tensegrity/devel/setup.bash
python src/smoke_test_communication.py
```

### 2. test_end_to_end_hybrid_mppi.py - Comprehensive Unit Tests

Complete unit and integration tests for all components.

**Usage:**
```bash
source ~/catkin_ws_tensegrity/devel/setup.bash
python src/test_end_to_end_hybrid_mppi.py

# Or use helper:
./run_tests.sh --unit
```

## Quick Start

### Prerequisites

1. Build ROS workspace:
   ```bash
   ./setup_workspace.sh
   source ~/catkin_ws_tensegrity/devel/setup.bash
   ```

2. Install dependencies (MuJoCo, PyTorch, ROS, numpy, scipy, cv2)

### Running Tests

**Easiest: Use the helper script**
```bash
./run_tests.sh
```

## Troubleshooting

### Common Issues

**"Cannot import tensegrity messages"**
```bash
source ~/catkin_ws_tensegrity/devel/setup.bash
```

**"Simulator UDP: No response"**
```bash
# Check if running:
ps aux | grep tensegrity_udp_simulator

# Start it:
python src/mujoco_simulator/tensegrity_udp_simulator.py xml_models/3bar_new_platform_all_cables.xml --no-viz
```

**"No messages received"**
- Check all components are running
- Check topics: `rostopic list`
- Monitor activity: `rostopic hz /control_msg`

## Test Coverage

### Communication Paths Tested

- [x] Simulator → Controller (UDP sensor data)
- [x] Controller → Simulator (UDP motor commands)
- [x] Simulator → Tracker (UDP pose query)
- [x] Controller → Tracker (ROS /control_msg)
- [x] Tracker → Planner (ROS /pose_msg)
- [x] Planner → Controller (ROS /action_mppi_msg)
- [x] Controller → Planner (ROS /state_msg)

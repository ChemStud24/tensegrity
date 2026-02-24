#!/bin/bash
#
# Direct launcher for the Hybrid MPPI Tensegrity Simulation System
# This version runs scripts directly without requiring catkin_make.
#
# Usage:
#   ./launch_simulation_direct.sh                    # Launch with defaults
#   ./launch_simulation_direct.sh --no-viz           # Launch without MuJoCo visualization
#   ./launch_simulation_direct.sh --help             # Show all options
#

set -e

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Default options
VISUALIZE=""
SENSOR_NOISE=""
PHYSICS_RATE="1000.0"
SENSOR_RATE="20.0"
XML_MODEL="${SCRIPT_DIR}/src/mujoco_simulator/xml_models/3bar_new_platform_all_cables.xml"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --no-viz|--headless)
            VISUALIZE="--no-viz"
            shift
            ;;
        --noise|--sensor-noise)
            SENSOR_NOISE="--sensor-noise"
            shift
            ;;
        --physics-rate)
            PHYSICS_RATE="$2"
            shift 2
            ;;
        --sensor-rate)
            SENSOR_RATE="$2"
            shift 2
            ;;
        --xml|--model)
            XML_MODEL="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Direct launcher for the Hybrid MPPI Tensegrity Simulation System"
            echo "(Does not require catkin_make rebuild)"
            echo ""
            echo "Options:"
            echo "  --no-viz, --headless     Disable MuJoCo visualization"
            echo "  --noise, --sensor-noise  Enable realistic sensor noise"
            echo "  --physics-rate RATE      Physics simulation rate in Hz (default: 1000)"
            echo "  --sensor-rate RATE       Sensor broadcast rate in Hz (default: 20)"
            echo "  --xml, --model PATH      Custom MuJoCo XML model path"
            echo "  -h, --help               Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help to see available options"
            exit 1
            ;;
    esac
done

echo "=============================================="
echo "Hybrid MPPI Tensegrity Simulation (Direct)"
echo "=============================================="

# Check if micromamba is available
if ! command -v micromamba &> /dev/null; then
    echo "Error: micromamba not found. Please install micromamba first."
    exit 1
fi

# Activate micromamba environment
echo "[1/3] Activating ros_env environment..."
eval "$(micromamba shell hook --shell bash)"
micromamba activate ros_env

# Source the catkin workspace
CATKIN_WS="${HOME}/catkin_ws_tensegrity"
if [ -f "${CATKIN_WS}/devel/setup.bash" ]; then
    echo "[2/3] Sourcing catkin workspace: ${CATKIN_WS}"
    source "${CATKIN_WS}/devel/setup.bash"
else
    echo "Error: Catkin workspace not found at ${CATKIN_WS}"
    echo "Please run: ./setup_workspace.sh"
    exit 1
fi

# Create a cleanup function
cleanup() {
    echo ""
    echo "Shutting down all processes..."
    kill $SIMULATOR_PID $TRACKER_PID $CONTROLLER_PID $PLANNER_PID 2>/dev/null || true
    wait 2>/dev/null || true
    echo "All processes stopped."
}

# Set up trap to cleanup on exit
trap cleanup EXIT INT TERM

echo "[3/3] Starting ROS nodes..."
echo ""
echo "Press Ctrl+C to stop all nodes"
echo "=============================================="
echo ""

# Check if roscore is running, start if not
if ! rostopic list &>/dev/null; then
    echo "Starting roscore..."
    roscore &
    ROSCORE_PID=$!
    sleep 2
else
    echo "roscore already running"
    ROSCORE_PID=""
fi

# Start MuJoCo UDP Simulator
echo "Starting MuJoCo UDP Simulator..."
python "${SCRIPT_DIR}/src/mujoco_simulator/tensegrity_udp_simulator.py" \
    "${XML_MODEL}" \
    --physics-rate "${PHYSICS_RATE}" \
    --sensor-rate "${SENSOR_RATE}" \
    ${VISUALIZE} ${SENSOR_NOISE} &
SIMULATOR_PID=$!
sleep 2

# Start Mock Tracking Service
echo "Starting Mock Tracking Service..."
python "${SCRIPT_DIR}/src/perception/scripts/mock_tracking_service.py" &
TRACKER_PID=$!
sleep 1

# Start Hybrid MPPI Controller
echo "Starting Hybrid MPPI Controller..."
python "${SCRIPT_DIR}/src/run_tensegrity_hybrid_mppi_simulator.py" &
CONTROLLER_PID=$!
sleep 2

# Start MPPI Planner
echo "Starting MPPI Planner..."
python "${SCRIPT_DIR}/src/mppi_planner.py" &
PLANNER_PID=$!

echo ""
echo "All nodes started:"
echo "  - MuJoCo Simulator (PID: $SIMULATOR_PID)"
echo "  - Mock Tracker (PID: $TRACKER_PID)"
echo "  - Controller (PID: $CONTROLLER_PID)"
echo "  - Planner (PID: $PLANNER_PID)"
echo ""
echo "Waiting for processes... (Ctrl+C to stop)"

# Wait for any process to exit
wait -n $SIMULATOR_PID $TRACKER_PID $CONTROLLER_PID $PLANNER_PID 2>/dev/null || true

# If we get here, something exited
echo ""
echo "A process exited. Cleaning up..."

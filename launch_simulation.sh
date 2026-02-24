#!/bin/bash
#
# One-command launcher for the Hybrid MPPI Tensegrity Simulation System
#
# This script:
# 1. Activates the micromamba ros_env environment
# 2. Sources the catkin workspace
# 3. Launches all ROS nodes via roslaunch
#
# Usage:
#   ./launch_simulation.sh                    # Launch with defaults
#   ./launch_simulation.sh --no-viz           # Launch without MuJoCo visualization
#   ./launch_simulation.sh --noise            # Launch with sensor noise
#   ./launch_simulation.sh --help             # Show all options
#

set -e

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Default options
VISUALIZE="true"
SENSOR_NOISE="false"
PHYSICS_RATE="1000.0"
SENSOR_RATE="20.0"
CAMERA_RATE="30.0"
POSITION_NOISE="0.0"
ORIENTATION_NOISE="0.0"
XML_MODEL=""

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --no-viz|--headless)
            VISUALIZE="false"
            shift
            ;;
        --noise|--sensor-noise)
            SENSOR_NOISE="true"
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
        --camera-rate)
            CAMERA_RATE="$2"
            shift 2
            ;;
        --position-noise)
            POSITION_NOISE="$2"
            shift 2
            ;;
        --orientation-noise)
            ORIENTATION_NOISE="$2"
            shift 2
            ;;
        --xml|--model)
            XML_MODEL="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Launch the Hybrid MPPI Tensegrity Simulation System"
            echo ""
            echo "Options:"
            echo "  --no-viz, --headless     Disable MuJoCo visualization (faster)"
            echo "  --noise, --sensor-noise  Enable realistic sensor noise"
            echo "  --physics-rate RATE      Physics simulation rate in Hz (default: 1000)"
            echo "  --sensor-rate RATE       Sensor broadcast rate in Hz (default: 20)"
            echo "  --camera-rate RATE       Camera publish rate in Hz (default: 30)"
            echo "  --position-noise STD     Position tracking noise std in meters (default: 0.0)"
            echo "  --orientation-noise STD  Orientation tracking noise std in radians (default: 0.0)"
            echo "  --xml, --model PATH      Custom MuJoCo XML model path"
            echo "  -h, --help               Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                       # Launch with defaults (with visualization)"
            echo "  $0 --no-viz              # Launch headless (faster)"
            echo "  $0 --noise               # Launch with sensor noise"
            echo "  $0 --no-viz --noise      # Headless with noise"
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
echo "Hybrid MPPI Tensegrity Simulation Launcher"
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

# Build roslaunch command with arguments
LAUNCH_CMD="roslaunch tensegrity hybrid_mppi_sim.launch"
LAUNCH_CMD+=" visualize:=${VISUALIZE}"
LAUNCH_CMD+=" sensor_noise:=${SENSOR_NOISE}"
LAUNCH_CMD+=" physics_rate:=${PHYSICS_RATE}"
LAUNCH_CMD+=" sensor_rate:=${SENSOR_RATE}"
LAUNCH_CMD+=" camera_rate:=${CAMERA_RATE}"
LAUNCH_CMD+=" position_noise_std:=${POSITION_NOISE}"
LAUNCH_CMD+=" orientation_noise_std:=${ORIENTATION_NOISE}"

if [ -n "${XML_MODEL}" ]; then
    LAUNCH_CMD+=" xml_model:=${XML_MODEL}"
fi

echo "[3/3] Launching simulation..."
echo ""
echo "Configuration:"
echo "  - Visualization: ${VISUALIZE}"
echo "  - Sensor noise: ${SENSOR_NOISE}"
echo "  - Physics rate: ${PHYSICS_RATE} Hz"
echo "  - Sensor rate: ${SENSOR_RATE} Hz"
echo "  - Camera rate: ${CAMERA_RATE} Hz"
echo "  - Position noise: ${POSITION_NOISE} m"
echo "  - Orientation noise: ${ORIENTATION_NOISE} rad"
if [ -n "${XML_MODEL}" ]; then
    echo "  - XML model: ${XML_MODEL}"
fi
echo ""
echo "Press Ctrl+C to stop all nodes"
echo "=============================================="
echo ""

# Run roslaunch
exec ${LAUNCH_CMD}

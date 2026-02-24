#!/bin/bash
#
# Benchmark launcher for the Hybrid MPPI Tensegrity Simulation System
#
# Runs the full simulation pipeline N times, recording whether the robot
# reaches the goal within a time limit, then computes success rate and
# average time.
#
# Usage:
#   ./benchmark.sh --trials 5 --time-limit 120 --no-viz
#   ./benchmark.sh --help
#

set -e

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

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

# Start roscore if not running
ROSCORE_PID=""
if ! rostopic list &>/dev/null 2>&1; then
    echo "Starting roscore..."
    roscore &
    ROSCORE_PID=$!
    sleep 2
else
    echo "roscore already running"
fi

cleanup() {
    if [ -n "$ROSCORE_PID" ]; then
        echo "Stopping roscore (PID: $ROSCORE_PID)..."
        kill $ROSCORE_PID 2>/dev/null || true
        wait $ROSCORE_PID 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

echo "[3/3] Starting benchmark..."
echo ""

# Forward all arguments to Python orchestrator
python "${SCRIPT_DIR}/src/benchmark_orchestrator.py" "$@"

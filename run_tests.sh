#!/bin/bash
# Helper script to run smoke tests for Hybrid MPPI communication
#
# Usage:
#   ./run_tests.sh          # Interactive mode - you start components manually
#   ./run_tests.sh --help   # Show this help

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

function print_header() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}========================================${NC}"
}

function print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

function print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

function print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

function show_help() {
    cat << 'HELP'
Hybrid MPPI Smoke Test Runner

This script helps run communication smoke tests for the tensegrity system.

Usage:
  ./run_tests.sh          Run smoke test (requires manual component startup)
  ./run_tests.sh --help   Show this help

Manual Test Workflow:
  1. Open 6 separate terminals
  2. In each terminal, cd to this directory
  3. Run the following commands in order:

  Terminal 1 (ROS Master - REQUIRED):
    roscore

  Terminal 2 (Simulator):
    python src/mujoco_simulator/tensegrity_udp_simulator.py src/mujoco_simulator/xml_models/3bar_new_platform_all_cables.xml --no-viz

  Terminal 3 (Mock Tracker):
    source ~/catkin_ws_tensegrity/devel/setup.bash
    python src/perception/scripts/mock_tracking_service.py

  Terminal 4 (Controller):
    source ~/catkin_ws_tensegrity/devel/setup.bash
    python src/run_tensegrity_hybrid_mppi_simulator.py

  Terminal 5 (Planner - optional):
    source ~/catkin_ws_tensegrity/devel/setup.bash
    python src/mppi_planner.py

  Terminal 6 (This test):
    ./run_tests.sh

Unit Tests:
  To run comprehensive unit tests:
    source ~/catkin_ws_tensegrity/devel/setup.bash
    python src/test_end_to_end_hybrid_mppi.py

HELP
}

function check_ros_workspace() {
    if [ ! -d "$HOME/catkin_ws_tensegrity/devel" ]; then
        print_warning "ROS workspace not found at ~/catkin_ws_tensegrity"
        print_info "Setting up workspace..."
        
        if [ -f "./setup_workspace.sh" ]; then
            ./setup_workspace.sh
        else
            print_error "setup_workspace.sh not found!"
            exit 1
        fi
    fi
}

function check_xml_model() {
    if [ ! -f "src/mujoco_simulator/xml_models/3bar_new_platform_all_cables.xml" ]; then
        print_warning "Default XML model not found"
        print_info "Looking for alternative XML models..."

        if ls src/mujoco_simulator/xml_models/*.xml 1> /dev/null 2>&1; then
            local xml_file=$(ls src/mujoco_simulator/xml_models/*.xml | grep -v common | head -n 1)
            print_info "Found: $xml_file"
        else
            print_error "No XML models found in src/mujoco_simulator/xml_models/"
            exit 1
        fi
    else
        print_info "Default XML model found"
    fi
}

function run_smoke_test() {
    print_header "Running Smoke Test"
    
    # Check ROS workspace
    check_ros_workspace
    
    # Check XML model
    check_xml_model
    
    # Source ROS workspace if available
    if [ -f "$HOME/catkin_ws_tensegrity/devel/setup.bash" ]; then
        source "$HOME/catkin_ws_tensegrity/devel/setup.bash"
        print_info "Sourced ROS workspace"
    else
        print_warning "ROS workspace not sourced"
    fi
    
    # Run smoke test
    print_info "Starting smoke test..."
    python src/smoke_test_communication.py
}

function run_unit_tests() {
    print_header "Running Unit Tests"
    
    # Source ROS workspace
    if [ -f "$HOME/catkin_ws_tensegrity/devel/setup.bash" ]; then
        source "$HOME/catkin_ws_tensegrity/devel/setup.bash"
    else
        print_error "ROS workspace not found!"
        exit 1
    fi
    
    # Run unit tests
    python src/test_end_to_end_hybrid_mppi.py
}

# Main script
case "${1:-}" in
    --help|-h)
        show_help
        exit 0
        ;;
    --unit)
        run_unit_tests
        ;;
    *)
        run_smoke_test
        ;;
esac

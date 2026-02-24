#!/bin/bash
# Convenience script to run mock tracking service with proper ROS environment

# Source ROS workspace
source ~/catkin_ws_tensegrity/devel/setup.bash

# Run the mock tracking service
python3 "$(dirname "$0")/mock_tracking_service.py" "$@"

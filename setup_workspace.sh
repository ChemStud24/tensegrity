#!/bin/bash
#
# Setup script to create a catkin workspace for the tensegrity package
#

set -e

PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$HOME/catkin_ws_tensegrity"

echo "======================================================================"
echo "Setting up Catkin Workspace for Tensegrity Package"
echo "======================================================================"
echo ""
echo "Package location: $PACKAGE_DIR"
echo "Workspace will be created at: $WORKSPACE_DIR"
echo ""

# Check if workspace already exists
if [ -d "$WORKSPACE_DIR" ]; then
    echo "Workspace already exists at $WORKSPACE_DIR"
    read -p "Remove and recreate? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "Removing existing workspace..."
        rm -rf "$WORKSPACE_DIR"
    else
        echo "Aborting."
        exit 1
    fi
fi

# Create workspace structure
echo "Creating workspace structure..."
mkdir -p "$WORKSPACE_DIR/src"

# Initialize catkin workspace
cd "$WORKSPACE_DIR/src"
echo "Initializing catkin workspace..."
catkin_init_workspace

# Create symlink to tensegrity package
echo "Linking tensegrity package..."
ln -s "$PACKAGE_DIR" "$WORKSPACE_DIR/src/tensegrity"

# Also link tensegrity_perception if it exists
PERCEPTION_DIR="$(dirname "$PACKAGE_DIR")/tensegrity_perception"
if [ -d "$PERCEPTION_DIR" ]; then
    echo "Linking tensegrity_perception package..."
    ln -s "$PERCEPTION_DIR" "$WORKSPACE_DIR/src/tensegrity_perception"
fi

# Build workspace
echo ""
echo "Building workspace..."
cd "$WORKSPACE_DIR"

# Check if catkin_make is available
if ! command -v catkin_make &> /dev/null; then
    echo ""
    echo "ERROR: catkin_make not found!"
    echo ""
    echo "Make sure you have ROS installed and sourced."
    echo "If using conda/micromamba, activate your ROS environment first:"
    echo "  micromamba activate ros_env"
    echo "  # or: conda activate ros_env"
    echo ""
    echo "Then run this script again."
    exit 1
fi

catkin_make

echo ""
echo "======================================================================"
echo "✓ Workspace setup complete!"
echo "======================================================================"
echo ""
echo "To use this workspace:"
echo "  source $WORKSPACE_DIR/devel/setup.bash"
echo ""
echo "To run tests:"
echo "  source $WORKSPACE_DIR/devel/setup.bash"
echo "  cd $PACKAGE_DIR"
echo "  ./run_tests.sh"
echo ""
echo "Or add to your ~/.bashrc:"
echo "  echo 'source $WORKSPACE_DIR/devel/setup.bash' >> ~/.bashrc"
echo ""

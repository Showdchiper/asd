#!/bin/bash

# Mars Rover ROS 2 Navigation Setup and Build Script

echo "=========================================="
echo "Mars Rover Navigation - Setup Script"
echo "=========================================="

# Check if ROS 2 is installed
if [ -z "$ROS_DISTRO" ]; then
    echo "Error: ROS 2 not sourced. Please run:"
    echo "  source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "Using ROS 2 distribution: $ROS_DISTRO"
echo ""

# Navigate to workspace
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR"

cd "$WS_DIR"

echo "Installing dependencies..."
rosdep install --from-paths src --ignore-src -r -y

echo ""
echo "Building package..."
colcon build --packages-select mars_rover_navigation --symlink-install

if [ $? -eq 0 ]; then
    echo ""
    echo "=========================================="
    echo "Build successful!"
    echo "=========================================="
    echo ""
    echo "To use the package, run:"
    echo "  source $WS_DIR/install/setup.bash"
    echo ""
    echo "Then launch navigation:"
    echo "  ros2 launch mars_rover_navigation rover_navigation.launch.py"
    echo ""
else
    echo ""
    echo "Build failed. Please check the error messages above."
    exit 1
fi

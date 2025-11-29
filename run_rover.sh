#!/bin/bash

# Run Mars Rover Simulation - Complete Setup Script

echo "=== Mars Rover Simulation ==="
echo ""

# Set resource paths
export IGN_GAZEBO_RESOURCE_PATH="${IGN_GAZEBO_RESOURCE_PATH}:/home/sakshi/ignitio"
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}:/home/sakshi/ignitio"

# Source ROS2
source /opt/ros/humble/setup.bash
source /home/sakshi/ignitio/ros2_ws/install/setup.bash

echo "Step 1: Starting Gazebo Simulation..."
echo "       (Press PLAY button in Gazebo to start simulation)"
ign gazebo /home/sakshi/ignitio/MyModel.world &
GAZEBO_PID=$!
sleep 5

echo ""
echo "Step 2: Starting ROS-Gazebo Bridge..."
ros2 run ros_gz_bridge parameter_bridge \
    /lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan \
    /odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry \
    /cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist &
BRIDGE_PID=$!
sleep 2

echo ""
echo "Step 3: Starting Object Detector..."
ros2 run mars_rover_navigation object_detector &
DETECTOR_PID=$!
sleep 1

echo ""
echo "=== ROVER IS READY ==="
echo ""
echo "To move the rover, open a new terminal and run:"
echo "  source /opt/ros/humble/setup.bash"
echo "  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.5}, angular: {z: 0.0}}\" -r 10"
echo ""
echo "To stop the rover:"
echo "  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.0}, angular: {z: 0.0}}\" --once"
echo ""
echo "To rotate the rover:"
echo "  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.0}, angular: {z: 0.5}}\" -r 10"
echo ""
echo "View lidar data:"
echo "  ros2 topic echo /lidar"
echo ""
echo "View detected objects:"
echo "  ros2 topic echo /detected_objects"
echo ""
echo "Press Ctrl+C to stop all processes..."
echo ""

# Wait for Gazebo to exit
wait $GAZEBO_PID

# Cleanup
kill $BRIDGE_PID 2>/dev/null
kill $DETECTOR_PID 2>/dev/null
echo "Simulation stopped."

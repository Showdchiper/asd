#!/bin/bash

# Bridge Gazebo topics to ROS 2

echo "Starting Gazebo-ROS2 Bridge for Mars Rover..."
echo "Bridging lidar topic from Gazebo to ROS 2"

# Bridge lidar topic
ros2 run ros_gz_bridge parameter_bridge /lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan &

echo "Bridge started. Lidar data available on ROS 2 topic: /lidar"
echo "Check with: ros2 topic list"
echo "View data: ros2 topic echo /lidar"

wait

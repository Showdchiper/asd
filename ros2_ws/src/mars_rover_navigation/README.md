# Mars Rover Navigation with Object Detection and Path Planning

This package provides autonomous navigation for the Mars Rover using ROS 2, with lidar-based obstacle detection and avoidance.

## Features

- **Object/Cluster Detection**: Geometry-based detection using lidar data
- **Path Planning**: Nav2 stack with NavFn planner
- **Obstacle Avoidance**: DWB local planner with dynamic obstacle avoidance
- **Cost Map**: Local and global costmaps for navigation
- **Autonomous Navigation**: Full Nav2 integration

## Prerequisites

```bash
# Install ROS 2 Humble (or your ROS 2 version)
sudo apt update
sudo apt install ros-humble-desktop

# Install Nav2
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# Install Gazebo-ROS bridge
sudo apt install ros-humble-ros-gz-bridge ros-humble-ros-gz-sim

# Install additional dependencies
sudo apt install ros-humble-robot-localization \
                 ros-humble-robot-state-publisher \
                 ros-humble-tf2-ros \
                 ros-humble-pcl-ros
```

## Building the Package

```bash
# Navigate to workspace
cd ~/ignitio/ros2_ws

# Build the package
colcon build --packages-select mars_rover_navigation

# Source the workspace
source install/setup.bash
```

## Running the Simulation

### Step 1: Launch Gazebo with the Rover

```bash
cd ~/ignitio
./launch_model.sh
```

### Step 2: Launch ROS 2 Navigation Stack

In a new terminal:

```bash
cd ~/ignitio/ros2_ws
source install/setup.bash
ros2 launch mars_rover_navigation rover_navigation.launch.py
```

### Step 3: Visualize in RViz2

In another terminal:

```bash
ros2 run rviz2 rviz2
```

Add these displays in RViz2:
- **LaserScan** → Topic: `/scan`
- **Map** → Topic: `/local_costmap/costmap`
- **Map** → Topic: `/global_costmap/costmap`
- **MarkerArray** → Topic: `/detected_objects` (for object visualization)
- **Path** → Topic: `/plan`
- **TF** → Show robot frames

### Step 4: Send Navigation Goals

You can send navigation goals in multiple ways:

**Method 1: Using RViz2**
- Click "2D Goal Pose" button in RViz2
- Click on the map to set goal location

**Method 2: Using Command Line**
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'odom'}, \
    pose: {position: {x: 5.0, y: 5.0, z: 0.0}, \
           orientation: {w: 1.0}}}"
```

**Method 3: Using Nav2 Simple Commander (Python)**
```python
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy

rclpy.init()
navigator = BasicNavigator()

# Set goal pose
goal_pose = PoseStamped()
goal_pose.header.frame_id = 'odom'
goal_pose.header.stamp = navigator.get_clock().now().to_msg()
goal_pose.pose.position.x = 5.0
goal_pose.pose.position.y = 5.0
goal_pose.pose.orientation.w = 1.0

navigator.goToPose(goal_pose)
```

## Testing Object Detection

The object detector automatically processes lidar data and publishes detected objects:

```bash
# View detected objects
ros2 topic echo /detected_objects

# Adjust detection parameters
ros2 param set /object_detector min_cluster_size 5
ros2 param set /object_detector max_cluster_distance 0.3
```

## Configuration

### Navigation Parameters
Edit `config/nav2_params.yaml` to adjust:
- **Robot dimensions**: `robot_radius`
- **Speed limits**: `max_vel_x`, `max_vel_theta`
- **Inflation radius**: `inflation_radius` (safety margin around obstacles)
- **Costmap resolution**: `resolution`

### Object Detection Parameters
- `min_cluster_size`: Minimum points to form an object (default: 3)
- `max_cluster_distance`: Maximum distance between points in cluster (default: 0.5m)
- `min_object_distance`: Minimum distance to consider object (default: 0.2m)

## Topics

### Subscribed Topics
- `/scan` - Lidar data from sensor
- `/odom` - Odometry from differential drive

### Published Topics
- `/cmd_vel` - Velocity commands to rover
- `/detected_objects` - Visualization of detected obstacles
- `/local_costmap/costmap` - Local navigation costmap
- `/global_costmap/costmap` - Global navigation costmap
- `/plan` - Planned path

## Troubleshooting

### Rover not moving
- Check if Nav2 is running: `ros2 node list`
- Verify cmd_vel is being published: `ros2 topic echo /cmd_vel`
- Check bridge is working: `ros2 topic list | grep -E "(scan|odom|cmd_vel)"`

### No lidar data
- Verify Gazebo simulation is running
- Check bridge: `ros2 topic hz /scan`
- Restart gz_bridge node

### Navigation fails
- Check costmaps in RViz2
- Verify robot localization: `ros2 topic echo /odom`
- Adjust `robot_radius` if rover is too large/small
- Increase `inflation_radius` for more conservative planning

## Architecture

```
┌─────────────────┐
│  Gazebo Sim     │
│  (Rover Model)  │
└────────┬────────┘
         │
    ┌────▼─────┐
    │ ROS-GZ   │
    │  Bridge  │
    └────┬─────┘
         │
    ┌────▼──────────────────┐
    │  Object Detector      │
    │  (Cluster Detection)  │
    └────┬──────────────────┘
         │
    ┌────▼─────────────┐
    │   Nav2 Stack     │
    │ ┌──────────────┐ │
    │ │  Costmaps    │ │
    │ │  Planner     │ │
    │ │  Controller  │ │
    │ └──────────────┘ │
    └────┬─────────────┘
         │
    ┌────▼─────┐
    │  Rover   │
    │ Movement │
    └──────────┘
```

## Next Steps

1. **Add obstacle models** to your Gazebo world for testing
2. **Tune parameters** in `nav2_params.yaml` for your environment
3. **Add waypoint navigation** for autonomous missions
4. **Integrate SLAM** for mapping unknown environments
5. **Add 3D obstacle detection** using elevation mapping

## License

Apache 2.0

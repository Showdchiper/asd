# Mars Rover Simulation - ROS 2 & Gazebo

A complete Mars Rover simulation with lidar-based navigation, object detection, and autonomous path planning using ROS 2 Humble and Gazebo Ignition.

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![Gazebo](https://img.shields.io/badge/Gazebo-Ignition-orange)
![License](https://img.shields.io/badge/License-Apache%202.0-green)

## Features

- 🚀 **Mars Curiosity Rover Model** - Realistic rover with 4-wheel differential drive
- 📡 **360° GPU Lidar** - 30m range, 8Hz update rate
- 🎯 **Object Detection** - Real-time obstacle clustering from lidar data
- 🗺️ **Nav2 Integration** - Full autonomous navigation stack
- 🛡️ **Obstacle Avoidance** - Dynamic path planning with DWB controller

## Project Structure

```
ignitio/
├── README.md                 # This file
├── MyModel.world             # Gazebo world file
│
├── MarsRover/                # Mars Curiosity Rover model
│   ├── model.config          # Model metadata
│   ├── model.sdf             # Robot description with lidar & wheels
│   ├── mars_rover.dae        # 3D mesh
│   └── Image_*.png           # Textures (7 files)
│
├── MyModel/                  # Terrain/environment model
│   ├── model.config          # Model metadata
│   ├── model.sdf             # Terrain description
│   ├── Untitled.dae          # 3D mesh
│   ├── Diffuse.jpg           # Texture
│   └── Rock002_1K_Color.jpg  # Rock texture
│
└── ros2_ws/                  # ROS 2 Workspace
    └── src/mars_rover_navigation/
        ├── CMakeLists.txt        # Build configuration
        ├── package.xml           # Package dependencies
        ├── src/
        │   └── object_detector.cpp   # Lidar object detection node
        ├── launch/
        │   ├── rover_navigation.launch.py  # Full Nav2 launch
        │   └── test_lidar.launch.py        # Simple lidar test
        └── config/
            └── nav2_params.yaml  # Navigation parameters
```

## Prerequisites

```bash
# ROS 2 Humble
sudo apt install ros-humble-desktop

# Navigation2
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# Gazebo-ROS Bridge
sudo apt install ros-humble-ros-gz-bridge ros-humble-ros-gz-sim

# Additional dependencies
sudo apt install ros-humble-robot-localization ros-humble-tf2-ros ros-humble-pcl-ros
```

## Installation

```bash
# Clone the repository
git clone https://github.com/Showdchiper/qwer.git
cd qwer

# Build ROS 2 package
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mars_rover_navigation
source install/setup.bash
```

## Running the Project

Follow these steps in order. You need **3 separate terminals**.

### Step 1: Kill Any Existing Processes

```bash
pkill -9 ign 2>/dev/null; pkill -9 gz 2>/dev/null; pkill -9 ruby 2>/dev/null
```

### Step 2: Start Gazebo Simulation (Terminal 1)

```bash
cd ~/ignitio
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:$(pwd)
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)
ign gazebo MyModel.world
```

> ⏳ Wait for Gazebo to fully load before proceeding.

### Step 3: Start ROS-Gazebo Bridge (Terminal 2)

```bash
source /opt/ros/humble/setup.bash
source ~/ignitio/ros2_ws/install/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
    /lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan \
    /odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry \
    /cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist
```

### Step 4: Start Object Detector (Terminal 3)

```bash
source /opt/ros/humble/setup.bash
source ~/ignitio/ros2_ws/install/setup.bash
ros2 run mars_rover_navigation object_detector
```

### Step 5: Start the Simulation

> ⚠️ **Important:** Press the **PLAY ▶️ button** in Gazebo to start the simulation!

### Step 6: Verify Everything is Running

```bash
source /opt/ros/humble/setup.bash
ros2 topic list
```

You should see:
```
/cmd_vel
/lidar
/odom
/detected_objects
/parameter_events
/rosout
```

## Controlling the Rover

### Move Forward
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}" -r 10
```

### Rotate
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}" -r 10
```

### Stop
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

## ROS 2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/lidar` | `sensor_msgs/LaserScan` | 360° lidar scan data |
| `/odom` | `nav_msgs/Odometry` | Rover odometry |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
| `/detected_objects` | `visualization_msgs/MarkerArray` | Detected obstacles |

## View Data

```bash
# List all topics
ros2 topic list

# View lidar data
ros2 topic echo /lidar

# Check update rate
ros2 topic hz /lidar

# View detected objects
ros2 topic echo /detected_objects
```

## Full Navigation (Optional)

For autonomous navigation with Nav2:

```bash
ros2 launch mars_rover_navigation rover_navigation.launch.py
```

Then use RViz2 to set navigation goals:
```bash
ros2 run rviz2 rviz2
```

## Configuration

### Rover Speed (edit `config/nav2_params.yaml`)
```yaml
max_vel_x: 0.5        # m/s
max_vel_theta: 1.0    # rad/s
```

### Object Detection Parameters
```bash
ros2 param set /object_detector min_cluster_size 5
ros2 param set /object_detector max_cluster_distance 0.4
```

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Gazebo not starting | `pkill -9 ign && pkill -9 gz` then retry |
| No lidar data | Press PLAY in Gazebo |
| Bridge not working | `sudo apt install ros-humble-ros-gz-bridge` |
| Package not found | `source ~/ignitio/ros2_ws/install/setup.bash` |

## License

Apache 2.0

## Author

Sakshi - [@Showdchiper](https://github.com/Showdchiper)

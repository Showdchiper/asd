# Mars Rover - Quick Start Guide

## Object Detection and Path Planning with Obstacle Avoidance

This setup provides autonomous navigation for your Mars Rover using only **lidar sensor** and **ROS 2 Nav2** packages.

---

## What You Get

✅ **Geometry-based Object/Cluster Detection** from lidar scans  
✅ **Path Planning** using Nav2's NavFn planner  
✅ **Obstacle Avoidance** using DWB local planner  
✅ **Dynamic Costmaps** for safe navigation  
✅ **ROS 2 Integration** ready for autonomous missions

---

## Quick Start (3 Steps)

### 1️⃣ Build the ROS 2 Package

```bash
cd ~/ignitio/ros2_ws
source /opt/ros/humble/setup.bash  # or your ROS 2 distro
./build_rover.sh
```

### 2️⃣ Launch Gazebo Simulation

```bash
cd ~/ignitio
./launch_model.sh
```

### 3️⃣ Start Navigation Stack

**New terminal:**
```bash
cd ~/ignitio/ros2_ws
source install/setup.bash
ros2 launch mars_rover_navigation rover_navigation.launch.py
```

---

## How It Works

```
Lidar Scan → Object Detection → Costmap → Path Planning → Obstacle Avoidance → Rover Movement
```

1. **Lidar** scans the environment (270° FOV, 30m range)
2. **Object Detector** clusters nearby points into objects
3. **Costmap** marks obstacles and inflates safety zones
4. **Planner** computes optimal path avoiding obstacles
5. **Controller** follows path while dynamically avoiding new obstacles
6. **Rover** navigates safely to goal

---

## Send Navigation Commands

### Option 1: RViz2 (Recommended)

```bash
ros2 run rviz2 rviz2
```

1. Add displays: LaserScan (`/scan`), Map (`/local_costmap/costmap`), Path (`/plan`)
2. Click "2D Goal Pose" toolbar button
3. Click on map to set destination
4. Rover automatically plans path and navigates!

### Option 2: Command Line

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'odom'}, \
    pose: {position: {x: 10.0, y: 5.0, z: 0.0}, orientation: {w: 1.0}}}"
```

---

## Key Features

### 🎯 Object Detection
- Clusters lidar points into objects
- Adjustable sensitivity via parameters
- Publishes visualization markers

### 🗺️ Path Planning
- Global path using NavFn planner
- Considers all known obstacles
- Replans if path blocked

### 🚧 Obstacle Avoidance
- Real-time local planning with DWB
- Dynamic obstacle detection
- Safe navigation with inflation layers

### ⚙️ Tunable Parameters

**Detection sensitivity:**
```bash
ros2 param set /object_detector min_cluster_size 5
ros2 param set /object_detector max_cluster_distance 0.4
```

**Speed limits** (edit `config/nav2_params.yaml`):
```yaml
max_vel_x: 0.5  # m/s
max_vel_theta: 1.0  # rad/s
```

**Safety margin:**
```yaml
inflation_radius: 0.8  # meters around obstacles
robot_radius: 0.6  # robot size
```

---

## Monitoring

### View Topics
```bash
# Lidar data
ros2 topic hz /scan

# Detected objects
ros2 topic echo /detected_objects

# Velocity commands
ros2 topic echo /cmd_vel

# Navigation status
ros2 topic echo /plan
```

### Check Nodes
```bash
ros2 node list
# Should show: object_detector, controller_server, planner_server, etc.
```

---

## Testing Tips

### Add Test Obstacles
Add boxes to your Gazebo world (`MyModel.world`):

```xml
<model name="obstacle_1">
  <static>true</static>
  <pose>5 5 0.5 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry>
        <box><size>1 1 1</size></box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box><size>1 1 1</size></box>
      </geometry>
    </visual>
  </link>
</model>
```

### Test Scenarios
1. **Static obstacles**: Place boxes, rover navigates around them
2. **Narrow passages**: Test path planning through tight spaces
3. **Moving obstacles**: Manually move objects, test dynamic avoidance

---

## Architecture

- **Object Detector Node**: Processes `/scan`, publishes `/detected_objects`
- **Nav2 Stack**: Handles planning, control, and behaviors
- **Local Costmap**: 10x10m rolling window around rover
- **Global Costmap**: Full known environment
- **DWB Controller**: Dynamic window approach for smooth motion

---

## Next Steps

1. ✅ Test basic navigation
2. 📍 Add waypoint following for multi-point missions
3. 🗺️ Integrate SLAM for mapping (slam_toolbox)
4. 🎥 Add camera for visual detection
5. 🌐 Add 3D obstacle avoidance with octomap

---

## Troubleshooting

**Problem**: Rover doesn't move  
**Solution**: Check `ros2 topic echo /cmd_vel`, verify bridge is running

**Problem**: No obstacles detected  
**Solution**: Check `ros2 topic hz /scan`, adjust detection parameters

**Problem**: Rover gets stuck  
**Solution**: Increase `inflation_radius` or decrease `robot_radius`

**Problem**: Path planning fails  
**Solution**: Set goal closer, check costmap in RViz2

---

## Files Created

```
ros2_ws/src/mars_rover_navigation/
├── CMakeLists.txt                    # Build configuration
├── package.xml                       # Package dependencies
├── README.md                         # Full documentation
├── src/
│   └── object_detector.cpp          # Object detection node
├── launch/
│   └── rover_navigation.launch.py  # Main launch file
└── config/
    └── nav2_params.yaml            # Nav2 configuration
```

## Support

For issues, check the full README.md in the package directory.

Happy navigating! 🚀🤖

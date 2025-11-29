# Mars Rover Navigation System - Complete Setup

## ✅ What Has Been Created

### 1. **Custom Rover Model** (`CustomRover/`)
- 4-wheel differential drive rover
- Lidar sensor (270° FOV, 30m range)
- Proper physics and collision
- Positioned correctly on ground

### 2. **ROS 2 Navigation Package** (`ros2_ws/src/mars_rover_navigation/`)

#### Object Detection Node (`src/object_detector.cpp`)
- **Geometry-based clustering** of lidar points
- **Real-time object detection** from laser scans
- **Adjustable parameters** for sensitivity
- **Visualization markers** for detected objects

#### Navigation Launch File (`launch/rover_navigation.launch.py`)
- Gazebo-ROS bridge for topics
- Object detector node
- Robot state publisher for TF
- Complete Nav2 navigation stack

#### Nav2 Configuration (`config/nav2_params.yaml`)
- **Local & Global Costmaps** for obstacle representation
- **DWB Local Planner** for dynamic obstacle avoidance
- **NavFn Global Planner** for path planning
- **Behavior Server** for recovery behaviors
- Tuned for Mars rover specifications

---

## 🎯 Key Features Implemented

### Object/Cluster Detection
✅ Scans lidar data at 10 Hz  
✅ Groups nearby points into clusters  
✅ Filters out noise with minimum cluster size  
✅ Publishes detected objects as visualization markers  

### Path Planning
✅ Global path planning using NavFn algorithm  
✅ Considers static and dynamic obstacles  
✅ Replans automatically if path becomes blocked  
✅ Smooth path following with velocity smoothing  

### Obstacle Avoidance  
✅ Dynamic Window Approach (DWB) for local planning  
✅ Real-time obstacle detection and avoidance  
✅ Cost-based trajectory optimization  
✅ Safety inflation zones around obstacles  
✅ Recovery behaviors (spin, backup, wait)  

---

## 📦 Package Structure

```
ignitio/
├── CustomRover/                    # Rover model
│   ├── model.sdf                  # Robot description
│   └── model.config               # Model metadata
│
├── MyModel.world                   # Gazebo world with rover
│
├── ros2_ws/
│   └── src/mars_rover_navigation/
│       ├── CMakeLists.txt         # Build config
│       ├── package.xml            # Dependencies
│       ├── README.md              # Full documentation
│       │
│       ├── src/
│       │   └── object_detector.cpp    # Object detection
│       │
│       ├── launch/
│       │   └── rover_navigation.launch.py  # Main launch
│       │
│       └── config/
│           └── nav2_params.yaml   # Nav2 configuration
│
├── build_rover.sh                  # Build script
├── NAVIGATION_QUICKSTART.md        # Quick start guide
└── launch_model.sh                 # Gazebo launcher
```

---

## 🚀 How to Use

### Step 1: Build ROS 2 Package
```bash
cd ~/ignitio/ros2_ws
source /opt/ros/humble/setup.bash
./build_rover.sh
```

### Step 2: Launch Gazebo
```bash
cd ~/ignitio
./launch_model.sh
```

### Step 3: Launch Navigation
```bash
# New terminal
cd ~/ignitio/ros2_ws
source install/setup.bash
ros2 launch mars_rover_navigation rover_navigation.launch.py
```

### Step 4: Visualize & Navigate
```bash
# New terminal
ros2 run rviz2 rviz2
```

Add displays:
- LaserScan → `/scan`
- Map → `/local_costmap/costmap`
- MarkerArray → `/detected_objects`
- Path → `/plan`

Click "2D Goal Pose" and set destination!

---

## 🔧 Configuration

### Adjust Object Detection
```bash
ros2 param set /object_detector min_cluster_size 5
ros2 param set /object_detector max_cluster_distance 0.4
```

### Modify Speed Limits
Edit `config/nav2_params.yaml`:
```yaml
max_vel_x: 0.5        # Linear speed (m/s)
max_vel_theta: 1.0    # Angular speed (rad/s)
```

### Safety Margins
```yaml
robot_radius: 0.6      # Robot size
inflation_radius: 0.8  # Safety zone
```

---

## 📊 System Architecture

```
┌──────────────┐
│   Gazebo     │  Simulates rover, lidar, physics
└──────┬───────┘
       │
┌──────▼────────┐
│  ros_gz_bridge│  Connects Gazebo ↔ ROS 2
└──────┬────────┘
       │
       ├─────→ /scan ─────────┐
       ├─────→ /odom          │
       └─────← /cmd_vel       │
                              │
                    ┌─────────▼──────────┐
                    │  Object Detector   │  Clusters lidar points
                    └─────────┬──────────┘
                              │
                    ┌─────────▼──────────┐
                    │   Local Costmap    │  Marks obstacles
                    └─────────┬──────────┘
                              │
                    ┌─────────▼──────────┐
                    │  Global Planner    │  Plans path to goal
                    └─────────┬──────────┘
                              │
                    ┌─────────▼──────────┐
                    │  Local Planner     │  Avoids obstacles
                    │  (DWB Controller)  │
                    └─────────┬──────────┘
                              │
                              ▼
                        Rover moves safely!
```

---

## 🧪 Testing

### Add Test Obstacles to World
Edit `MyModel.world`, add before `</world>`:

```xml
<model name="test_obstacle">
  <static>true</static>
  <pose>5 0 0.5 0 0 0</pose>
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
      <material>
        <ambient>1 0 0 1</ambient>
        <diffuse>1 0 0 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

### Send Test Goal
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'odom'}, \
    pose: {position: {x: 10.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

Watch rover navigate around the obstacle!

---

## 📝 Important Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | LaserScan | Lidar measurements |
| `/odom` | Odometry | Robot position/velocity |
| `/cmd_vel` | Twist | Velocity commands |
| `/detected_objects` | MarkerArray | Detected obstacles |
| `/local_costmap/costmap` | OccupancyGrid | Local obstacle map |
| `/global_costmap/costmap` | OccupancyGrid | Global map |
| `/plan` | Path | Planned trajectory |

---

## 🎓 Next Steps

1. **Test navigation** with simple goals
2. **Add more obstacles** to test avoidance
3. **Tune parameters** for your environment
4. **Add SLAM** for mapping: `ros-humble-slam-toolbox`
5. **Waypoint missions** for autonomous exploration
6. **3D mapping** with elevation costmap

---

## 📚 Dependencies Required

```bash
# Core ROS 2
sudo apt install ros-humble-desktop

# Navigation
sudo apt install ros-humble-navigation2 \
                 ros-humble-nav2-bringup

# Gazebo bridge
sudo apt install ros-humble-ros-gz-bridge \
                 ros-humble-ros-gz-sim

# Additional tools
sudo apt install ros-humble-robot-localization \
                 ros-humble-robot-state-publisher \
                 ros-humble-tf2-ros \
                 ros-humble-rviz2
```

---

## ✨ Summary

You now have a **complete autonomous navigation system** for your Mars rover:

✅ Lidar-based obstacle detection  
✅ Real-time path planning  
✅ Dynamic obstacle avoidance  
✅ ROS 2 integration  
✅ Ready for autonomous missions  

All using **geometry-only detection** from the **lidar sensor** and **ROS 2 Nav2** packages!

Read `NAVIGATION_QUICKSTART.md` for detailed usage instructions.

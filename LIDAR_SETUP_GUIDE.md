# Mars Rover - Complete Setup & Testing Guide

## Fixed Issues ✅

1. ✅ Lidar sensor configured: 720 samples, 10 Hz, 0.05-30m range
2. ✅ ROS 2 package builds successfully
3. ✅ Gazebo sensor system plugin added
4. ✅ Object detection node ready
5. ✅ Lidar topic remapping configured

---

## Step-by-Step Startup

### Terminal 1: Launch Gazebo Simulation

```bash
cd ~/ignitio
./launch_model.sh
```

**Wait for Gazebo to fully load** (you should see the rover in the window)

---

### Terminal 2: Check Gazebo Lidar Topic

```bash
# Check if lidar is publishing
gz topic -l | grep lidar

# View lidar data (should show: /lidar)
gz topic -e -t /lidar
```

You should see laser scan data streaming!

---

### Terminal 3: Bridge Gazebo to ROS 2

**Install bridge if needed:**
```bash
sudo apt install ros-humble-ros-gz-bridge
```

**Start the bridge:**
```bash
cd ~/ignitio/ros2_ws
source /opt/ros/humble/setup.bash

# Bridge lidar topic from Gazebo to ROS 2
ros2 run ros_gz_bridge parameter_bridge /lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan
```

**Keep this terminal running!**

---

### Terminal 4: Test ROS 2 Lidar Topic

```bash
cd ~/ignitio/ros2_ws
source /opt/ros/humble/setup.bash

# Check ROS 2 topics
ros2 topic list | grep lidar

# View lidar data in ROS 2
ros2 topic echo /lidar --once
```

You should see LaserScan messages!

---

### Terminal 5: Launch Object Detection (Optional)

```bash
cd ~/ignitio/ros2_ws
source install/setup.bash

# Launch test with object detector
ros2 launch mars_rover_navigation test_lidar.launch.py
```

This will:
- Start object detection node
- Process lidar data
- Detect and cluster obstacles
- Publish `/detected_objects` markers

---

## Verify Everything is Working

### Check All Topics
```bash
ros2 topic list
```

Should show:
- `/lidar` - Lidar scan data
- `/detected_objects` - Detected obstacles (if running detector)
- `/parameter_events`
- `/rosout`

### Check TF Frames
```bash
ros2 run tf2_tools view_frames
```

Should show: `odom` → `base_link` → `lidar_link`

### Monitor Lidar Data
```bash
ros2 topic hz /lidar
```

Should show ~10 Hz update rate

### View in RViz2
```bash
ros2 run rviz2 rviz2
```

Add displays:
1. **LaserScan** → Topic: `/lidar`, Fixed Frame: `base_link`
2. **TF** → Show all transforms
3. **MarkerArray** → Topic: `/detected_objects` (if detector running)

---

## Common Issues & Fixes

### Issue: "Package not found"
```bash
cd ~/ignitio/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash  # ← Important! Source workspace
```

### Issue: "No lidar topic in Gazebo"
- Wait longer for Gazebo to fully start
- Check sensors plugin loaded: `gz plugin --list | grep Sensors`
- Restart Gazebo if needed

### Issue: "Bridge not working"
```bash
# Install bridge
sudo apt install ros-humble-ros-gz-bridge

# Use correct message type
ros2 run ros_gz_bridge parameter_bridge /lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan
```

### Issue: "Gazebo crashes"
- Remove `render_engine` specification (already fixed)
- Reduce lidar samples if needed
- Check GPU drivers

---

## Lidar Specifications Implemented

✅ **Horizontal samples**: 720  
✅ **Update rate**: 10 Hz  
✅ **Min range**: 0.05 m  
✅ **Max range**: 30 m  
✅ **Field of view**: 360° horizontal  
✅ **Resolution**: 0.01 m  
✅ **Noise**: Gaussian (mean=0, stddev=0.01)  

---

## Topic Connections Verified

```
Gazebo Simulation
       ↓
  /lidar (gz.msgs.LaserScan)
       ↓
  ros_gz_bridge
       ↓
  /lidar (sensor_msgs/LaserScan) ← ROS 2
       ↓
  object_detector node
       ↓
  /detected_objects (visualization_msgs/MarkerArray)
```

---

## Quick Test Commands

```bash
# 1. Start Gazebo
cd ~/ignitio && ./launch_model.sh &

# 2. Wait 10 seconds, then check Gazebo topic
sleep 10 && gz topic -l | grep lidar

# 3. Bridge to ROS 2 (new terminal)
source /opt/ros/humble/setup.bash
ros2 run ros_gz_bridge parameter_bridge /lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan &

# 4. Check ROS 2 topic
sleep 2 && ros2 topic hz /lidar

# 5. View data
ros2 topic echo /lidar --once
```

---

## Files Created

```
ignitio/
├── bridge_gazebo_ros.sh              # Bridge script
├── MarsRover/model.sdf               # Rover with lidar (720 samples, 10Hz)
├── MyModel.world                     # World with sensors plugin
└── ros2_ws/
    └── src/mars_rover_navigation/
        ├── src/object_detector.cpp   # Object detection
        ├── launch/
        │   ├── test_lidar.launch.py # Simple test launch
        │   └── rover_navigation.launch.py  # Full navigation
        └── config/nav2_params.yaml   # Navigation config
```

---

## Success Checklist

- [ ] Gazebo launches without crashing
- [ ] `gz topic -l` shows `/lidar`
- [ ] Bridge connects Gazebo to ROS 2
- [ ] `ros2 topic list` shows `/lidar`
- [ ] `ros2 topic hz /lidar` shows ~10 Hz
- [ ] RViz2 displays laser scan
- [ ] Object detector processes scans

---

## Next Steps

Once lidar is working:
1. Add obstacles to test object detection
2. Add wheel control for navigation
3. Integrate Nav2 for path planning
4. Add waypoint following

For full autonomous navigation, see `ROVER_NAVIGATION_SUMMARY.md`

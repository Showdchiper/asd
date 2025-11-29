# Mars Rover Lidar Integration - Status Report

## ✅ What's Been Fixed

### 1. Lidar Sensor Configuration
- **Horizontal samples**: 720 (as requested)
- **Update rate**: 10 Hz
- **Min range**: 0.05 m  
- **Max range**: 30 m
- **Noise**: Added Gaussian noise for realism
- **Location**: Added to `MarsRover/model.sdf`

### 2. ROS 2 Package
- ✅ Package builds successfully
- ✅ Object detector node compiled
- ✅ Test launch file created (`test_lidar.launch.py`)
- ✅ Topic remapping configured (`/lidar` topic)

### 3. World Configuration
- ✅ Sensors system plugin added
- ✅ Ground plane added
- ✅ Render engine issue fixed (removed ogre2 spec)

---

## ⚠️ Current Issues

### Gazebo Stability
**Problem**: Gazebo crashes when loading the full world with rover model

**Possible causes**:
1. Ray sensor with high sample count (720) may be too heavy
2. Conflict between MyModel and rover
3. Graphics rendering issues

**Solutions to try**:
1. Reduce lidar samples temporarily (360 instead of 720)
2. Test rover alone without MyModel
3. Use CPU ray sensor instead of GPU

---

## 🔧 Recommended Next Steps

### Option 1: Reduce Lidar Complexity (Quick Fix)

Edit `/home/sakshi/ignitio/MarsRover/model.sdf`:

```xml
<horizontal>
  <samples>360</samples>  <!-- Reduce from 720 -->
  <resolution>1</resolution>
  <min_angle>-3.14159</min_angle>
  <max_angle>3.14159</max_angle>
</horizontal>
```

### Option 2: Test Rover Separately

Create simple test world:
```bash
cd ~/ignitio
cat > test_rover.world << 'EOF'
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="test_world">
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
    <plugin filename="ignition-gazebo-physics-system"
            name="ignition::gazebo::systems::Physics" />
    <plugin filename="ignition-gazebo-sensors-system"
            name="ignition::gazebo::systems::Sensors" />
    <plugin filename="ignition-gazebo-scene-broadcaster-system"
            name="ignition::gazebo::systems::SceneBroadcaster" />
    
    <light type="directional" name="sun">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
    </light>
    
    <include>
      <uri>model://MarsRover</uri>
      <pose>0 0 0.5 0 0 0</pose>
    </include>
    
    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal></plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane><normal>0 0 1</normal><size>100 100</size></plane>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
EOF
```

Then test:
```bash
ign gazebo test_rover.world
```

### Option 3: Use CPU Ray Sensor

The current sensor uses GPU acceleration which might be causing issues. To use CPU-based ray sensor, the type should work as-is (`type="ray"`).

---

## 📋 Complete Startup Procedure (Once Fixed)

```bash
# Terminal 1: Start Gazebo
cd ~/ignitio
./launch_model.sh

# Terminal 2: Check Gazebo lidar topic (wait 10 sec)
gz topic -l | grep lidar
# Should show: /lidar

# Terminal 3: Bridge to ROS 2
source /opt/ros/humble/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  /lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan

# Terminal 4: Verify ROS 2 topic
cd ~/ignitio/ros2_ws
source install/setup.bash
ros2 topic list | grep lidar
ros2 topic hz /lidar  # Should show ~10 Hz

# Terminal 5: Start object detection
ros2 launch mars_rover_navigation test_lidar.launch.py
```

---

## 🎯 Lidar Topic Connection Map

```
┌─────────────────────────────────────┐
│   Gazebo Simulation                 │
│   MarsRover/model.sdf               │
│   <sensor name="lidar_sensor">      │
│   <topic>lidar</topic>              │
└──────────────┬──────────────────────┘
               │
               ↓ gz.msgs.LaserScan
               │
┌──────────────▼──────────────────────┐
│   Gazebo Internal Topic             │
│   /lidar                            │
└──────────────┬──────────────────────┘
               │
               ↓
┌──────────────▼──────────────────────┐
│   ros_gz_bridge                     │
│   Converts: gz.msgs → ROS 2 msgs   │
└──────────────┬──────────────────────┘
               │
               ↓ sensor_msgs/LaserScan
               │
┌──────────────▼──────────────────────┐
│   ROS 2 Topic                       │
│   /lidar                            │
└──────────────┬──────────────────────┘
               │
               ↓
┌──────────────▼──────────────────────┐
│   object_detector node              │
│   Subscribes to: /scan              │
│   (remapped from /lidar)            │
└──────────────┬──────────────────────┘
               │
               ↓
┌──────────────▼──────────────────────┐
│   /detected_objects                 │
│   (MarkerArray)                     │
└─────────────────────────────────────┘
```

---

## 📦 Files Modified

1. `/home/sakshi/ignitio/MarsRover/model.sdf`
   - Added lidar sensor with 720 samples, 10 Hz
   - Added noise model
   - Added lidar visual marker (blue cylinder)

2. `/home/sakshi/ignitio/MyModel.world`
   - Added Sensors system plugin
   - Added ground plane
   - Removed ogre2 render engine spec

3. `/home/sakshi/ignitio/ros2_ws/src/mars_rover_navigation/`
   - `launch/test_lidar.launch.py` - Simple test launch
   - Object detector configured for `/lidar` topic

4. Created helper scripts:
   - `bridge_gazebo_ros.sh` - Bridge script
   - `LIDAR_SETUP_GUIDE.md` - Complete guide

---

## ✅ Verification Commands

```bash
# Check Gazebo is running
ps aux | grep gz

# List Gazebo topics
gz topic -l

# Echo lidar data in Gazebo
gz topic -e -t /lidar

# List ROS 2 topics
ros2 topic list

# Check lidar frequency
ros2 topic hz /lidar

# View lidar data
ros2 topic echo /lidar --once

# Check TF frames
ros2 run tf2_tools view_frames
```

---

## 🆘 If Still Having Issues

1. **Reduce sample count**: Change 720 → 360 in model.sdf
2. **Test simple world**: Use test_rover.world (see Option 2)
3. **Check GPU**: Gazebo might need better graphics support
4. **Use different sensor**: Try gpu_ray or gpu_lidar type

Read `LIDAR_SETUP_GUIDE.md` for detailed troubleshooting.

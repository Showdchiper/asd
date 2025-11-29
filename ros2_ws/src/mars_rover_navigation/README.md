# Mars Rover Exploration System

This system provides autonomous exploration capabilities for the Mars Rover, including:
- **Object Detection** - Detects rocks, craters, walls, and potential habitat sites using LiDAR
- **Path Recording** - Records and saves the rover's travel path
- **Habitat Finding** - Analyzes terrain to find suitable habitat locations
- **Navigation Control** - Autonomous navigation with obstacle avoidance

## ROS2 Topics

### Subscribed Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/lidar` | sensor_msgs/LaserScan | LiDAR sensor data |
| `/odom` | nav_msgs/Odometry | Rover odometry |
| `/navigation_command` | std_msgs/String | Navigation commands |

### Published Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | geometry_msgs/Twist | Velocity commands |
| `/detected_objects` | visualization_msgs/MarkerArray | Detected object markers |
| `/detected_objects_data` | std_msgs/String | JSON object data |
| `/recorded_path` | nav_msgs/Path | Recorded travel path |
| `/habitat_markers` | visualization_msgs/MarkerArray | Habitat location markers |
| `/habitat_candidates` | std_msgs/String | JSON habitat candidates |
| `/best_habitat` | geometry_msgs/PoseStamped | Best habitat location |
| `/navigation_status` | std_msgs/String | Navigation status |

## Navigation Commands

Send commands to `/navigation_command` topic:
- `explore` - Start autonomous exploration
- `stop` - Stop the rover
- `idle` - Set to idle mode
- `goto x y` - Navigate to specific coordinates

## Object Types Detected
- **ROCK** - Small obstacles (< 0.8m)
- **CRATER** - Medium circular depressions (< 2.0m)
- **WALL** - Large elongated obstacles (> 3.0m aspect ratio)
- **POTENTIAL_HABITAT** - Large flat areas suitable for habitat

## Habitat Quality Ratings
- **EXCELLENT** (Score >= 0.8) - Green markers
- **GOOD** (Score >= 0.6) - Yellow-green markers
- **FAIR** (Score >= 0.4) - Yellow markers
- **POOR** (Score >= 0.2) - Orange markers
- **UNSUITABLE** (Score < 0.2) - Red markers

## Data Output

### Path Data
Saved to `/home/sakshi/ignitio/ros2_ws/data/paths/`
- JSON files with full path data
- CSV files for easy analysis

### Habitat Data
Saved to `/home/sakshi/ignitio/ros2_ws/data/habitats/`
- JSON files with habitat candidates and scores

## Usage

### Build the package
```bash
cd /home/sakshi/ignitio/ros2_ws
colcon build --packages-select mars_rover_navigation
source install/setup.bash
```

### Launch Gazebo simulation
```bash
export IGN_GAZEBO_RESOURCE_PATH="${IGN_GAZEBO_RESOURCE_PATH}:/home/sakshi/ignitio"
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}:/home/sakshi/ignitio"
ign gazebo /home/sakshi/ignitio/MyModel.world
```

### Launch exploration system
```bash
ros2 launch mars_rover_navigation exploration_system.launch.py
```

### Manual navigation commands
```bash
# Start exploration
ros2 topic pub --once /navigation_command std_msgs/msg/String "{data: 'explore'}"

# Stop rover
ros2 topic pub --once /navigation_command std_msgs/msg/String "{data: 'stop'}"

# Navigate to specific location
ros2 topic pub --once /navigation_command std_msgs/msg/String "{data: 'goto 5.0 3.0'}"
```

### View topics
```bash
# See detected objects
ros2 topic echo /detected_objects_data

# See habitat candidates
ros2 topic echo /habitat_candidates

# See navigation status
ros2 topic echo /navigation_status

# See recorded path
ros2 topic echo /recorded_path
```

## Parameters

### Object Detector
- `min_cluster_size`: Minimum points to form object (default: 3)
- `max_cluster_distance`: Max distance between points in cluster (default: 0.5m)
- `rock_size_threshold`: Max size for rock classification (default: 0.8m)
- `crater_size_threshold`: Max size for crater (default: 2.0m)

### Path Recorder
- `save_directory`: Path save location
- `save_interval`: Auto-save interval (default: 30s)
- `min_distance_threshold`: Min distance to record point (default: 0.1m)

### Habitat Finder
- `analysis_radius`: Terrain analysis radius (default: 15m)
- `flatness_threshold`: Max variance for flat terrain (default: 0.3)
- `min_open_space`: Minimum open space ratio (default: 0.4)
- `shelter_distance`: Ideal distance to shelter (default: 3.0m)

### Navigation Controller
- `max_linear_speed`: Maximum forward speed (default: 0.5 m/s)
- `max_angular_speed`: Maximum rotation speed (default: 1.0 rad/s)
- `obstacle_distance_threshold`: Obstacle detection distance (default: 1.5m)
- `critical_distance`: Emergency stop distance (default: 0.5m)

# Mars Curiosity Rover - Gazebo Model

## ✅ Completed Tasks

### 1. Model Conversion
- ✓ Converted `Mars Curiosity Rover.glb` to `mars_rover.dae` format
- ✓ Applied proper rotation and transformations for Gazebo
- ✓ Optimized materials for Gazebo rendering

### 2. Texture Extraction
- ✓ Extracted all 7 textures from the GLB model:
  - Image_0.png (362 KB)
  - Image_1.png (891 KB)
  - Image_2.png (1.3 MB)
  - Image_3.png (1.1 MB)
  - Image_4.png (1.6 MB)
  - Image_5.png (626 KB)
  - Image_6.png (741 KB)
- ✓ Textures automatically applied to the DAE model

### 3. Model Configuration
- ✓ Created `model.config` with proper metadata
- ✓ Created `model.sdf` with:
  - Proper physics (mass: 900 kg, matching real Curiosity rover)
  - Collision geometry
  - Visual rendering with material properties
  - Sensor mount point for future lidar attachment
  - ROS 2 integration preparation

### 4. World Integration
- ✓ Added Mars Rover to `MyModel.world`
- ✓ Positioned at coordinates (0, 0, 0.5)
- ✓ Ready to spawn alongside other models

## 📁 File Structure

```
MarsRover/
├── model.config          # Model metadata
├── model.sdf            # SDF definition with physics
├── mars_rover.dae       # 3D mesh (12 MB)
├── Image_0.png          # Texture 1
├── Image_1.png          # Texture 2
├── Image_2.png          # Texture 3
├── Image_3.png          # Texture 4
├── Image_4.png          # Texture 5
├── Image_5.png          # Texture 6
└── Image_6.png          # Texture 7
```

## 🚀 How to Launch

Run the simulation:
```bash
./launch_model.sh
```

Or manually:
```bash
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:$(pwd)
ign gazebo MyModel.world
```

## ✨ Model Features

- **Realistic Physics**: 900 kg mass matching real Curiosity rover
- **Proper Textures**: All 7 textures applied and rendering correctly
- **Surface Friction**: Configured for Mars-like terrain
- **Collision Detection**: Full collision geometry enabled
- **Sensor Ready**: Mount point prepared for lidar sensor

## 🔄 Next Steps (ROS 2 Integration)

When ready for ROS 2 and lidar sensor integration:

1. **Add ROS 2 Bridge Plugin** to `model.sdf`
2. **Configure Lidar Sensor** (e.g., Velodyne, RPLidar)
3. **Setup ROS 2 Control** for rover movement
4. **Add Joint Controllers** for wheels/arms
5. **Configure TF Tree** for coordinate frames
6. **Setup ROS 2 Topics** for sensor data

The model is now ready and waiting for your signal to proceed with ROS 2 integration! 🎉

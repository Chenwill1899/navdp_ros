# NavDP ROS Integration

ROS Noetic integration for NavDP (Navigation Diffusion Policy) on NVIDIA AGX Orin platform.

## Overview

This package provides ROS nodes to run NavDP visual navigation policy with RGB-D camera input, publishing predicted trajectories for mobile robot navigation.

## Features

- **Real-time Inference**: NavDP model running on CUDA for trajectory prediction
- **ROS Integration**: Subscribes to camera topics, publishes navigation trajectories
- **World Coordinate Transform**: Automatic coordinate transformation from camera frame to world frame
- **RViz Visualization**: Complete visualization of inputs, outputs, and trajectories
- **AGX Orin Optimized**: Configured for NVIDIA AGX Orin with libffi workarounds

## System Requirements

- **Platform**: NVIDIA AGX Orin (ARM64)
- **OS**: Ubuntu 20.04
- **ROS**: Noetic
- **Python**: 3.8
- **CUDA**: Compatible with AGX Orin
- **Camera**: Intel RealSense or compatible RGB-D camera

## Installation

```bash
# Clone repository
cd ~/catkin_ws/src
git clone -b ros1 https://github.com/Chenwill1899/navdp_ros.git

# Install dependencies
cd navdp_ros/src/NavDP
pip install -r requirements.txt

cd ../navdp_bridge
pip install -r requirements.txt

# Build workspace
cd ~/catkin_ws
catkin_make

# Source workspace
source devel/setup.bash
```

## Package Structure

```
src/
├── NavDP/              # NavDP policy implementation
│   ├── baselines/      # Navigation baselines including NavDP
│   ├── configs/        # Robot and scene configurations
│   └── utils_tasks/    # Utility functions
└── navdp_bridge/       # ROS bridge package
    ├── launch/         # Launch files
    ├── scripts/        # ROS nodes
    └── requirements.txt
```

## Camera Topics

The node subscribes to:
- `/camera/color/image_raw` - RGB image (sensor_msgs/Image)
- `/camera/aligned_depth_to_color/image_raw` - Aligned depth (sensor_msgs/Image)
- `/camera/color/camera_info` - Camera intrinsics (optional if configured in launch file)
- `/Odometry` - Robot odometry for coordinate transforms
- `/move_base_simple/goal` - Goal from RViz 2D Nav Goal tool

## Output Topics

The node publishes:
- `/navdp/trajectory` - Best predicted trajectory (nav_msgs/Path)
- `/navdp/all_trajectories` - All candidate trajectories (visualization_msgs/MarkerArray)
- `/navdp/current_goal` - Current goal marker (visualization_msgs/Marker)
- `/navdp/rgb_viz` - RGB visualization (sensor_msgs/Image)
- `/navdp/depth_viz` - Depth visualization (sensor_msgs/Image)
- `/navdp/trajectory_overlay` - Trajectory overlay image (sensor_msgs/Image)

## Usage

### Basic Launch

```bash
roslaunch navdp_bridge navdp.launch
```

### Launch Parameters

- `checkpoint`: Path to NavDP model weights (default: `navdp-weights.ckpt`)
- `use_rviz`: Launch RViz for visualization (default: true)
- `device`: CUDA device (default: cuda:0)
- `camera_fx`, `camera_fy`, `camera_cx`, `camera_cy`: Camera intrinsics (optional)

### Setting Navigation Goals

1. Open RViz (automatically launched)
2. Use "2D Nav Goal" tool in toolbar
3. Click and drag to set goal position in `map` frame
4. NavDP will predict trajectory and publish to `/navdp/trajectory`

## Coordinate Systems

- **map**: Static world frame (RViz Fixed Frame)
- **base_link**: Robot frame, updated from `/Odometry`
- **camera_link**: Camera frame

Goals are set in `map` frame, transformed to camera frame for NavDP inference, then results transformed back to `map` frame for visualization.

## Configuration

### Camera Intrinsics

Configure in launch file if camera_info topic unavailable:

```xml
<param name="camera_fx" value="601.108"/>
<param name="camera_fy" value="599.911"/>
<param name="camera_cx" value="321.224"/>
<param name="camera_cy" value="249.420"/>
```

### Model Checkpoint

Place NavDP model weights at workspace root or specify path:

```bash
roslaunch navdp_bridge navdp.launch checkpoint:=/path/to/weights.ckpt
```

## Troubleshooting

### libffi Symbol Error

The launch file includes `LD_PRELOAD` workaround for libffi conflicts on AGX Orin:
```xml
<env name="LD_PRELOAD" value="/usr/lib/aarch64-linux-gnu/libffi.so.7"/>
```

### RViz OpenGL Crash (SSH)

Software rendering is enabled for remote X11:
```xml
<env name="LIBGL_ALWAYS_SOFTWARE" value="1"/>
```

### No Trajectory Output

Check:
1. Camera topics publishing: `rostopic hz /camera/color/image_raw`
2. Odometry available: `rostopic echo /Odometry -n 1`
3. Goal set in RViz
4. Model initialized: Look for "✓✓ NavDP agent initialized" in logs

## Performance

- **Inference Rate**: ~10Hz on AGX Orin
- **Image Resolution**: 640x480 (resized to 224x224 for model)
- **Trajectory Horizon**: 24 waypoints
- **Candidate Trajectories**: 16 sampled paths

## Citation

If you use this code, please cite the original NavDP paper:

```bibtex
@article{navdp2024,
  title={NavDP: Navigation Diffusion Policy for Visual Navigation},
  author={[Authors]},
  journal={[Conference/Journal]},
  year={2024}
}
```

## License

This project integrates NavDP with ROS. Please refer to the original NavDP repository for licensing terms.

## Contact

- GitHub: [Chenwill1899](https://github.com/Chenwill1899)
- Repository: https://github.com/Chenwill1899/navdp_ros

## Acknowledgments

- NavDP original implementation
- ROS community
- NVIDIA for AGX Orin platform support

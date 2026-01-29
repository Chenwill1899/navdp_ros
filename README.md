# NavDP ROS Integration

ROS Noetic integration for NavDP (Navigation Diffusion Policy) on NVIDIA AGX Orin platform.

## Overview

This package provides ROS nodes to run NavDP visual navigation policy with RGB-D camera input, publishing predicted trajectories for mobile robot navigation.

## Update Log (更新日志)

### 2026-01-29
- **仿真环境集成**: 将 `ugv_sim` 仿真环境代码合并入主仓库。
- **文档更新**: 整理项目时间线与功能说明。

### 2026-01-28
- **可视化增强**: 
  - 新增 `/navdp/all_trajectories` 话题，在 RViz 中以热力图形式绘制所有 16 条候选轨迹。
  - 新增 `/navdp/current_goal` 话题，直观显示当前导航目标点。
  - 新增 `/navdp/debug_image`，显示模型生成的轨迹掩码投影。
- **AGX Orin 兼容性修复**: 
  - 解决了 `libffi` 版本冲突问题（通过 `launch` 文件注入 `LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libffi.so.7`）。
  - 修复了 macOS/Windows 远程连接时的 X11 Forwarding `OpenGL` 崩溃问题（支持设置 `LIBGL_ALWAYS_SOFTWARE=1`）。

### 2026-01-27
- **项目初始化**: 创建 `navdp_bridge` ROS 包。
- **核心节点实现**: 完成 `navdp_node.py`，打通了从 ROS 图片订阅 -> NavDP 推理 -> 轨迹 Path 发布的链路。
- **控制器接口**: 提供了 MPC (Model Predictive Control) 和 Pure Pursuit (纯追踪) 控制器的基础实现。

---

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
git clone https://github.com/Chenwill1899/navdp_ros.git

# Install dependencies (NavDP)
cd navdp_ros/src/NavDP
pip install -r requirements.txt

# Install dependencies (Bridge)
cd ../navdp_bridge
pip install -r requirements.txt
# Additional libs for Orin
pip install cvxpy numpy cv_bridge

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
├── navdp_bridge/       # ROS bridge package
│   ├── launch/         # Launch files (navdp.launch)
│   ├── scripts/        # ROS nodes (navdp_node.py, mpc_controller.py)
│   └── requirements.txt
└── ugv_sim/            # UGV Simulation Environment (Integrated)
```

## Camera Topics

The node subscribes to:
- `/camera/color/image_raw` - RGB image (sensor_msgs/Image)
- `/camera/depth/image_raw` - Depth image (sensor_msgs/Image, 16UC1 or 32FC1)
- `/camera/color/camera_info` - Camera intrinsics (optional if configured in launch file)
- `/odom` - Robot odometry
- `/move_base_simple/goal` - Goal from RViz 2D Nav Goal tool

## Output Topics

The node publishes:
- `/navdp/trajectory` - Best predicted trajectory for control (nav_msgs/Path)
- `/navdp/all_trajectories` - All candidate trajectories (visualization_msgs/MarkerArray)
- `/navdp/current_goal` - Current goal marker (visualization_msgs/Marker)
- `/navdp/debug_image` - Trajectory mask visualization (sensor_msgs/Image)

## Usage

### Basic Launch

```bash
roslaunch navdp_bridge navdp.launch
```

### Launch Parameters

- `checkpoint`: Path to NavDP model weights (default: `navdp-weights.ckpt`)
- `use_rviz`: Launch RViz for visualization (default: true)
- `device`: CUDA device (default: cuda:0)
- `stop_threshold`: Threshold to stop navigation (default: 0.5)

### Running on AGX Orin / Jetson

If you encounter `libffi` or Python extension errors, the launch file is already configured to preload the system library:
```xml
<env name="LD_PRELOAD" value="/usr/lib/aarch64-linux-gnu/libffi.so.7"/>
```

### Remote Visualization (Mac/Windows)
The launch file supports software rendering for X11 forwarding:
```xml
<env name="LIBGL_ALWAYS_SOFTWARE" value="1"/>
```

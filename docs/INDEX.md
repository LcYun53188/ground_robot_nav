# 文档索引

本文档集只服务当前地面全向轮机器人导航项目。当前主线是：

```text
OAK-D + Isaac ROS Visual SLAM / cuVSLAM + nvblox + Nav2 + ground_serial_bridge
```

第一版不使用可靠轮速里程计，不把 MID360 放入主定位链路，也不处理 OAK-D 与
MID360 的复杂跨设备时间戳对齐。

## 核心文档

- [项目 README](../README.md)：项目目标、环境配置、构建、硬件验证和当前进度。
- [docs README](./README.md)：本目录文档阅读顺序。
- [安装与构建指南](./INSTALLATION.md)：`.venv`、`uv`、Python 依赖和 colcon 构建。
- [CUDA Toolkit 13.2 安装指南](./CUDA_TOOLKIT_13_2_INSTALLATION.md)：Isaac ROS / nvblox 构建所需 CUDA 环境。
- [OAK-D Visual SLAM 与 RViz 验证](./OAKD_VISUAL_SLAM_RVIZ.md)：OAK-D 双目、IMU、TF、Visual SLAM 和 RViz 验证。
- [NVIDIA 3D 导航架构](./NVIDIA_3D_NAV_ARCHITECTURE.md)：当前 OAK-D + cuVSLAM + nvblox + Nav2 架构，以及 VINS 残留状态说明。
- [NVIDIA 3D 导航项目计划](./NVIDIA_3D_NAV_PROJECT_PLAN.md)：阶段计划、最小可行版本边界和后续 ESS / MID360 安排。
- [Gazebo Harmonic 仿真](./GAZEBO_HARMONIC_SIMULATION.md)：当前推荐的无硬件仿真路径，使用 Gazebo Harmonic + `ros_gz`。
- [Isaac Sim 4.5 / 轻量仿真验证](./ISAAC_SIM_SIMULATION.md)：无真实硬件时验证 ROS 2 侧 Nav2 / nvblox 闭环。

## 当前运行入口

OAK-D + Visual SLAM 硬件验证：

```bash
env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
./scripts/with_venv.sh ros2 launch src/omni_bringup/launch/oakd_visual_slam_rviz.launch.py
```

完整 NVIDIA 3D 导航入口：

```bash
env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
./scripts/with_venv.sh ros2 launch omni_bringup nvidia_3d_nav.launch.py
```

Gazebo Harmonic 仿真入口：

```bash
./simulation/scripts/run_gazebo_harmonic_nav.sh
```

Isaac Sim 最小仿真入口：

```bash
./simulation/scripts/run_isaac_sim_nav.sh
./scripts/with_venv.sh ros2 launch omni_bringup isaac_sim_nav.launch.py
```

Isaac Sim 4.5 UI 入口：

```bash
./simulation/scripts/run_isaac_sim_45_ui.sh
```

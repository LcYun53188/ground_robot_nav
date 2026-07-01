# 文档目录

本目录保存地面全向轮导航栈的项目文档。文档内容应围绕地面机器人、
OAK-D、Isaac ROS Visual SLAM、nvblox、Nav2 和底盘串口桥展开。

建议阅读顺序：

- [项目 README](../README.md)：项目目标、环境配置、构建、硬件验证和当前进度。
- [文档索引](./INDEX.md)：完整文档入口。
- [安装与构建指南](./INSTALLATION.md)：虚拟环境、依赖安装与构建步骤。
- [CUDA Toolkit 13.2 安装指南](./CUDA_TOOLKIT_13_2_INSTALLATION.md)：Isaac ROS / nvblox 构建所需 CUDA 环境。
- [OAK-D Visual SLAM 与 RViz 验证](./OAKD_VISUAL_SLAM_RVIZ.md)：OAK-D 双目 + IMU + Visual SLAM 硬件验证。
- [NVIDIA 3D 导航架构](./NVIDIA_3D_NAV_ARCHITECTURE.md)：当前 OAK-D + cuVSLAM + nvblox + Nav2 架构。
- [NVIDIA 3D 导航项目计划](./NVIDIA_3D_NAV_PROJECT_PLAN.md)：分阶段迁移计划。
- [Gazebo Harmonic 仿真](./GAZEBO_HARMONIC_SIMULATION.md)：当前推荐的无硬件仿真路径。
- [Isaac Sim 最小仿真验证](./ISAAC_SIM_SIMULATION.md)：无真实硬件时验证 ROS 2 侧 Nav2 / nvblox 闭环。

新增文档应只描述当前地面机器人项目，不再扩展无人机主线。

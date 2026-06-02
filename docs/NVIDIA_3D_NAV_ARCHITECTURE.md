# NVIDIA 3D 导航架构

本文档描述当前地面全向轮机器人的 NVIDIA / Isaac ROS 3D 导航主线。该主线以
OAK-D 为第一版核心传感器，使用 Isaac ROS Visual SLAM / cuVSLAM 做视觉里程计，
使用 nvblox 做 3D 建图，使用 Nav2 做规划和控制。

第一版明确不依赖可靠轮速里程计，不引入 OAK-D 与 MID360 的复杂跨设备时间戳
对齐，也不把 MID360 放进主定位链路。

## 核心数据链路

```text
OAK-D 左右目矫正图 + OAK-D IMU
    -> isaac_ros_visual_slam
    -> odom -> base_link
    -> /visual_slam/tracking/odometry

OAK-D depth image + depth camera info
    -> nvblox_ros
    -> /nvblox_node/static_map_slice

/nvblox_node/static_map_slice
    -> nvblox_nav2 costmap layer
    -> Nav2 planner/controller
    -> /cmd_vel
    -> ground_serial_bridge
```

OAK-D 的 stereo、IMU、depth 和 CameraInfo 都来自同一设备时钟域，能避免首版就处理
跨设备同步问题。MID360 后续只作为局部安全障碍层考虑，不参与 cuVSLAM、ESS 或
nvblox 主地图。

## 当前启动入口

完整 NVIDIA 3D 导航入口：

```bash
env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
./scripts/with_venv.sh ros2 launch omni_bringup nvidia_3d_nav.launch.py
```

只验证 OAK-D + Visual SLAM，不启动 nvblox、Nav2、ESS 或底盘桥：

```bash
env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
./scripts/with_venv.sh ros2 launch src/omni_bringup/launch/oakd_visual_slam_rviz.launch.py
```

只启动 OAK-D 和静态 TF，用于检查硬件数据完整性：

```bash
env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
./scripts/with_venv.sh ros2 launch omni_bringup nvidia_3d_nav.launch.py \
  launch_visual_slam:=false \
  launch_nvblox:=false \
  launch_nav2:=false \
  launch_ground_bridge:=false
```

## OAK-D 必需输出

第一版主线需要这些 OAK-D topic：

- `/oakd/left/image_raw`
- `/oakd/right/image_raw`
- `/oakd/left/camera_info`
- `/oakd/right/camera_info`
- `/oakd/imu/raw`
- `/oakd/depth/image`
- `/oakd/depth/camera_info`

`/oakd/left/image_raw` 和 `/oakd/right/image_raw` 名称沿用旧接口，但当前内容来自
DepthAI `StereoDepth` 的 rectified stereo 输出，而不是未矫正的 mono 原始图。

VIO-only 验证入口默认使用 `oakd_stereo_quality_mode:=low_latency`，在不发布
depth/pointcloud 时关闭 subpixel、left-right check 和深度后处理，避免把 OAK-D
算力消耗在 Visual SLAM 不使用的深度质量上。

完整导航入口默认使用 `oakd_stereo_quality_mode:=auto`。当启用 depth/nvblox 时，
仍保留深度质量相关配置。

## 依赖与 vendor 代码

当前 NVIDIA 路径使用仓库内 vendor/submodule 源码补齐最小 ROS 安装中通常没有的包：

- `src/isaac_ros_visual_slam`：Isaac ROS Visual SLAM / cuVSLAM。
- `src/isaac_ros_nvblox`：nvblox ROS 节点和 `nvblox_nav2`。
- `src/isaac_ros_common`：Isaac ROS CMake 和 launch 公共工具。
- `src/isaac_ros_nitros`：Isaac ROS NITROS / GXF 传输相关包。
- `src/navigation2`：Nav2 Jazzy 源码，包括 `nav2_mppi_controller`。
- `src/negotiated`：NITROS 需要的 REP-2009 type negotiation 支持。
- `src/isaac_ros_image_pipeline`：主要构建 `isaac_ros_vpi_utils`，其他演示/测试包通过补丁排除。

初始化 submodule 并应用本地补丁：

```bash
git submodule update --init --recursive
./scripts/apply_vendor_patches.sh
```

安装 ROS 系统依赖：

```bash
./scripts/install_nvidia_3d_nav_rosdeps.sh
```

下载 Isaac ROS 需要的 Git LFS vendor 二进制资源：

```bash
./scripts/install_vendor_lfs_assets.sh
```

构建 NVIDIA 3D 导航依赖：

```bash
./scripts/build_nvidia_3d_nav_deps.sh
```

`patches/vendor/` 用于保存对上游/vendor 代码的本地适配，避免直接修改上游源码而无法追踪。

## 验证命令

静态配置和包可用性检查：

```bash
STATIC_ONLY=true ./scripts/check_nvidia_3d_nav_mvp.sh
```

启动完整 NVIDIA 路径后运行：

```bash
./scripts/check_nvidia_3d_nav_mvp.sh
```

运行期重点检查：

- `/visual_slam/tracking/odometry`
- `odom -> base_link`
- `/nvblox_node/static_map_slice`
- `/cmd_vel`
- Nav2 lifecycle 状态

如果只检查 OAK-D 硬件和 Visual SLAM，请优先使用
[OAK-D Visual SLAM 与 RViz 验证](./OAKD_VISUAL_SLAM_RVIZ.md)。

## Nav2 与 MPPI

`nav2_3d_nav.yaml` 当前使用 `nav2_mppi_controller::MPPIController`，运动模型按
全向轮设置为 `Omni`，速度和加速度限制采用保守参数。后续闭环跑通后，再针对真实
底盘能力逐步调大速度、采样数和代价项权重。

## ESS 后续路径

ESS 默认不启用。原因是 ESS 需要 TensorRT engine，并且要求左右目矫正图和
CameraInfo 稳定。主闭环先使用 OAK-D 原生 depth 跑通。

启用 ESS 的基本形式：

```bash
./scripts/with_venv.sh ros2 launch omni_bringup nvidia_3d_nav.launch.py \
  launch_ess:=true \
  ess_engine_file:=/absolute/path/to/ess.engine
```

启用后需要把 `nvblox_depth_image_topic` 和 `nvblox_depth_camera_info_topic`
切换到 ESS / disparity-to-depth 输出。ESS 的接入应在 OAK-D 原生 depth 闭环稳定后
再进行。

## 当前核心保留包

- `oakd_perception`：OAK-D 图像、IMU、深度和可选点云输入。
- `isaac_ros_visual_slam`：视觉里程计和 `odom -> base_link`。
- `nvblox_ros`：3D TSDF/ESDF 建图。
- `nvblox_nav2`：Nav2 costmap 接入。
- `nav2_*`：目标跟随、规划、局部控制和恢复行为。
- `ground_serial_bridge`：把 Nav2 速度命令转换为底盘串口命令。

## 不在当前核心路径中的内容

- `FAST_LIO_ROS2`：第一版 NVIDIA 架构不使用。
- `VINS-Fusion-ros2`：由 `isaac_ros_visual_slam` 替代。源码目录仍保留，但带 `COLCON_IGNORE`，默认不构建，也不作为当前可用定位链路。
- `nav_mapping/local_map_builder`：由 nvblox 替代。
- 自研 SE(2) DWA：由 Nav2 controller server 和 MPPI 替代。
- MID360 主定位：不进入第一版主链路。

## VINS 残留状态

当前仓库中仍能看到：

- `src/VINS-Fusion-ros2`
- `src/imu_fusion`
- 旧 `scripts/run_nav_stack.sh` 中的 `vio` / `enable_vins` 逻辑

这些内容只表示历史源码和旧入口残留，不表示当前 VINS 链路可直接使用。原因：

- `src/VINS-Fusion-ros2/COLCON_IGNORE` 使 VINS 默认不参与 `colcon build`。
- `src/imu_fusion/COLCON_IGNORE` 使旧 IMU fusion 辅助链路默认不构建。
- 旧 `run_nav_stack.sh` 启动的是 `uav_bringup nav_stack.launch.py`，不是当前地面
  `nvidia_3d_nav.launch.py` 主线。
- 旧 VINS OAK-D 配置需要重新核对 OAK-D 内参、双目外参、IMU 外参和时间戳策略。

如果后续确实需要恢复 VINS，必须作为独立任务处理：移除 `COLCON_IGNORE`、修复依赖、
重新标定、编写地面专用 launch，并重新验证 `/vio/odometry`、TF 和 Nav2 接入。

## 风险边界

- 无轮速里程计兜底时，Visual SLAM 丢跟踪会直接影响定位。
- OAK-D 图像帧率和左右目同步必须稳定。
- 低纹理、弱光、过曝或运动过快都会降低 cuVSLAM 可靠性。
- nvblox 对 depth 和 TF 连续性敏感，必须先确认 `odom -> base_link` 稳定。

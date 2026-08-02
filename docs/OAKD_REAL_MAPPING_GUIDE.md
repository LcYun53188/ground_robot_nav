# OAK-D 真实环境建图指南

本文档说明如何使用真实 OAK-D 完成当前项目的在线建图验证。当前主线使用 OAK-D
左右目矫正图和 IMU 运行 Isaac ROS Visual SLAM / cuVSLAM，以 OAK-D 原生深度图
驱动 nvblox 建立 TSDF/ESDF 地图，再把二维距离切片交给 Nav2。

## 建图链路

```text
OAK-D 左右目 + IMU
    -> Isaac ROS Visual SLAM / cuVSLAM
    -> map -> odom
    -> /visual_slam/tracking/odometry
    -> visual_odom_guard
    -> robot_localization EKF
    -> odom -> base_link

OAK-D depth + CameraInfo
    -> nvblox TSDF / ESDF
    -> /nvblox_node/static_map_slice
    -> Nav2 local/global costmap
```

完整 NVIDIA 路径默认不需要主机生成 `/oakd/points`；nvblox 直接消费深度图，避免
PointCloud2 转换带来的 CPU 开销。

## 1. 建图前检查

### 硬件和环境

- OAK-D 固定牢靠，运行过程中相对 `base_link` 不发生移动。
- 镜头无遮挡、无明显污渍，USB 连接稳定。
- 环境具有足够光照和纹理，避免大面积纯色墙、强反光和阳光直射。
- NVIDIA 驱动、CUDA、Isaac ROS、nvblox 和工作区已经构建完成。
- 启动时机器人保持静止 `5-10s`，等待 IMU 和视觉跟踪稳定。

### 相机安装外参

真实入口默认安装参数为：

```text
oakd_x     = 0.12
oakd_y     = 0.0
oakd_z     = 0.28
oakd_yaw   = 0.0
oakd_pitch = pi/2
oakd_roll  = pi
```

前三项必须按实际安装位置测量。默认旋转假设相机向前、水平安装，并把 OAK-D 原始
IMU/光学轴转换到 ROS `base_link` 约定。不要通过观察 RViz 猜测平移外参。

例如相机位于车体中心前方 `0.18m`、上方 `0.22m`：

```bash
env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
./scripts/with_venv.sh ros2 launch omni_bringup nvidia_3d_nav.launch.py \
  oakd_x:=0.18 oakd_y:=0.0 oakd_z:=0.22
```

## 2. 单独验证 OAK-D 和 cuVSLAM

建图前先运行 VIO-only 入口：

```bash
env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
./scripts/with_venv.sh ros2 launch omni_bringup oakd_visual_slam_rviz.launch.py
```

检查左右目和 IMU：

```bash
env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
timeout 8s ./scripts/with_venv.sh ros2 topic hz /oakd/left/image_raw

env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
timeout 8s ./scripts/with_venv.sh ros2 topic hz /oakd/right/image_raw

env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
timeout 8s ./scripts/with_venv.sh ros2 topic hz /oakd/imu/raw
```

缓慢前后、横向和旋转机器人，确认：

- `/visual_slam/tracking/odometry` 连续更新。
- RViz 中 `base_link` 的运动方向与真实机器人一致。
- 位姿没有突然跳到远处，静止时没有持续快速漂移。
- `/visual_slam/status` 没有持续报告跟踪丢失。

详细排查方法见 [OAK-D Visual SLAM 与 RViz 验证](./OAKD_VISUAL_SLAM_RVIZ.md)。

## 3. 启动真实在线建图

退出 VIO-only 入口，确保没有其他进程占用 OAK-D，然后启动完整链路：

```bash
env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
./scripts/with_venv.sh ros2 launch omni_bringup nvidia_3d_nav.launch.py
```

该入口默认启动 OAK-D、cuVSLAM、里程计跳变保护、EKF、nvblox、Nav2 和底盘串口桥。
首次测试如果不希望连接真实底盘，可以关闭串口桥，并手动移动整台机器人：

```bash
env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
./scripts/with_venv.sh ros2 launch omni_bringup nvidia_3d_nav.launch.py \
  launch_nav2:=false launch_ground_bridge:=false
```

手动移动时必须保持 OAK-D 与机器人刚性连接，不要只拿着相机移动，否则
`base_link -> camera` 外参假设不成立。

## 4. 检查深度、TF 和地图

```bash
env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
timeout 8s ./scripts/with_venv.sh ros2 topic hz /oakd/depth/image

env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
./scripts/with_venv.sh ros2 topic echo --once /oakd/depth/image/header

env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
./scripts/with_venv.sh ros2 topic echo --once /oakd/depth/camera_info/header

env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
./scripts/with_venv.sh ros2 run tf2_ros tf2_echo odom oakd_camera_optical_frame
```

检查 nvblox 输出：

```bash
env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
timeout 8s ./scripts/with_venv.sh ros2 topic hz /nvblox_node/static_map_slice

env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
./scripts/with_venv.sh ros2 topic echo --once --no-arr \
  /nvblox_node/static_map_slice
```

另开 RViz：

```bash
env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
./scripts/with_venv.sh ros2 run rviz2 rviz2 \
  -d src/omni_bringup/rviz/nvblox_map_check.rviz
```

真实配置位于 `src/omni_bringup/config/nvblox_3d_nav.yaml`。当前主要参数为：

- `global_frame: odom`
- `voxel_size: 0.035`
- `mapping_type: static_tsdf`
- `min_height: 0.10`
- `max_height: 0.50`
- `back_projection_subsampling: 2`
- `publish_mesh: false`

如果需要临时观察三维 mesh，可把 `publish_mesh` 改为 `true` 后重启，但会增加 GPU、
CPU 和 RViz 负载。导航调试完成后应恢复为 `false`。

## 5. 建图运动方法

- 初期将线速度限制在 `0.2-0.35m/s`，角速度限制在 `0.4-0.8rad/s`。
- 从光照和纹理较好的区域开始，先走短直线，再做缓慢转向。
- 相邻视野保持足够重叠，不要高速原地旋转。
- 定期回到已经走过的位置，观察 cuVSLAM 是否稳定闭环。
- OAK-D 为前向传感器，倒车和侧向运动前先让相机观察目标方向附近的障碍。
- 遇到玻璃、镜面、黑色吸光物体和过近物体时，不要假设深度一定可靠。
- 持续查看 `/visual_slam/odom_guard/status`，发现连续拒绝时立即停止机器人。

查看跳变保护状态：

```bash
env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
./scripts/with_venv.sh ros2 topic echo /visual_slam/odom_guard/status
```

## 6. 保存两类地图

真实环境需要分别保存 cuVSLAM 特征地图和 nvblox 几何地图。二者用途不同，不能互相
替代。

创建父目录：

```bash
MAP_DIR="$(pwd)/maps/oakd_site_01"
mkdir -p "$MAP_DIR"
```

### 保存 cuVSLAM 特征地图

该地图用于后续视觉重定位：

```bash
env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
./scripts/with_venv.sh ros2 service call \
  /visual_slam/save_map isaac_ros_visual_slam_interfaces/srv/FilePath \
  "{file_path: '$MAP_DIR/cuvslam'}"
```

`cuvslam` 是由服务创建的地图目录。保存前必须已经收到有效里程计，并且配置中的
`enable_localization_n_mapping` 保持为 `true`。

### 保存 nvblox 几何地图

```bash
env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
./scripts/with_venv.sh ros2 service call \
  /nvblox_node/save_map nvblox_msgs/srv/FilePath \
  "{file_path: '$MAP_DIR/site_01.nvblx'}"
```

可选导出 PLY：

```bash
env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
./scripts/with_venv.sh ros2 service call \
  /nvblox_node/save_ply nvblox_msgs/srv/FilePath \
  "{file_path: '$MAP_DIR/site_01.ply'}"
```

所有服务都应返回 `success: true`。保存后检查：

```bash
find "$MAP_DIR" -maxdepth 2 -type f -ls
```

## 7. 加载和视觉重定位

先启动真实建图入口并等待 OAK-D 跟踪初始化，然后在机器人位于已知地图附近时请求
视觉重定位。下面的单位姿态提示表示机器人接近建图原点：

```bash
MAP_DIR="$(pwd)/maps/oakd_site_01"

env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
./scripts/with_venv.sh ros2 service call \
  /visual_slam/localize_in_map \
  isaac_ros_visual_slam_interfaces/srv/LocalizeInMap \
  "{map_folder_path: '$MAP_DIR/cuvslam', pose_hint: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

如果机器人不在原点附近，应把 `pose_hint` 改为机器人在旧地图中的近似位置和朝向。
服务接受请求只表示异步重定位已经开始；还要继续观察 `/visual_slam/status` 和
`map -> odom` 是否稳定。

加载 nvblox 地图：

```bash
env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
./scripts/with_venv.sh ros2 service call \
  /nvblox_node/load_map nvblox_msgs/srv/FilePath \
  "{file_path: '$MAP_DIR/site_01.nvblx'}"
```

## 8. 当前持久化限制

当前 nvblox 和 Nav2 costmap 使用 `odom`，local/global costmap 都是滚动窗口。cuVSLAM
虽然发布 `map -> odom` 并支持保存特征地图，但 nvblox 地图没有直接建立在 `map`
坐标系中。因此：

- 当前配置主要面向边走边建图、在线避障和短中距离导航。
- `.nvblx` 可以保存和加载，但跨进程重启后必须保证新的 `odom` 与保存地图坐标一致。
- 只完成 cuVSLAM 重定位，不代表旧 `.nvblx` 会自动与新的 `odom` 对齐。
- 当前不会直接生成传统 Nav2 `map.yaml + map.pgm` 静态占据地图。

如果目标是断电重启后复用完整场地地图，应另行完成坐标系统一：让持久化 nvblox
地图和 global costmap 使用 `map`，验证 cuVSLAM 重定位后的 `map -> odom`，并设计
启动时的地图加载顺序。在完成该改造前，不应把当前 `.nvblx` 加载流程当作生产级
长期定位方案。

## 9. 验收标准

一次真实建图测试至少满足：

- 左右目图像、IMU、深度图和 CameraInfo 持续发布。
- `map -> odom -> base_link -> oakd_camera_optical_frame` TF 链完整。
- cuVSLAM 位姿方向正确，无持续跳变或高速漂移。
- nvblox map slice 在机器人移动后扩展，障碍与真实环境大致一致。
- Nav2 local/global costmap 能收到 nvblox layer。
- cuVSLAM 和 nvblox 保存服务均返回成功，输出文件可读。
- 急停、人工接管和底盘停止方法已经实际验证。

## 10. 常见问题

### `X_LINK_DEVICE_ALREADY_IN_USE`

已有进程占用 OAK-D。退出旧 launch，确认 `oakd_unified`、Visual SLAM 和相关容器停止
后再启动；不要同时运行 VIO-only 和完整建图入口。

### 深度有数据但地图为空

检查 depth 与 CameraInfo 的 frame 是否相同，以及
`odom -> oakd_camera_optical_frame` 是否能在对应时间戳查询。TF 时间和深度时间不一致
时，nvblox 会丢弃输入。

### 地图重影或弯曲

优先检查相机刚性、安装外参、左右目同步、光照和 cuVSLAM 漂移。nvblox 只能按输入
位姿融合深度，不能修复错误里程计。

### 低矮或悬空障碍没有进入 costmap

检查 `min_height`、`max_height` 和 `slice_height`。当前真实配置只处理约
`0.10-0.50m` 的高度带，修改后必须结合机器人高度、通道尺寸和实际障碍重新验证。


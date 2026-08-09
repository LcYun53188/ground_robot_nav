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
    +-> nvblox TSDF / ESDF
    |   -> /nvblox_node/static_map_slice（正障碍）
    +-> cliff_detector local elevation grid
        -> /perception/cliff_points（负障碍边缘）
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
- `multi_mapper.experimental_use_ground_plane_estimation: true`
- `multi_mapper.max_ground_plane_slope_deg: 45.0`（大于 45° 判为障碍）
- `multi_mapper.min_reversed_traversable_slope_deg: 5.0`
- `multi_mapper.traversable_elevation_height_tolerance_m: 0.02`
- `static_mapper.slice_height_above_plane_m: 0.03`
- `static_mapper.slice_height_thickness_m: 0.33`
- 固定高度回退范围：`static_mapper.esdf_slice_min_height: 0.03` 到
  `static_mapper.esdf_slice_max_height: 0.36`
- `back_projection_subsampling: 2`
- `publish_mesh: false`

二维障碍切片会优先跟随局部估计的地面平面，并对高度带内每个 TSDF 表面柱计算局部
法向。朝上的局部表面相对水平面的倾角不大于 45° 时从障碍中剔除；从坡底仰视时，
投影 TSDF 的梯度方向可能翻转，因此反向梯度仅在坡度为 5-45° 时按缓坡处理。近水平的
朝下表面仍视为悬空障碍，大于 45°、法向不可靠，或与同一栅格内其他障碍重叠时也仍
作为障碍。因此缓坡不会仅因观察方向或在 `odom` 中累计升高超过 10 cm 就被整片标成
障碍。台阶、断崖及多层地面仍需要在实车上保守验证。
法向计算会在两体素范围内寻找最近的有效 TSDF 邻居，中心差分不可用时回退到单边差分，
以减少 OAK-D 稀疏深度、坡边缘和 TSDF 块边界导致的小坡误判。同一垂直栅格柱采用
置信度计数：可靠缓坡证据必须多于陡坡/朝下证据，少量无效法向不再一票否决整列坡面。
如果一个栅格柱只有不可靠法向，系统还会提取与观察方向无关的 TSDF 零交叉高度；只有
至少两个邻格共同形成 5-45° 连续坡面、邻域高度残差不超过 2 cm，且没有任何可靠陡坡
证据时才补判为可通行。高程突变会否决法向清障；水平悬空底面和孤立噪点也不会通过
该补偿规则。下降台阶仍需由独立的负障碍/断崖检测补充，不能通过向地面下方扩大主
障碍切片实现，否则会把正常地面 TSDF 一并投影为障碍。

### 独立负障碍/断崖层

真实 NVIDIA 入口默认启动 `oakd_perception/cliff_detector`。它直接复用已有深度图，
不额外生成全分辨率主机点云；节点将采样点按时间戳变换到 `base_link`，建立局部高程
栅格，并把超过 45° 坡度模型且落差至少 8 cm 的下降边缘发布到：

```text
/perception/cliff_points  (sensor_msgs/msg/PointCloud2)
```

节点还发布 `/perception/cliff_clear_points`，只用于清除视野中已经消失的旧断崖标记，
不会标记新障碍。costmap 每帧先用已观察地形清障，再重新写入当前断崖上沿，避免偶发
深度噪点永久残留在滚动地图中。

`nav2_3d_nav.yaml` 的 local/global costmap 都在 nvblox 层与膨胀层之间加载独立的
`cliff_layer`。因此断崖边缘作为致命障碍参与规划和控制，但不会改变 nvblox 正障碍
切片，也不会再把正常地面一起投影为不可达。

检查节点和输出频率：

```bash
./scripts/with_venv.sh ros2 node info /oakd_cliff_detector
./scripts/with_venv.sh ros2 topic hz /perception/cliff_points
./scripts/with_venv.sh ros2 topic hz /perception/cliff_clear_points
./scripts/with_venv.sh ros2 topic echo --once --no-arr \
  /perception/cliff_points
```

在 RViz 添加 `PointCloud2`，Topic 选择 `/perception/cliff_points`，同时观察 local
costmap。平地时消息仍持续发布，但 `width` 应为 0；对着下降台阶时，点应落在上沿，
并经 `cliff_layer` 膨胀成不可通行带。

检测参数位于 `src/oakd_perception/config/cliff_detector.yaml`：

- `expected_ground_z_m`：支撑地面在 `base_link` 中的 z，当前默认 -0.11 m。
- `min_drop_height_m`：最小断崖落差，当前 0.08 m；误报多时增大。
- `max_traversable_slope_deg`：可通行最大坡度，应与 nvblox 保持 45°。
- `grid_resolution_m`：局部高程格分辨率，当前 0.08 m。
- `min_lower_neighbors`：下降证据的最少邻格数，增大可抑制孤立深度噪点。
- `max_detectable_drop_m`：参与判断的最大下层深度差，当前 0.50 m。
- `max_terrain_height_change_m`：允许前方坡面相对当前支撑面升高的上限，当前 0.75 m；
  避免上坡面在检测侧缘之前被固定高度过滤掉。
- `min_depth_jump_m`：图像空间深度突变阈值，当前 0.12 m；下层表面被遮挡时，标记
  可见的近侧边缘。
- `detect_missing_depth_edges`：把连续无深度回波的可见边界作为保守断崖候选。

在不使用侧向传感器的 OAK-D-only 配置中，Nav2 默认使用 `DiffDrive` 安全运动模型，
将横移、倒车速度限制为 0，使麦轮底盘先转向、再朝相机视场内前进。断崖点同时接入
半径 0.48 m 的 collision monitor 停车区。这样牺牲了导航层的全向移动能力，但单个
前视相机无法为视场外横移或倒车提供断崖安全保证。

你给出的 `base_link -> oakd_camera_optical_frame` 矩阵第三列为
`[0.985, 0, -0.174]`，即光学前向轴相对水平面向下约 10°，TF 与实际俯视安装一致。
节点会使用这个 TF，因此不需要再在检测参数中重复填写相机俯角。启动最初出现一次
`base_link` 不存在、随后能持续输出变换，通常只是 TF 发布节点尚未完成启动。

这条检测规则必须同时看到上层和至少部分下层表面。若断崖下方超出量程/视场，或因
玻璃、强反光、黑色吸光材料而没有深度回波，单纯高程突变检测不会报警。此类场景需要
另加“预期地面回波缺失”安全停机规则，并在实车低速下标定；不能把未知空间直接全部
设为障碍，否则又会大面积阻塞正常导航。

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

检查 `static_mapper.slice_height_above_plane_m`、`slice_height_thickness_m` 以及
`esdf_slice_min_height/max_height`。当前真实配置处理拟合地面以上约 `0.03-0.36m` 的
高度带：下限用于避开地面本身；上限覆盖 `base_link` 离地约
`0.11m` 加 OAK-D 顶部约 `0.243m` 的整机包络。修改机器人或传感器安装高度后必须
同步调整并重新验证。可订阅
`/nvblox_node/groundplane_estimator_estimated_plane` 检查地面拟合是否稳定。

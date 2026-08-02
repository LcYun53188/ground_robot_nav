# Gazebo 建图测试指南

本文档说明如何使用项目内置的 RMUC 2025 Gazebo 场地验证仿真 OAK-D、
nvblox 和 Nav2 建图闭环。该流程使用 Gazebo 真值里程计替代真实 cuVSLAM，适合在
接入相机和底盘前检查深度图、TF、三维建图、二维地图切片和导航控制。

## 建图链路

```text
Gazebo RGB-D 相机
    -> /rgbd_camera/depth_image
    -> /rgbd_camera/camera_info
    -> nvblox TSDF / ESDF
    -> /nvblox_node/static_map_slice
    -> Nav2 local/global costmap

Gazebo 全向底盘里程计
    -> /visual_slam/tracking/odometry
    -> odom -> base_link
```

仿真不会启动真实 OAK-D、真实 cuVSLAM 或底盘串口桥，不能用于验证相机标定、
视觉跟踪质量或真实底盘通信。

## 1. 构建

修改过 Gazebo world、模型、launch 或配置后先构建：

```bash
./scripts/with_venv.sh colcon build --symlink-install \
  --packages-up-to omni_bringup nvblox_ros nvblox_nav2
source install/setup.bash
```

如果工作区已经构建完成，可以直接启动。

## 2. 启动 RMUC 2025 建图

```bash
./simulation/scripts/run_rmuc_2025_sim.sh
```

该入口默认启动：

- RMUC 2025 Gazebo 场地和四轮全向机器人。
- 仿真 OAK-D RGB-D、CameraInfo 和 IMU bridge。
- Gazebo 里程计与 TF bridge。
- nvblox、Nav2 和 RViz。
- OAK-D bridge 开启，MID360 bridge 关闭。
- 自动目标关闭，由操作者手动发送目标。

脚本启动前会清理本工作区遗留的 Gazebo launch，并通过锁文件阻止重复实例。

需要自动巡航扩展地图时运行：

```bash
./simulation/scripts/run_rmuc_2025_sim.sh launch_auto_goals:=true
```

只验证传感器、不启动 nvblox/Nav2：

```bash
./simulation/scripts/run_rmuc_2025_sim.sh launch_navigation:=false
```

## 3. 启动后的基础检查

另开终端执行：

```bash
env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
./scripts/with_venv.sh ros2 topic list --no-daemon | \
  grep -E 'clock|rgbd_camera|visual_slam/tracking/odometry|nvblox_node/static_map_slice'
```

检查深度和地图切片频率：

```bash
env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
timeout 8s ./scripts/with_venv.sh ros2 topic hz /rgbd_camera/depth_image

env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
timeout 8s ./scripts/with_venv.sh ros2 topic hz /nvblox_node/static_map_slice
```

检查 TF：

```bash
env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
timeout 5s ./scripts/with_venv.sh ros2 run tf2_ros tf2_echo odom base_link
```

启动最初几秒可能出现等待 `odom` 的提示；Gazebo bridge 开始发布后应自动恢复。
如果该提示持续出现，先检查 `/clock`、`/tf` 和
`/visual_slam/tracking/odometry` 是否有数据。

## 4. 移动机器人并扩展地图

在 RViz 顶部选择 `2D Goal Pose`，在可通行区域拖出目标位置和朝向。也可以从命令行
发送项目默认的第一个短距离测试目标：

```bash
env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
./scripts/with_venv.sh ros2 action send_goal \
  /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: odom}, pose: {position: {x: 1.35, y: -1.15, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
```

建图测试不能只观察静止帧。至少让机器人完成一次平移和转向，并确认：

- RViz 中 nvblox map slice 随机器人视野增加。
- `/visual_slam/tracking/odometry` 的位置随运动变化。
- local/global costmap 中出现相机观察到的障碍。
- Nav2 目标最终返回 `SUCCEEDED`，或失败时能给出明确的规划/控制原因。

查看地图元数据而不打印完整数组：

```bash
env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
./scripts/with_venv.sh ros2 topic echo --once --no-arr \
  /nvblox_node/static_map_slice
```

仿真配置位于 `src/omni_bringup/config/nvblox_isaac_sim.yaml`。当前主要参数为：

- `global_frame: odom`
- `voxel_size: 0.035`
- `mapping_type: static_tsdf`
- `min_height: 0.03`
- `max_height: 1.20`
- `publish_mesh: true`
- `decay_tsdf_rate_hz: 0.0`：关闭静态 TSDF 衰减。
- `map_clearing_radius_m: -1.0`：关闭机器人半径外地图清理。

因此 nvblox 底层地图会保留本次进程启动以来集成的全部区域，`save_map` 和
`save_ply` 保存的是完整累计地图。Nav2 local/global costmap 仍然是滚动窗口，RViz
中的 costmap 随机器人移动而刷新不代表底层地图被删除。

修改这些参数后必须重启建图进程；参数修改前已经被清理或衰减的数据无法恢复，需要
重新遍历场地建图。关闭清理后，GPU 显存和主机内存会随探索范围持续增长。

## 5. 保存建图结果

创建输出目录：

```bash
MAP_DIR="$(pwd)/maps/rmuc_2025"
mkdir -p "$MAP_DIR"
```

保存可重新加载的 nvblox 地图：

```bash
env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
./scripts/with_venv.sh ros2 service call \
  /nvblox_node/save_map nvblox_msgs/srv/FilePath \
  "{file_path: '$MAP_DIR/rmuc_2025.nvblx'}"
```

导出 PLY，供 CloudCompare 或 MeshLab 检查：

```bash
env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
./scripts/with_venv.sh ros2 service call \
  /nvblox_node/save_ply nvblox_msgs/srv/FilePath \
  "{file_path: '$MAP_DIR/rmuc_2025.ply'}"
```

检查服务响应为 `success: true`，并确认文件存在：

```bash
ls -lh "$MAP_DIR"
```

当前 nvblox 版本保存 `.nvblx` 时可能提示 mesh layer 无法序列化，但 TSDF、ESDF 等
体素层仍可成功保存。需要三维网格文件时应另外调用 `save_ply`。

### 5.1 离线查看 PLY 三维网格

`.ply` 是可直接查看的三维网格文件，适合检查重建范围、表面完整性和异常噪点。
推荐使用 MeshLab 或 CloudCompare。Ubuntu 可以任选其一安装：

```bash
sudo apt update
sudo apt install meshlab
# 或者：sudo apt install cloudcompare
```

在工作区根目录直接打开保存的地图：

```bash
MAP_DIR="$(pwd)/maps/rmuc_2025"
meshlab "$MAP_DIR/rmuc_2025.ply"
# CloudCompare 对应命令：CloudCompare "$MAP_DIR/rmuc_2025.ply"
```

也可以启动查看器后，通过 `File -> Open/Import Mesh` 选择
`maps/rmuc_2025/rmuc_2025.ply`。PLY 用于离线查看，不需要启动 ROS、Gazebo 或
nvblox。

### 5.2 加载 NVBLX 并在 RViz 查看

`.nvblx` 是 nvblox 的可重新加载地图，不能作为普通三维模型直接打开。先启动包含
nvblox 和 RViz 的仿真链路：

```bash
./simulation/scripts/run_rmuc_2025_sim.sh
```

等待 `/nvblox_node/load_map` 服务可用后，在另一个终端加载地图：

```bash
MAP_DIR="$(pwd)/maps/rmuc_2025"

env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
./scripts/with_venv.sh ros2 service call \
  /nvblox_node/load_map nvblox_msgs/srv/FilePath \
  "{file_path: '$MAP_DIR/rmuc_2025.nvblx'}"
```

服务返回 `success: true` 后，在 RViz 中展开并查看：

- `Nvblox 3D -> Reconstructed Mesh`：三维重建网格。
- `Built Maps -> Nvblox Static Occupancy`：二维占据地图。
- `Built Maps -> Local Costmap`：局部导航代价地图。

如果启动入口未打开 RViz，可以另开终端：

```bash
RVIZ_FIXED_FRAME=odom ./scripts/run_rviz_nav.sh \
  -d src/omni_bringup/rviz/gazebo_sensor_check.rviz
```

`.nvblx` 中的 mesh layer 可能无法被当前版本完整序列化；此时二维地图和体素层仍可
恢复，但 `Reconstructed Mesh` 可能为空。需要可靠查看保存时的三维网格，应使用同次
保存生成的 `.ply` 文件。

## 6. 当前测试基线

2026-08-02 在 RMUC 2025 场地完成过一次短距离冒烟测试：

- 仿真深度图约 `10.7 Hz`。
- nvblox map slice 约 `4.1 Hz`。
- 机器人从原点到达约 `(1.31, -1.09)`，Nav2 返回 `SUCCEEDED`。
- 移动后 map slice 分辨率为 `0.035m`，一次采样尺寸为 `217 x 264`。
- `.nvblx` 保存服务成功，短距离测试地图约 `17MB`。

频率和地图尺寸会随 GPU 负载、场景可见范围和机器人位置变化，不应作为严格实时指标。

## 7. 常见问题

### 地图为空

依次检查：

```bash
./scripts/with_venv.sh ros2 topic echo --once /rgbd_camera/depth_image/header
./scripts/with_venv.sh ros2 topic echo --once /rgbd_camera/camera_info/header
./scripts/with_venv.sh ros2 run tf2_ros tf2_echo odom oakd_camera_optical_frame
```

深度图和 CameraInfo 的 frame 必须一致，并且该 frame 必须能转换到 `odom`。

### RViz 出现 GLSL 或 geometry shader 提示

nvblox RViz 插件会在缺少 geometry shader 时使用回退渲染。若地图和 costmap 仍持续
刷新，该提示通常不影响建图；若 RViz 崩溃，可关闭内置 RViz 后使用精简配置重启：

```bash
./simulation/scripts/run_rmuc_2025_sim.sh launch_rviz:=false
RVIZ_FIXED_FRAME=odom ./scripts/run_rviz_nav.sh \
  -d src/omni_bringup/rviz/nvblox_map_check.rviz
```

### 机器人不能横移或旋转

不要把 Gazebo 物理引擎改为 Bullet Featherstone。当前麦轮模型依赖 DART 对各向异性
接触摩擦的支持。

### 出现重复 TF 或机器人跳动

只保留一个 Gazebo 实例。重新运行 `run_rmuc_2025_sim.sh` 会尝试清理旧实例；若仍有
残留，先退出旧 Gazebo 和 ROS launch，再重新启动。

# OAK-D Visual SLAM 与 RViz 验证

本文档用于启动并验证 OAK-D 双目 + IMU 的 Isaac ROS Visual SLAM 视觉里程计。该入口只启动 OAK-D、Visual SLAM、静态 TF 和 RViz，不启动 nvblox、Nav2、ESS 或底盘串口桥，适合排查 VIO、坐标轴和 RViz 显示问题。

## 启动命令

源码路径启动，不需要重新 install：

```bash
env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
./scripts/with_venv.sh ros2 launch src/omni_bringup/launch/oakd_visual_slam_rviz.launch.py
```

如果已经重新 build/install，也可以用包名启动：

```bash
env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
./scripts/with_venv.sh ros2 launch omni_bringup oakd_visual_slam_rviz.launch.py
```

如果 RViz 没有加载预设，强制指定源码 RViz 配置：

```bash
env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
./scripts/with_venv.sh ros2 launch src/omni_bringup/launch/oakd_visual_slam_rviz.launch.py \
  rviz_config:=/home/nuc/Program/ground_robot_nav_ws/src/omni_bringup/rviz/visual_slam_check.rviz
```

## 启动内容

该 launch 文件会启动：

- `oakd_unified_node`
- Isaac ROS Visual SLAM
- `base_link -> oakd_imu_link` 静态 TF
- `oakd_imu_link -> oakd_camera_optical_frame` 静态 TF
- RViz，默认配置为 `visual_slam_check.rviz`

该 launch 文件不会启动：

- nvblox
- Nav2
- ESS
- `ground_serial_bridge`
- OAK-D depth image
- OAK-D PointCloud2

## 默认坐标关系

默认假设 OAK-D 向前、水平放置，且机器人中心暂时等于 OAK-D 中心：

```text
oakd_x = 0
oakd_y = 0
oakd_z = 0
oakd_yaw = 0
oakd_pitch = pi/2
oakd_roll = pi
```

这个安装旋转用于保持：

```text
base_link:
  X forward
  Y left
  Z up

oakd_camera_optical_frame:
  Z forward
  X right
  Y down
```

RViz 中颜色含义：

```text
红色 = X
绿色 = Y
蓝色 = Z
```

判断机器人姿态时看 `base_link`。判断相机成像方向时看 `oakd_camera_optical_frame` 的蓝色 Z 轴。

## RViz 预设

默认 RViz 配置：

```text
src/omni_bringup/rviz/visual_slam_check.rviz
```

默认显示：

- Fixed Frame: `odom`
- TF 坐标轴
- `/visual_slam/tracking/odometry`
- `/visual_slam/tracking/vo_path`
- `/oakd/left/image_raw`

默认关闭：

- `/visual_slam/tracking/slam_path`

VIO-only 配置不发布动态 `map -> odom`，所以不要用 `map` 判断 OAK-D 姿态。验证 yaw、pitch、roll 时保持 RViz `Fixed Frame = odom`，主要看 `base_link`。

## 常用检查命令

查看 Visual SLAM 状态：

```bash
./scripts/with_venv.sh ros2 topic echo --once /visual_slam/status
```

查看视觉里程计：

```bash
./scripts/with_venv.sh ros2 topic echo --once /visual_slam/tracking/odometry
```

确认 odometry 只有一个发布者：

```bash
./scripts/with_venv.sh ros2 topic info -v /visual_slam/tracking/odometry
```

检查 OAK-D 图像频率：

```bash
timeout 5s ./scripts/with_venv.sh ros2 topic hz /oakd/left/image_raw
```

检查 VIO 输出频率：

```bash
timeout 5s ./scripts/with_venv.sh ros2 topic hz /visual_slam/tracking/odometry
```

检查 TF：

```bash
timeout 5s env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
./scripts/with_venv.sh ros2 run tf2_ros tf2_echo base_link oakd_camera_optical_frame
```

期望关系：

```text
camera Z ~= base +X
camera X ~= base -Y
camera Y ~= base -Z
```

## 重置 VIO

如果 RViz 中 OAK-D 位置突然跳到很远，先重置 Visual SLAM：

```bash
./scripts/with_venv.sh ros2 service call /visual_slam/reset isaac_ros_visual_slam_interfaces/srv/Reset {}
```

重置后保持 OAK-D 静止 5-10 秒，再小幅移动验证。

## 停止命令

```bash
pkill -f rviz2
pkill -f "ros2 launch.*oakd_visual_slam_rviz"
pkill -f "ros2 launch.*nvidia_3d_nav"
pkill -f "oakd_unified|visual_slam|static_transform_publisher"
```

## 常见问题

### RViz 没有加载预设

优先用源码路径启动，或显式传入 `rviz_config`：

```bash
rviz_config:=/home/nuc/Program/ground_robot_nav_ws/src/omni_bringup/rviz/visual_slam_check.rviz
```

### 位置乱跳或跳到百米级

通常是 Visual SLAM 初始化或跟踪发散。先确认：

- `/visual_slam/tracking/odometry` 只有一个发布者
- `Fixed Frame = odom`
- OAK-D 启动后静止 5-10 秒
- 画面有足够纹理和光照

然后调用 `/visual_slam/reset`。

### 抬头/低头方向看起来相反

先确认你看的是 `base_link`，不是 `oakd_camera_optical_frame`。相机光学坐标系的轴定义和机器人本体坐标不同。

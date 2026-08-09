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

VIO-only 入口默认使用 `oakd_stereo_quality_mode:=low_latency`。该模式仍通过
OAK-D `StereoDepth` 输出左右矫正图，但会关闭深度质量相关的 subpixel、
left-right check 和深度后处理，避免在不发布 depth/pointcloud 时把算力消耗在
Visual SLAM 不使用的深度质量上。

默认图像参数：

```text
oakd_image_frequency = 25 Hz
oakd_image_poll_frequency = 75 Hz
oakd_image_queue_size = 2
oakd_image_qos_depth = 4
oakd_image_inter_publish_delay_ms = 1.0
```

`oakd_image_queue_size` 默认较小，目的是减少旧帧堆积，让 Visual SLAM 更快拿到
最新左右目图像。`oakd_image_qos_depth` 和 `oakd_image_inter_publish_delay_ms`
用于稳定 ROS/DDS 侧连续发布的左右大图像：如果左右图像背靠背发布，第二个图像
topic 在 best-effort 链路上更容易丢样本；默认 1ms 间隔用于把两张大图像错开。

## 默认坐标关系

默认值与完整导航的低台阶安全安装一致：相机前移、降低并向下俯视 18°：

```text
oakd_x = 0.18
oakd_y = 0
oakd_z = 0.16
oakd_yaw = 0
oakd_pitch = 0.6*pi
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
- `/visual_slam/guarded_odometry`
- `/odometry/filtered`
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

查看跳变保护后的视觉里程计：

```bash
./scripts/with_venv.sh ros2 topic echo --once /visual_slam/guarded_odometry
```

查看跳变保护状态：

```bash
./scripts/with_venv.sh ros2 topic echo /visual_slam/odom_guard/status
```

查看 EKF 输出：

```bash
./scripts/with_venv.sh ros2 topic echo --once /odometry/filtered
```

确认 odometry 只有一个发布者：

```bash
./scripts/with_venv.sh ros2 topic info -v /visual_slam/tracking/odometry
```

检查 OAK-D 图像频率：

```bash
timeout 5s ./scripts/with_venv.sh ros2 topic hz /oakd/left/image_raw
```

注意：当前 ROS 2 Jazzy 的 `ros2 topic hz` 不能为图像 topic 指定
`best_effort` QoS，默认可靠订阅可能与 OAK-D 图像发布 QoS 不兼容。图像频率
精确排查时应使用 best-effort 订阅脚本或自定义探针；`ros2 topic hz` 的结果只
适合作为粗略参考。

如果仍出现 `Delta between current and previous frame` 或 OAK-D 图像最大间隔
接近 `0.078s`，先用较宽左右目配对阈值复测：

```bash
env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
./scripts/with_venv.sh ros2 launch src/omni_bringup/launch/oakd_visual_slam_rviz.launch.py \
  oakd_image_pair_max_dt_ms:=12.0
```

然后分别检查左右目、IMU 和 VIO 频率：

```bash
timeout 8s ./scripts/with_venv.sh ros2 topic hz /oakd/left/image_raw
timeout 8s ./scripts/with_venv.sh ros2 topic hz /oakd/right/image_raw
timeout 8s ./scripts/with_venv.sh ros2 topic hz /oakd/imu/raw
timeout 8s ./scripts/with_venv.sh ros2 topic hz /visual_slam/tracking/odometry
./scripts/with_venv.sh ros2 topic echo --once /visual_slam/status
```

2026-06-02 实测优化后，在 RViz 开启、默认参数下，best-effort 探针 20 秒采样：

```text
/oakd/left/image_raw:  25.000 Hz, max interval 0.040s
/oakd/right/image_raw: 25.000 Hz, max interval 0.040s
/visual_slam/tracking/odometry: 25.000 Hz, max interval 0.040s
/visual_slam/status: SUCCESS
```

如果关闭 `oakd_image_inter_publish_delay_ms` 或把发布顺序改为单侧优先，第二个
发布的大图像 topic 可能下降到约 17-21Hz；因此默认保留 1ms 间隔。

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

## Odom 跳变保护

RViz 验证入口默认启动 `nav_guard/visual_odom_guard` 和
`robot_localization/ekf_node`。链路为：

```text
/visual_slam/tracking/odometry
  -> /visual_slam/guarded_odometry
  -> /odometry/filtered
  -> odom -> base_link
```

默认由 EKF 发布 `odom -> base_link`；launch 会关闭 Visual SLAM 原始 TF，避免
双发布。若要只看原始 Visual SLAM TF，可启动：

RViz 验证入口默认使用 `ekf_visual_slam_3d.yaml`，关闭 `two_d_mode`，完整保留
cuVSLAM 的 `odom -> base_link` 姿态。这样验证 roll、pitch、yaw 时不会被二维
导航约束压平。完整导航入口 `nvidia_3d_nav.launch.py` 默认仍使用
`ekf_visual_slam.yaml`，该配置启用二维约束，适合 Nav2 使用。

```bash
env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
./scripts/with_venv.sh ros2 launch src/omni_bringup/launch/oakd_visual_slam_rviz.launch.py \
  launch_robot_localization:=false \
  odom_guard_publish_tf:=false
```

独立 IMU 不在 RViz 验证入口默认启用。需要验证独立 IMU 时，使用
`ekf_visual_slam_with_independent_imu.yaml`，并先确认 IMU 静态 TF 和轴向。

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

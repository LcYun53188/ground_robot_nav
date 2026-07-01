# Gazebo Harmonic / ros_gz 仿真

本文档说明当前推荐的无实物仿真路径。该路径不启动真实 OAK-D、不启动真实
cuVSLAM，而是用 Gazebo Harmonic 发布仿真 RGB-D、IMU、里程计、TF 和 `/clock`，
ROS 侧继续复用现有 nvblox + Nav2 导航配置。

## 范围

当前 Gazebo 入口用于验证：

- 四轮全向底盘在 Gazebo 中响应 `/cmd_vel`。
- `ros_gz_bridge` 把 Gazebo `/clock`、里程计、TF、IMU 和 CameraInfo 接入 ROS。
- `ros_gz_image` 把 RGB-D 图像接入 ROS。
- ROS 侧 `nvidia_3d_nav.launch.py` 在仿真模式下关闭真实 OAK-D 和真实 cuVSLAM。
- nvblox 和 Nav2 可以使用仿真 RGB-D 与 Gazebo 里程计跑闭环。

它不用于验证真实 OAK-D 标定、真实 cuVSLAM 跟踪质量或真实底盘串口协议。

## 文件入口

- Launch：`src/omni_bringup/launch/gazebo_harmonic_nav.launch.py`
- World：`src/omni_bringup/gazebo/worlds/omni_harmonic_demo.sdf`
- Bridge：`src/omni_bringup/gazebo/config/omni_gazebo_bridge.yaml`
- 启动脚本：`simulation/scripts/run_gazebo_harmonic_nav.sh`

## 启动

先确认已构建并 source 当前工作区：

```bash
./scripts/with_venv.sh colcon build --symlink-install --packages-select omni_bringup
source install/setup.bash
```

启动 Gazebo + ROS 侧导航：

```bash
./simulation/scripts/run_gazebo_harmonic_nav.sh
```

默认会同时启动：

- Gazebo Harmonic UI。
- `ros_gz_bridge` / `ros_gz_image`。
- nvblox + Nav2。
- RViz，使用 `src/omni_bringup/rviz/nvblox_map_check.rviz`。
- `gazebo_auto_goals`，循环向 Nav2 发送目标点。

另开终端启动 RViz：

```bash
RVIZ_FIXED_FRAME=odom ./scripts/run_rviz_nav.sh -d src/omni_bringup/rviz/nvblox_map_check.rviz
```

如果要手动启动 RViz，可关闭 launch 内置 RViz：

```bash
./simulation/scripts/run_gazebo_harmonic_nav.sh launch_rviz:=false
```

只启动 Gazebo 和 bridge，不启动 nvblox/Nav2/自动目标/RViz：

```bash
./simulation/scripts/run_gazebo_harmonic_nav.sh \
  launch_navigation:=false launch_auto_goals:=false launch_rviz:=false
```

只启动 Gazebo server，不打开 Gazebo GUI：

```bash
./simulation/scripts/run_gazebo_harmonic_nav.sh \
  launch_rviz:=false \
  gz_args:="-r -s -v 3 /home/nuc/Program/ground_robot_nav_ws/install/omni_bringup/share/omni_bringup/gazebo/worlds/omni_harmonic_demo.sdf"
```

只启动 ROS 侧导航，连接已经运行的 Gazebo：

```bash
./simulation/scripts/run_gazebo_harmonic_nav.sh launch_gazebo:=false
```

## 话题映射

Gazebo 侧通过 `ros_gz` 接入当前导航主线：

| ROS 话题 | 来源 | 作用 |
| --- | --- | --- |
| `/clock` | Gazebo | 仿真时间 |
| `/cmd_vel` | Nav2 -> Gazebo | 全向底盘速度命令 |
| `/visual_slam/tracking/odometry` | Gazebo mecanum odom | 仿真中替代真实 cuVSLAM 里程计 |
| `/tf` | Gazebo | `odom -> base_link` 和模型 TF |
| `/rgbd_camera/image` | Gazebo RGB-D | nvblox color 输入 |
| `/rgbd_camera/depth_image` | Gazebo RGB-D | nvblox depth 输入 |
| `/rgbd_camera/camera_info` | Gazebo RGB-D | nvblox CameraInfo 输入 |
| `/oakd/imu/raw` | Gazebo IMU | IMU 仿真输出，当前导航闭环不强依赖 |

`gazebo_harmonic_nav.launch.py` 会把现有 `nvidia_3d_nav.launch.py` 配成仿真模式：

- `use_sim_time:=true`
- `launch_oakd:=false`
- `launch_visual_slam:=false`
- `launch_ground_bridge:=false`
- `launch_odom_guard:=false`
- `launch_robot_localization:=false`

因此 Gazebo 里程计直接作为仿真定位源。真实硬件运行仍使用
`nvidia_3d_nav.launch.py` 的默认 cuVSLAM 主线。

## 自定义地图和机器人

替换 world：

```bash
./simulation/scripts/run_gazebo_harmonic_nav.sh \
  world:=/absolute/path/to/custom_world.sdf
```

当前最小 world 内联了一个简化四轮 mecanum 模型。后续导入真实机器人模型时，保持
这些接口不变即可复用 ROS 侧配置：

- Gazebo 订阅 `/cmd_vel`，类型 `gz.msgs.Twist`。
- Gazebo 发布 `/visual_slam/tracking/odometry`，类型 `gz.msgs.Odometry`。
- Gazebo 发布 `/tf`，包含 `odom -> base_link`。
- RGB-D 相机发布 `/rgbd_camera/image`、`/rgbd_camera/depth_image` 和
  `/rgbd_camera/camera_info`。

## 自动目标点

默认目标点在 `odom` 坐标系中循环：

```json
[
  {"x": 1.35, "y": -1.15, "yaw": 0.0},
  {"x": -1.20, "y": 1.20, "yaw": 1.57},
  {"x": 1.15, "y": 1.25, "yaw": 3.14},
  {"x": -1.35, "y": -1.05, "yaw": -1.57}
]
```

覆盖目标点：

```bash
./simulation/scripts/run_gazebo_harmonic_nav.sh \
  auto_goals_json:='[{"x": 1.0, "y": 0.8, "yaw": 0.0}, {"x": -1.0, "y": -0.8, "yaw": 3.14}]'
```

关闭自动目标：

```bash
./simulation/scripts/run_gazebo_harmonic_nav.sh launch_auto_goals:=false
```

## 快速检查

```bash
./scripts/with_venv.sh ros2 topic list | grep -E 'clock|cmd_vel|visual_slam|rgbd_camera'
./scripts/with_venv.sh ros2 topic hz /visual_slam/tracking/odometry
./scripts/with_venv.sh ros2 topic hz /rgbd_camera/depth_image
./scripts/with_venv.sh ros2 topic echo --once /clock
```

如果 Gazebo UI 没显示或渲染失败，先用无导航模式验证 Gazebo 本身：

```bash
./simulation/scripts/run_gazebo_harmonic_nav.sh launch_navigation:=false
```

# Gazebo Harmonic / ros_gz 仿真

本文档说明当前推荐的无实物仿真路径。该路径不启动真实 OAK-D、不启动真实
cuVSLAM，而是用 Gazebo Harmonic 发布仿真 RGB-D、MID360 点云/IMU、里程计、TF
和 `/clock`，ROS 侧继续复用现有 nvblox + Nav2 导航配置。

## 范围

当前 Gazebo 入口用于验证：

- 四轮全向底盘在 Gazebo 中响应 `/cmd_vel`。
- `ros_gz_bridge` 把 Gazebo `/clock`、里程计、TF、IMU 和 CameraInfo 接入 ROS。
- `ros_gz_image` 把 RGB-D 图像接入 ROS。
- `ros_gz_bridge` 把仿真 MID360 点云和 IMU 接入 ROS。
- ROS 侧 `nvidia_3d_nav.launch.py` 在仿真模式下关闭真实 OAK-D 和真实 cuVSLAM。
- nvblox 和 Nav2 可以使用仿真 RGB-D 与 Gazebo 里程计跑闭环。
- OAK-D 深度断崖检测接入独立 costmap 层和 collision monitor；该链路不使用 MID360。

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

选择项目内置 RMUC/RMUL 场地：

```bash
./simulation/scripts/run_gazebo_harmonic_nav.sh arena:=rmuc_2024
./simulation/scripts/run_gazebo_harmonic_nav.sh arena:=rmuc_2025
./simulation/scripts/run_gazebo_harmonic_nav.sh arena:=rmul_2024
./simulation/scripts/run_gazebo_harmonic_nav.sh arena:=rmul_2025
```

场地被平移到参考项目红方机器人出生点为 `odom` 原点，因此现有 Nav2、真值里程计
和默认局部目标仍使用以启动位置为原点的坐标，不需要引入机器人命名空间。
启动入口默认选择 DART 物理引擎。四个麦轮通过 `fdir1` 各向异性摩擦模拟滚子方向，
该模型在 Bullet Featherstone 下会退化为只能沿车体 X 轴运动；覆盖 `gz_args` 时应
保留 `--physics-engine gz-physics-dartsim-plugin`。当前 Gazebo Harmonic / DART
可以正确加载这些场地的静态三角网格碰撞。

轮面主摩擦系数为 `mu=2.0`，OAK-D-only 安全配置把滚子方向摩擦提高到 `mu2=0.6`，
减少坡面横向侧滑；底盘碰撞体底面相对轮底留有 `60mm` 间隙，避免在坡顶折角托底。
当前参数已用 `0.35m/s` 前进速度通过 `15deg`、长 `2m` 的斜坡和坡顶过渡。更陡的
坡仍应按实车允许坡度单独验证，不应仅因 nvblox 允许最高 `45deg` 地面坡度，就视为
底盘具备相同的物理爬坡能力。

默认会同时启动：

- Gazebo Harmonic UI。
- `ros_gz_bridge` / `ros_gz_image`。
- nvblox + Nav2。
- RViz，使用 `src/omni_bringup/rviz/nvblox_map_check.rviz`。
- `gazebo_auto_goals`，循环向 Nav2 发送目标点。

提高 `mu2` 会牺牲一部分麦轮横移真实性，这与默认禁用横移的安全策略一致。虽然
Gazebo 底盘本身仍支持麦轮全向运动，默认 OAK-D-only 导航安全配置使用
`DiffDrive` 运动模型并禁止横移、倒车，让机器人先朝行驶方向转向。原因是单个前视
OAK-D 无法保护视场外的侧向和后向断崖；这不是 MID360 融合方案。

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
| `/mid360/points` | Gazebo GPU lidar | MID360 标准 `PointCloud2` 仿真输出 |
| `/mid360/imu` | Gazebo IMU | MID360 同机 IMU 仿真输出 |

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
- MID360 发布 `/mid360/points` 和 `/mid360/imu`，frame 为 `mid360_link`。

MID360 位于车体中心右前方 `45deg`：以 `base_link` 的 `+X` 为前方、`-Y` 为
右侧，水平安装半径为 `0.16m`，坐标约为 `(0.1131, -0.1131, 0.18)m`。其坐标轴
yaw 保持 `45deg`，不改变 360° 点云覆盖范围。

OAK-D 的机械安装坐标系为 `oakd_camera_link`，相对 `base_link` 位于
`(0.18, 0, 0.22)`，其 Gazebo 视轴 `+X` 与车体前进方向一致，并绕车体 `Y` 轴向
地面俯视 `10deg`。ROS 图像使用独立的标准光学坐标系
`oakd_camera_optical_frame`。Gazebo 中车体、OAK-D 和 MID360 分别使用蓝色、橙色
和绿色，并在 OAK-D 前表面使用黑色双镜头标记前向。

## RMUC/RMUL 场地资源

项目内置以下场地，不需要克隆或构建其他 ROS 项目：

- `rmuc_2024`
- `rmuc_2025`
- `rmul_2024`
- `rmul_2025`

场地 mesh 来自 `SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator` 的
`36e3cf423448cbef12fa2c56f61f81eade8e286d` 提交。来源和许可证副本保存在
`src/omni_bringup/gazebo/models/THIRD_PARTY_NOTICES.md` 和
`src/omni_bringup/gazebo/models/rmu_gazebo_simulator_LICENSE`。

## MID360 仿真边界

仿真 MID360 参照参考模型采用 `20Hz`、水平 `1875` 点、垂直 `32` 线、约
`-7deg` 到 `52deg` 的 GPU lidar 扫描，并使用相同的倾斜安装姿态。它是规则栅格
扫描近似，不模拟真实 Livox 非重复扫描 pattern，也不提供 Livox 自定义消息中的
逐点时间、line、tag 等字段。因此 `/mid360/points` 可直接用于障碍感知和
PointCloud2 算法验证，但不能用于验证依赖真实 Livox 逐点时序的 FAST-LIO 行为。

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
./scripts/with_venv.sh ros2 topic list | grep -E 'clock|cmd_vel|visual_slam|rgbd_camera|mid360'
./scripts/with_venv.sh ros2 topic hz /visual_slam/tracking/odometry
./scripts/with_venv.sh ros2 topic hz /rgbd_camera/depth_image
./scripts/with_venv.sh ros2 topic hz /mid360/points
./scripts/with_venv.sh ros2 topic echo --once /mid360/imu
./scripts/with_venv.sh ros2 topic echo --once /clock
```

如果 Gazebo UI 没显示或渲染失败，先用无导航模式验证 Gazebo 本身：

```bash
./simulation/scripts/run_gazebo_harmonic_nav.sh launch_navigation:=false
```

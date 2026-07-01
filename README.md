# 地面全向轮机器人导航工作区

本工作区面向地面全向轮机器人，不再按无人机导航栈设计。当前主线是以
OAK-D 为第一版核心传感器，使用 Isaac ROS Visual SLAM / cuVSLAM 提供视觉
里程计，使用 nvblox 构建 3D 障碍地图，使用 Nav2 和 `ground_serial_bridge`
完成规划、控制与底盘串口输出。

第一版约束：

- 不依赖可靠轮速里程计。
- 不在首版引入 OAK-D 与 MID360 的复杂跨设备时间戳对齐。
- MID360 暂不参与主定位，后续作为局部安全障碍层接入。
- OAK-D + Visual SLAM + nvblox + Nav2 是当前优先闭环。

## 文档入口

- [docs/INDEX.md](docs/INDEX.md)：文档索引。
- [docs/INSTALLATION.md](docs/INSTALLATION.md)：虚拟环境、依赖安装与构建说明。
- [docs/CUDA_TOOLKIT_13_2_INSTALLATION.md](docs/CUDA_TOOLKIT_13_2_INSTALLATION.md)：CUDA Toolkit 13.2 / Isaac ROS 构建环境。
- [docs/OAKD_VISUAL_SLAM_RVIZ.md](docs/OAKD_VISUAL_SLAM_RVIZ.md)：OAK-D + Visual SLAM + RViz 硬件验证。
- [docs/NVIDIA_3D_NAV_ARCHITECTURE.md](docs/NVIDIA_3D_NAV_ARCHITECTURE.md)：NVIDIA 3D 导航架构说明。
- [docs/NVIDIA_3D_NAV_PROJECT_PLAN.md](docs/NVIDIA_3D_NAV_PROJECT_PLAN.md)：分阶段迁移计划。
- [docs/GAZEBO_HARMONIC_SIMULATION.md](docs/GAZEBO_HARMONIC_SIMULATION.md)：Gazebo Harmonic + ros_gz 仿真入口。
- [docs/ISAAC_SIM_SIMULATION.md](docs/ISAAC_SIM_SIMULATION.md)：Isaac Sim 4.5 / 轻量仿真验证。

## 环境配置

项目约定：

- 使用 `.venv` 虚拟环境。
- 使用 `uv` 作为 Python 包管理器。
- ROS 命令优先通过 `./scripts/with_venv.sh` 执行，避免系统 Python 与虚拟环境混用。
- Isaac ROS / nvblox 构建需要可用的 NVIDIA 驱动、CUDA Toolkit、VPI 和相关 ROS 依赖。

基础环境：

```bash
cd /home/nuc/Program/ground_robot_nav_ws

# 如 .venv 尚未创建，按 docs/INSTALLATION.md 完整配置。
source .venv/bin/activate

# 推荐通过包装脚本运行 ROS 命令。
./scripts/with_venv.sh ros2 topic list
```

CUDA / Isaac ROS 相关构建前，先确认：

```bash
nvidia-smi
nvcc --version
echo "$CUDA_HOME"
echo "$CUDAToolkit_ROOT"
```

详细环境步骤见 [docs/INSTALLATION.md](docs/INSTALLATION.md) 和
[docs/CUDA_TOOLKIT_13_2_INSTALLATION.md](docs/CUDA_TOOLKIT_13_2_INSTALLATION.md)。

## 构建项目

构建当前地面导航栈：

```bash
./scripts/build_ground_stack.sh
source install/setup.bash
```

调试 OAK-D / Visual SLAM 相关改动时，可以只构建相关包：

```bash
./scripts/with_venv.sh colcon build --symlink-install \
  --packages-select oakd_perception omni_bringup
source install/setup.bash
```

如果构建 Isaac ROS / nvblox 相关包失败，优先查看：

- [docs/CUDA_TOOLKIT_13_2_INSTALLATION.md](docs/CUDA_TOOLKIT_13_2_INSTALLATION.md)
- [docs/NVIDIA_3D_NAV_ARCHITECTURE.md](docs/NVIDIA_3D_NAV_ARCHITECTURE.md)

## 验证硬件

### OAK-D Visual SLAM 单独验证

在调试完整 nvblox / Nav2 闭环前，先用 OAK-D VIO 验证入口确认双目、
IMU、TF 和 Visual SLAM 是否正常：

```bash
env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
./scripts/with_venv.sh ros2 launch src/omni_bringup/launch/oakd_visual_slam_rviz.launch.py
```

该入口只启动：

- `oakd_unified_node`
- Isaac ROS Visual SLAM
- OAK-D 静态 TF
- RViz

该入口不启动：

- nvblox
- Nav2
- ESS
- `ground_serial_bridge`
- OAK-D depth image
- OAK-D PointCloud2

VIO-only 入口默认使用 `oakd_stereo_quality_mode:=low_latency`。该模式仍通过
OAK-D `StereoDepth` 输出左右矫正图，但关闭 Visual SLAM 不使用的深度质量
相关计算，以降低双目图像链路延迟。

OAK-D 图像默认目标帧率为 `25Hz`，图像轮询频率为 `75Hz`，host 端左右目队列默认
为 `2`，图像 ROS publisher QoS depth 默认为 `4`，左右大图像之间默认错开 `1ms`
发布。队列较小用于减少旧帧堆积，QoS depth 和 1ms 间隔用于避免连续发布左右大图时
第二个 best-effort 图像 topic 明显掉样本。

如果出现 Visual SLAM 帧间隔警告或 OAK-D 图像间隔抖动，可先只放宽左右目配对阈值复测：

```bash
env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
./scripts/with_venv.sh ros2 launch src/omni_bringup/launch/oakd_visual_slam_rviz.launch.py \
  oakd_image_pair_max_dt_ms:=12.0
```

常用检查命令：

```bash
timeout 8s ./scripts/with_venv.sh ros2 topic hz /oakd/left/image_raw
timeout 8s ./scripts/with_venv.sh ros2 topic hz /oakd/right/image_raw
timeout 8s ./scripts/with_venv.sh ros2 topic hz /oakd/imu/raw
timeout 8s ./scripts/with_venv.sh ros2 topic hz /visual_slam/tracking/odometry
./scripts/with_venv.sh ros2 topic echo --once /visual_slam/status
```

注意：`ros2 topic hz` 对当前 best-effort 图像 topic 只能做粗略参考。精确验证左右目
频率时，应使用 best-effort QoS 探针。2026-06-02 在 RViz 开启、默认参数下，20 秒
采样结果为左右目和 `/visual_slam/tracking/odometry` 均 `25.000Hz`。

完整流程见 [docs/OAKD_VISUAL_SLAM_RVIZ.md](docs/OAKD_VISUAL_SLAM_RVIZ.md)。

### USB 与设备状态

如果 OAK-D 左右目频率达不到目标值，或最大间隔反复出现约 `0.078s`，优先确认
USB 3.x 链路、线材和供电：

```bash
lsusb -t
```

如果出现 `X_LINK_DEVICE_ALREADY_IN_USE`，先停止旧的 OAK-D / launch 进程后再重试。

## 运行导航栈

完整 NVIDIA 3D 导航入口：

```bash
env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
./scripts/with_venv.sh ros2 launch omni_bringup nvidia_3d_nav.launch.py
```

当前主链路：

```text
OAK-D stereo + IMU
    -> Isaac ROS Visual SLAM / cuVSLAM
    -> /visual_slam/tracking/odometry
    -> visual_odom_guard
    -> /visual_slam/guarded_odometry
    -> robot_localization EKF
    -> /odometry/filtered
    -> odom -> base_link
OAK-D depth
    -> nvblox 3D map
    -> Nav2 costmap
    -> Nav2 controller
    -> /cmd_vel
    -> ground_serial_bridge
```

常用运行参数：

```bash
# 只验证 OAK-D + Visual SLAM，不启动 nvblox/Nav2/底盘桥。
./scripts/with_venv.sh ros2 launch src/omni_bringup/launch/oakd_visual_slam_rviz.launch.py

# 完整 NVIDIA 3D 导航入口。
./scripts/with_venv.sh ros2 launch omni_bringup nvidia_3d_nav.launch.py

# 如需调整 OAK-D 安装外参。
./scripts/with_venv.sh ros2 launch omni_bringup nvidia_3d_nav.launch.py \
  oakd_x:=0.12 oakd_y:=0.0 oakd_z:=0.28
```

## 关键 Topic

OAK-D 输入：

- `/oakd/left/image_raw`
- `/oakd/right/image_raw`
- `/oakd/left/camera_info`
- `/oakd/right/camera_info`
- `/oakd/imu/raw`
- `/oakd/depth/image`
- `/oakd/depth/camera_info`
- `/oakd/points` 和 `/oakd/points_filtered`：可选，完整 NVIDIA 路径默认不依赖 host 端点云。

Visual SLAM / TF：

- `/visual_slam/tracking/odometry`：cuVSLAM 原始里程计，用于诊断和对比。
- `/visual_slam/guarded_odometry`：跳变保护后的里程计，作为 EKF 输入。
- `/visual_slam/odom_guard/status`：跳变保护状态；拒绝异常帧时会写明阈值原因。
- `/odometry/filtered`：`robot_localization` EKF 输出，完整导航入口默认给 Nav2 使用。
- `/visual_slam/tracking/vo_path`
- `/visual_slam/status`
- `odom -> base_link`

控制与安全：

- `/cmd_vel`
- `/nav/emergency`
- `/nav/safety_status`

## Odom 跳变保护

`nav_guard/visual_odom_guard` 会拒绝明显不符合地面机器人运动约束的 Visual SLAM
位姿跳变。默认阈值：

- 单帧 XY 位移 `> 0.20m`
- 单帧 Z 位移 `> 0.15m`
- 单帧 yaw 变化 `> 20deg`
- XY 速度 `> 1.2m/s`
- yaw 速度 `> 120deg/s`

异常帧会被 hold 为上一帧；如果连续拒绝超过 `2.0s`，保护节点会重新设定基准，
防止 Visual SLAM 初始化抖动后永久停在旧位姿。这个机制只能保护下游 Nav2 不立即
吃到离谱跳变，不能修复低纹理导致的 VIO 失效。

在 `nvidia_3d_nav.launch.py` 中，保护节点默认启用并发布受保护
`/visual_slam/guarded_odometry`；`robot_localization` 默认启用并发布
`/odometry/filtered` 和唯一的 `odom -> base_link` TF。此时 launch 会关闭
Visual SLAM 原始 `odom -> base_link` TF，避免 TF 双发布。

查看保护状态：

```bash
./scripts/with_venv.sh ros2 topic echo /visual_slam/odom_guard/status
```

## robot_localization 与独立 IMU

完整导航入口默认启动 `robot_localization/ekf_node`：

- 导航默认配置：`src/omni_bringup/config/ekf_visual_slam.yaml`
- 输入：`/visual_slam/guarded_odometry`
- 输出：`/odometry/filtered`
- TF：发布唯一 `odom -> base_link`

`ekf_visual_slam.yaml` 是二维导航配置，启用 `two_d_mode`，会把输出约束到
`z=0` 且 roll/pitch 为 0。它适合 Nav2 平面导航，但不适合验证 OAK-D 的
roll/pitch/yaw 轴向。

RViz OAK-D 验证入口默认使用
`src/omni_bringup/config/ekf_visual_slam_3d.yaml`。该配置关闭 `two_d_mode`，
完整保留 cuVSLAM 原始 `odom -> base_link` 姿态关系，适合检查 roll、pitch、
yaw 是否和接入 `robot_localization` 前一致。

独立 IMU 预留配置：

```bash
./scripts/with_venv.sh ros2 launch omni_bringup nvidia_3d_nav.launch.py \
  ekf_params_file:=/home/nuc/Program/ground_robot_nav_ws/src/omni_bringup/config/ekf_visual_slam_with_independent_imu.yaml
```

该配置默认使用 `/independent_imu/data`，且只融合 z 轴角速度。不要把它直接指向
`/oakd/imu/raw`，因为 cuVSLAM 已经使用 OAK-D IMU；重复融合同一个 IMU 会让权重
失真。独立 IMU 真正接入前，必须先补齐 `base_link -> independent_imu_frame`
静态 TF，并验证 IMU `frame_id`、轴向、时间戳和协方差。

## 坐标系约定

当前主 TF 关系：

```text
odom
└── base_link
    ├── base_footprint
    └── oakd_imu_link
        └── oakd_camera_optical_frame
            ├── oakd_left_camera_optical_frame
            └── oakd_right_camera_optical_frame
```

默认 OAK-D 安装假设：

- `base_link`：X 前、Y 左、Z 上。
- `oakd_camera_optical_frame`：Z 前、X 右、Y 下。
- `base_link -> oakd_imu_link` 表示整台 OAK-D 相对底盘的安装外参。
- `oakd_imu_link -> oakd_camera_optical_frame` 表示 OAK-D 内部 IMU/机身到相机光学坐标系的固定关系。

## 当前进度

已完成或正在使用：

- 地面机器人项目范围已明确，不再按无人机主线推进。
- `oakd_perception` 已提供 OAK-D IMU、左右目矫正图、CameraInfo、深度图和可选点云。
- OAK-D VIO-only 验证入口已建立：`oakd_visual_slam_rviz.launch.py`。
- VIO-only 入口已加入低延迟双目模式，减少不必要的深度质量计算。
- `nvidia_3d_nav.launch.py` 已作为 OAK-D + Visual SLAM + nvblox + Nav2 的主入口。
- Nav2 配置已转向 MPPI 控制器方向。
- Isaac Sim 最小验证路径已整理在 `simulation/` 和 [docs/ISAAC_SIM_SIMULATION.md](docs/ISAAC_SIM_SIMULATION.md)。
- VINS-Fusion 可作为实验兼容入口构建和启动，但实测 OAK-D 链路仍会触发异常速度/漂移保护，不作为当前可用定位链路。

当前重点：

- 稳定 OAK-D 双目帧率和 Visual SLAM 输出频率。
- 验证 `odom -> base_link` 连续性和坐标轴方向。
- 用 OAK-D depth + Visual SLAM TF 驱动 nvblox。
- 在 nvblox costmap 上跑通 Nav2 闭环。

当前实测 nvblox / Nav2 地图链路：

- `/oakd/depth/image` 已作为 nvblox 深度输入，关闭 PointCloud2 时仍会发布。
- `/nvblox_node/static_map_slice` 约 `4.5-4.8Hz`。
- `/local_costmap/nvblox_layer` 和 `/local_costmap/costmap` 约 `3.8-4Hz`。
- 该频率适合低速室内验证，建议线速度先限制在 `0.2-0.35m/s`，角速度先限制在 `0.4-0.8rad/s`。
- nvblox 默认关闭 mesh 发布并对 depth 反投影做降采样，以优先保证导航实时性。

查看 nvblox 地图：

```bash
env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
./scripts/with_venv.sh ros2 launch src/omni_bringup/launch/nvidia_3d_nav.launch.py \
  launch_ground_bridge:=false
```

另开终端：

```bash
env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
./scripts/with_venv.sh ros2 run rviz2 rviz2 \
  -d src/omni_bringup/rviz/nvblox_map_check.rviz
```

RViz 默认只打开局部地图层：

- `Local Costmap`
- `Local nvblox Layer`
- `Filtered Odometry`
- `Local Footprint`

如果需要临时查看三维内容，可在 `src/omni_bringup/config/nvblox_3d_nav.yaml` 中把
`publish_mesh` 改为 `true` 后重启，但这会增加可视化和建图负载。导航验证时默认
只看局部 2D costmap / nvblox layer。

后续计划：

- 闭环稳定后继续调 MPPI 参数。
- 再接入 ESS 改善深度质量。
- 最后将 MID360 作为局部安全障碍层接入，不作为第一版主定位输入。

## 代码包概览

- `src/omni_bringup`：地面导航 launch、Nav2/nvblox/Visual SLAM 配置。
- `src/oakd_perception`：OAK-D 统一驱动、IMU、双目、深度和点云接口。
- `src/ground_serial_bridge`：`/cmd_vel` 到底盘 MCU 的串口桥。
- `src/isaac_ros_visual_slam`：Isaac ROS Visual SLAM / cuVSLAM 上游代码。
- `src/isaac_ros_nvblox`：nvblox 3D 建图和 Nav2 costmap 插件相关代码。
- `src/navigation2`：Nav2 上游代码。
- `src/VINS-Fusion-ros2`：旧 VINS 残留源码，当前由 `isaac_ros_visual_slam` 替代；可实验启动，但不要作为当前导航入口使用。
- `patches/`：对 vendor / upstream 代码的本地补丁。
- `simulation/`：Isaac Sim 最小仿真验证入口。

## 常见问题

### OAK-D 图像频率低于 25Hz

当前默认目标帧率是 `25Hz`。如果左右目长期低于该值，先确认 RViz 是否影响不大，
并确认没有覆盖 `oakd_image_qos_depth:=4` 和
`oakd_image_inter_publish_delay_ms:=1.0`。再检查 USB 3.x、线材和供电。若仍不稳定，
可以临时降低 `oakd_image_frequency` 做稳定性验证。

### Visual SLAM 输出频率明显低于图像频率

先看 `/visual_slam/status`，再检查左右目同步、画面纹理、光照和 TF。若图像最大
间隔反复到 `0.078s`，优先解决 OAK-D 图像链路抖动。

### VINS 链路是否还能使用

当前不建议作为导航定位源使用。`src/VINS-Fusion-ros2` 已能实验构建并启动
`oakd_vins.launch.py`，但 OAK-D 实测仍会出现异常速度/漂移；代码中已恢复基础
failure detection，异常状态会 reset 并停止继续发布错误 `/vio/odometry`。

已修正的兼容项：

- VINS 的 OAK-D 左右目内参已对齐当前 `/oakd/left/camera_info` 和
  `/oakd/right/camera_info`。
- `body_T_cam0/body_T_cam1` 已对齐实时
  `oakd_imu_link -> oakd_left/right_camera_optical_frame` TF。
- VINS IMU/image 订阅使用 sensor-data QoS，避免收不到 OAK-D best-effort 数据。

仍未解决的问题：

- 当前 VINS 视觉跟踪会反复出现 `unstable tracking`。
- VIO 初始化后会触发异常速度保护，例如 `velocity too large`，因此不会放行
  `/vio/odometry` 给导航链路。
- 若要继续恢复 VINS，需要单独做 OAK-D 图像质量/特征跟踪、IMU 轴约定、时间同步
  和 VINS 参数的专项调试。

### RViz 中姿态方向看起来反了

先确认观察的是 `base_link` 还是 `oakd_camera_optical_frame`。相机光学坐标系与
机器人本体坐标系定义不同，详细说明见
[docs/OAKD_VISUAL_SLAM_RVIZ.md](docs/OAKD_VISUAL_SLAM_RVIZ.md)。

### OAK-D 被占用

停止旧进程：

```bash
pkill -f "ros2 launch.*oakd_visual_slam_rviz"
pkill -f "ros2 launch.*nvidia_3d_nav"
pkill -f "oakd_unified|visual_slam|static_transform_publisher"
```

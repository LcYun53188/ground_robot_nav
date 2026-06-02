# Isaac Sim 最小仿真验证

本文档说明如何用 Isaac Sim 作为仿真硬件侧，复用当前 ROS 2 侧的 NVIDIA 3D 导航栈。
该流程用于验证 Nav2 / nvblox 闭环、topic、TF 和 RViz 可视化，不用于验证真实
OAK-D cuVSLAM 跟踪质量。

## 架构

Isaac Sim 发布：

- `/clock`
- `/tf`
- `/visual_slam/tracking/odometry`
- `/oakd/left/image_raw`
- `/oakd/right/image_raw`
- `/oakd/left/camera_info`
- `/oakd/right/camera_info`
- `/oakd/depth/image`
- `/oakd/depth/camera_info`

Isaac Sim 订阅：

- `/cmd_vel`

仿真脚本默认通过 `simulation/scripts/run_isaac_sim_nav.sh` 启动 Isaac Sim 侧场景。
该脚本会在工作区 Python 环境下启动一个轻量 ROS 2 bridge 进程，以避免 Isaac Sim
Python 3.11 与 ROS Jazzy Python 3.12 的 `rclpy` 扩展混用。

ROS 2 侧使用 `omni_bringup/isaac_sim_nav.launch.py`，它会包装
`nvidia_3d_nav.launch.py` 并设置：

- `use_sim_time:=true`
- `launch_oakd:=false`
- `launch_visual_slam:=false`
- `launch_ground_bridge:=false`
- OAK-D image/depth 输入 remap 到 Isaac Sim 发布的仿真 topic

仿真中的 `/visual_slam/tracking/odometry` 来自 Isaac Sim ground truth。这样可以隔离
真实 OAK-D VIO 质量问题，专注验证 nvblox、Nav2 和控制闭环。

## 配置 Isaac Sim

编辑：

```text
simulation/config/isaac_sim_nav.yaml
```

设置 Isaac Sim Python：

```yaml
isaac_sim:
  python_executable: "/absolute/path/to/isaacsim/python.sh"
```

也可以在启动前设置环境变量：

```bash
export ISAAC_SIM_PYTHON=/absolute/path/to/isaacsim/python.sh
```

两个终端必须使用同一个 ROS domain：

```yaml
ros:
  domain_id: 0
```

脚本会把该值导出为 `ROS_DOMAIN_ID`。如果你在 ROS 2 终端手动设置
`ROS_DOMAIN_ID`，必须保持一致。

## 场景和模型资源

`simulation/config/isaac_sim_nav.yaml` 预留了这些资源路径：

```yaml
scene:
  robot_mesh_stl_path: ""
  scene_mesh_stl_path: ""
  world_usd_path: ""
```

保持为空时，脚本使用内置轻量 primitive 场景，适合最小烟雾测试。需要使用本地资源时，
优先放到 `simulation/assets/`，但不要提交大型 mesh / USD 文件。

当前流程优先直接加载 USD。STL 路径作为预留配置保留；如果当前 Isaac Sim 版本不能直接
导入 STL，请先在 Isaac Sim 内转换为 USD。

## 启动流程

终端 1，启动 Isaac Sim 仿真侧：

```bash
cd /home/nuc/Program/ground_robot_nav_ws
./simulation/scripts/run_isaac_sim_nav.sh
```

除非在 `simulation/config/isaac_sim_nav.yaml` 中设置
`ros.start_external_ros_bridge: false`，该命令也会启动
`simulation/scripts/ros_nav_bridge.py`。

终端 2，启动 ROS 2 导航侧：

```bash
cd /home/nuc/Program/ground_robot_nav_ws
./scripts/with_venv.sh ros2 launch omni_bringup isaac_sim_nav.launch.py
```

可选启动 RViz：

```bash
cd /home/nuc/Program/ground_robot_nav_ws
RVIZ_FIXED_FRAME=odom ./scripts/run_rviz_nav.sh
```

## 验收检查

检查 topic：

```bash
./scripts/with_venv.sh ros2 topic list -t
```

期望至少能看到：

- `/clock`
- `/visual_slam/tracking/odometry`
- `/oakd/depth/image`
- `/oakd/depth/camera_info`
- `/cmd_vel`

检查 TF：

```bash
timeout 5s ./scripts/with_venv.sh ros2 run tf2_ros tf2_echo odom base_link
```

检查 depth 频率：

```bash
timeout 8s ./scripts/with_venv.sh ros2 topic hz /oakd/depth/image
```

检查 Nav2：

```bash
./scripts/with_venv.sh ros2 lifecycle nodes
timeout 8s ./scripts/with_venv.sh ros2 topic hz /cmd_vel
```

RViz 中使用 `Fixed Frame = odom`，发送 Nav2 goal 后确认：

- Nav2 lifecycle 节点处于 active。
- `/cmd_vel` 有输出。
- Isaac Sim 中机器人运动。
- `/nvblox_node/static_map_slice` 出现。
- Nav2 costmap 随仿真 depth 更新。

## 常见问题

### 看不到 Isaac Sim topic

- 确认 Isaac Sim ROS 2 Bridge 已启用。
- 确认两个终端使用同一个 `ROS_DOMAIN_ID`。
- 确认 `RMW_IMPLEMENTATION` 与 Isaac Sim ROS 2 Bridge 兼容。

### `/clock` 不发布或不前进

- 先启动 Isaac Sim，再启动 ROS 2 导航侧。
- 确认 ROS 2 导航节点使用 `use_sim_time:=true`。

### 缺少 `odom -> base_link`

- 确认 `simulation/scripts/ground_nav_scene.py` 正在运行。
- 确认 `/visual_slam/tracking/odometry` 正在发布。

### Nav2 不接受 goal

- RViz 使用 `Fixed Frame = odom`。
- 确认 Nav2 lifecycle 节点已 active。
- 确认 nvblox 发布 `/nvblox_node/static_map_slice`。

### depth 有数据但 nvblox 为空

- 确认 `/oakd/depth/camera_info` 的 frame 与 depth image frame 匹配。
- 确认 `src/omni_bringup/config/nvblox_isaac_sim.yaml` 使用仿真时间。
- 确认 `odom -> base_link` 和相机 TF 连续可用。

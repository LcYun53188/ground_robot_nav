# Isaac Sim / 轻量 RGB-D 感知仿真验证

本文档说明如何用 Isaac Sim 作为仿真硬件侧，复用当前 ROS 2 侧的 NVIDIA 3D 导航栈。
该流程可在无 OAK-D、无底盘实物时验证 Nav2 / nvblox 闭环、topic、TF、RViz
可视化，以及由 Isaac 场景渲染得到的 RGB-D 感知输入。

仿真中的 `/visual_slam/tracking/odometry` 仍来自仿真 ground truth，不用于验证真实
OAK-D cuVSLAM 跟踪质量。

## 当前结论

本机已验证 Isaac Sim 5.1 与 Isaac Sim 4.5 都不能稳定作为 GUI/RTX 感知仿真后端使用。
5.1 在 RTX 4070 Laptop 8 GB 显存环境下未通过最低显存检查，并在 Kit UI/viewport
初始化后段错误。随后安装的 4.5 pip 环境可以解析启动器并加载到 `app ready`，但 Base
UI 和 headless `SimulationApp` 仍在 RTX viewport / `_wait_for_viewport` 初始化路径段错误。

因此，当前项目不再把 Isaac Sim GUI/RTX 作为近期仿真主线。本文保留 Isaac 相关配置和
`ros_bridge_only` 轻量回退，用于继续验证 ROS/RViz/Nav2/nvblox topic 与 TF 链路；真实
3D 引擎仿真改以 Gazebo Harmonic + `ros_gz` 作为下一步替代方案。

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

仿真脚本通过 `simulation/scripts/run_isaac_sim_nav.sh` 启动。当前默认是最轻量
`ros_bridge_only` 模式：只启动 ROS 2 bridge，不启动 Isaac Kit，不需要导入地图或机器人
模型。该模式发布 `/clock`、TF、里程计和固定 pattern RGB-D，用于验证 ROS/RViz/Nav2/nvblox
链路。

真实 Isaac RGB-D 模式仍保留：把 `isaac_sim.launch_mode` 改为 `"isaac"`，并把
`oakd.render_mode` 改为 `"isaac"` 后，脚本会启动 Isaac Sim 场景进程。

`oakd.render_mode: "isaac"` 时，ROS bridge 会通过本机 TCP 把当前机器人位姿发送给
Isaac 进程；Isaac 进程根据该位姿更新仿真机器人和相机，并把渲染出的 RGB-D 帧回传给
ROS bridge。ROS bridge 再发布 `/oakd/left/image_raw`、`/oakd/right/image_raw` 和
`/oakd/depth/image`，供 nvblox/Nav2 使用。

ROS 2 侧使用 `omni_bringup/isaac_sim_nav.launch.py`，它会包装
`nvidia_3d_nav.launch.py` 并设置：

- `use_sim_time:=true`
- `launch_oakd:=false`
- `launch_visual_slam:=false`
- `launch_ground_bridge:=false`
- OAK-D image/depth 输入 remap 到 Isaac Sim 发布的仿真 topic

如果 Isaac Camera API、RTX 渲染或 Kit UI/viewport 初始化不可用，使用默认
`ros_bridge_only + pattern` 路径继续验证 ROS 侧 topic、TF、nvblox 和 Nav2 链路。此
fallback 不能代表自定义地图中的真实障碍。

## 配置 Isaac Sim

编辑：

```text
simulation/config/isaac_sim_nav.yaml
```

设置 Isaac Sim Python：

```yaml
isaac_sim:
  target_version: "4.5"
  launch_mode: "ros_bridge_only"
  python_executable: "/home/nuc/Program/ground_robot_nav_ws/.venv-isaac-sim-4.5/bin/python"
  app_executable: "/home/nuc/Program/ground_robot_nav_ws/.venv-isaac-sim-4.5/bin/isaacsim"
```

也可以在启动前设置环境变量：

```bash
export ISAAC_SIM_PYTHON=/home/nuc/Program/ground_robot_nav_ws/.venv-isaac-sim-4.5/bin/python
```

当前项目默认以 Isaac Sim 4.5 为本机 UI/RTX 测试目标。原因是 RTX 4070 Laptop 8 GB
显存不满足 Isaac Sim 5.1 的最低显存检查。当前 4.5 pip 环境安装到：

```text
/home/nuc/Program/ground_robot_nav_ws/.venv-isaac-sim-4.5
```

如果安装到其他目录，设置：

```bash
export ISAAC_SIM_45_ROOT=/absolute/path/to/.venv-isaac-sim-4.5
export ISAAC_SIM_PYTHON=$ISAAC_SIM_45_ROOT/bin/python
```

两个终端必须使用同一个 ROS domain：

```yaml
ros:
  domain_id: 0
  frame_server_host: "127.0.0.1"
  frame_server_port: 47650

oakd:
  render_mode: "pattern"
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

真实感知仿真推荐使用 USD：

```yaml
scene:
  world_usd_path: "/home/nuc/Program/ground_robot_nav_ws/simulation/assets/my_world.usd"
```

默认就是固定深度烟雾测试：

```yaml
isaac_sim:
  launch_mode: "ros_bridge_only"

oakd:
  render_mode: "pattern"
```

要切回真实 Isaac 渲染，改为：

```yaml
isaac_sim:
  launch_mode: "isaac"
  python_executable: "/home/nuc/Program/ground_robot_nav_ws/.venv-isaac-sim-4.5/bin/python"

oakd:
  render_mode: "isaac"
```

要单独打开 Isaac Sim 4.5 UI：

```bash
cd /home/nuc/Program/ground_robot_nav_ws
./simulation/scripts/run_isaac_sim_45_ui.sh
```

## 启动流程

终端 1，启动轻量仿真侧：

```bash
cd /home/nuc/Program/ground_robot_nav_ws
./simulation/scripts/run_isaac_sim_nav.sh
```

默认会看到：

```text
[isaac_sim_nav] Starting lightweight ROS bridge only; Isaac Kit will not be launched.
```

这表示当前没有启动 Isaac Kit，所以不会打开 Isaac Sim GUI。可视化使用 RViz。

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
- 轻量模式下 TF/里程计中的机器人位姿运动；真实 Isaac 模式下 Isaac Sim 中机器人运动。
- `/nvblox_node/static_map_slice` 出现。
- Nav2 costmap 随 Isaac 渲染 depth 更新。

确认是否正在使用真实 Isaac RGB-D：

```bash
./scripts/with_venv.sh ros2 topic echo --once /oakd/depth/image/header
timeout 8s ./scripts/with_venv.sh ros2 topic hz /oakd/depth/image
```

轻量模式不会出现以下日志。真实 Isaac 模式下，终端 1 正常会出现：

```text
[isaac_sim_nav] RGB-D frame server listening on 127.0.0.1:47650
[isaac_sim_nav] ROS bridge connected to RGB-D frame server
[isaac_sim_nav] Isaac RGB-D cameras created
```

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

### 自定义地图不影响 depth/costmap

- 确认 `oakd.render_mode: "isaac"`，不要使用 `"pattern"`。
- 确认终端 1 出现 `Isaac RGB-D cameras created`；如果出现 camera API fallback，说明当前
  Isaac Sim Python/RTX 环境无法创建相机。
- 确认 `scene.world_usd_path` 指向存在的 USD 文件。STL/OBJ 需要先在 Isaac Sim 中转换为 USD。
- 确认机器人初始位姿和相机高度在自定义场景内，不要被墙体包围或放到地面以下。

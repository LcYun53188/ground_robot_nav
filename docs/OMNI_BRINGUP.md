# 全向轮启动包说明

`omni_bringup` 是地面全向轮机器人的独立启动入口。

## 启动范围

默认启动：

- OAK-D 感知入口，可通过参数关闭
- `nav_mapping/local_map_builder`
- `nav_planning/se2_dwa_local_planner`
- `nav_safety/safety_monitor`

可选启动：

- MID360 点云链路
- DWB bridge
- `ground_serial_bridge` 串口底盘桥接

## 常用命令

仅启动全向轮导航层，不发送到底盘：

```bash
./scripts/run_omni_nav.sh
```

常用参数优先在 `scripts/nav_launch.env` 里改变量：

```bash
OMNI_POINTCLOUD_SOURCE="oakd"
OMNI_ENABLE_BRIDGE="false"
OMNI_PLANNER="se2_dwa"
OMNI_ENABLE_OFFLINE_MAP="false"
OMNI_OFFLINE_MAP_YAML=""
```

如果某台车需要固定私有配置，新建 `scripts/nav_launch.local.env` 覆盖即可。
命令行参数仍可用于临时覆盖变量。

启用串口底盘桥接：

```bash
./scripts/run_omni_nav.sh --bridge
```

使用 MID360 作为障碍点云：

```bash
./scripts/run_omni_nav.sh --pointcloud-source mid360
```

同时使用 OAK-D 和 MID360：

```bash
./scripts/run_omni_nav.sh --pointcloud-source both
```

使用 DWB bridge 而不是默认 SE(2) DWA：

```bash
./scripts/run_omni_nav.sh --planner dwb
```

发布离线低分辨率 OccupancyGrid 地图：

```bash
./scripts/run_omni_nav.sh --offline-map /path/to/map.yaml
```

检查全向轮导航链路：

```bash
./scripts/check_omni_nav.sh
./scripts/check_omni_nav.sh --offline-map --bridge
./scripts/check_offline_map.sh
```

脚本后面可以继续追加底层 launch 参数：

```bash
./scripts/run_omni_nav.sh --pointcloud-source mid360 --bridge mid360_frame_id:=mid360_link
```

## 主要话题

输入：

```text
/oakd/points_filtered
/mid360/points
/nav/goal_pose
```

输出：

```text
/local_map/sensor_occupancy  # 仅启用 --offline-map 时作为实时局部图中间话题
/static_map/occupancy        # 仅启用 --offline-map 时发布
/local_map/occupancy
/nav/cmd_vel
/nav/emergency
/nav/safety_status
```

启用底盘桥接后，`ground_serial_bridge` 应订阅：

```text
/nav/cmd_vel
/nav/emergency
/nav/safety_status
```

串口桥接节点发布：

```text
/base/state
/base/status
/base/diagnostics
```

`ground_serial_bridge` 会在串口发送前把 `/nav/cmd_vel` 按 `header.frame_id` 转到 `command_output_frame`，默认是 `base_link`。因此下位机不需要维护自己的世界坐标系，也不应按下位机漂移的世界系解释速度方向。

推荐约定：

```text
上位机导航/定位：维护 map、odom、base_link
串口桥接：把 map/odom 速度转成 base_link 车体系速度
下位机：只按车体系 vx、vy、wz 控制电机
```

如果 TF 不可用，桥接节点会下发 0 速度，避免继续按错误方向执行。

如果下位机无法改成车体系速度接口，仍然必须按自己的漂移世界系执行速度，则可以开启动态补偿：

```yaml
mcu_velocity_frame: mcu_world
require_feedback: true
require_mcu_yaw_for_mcu_world: true
```

此模式要求下位机状态帧在原 10 字节 payload 后追加：

```text
yaw_mrad int16
```

完整状态 payload 变为：

```text
enabled uint8
estop uint8
vx_mm_s int16
vy_mm_s int16
wz_mrad_s int16
fault_code uint16
yaw_mrad int16
```

补偿流程：

```text
上位机 /nav/cmd_vel
  -> TF 转 mcu_yaw_reference_frame，例如 odom/map
  -> 从 TF 获取 yaw_base
  -> 从下位机状态帧获取 yaw_mcu
  -> 计算 yaw_error = yaw_mcu - yaw_base
  -> 使用 yaw_error 转到 mcu_world
  -> 串口发送给下位机
```

对应配置：

```yaml
mcu_yaw_reference_frame: odom
mcu_yaw_compensation_mode: yaw_error
```

`yaw_base` 来自 `mcu_yaw_reference_frame -> base_link` 的 TF。车体真实旋转时，`yaw_base` 和 `yaw_mcu` 应该一起变化；只有二者的差值才作为下位机坐标漂移补偿。

如果你的下位机接口实际需要“先转成 `base_link`，再用下位机 yaw 直接转到下位机世界系”，可以切回：

```yaml
mcu_yaw_compensation_mode: mcu_yaw
```

如果下位机 yaw 符号相反，设置：

```yaml
mcu_yaw_sign: -1.0
```

如果下位机 yaw 有固定安装/初始偏差，可设置：

```yaml
mcu_yaw_offset_rad: 0.0
```

若希望启动时把当前下位机 yaw 视为 0，可设置：

```yaml
mcu_yaw_auto_zero: true
```

## 配置文件

- `src/omni_bringup/config/omni_nav_stack.yaml`
- `src/omni_bringup/config/offline_map.yaml`
- `src/omni_bringup/config/ground_serial_bridge.yaml`
- `src/ground_serial_bridge/config/ground_serial_bridge.yaml`

`offline_map.yaml` 配置 `nav2_map_server` 和 lifecycle manager。启用后，
离线地图发布到 `/static_map/occupancy`；实时点云局部图发布到
`/local_map/sensor_occupancy`；`occupancy_grid_fusion` 会把离线地图中的占用格
叠加到实时局部图，并输出 `/local_map/occupancy` 给 SE(2) DWA 或 DWB。
这样静态墙体、禁行区和通道边界可以参与局部规划，动态障碍仍由实时点云更新。

离线地图必须满足：

- `frame_id` 与实时局部图一致，默认都是 `map`；
- 地图 YAML 使用 Nav2 map server 格式；
- 静态地图和 TF 对齐后再上车，否则静态障碍会被叠加到错误位置。

## 创建离线低图

离线低图使用 Nav2 `map_server` 的 2D OccupancyGrid 地图格式：一个
`map.yaml` 配一个图片文件，例如 `map.pgm` 或 `map.png`。低图只表达墙体、
通道边界和禁行区等稳定结构；动态障碍仍由 OAK-D / MID360 实时点云负责。

推荐目录：

```bash
mkdir -p maps/site_a
```

示例 `maps/site_a/map.yaml`：

```yaml
image: map.pgm
mode: trinary
resolution: 0.10
origin: [-5.0, -5.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

参数含义：

- `image`：地图图片路径，通常写相对 `map.yaml` 的路径；
- `resolution`：每个像素代表的米数，低图建议 `0.10` 到 `0.20`；
- `origin: [x, y, yaw]`：图片左下角在 `map` 坐标系中的位姿；
- `occupied_thresh` / `free_thresh`：像素转占用/空闲的阈值。

图片约定：

- 黑色或深色：占用，表示墙体、固定障碍或禁行区；
- 白色或浅色：空闲；
- 灰色：未知区域。

启用方式一：临时参数启动：

```bash
./scripts/run_omni_nav.sh --offline-map /home/nuc/Program/ground_robot_nav_ws/maps/site_a/map.yaml
```

启用方式二：写入 `scripts/nav_launch.env`：

```bash
OMNI_ENABLE_OFFLINE_MAP="true"
OMNI_OFFLINE_MAP_YAML="/home/nuc/Program/ground_robot_nav_ws/maps/site_a/map.yaml"
```

之后直接启动：

```bash
./scripts/run_omni_nav.sh
```

验证：

```bash
./scripts/check_offline_map.sh
./scripts/check_omni_nav.sh --offline-map
./scripts/run_rviz_nav.sh
```

RViz 中建议同时看：

```text
TF
/static_map/occupancy
/local_map/sensor_occupancy
/local_map/occupancy
/nav/cmd_vel
```

验收标准：

- `/static_map/occupancy`、`/local_map/sensor_occupancy`、`/local_map/occupancy`
  的 `frame_id` 都是 `map`；
- 静态地图中的墙体和现场真实墙体在 RViz 中重合；
- 车体 `base_link` 在地图中的位置和朝向正确；
- 融合后的 `/local_map/occupancy` 同时包含静态障碍和实时点云障碍。

调试记录建议使用：

```bash
./scripts/record_nav_debug_bag.sh --offline-map --omni
```

`omni_nav_stack.yaml` 中的速度、加速度、避障半径已经按地面全向轮做了初始收敛。上车前仍需按底盘能力重新标定：

```text
max_vel_x
max_vel_y
max_yaw_rate
max_acc_x
max_acc_y
max_yaw_accel
robot_radius
stop_clearance
```

## 下位机坐标系开关

默认关闭下位机世界坐标系，使用车体系速度：

```yaml
mcu_velocity_frame: base_link
```

含义：

```text
上位机 /nav/cmd_vel -> TF 转 base_link -> 串口发送 vx_base/vy_base/wz_base
```

这是推荐模式，要求下位机按 `x 前、y 左、z 上、wz 逆时针为正` 执行速度。

如果下位机必须按自己的世界系执行速度，启用：

```yaml
mcu_velocity_frame: mcu_world
mcu_yaw_compensation_mode: yaw_error
mcu_yaw_reference_frame: odom
require_feedback: true
require_mcu_yaw_for_mcu_world: true
```

影响：

- 必须回传 `yaw_mrad`。
- `yaw_base` 来自上位机 TF：`odom/map -> base_link`。
- `yaw_error = yaw_mcu - yaw_base` 只用于修正下位机坐标漂移。
- 目标追踪和 yaw 追踪仍由上位机闭环，`wz` 直接作为角速度下发。
- 回传帧率和延迟会影响边转边走时的方向稳定性，建议 `20Hz` 以上。

## 下位机坐标调试

1. 先用 `base_link` 模式验证轴向：

   ```text
   vx > 0：前进
   vy > 0：左移
   wz > 0：逆时针旋转
   ```

2. 若方向相反，优先修下位机协议或电机映射。下位机 yaw 符号相反时设置：

   ```yaml
   mcu_yaw_sign: -1.0
   ```

3. 再启用 `mcu_world` 补偿，观察 `/base/status` 与 `/base/diagnostics`。

4. 原地旋转时检查：

   ```text
   yaw_base 与 yaw_mcu 同号变化
   yaw_error 不应随真实旋转持续增大
   ```

5. TF 断开或下位机 yaw 缺失时，节点应下发 0 速度，不继续按旧方向执行。

## 下位机坐标验收

验收通过标准：

- `base_link` 轴向与实车一致。
- `mcu_velocity_frame: base_link` 可以稳定执行手动速度命令。
- 启用 `mcu_world` 后，`yaw_mcu`、`yaw_base`、`yaw_error` 变化符合预期。
- 边平移边转 yaw 时，车辆没有明显因下位机 yaw 漂移导致的方向偏转。
- 断开 TF、断开串口或底盘回传低于阈值时，系统进入停车/故障状态。
- 下位机世界坐标只用于补偿或诊断，不作为 ROS 主 TF 坐标源。

## 串口回传帧率

控制发送建议保持 `20-50Hz`。底盘状态回传建议不低于 `10Hz`，工程上更推荐 `20Hz`，这样故障码、使能状态和实际速度不会明显滞后。

`ground_serial_bridge` 默认 `require_feedback: false`，方便无回传协议时先调通速度下发。真实上车时建议改为：

```yaml
require_feedback: true
min_feedback_rate_hz: 10.0
feedback_timeout_sec: 0.3
```

启用后，如果底盘回传超时或统计帧率低于阈值，节点会进入 `FAULT` 并持续发送停车/急停帧。

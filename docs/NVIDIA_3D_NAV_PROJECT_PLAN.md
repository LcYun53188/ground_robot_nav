# NVIDIA 3D 导航项目计划

目标：将当前地面全向轮导航框架迁移到以 NVIDIA Isaac ROS 为核心的 3D 感知导航架构。第一阶段不引入 MID360，不使用底盘里程计，不做跨设备时间戳对齐。

## 1. 架构目标

第一版核心链路：

```text
OAK-D 双目 + IMU
    -> Isaac ROS Visual SLAM / cuVSLAM
    -> odom -> base_link

OAK-D 深度图
    -> nvblox_ros
    -> TSDF / ESDF / map slice

nvblox 地图切片
    -> nvblox_nav2 costmap layer
    -> Nav2 planner/controller
    -> /cmd_vel
    -> ground_serial_bridge
```

保留：

- `oakd_perception`
- `omni_bringup`
- `ground_serial_bridge`
- Nav2
- `nvblox_ros`
- `nvblox_nav2`

替换或降级：

- `VINS-Fusion-ros2`：由 `isaac_ros_visual_slam` 替代；源码可保留但默认 `COLCON_IGNORE`，不作为当前可用定位链路
- `FAST_LIO_ROS2`：第一版不进入核心链路
- `nav_mapping/local_map_builder`：由 nvblox 替代
- 自研 SE(2) DWA：由 Nav2 controller server 替代
- Nav2 DWB controller：闭环跑通后由 MPPI controller 替代
- `robot_localization`：第一版不作为必须项，除非后续需要融合额外位姿源

## 2. 设计原则

- 单传感器时序域优先：第一版只使用 OAK-D，不融合 MID360。
- 无底盘里程计：定位由 Visual SLAM 输出 `odom -> base_link`。
- 保守避障：nvblox 输出给 Nav2 costmap，低矮通道优先保证安全。
- 主线独立：当前文档和验证流程以 `nvidia_3d_nav.launch.py` 与 `oakd_visual_slam_rviz.launch.py` 为准，旧 `omni_nav.launch.py` 不再作为文档主入口。
- 分阶段接入 ESS：先用 OAK-D 原生 depth 跑通闭环，再启用 ESS。

## 3. 阶段计划

### 阶段 0：基线冻结

目标：明确当前系统可运行基线，避免迁移时失去回退点。

任务：

- 记录当前可运行 launch 命令。
- 记录当前 topic、TF、Nav2 行为。
- 保留旧源码以便追溯，但当前文档入口只维护 NVIDIA 3D 主线。

验收：

- 新增 NVIDIA 架构文件不影响当前地面栈构建。
- 被 `COLCON_IGNORE` 标记的旧包不会进入默认构建。

### 阶段 1：NVIDIA 3D 架构骨架

目标：建立独立 bringup 入口和配置文件。

任务：

- 新增 `nvidia_3d_nav.launch.py`。
- 新增 Visual SLAM、nvblox、Nav2 专用配置。
- 明确 OAK-D topic、TF frame、Nav2 odom topic。
- 新增架构说明和项目规划文档。

验收：

- `omni_bringup` 可编译。
- launch 文件 Python 语法检查通过。
- 新架构入口独立于旧 VINS / UAV 入口。

### 阶段 2：OAK-D 数据完整性

目标：确认 OAK-D 能提供 Visual SLAM 和 nvblox 所需输入。

任务：

- 确认 stereo image topic：
  - `/oakd/left/image_raw`
  - `/oakd/right/image_raw`
- 补齐或确认 left/right `CameraInfo`：
  - `/oakd/left/camera_info`
  - `/oakd/right/camera_info`
- 确认 IMU：
  - `/oakd/imu/raw`
- 确认 depth：
  - `/oakd/depth/image`
  - `/oakd/depth/camera_info`
- 检查 `base_link -> oakd_imu_link -> oakd_camera_optical_frame`。

验收：

- `ros2 topic hz` 显示图像、IMU、depth 稳定。
- 图像、IMU、depth 的 frame_id 正确。
- TF 树无断链。

### 阶段 3：Isaac ROS Visual SLAM

目标：无底盘里程计条件下，由 cuVSLAM 提供定位。

任务：

- 通过 submodule 引入、构建并 source `isaac_ros_visual_slam`。
- 通过 `patches/vendor/` 维护必要的上游源码适配。
- 使用 OAK-D stereo + IMU 启动 Visual SLAM。
- 输出：
  - `/visual_slam/tracking/odometry`
  - `map -> odom`
  - `odom -> base_link`
- 验证低速平移、原地旋转、坡面运动。

验收：

- `odom -> base_link` 连续发布。
- `/visual_slam/status` 正常。
- 原地旋转和短距离往返不明显发散。
- Nav2 能读取 `/visual_slam/tracking/odometry`。

### 阶段 4：nvblox 3D 建图

目标：用 OAK-D depth 和 Visual SLAM TF 建立 3D 障碍地图。

任务：

- 通过 submodule 引入、构建并启动 `nvblox_ros`。
- 通过 `patches/vendor/` 维护必要的上游源码适配。
- 输入 OAK-D depth 和 camera info。
- 使用 `odom` 作为 nvblox global frame。
- 输出：
  - `/nvblox_node/static_map_slice`
  - ESDF slice
  - mesh 可视化
- 调整低矮通道参数：
  - `voxel_size`
  - `min_height`
  - `max_height`
  - `slice_height`

验收：

- RViz/Foxglove 中可看到 nvblox map slice。
- 低矮障碍能进入 costmap。
- 空洞区域不会被误判为可靠 free space。

### 阶段 5：Nav2 接入

目标：Nav2 只消费 nvblox costmap，不再依赖自研 local_map_builder。

任务：

- 使用 `nvblox::nav2::NvbloxCostmapLayer`。
- 配置 local/global costmap rolling window。
- controller 使用全向轮速度限制。
- ground_serial_bridge 接收 `/cmd_vel`。

验收：

- Nav2 lifecycle 正常。
- local/global costmap 均来自 `/nvblox_node/static_map_slice`。
- 发送目标点后机器人能规划、避障、停止。

### 阶段 6：MPPI 控制器替换 DWB

目标：将 Nav2 controller 从 DWB 切换到 MPPI，让全向轮在 nvblox costmap 上获得更平滑、更适合狭窄空间的局部控制。

任务：

- 通过 Navigation2 submodule 引入、构建并确认 `nav2_mppi_controller` 可用。
- 通过 `patches/vendor/` 维护必要的上游源码适配。
- 将 `controller_server` 的 `FollowPath` 插件从 `dwb_core::DWBLocalPlanner` 替换为 `nav2_mppi_controller::MPPIController`。
- 保留 nvblox local/global costmap，不改变建图链路。
- 配置全向轮速度边界：
  - `vx_max`
  - `vx_min`
  - `vy_max`
  - `wz_max`
- 配置 MPPI 采样和代价项：
  - batch size
  - time steps
  - model dt
  - obstacle critic
  - path follow critic
  - goal critic
- 保持 `ground_serial_bridge` 接收 `/cmd_vel`。
- 先低速测试，再逐步提高速度。

验收：

- Nav2 lifecycle 正常。
- MPPI 能输出连续 `/cmd_vel`。
- 全向横移、前进、后退、原地旋转均正常。
- 在 250 mm 低矮通道前能稳定减速和绕避。
- 无明显振荡、反复倒车、卡死。

### 阶段 7：ESS 深度升级

目标：在主闭环跑通后，用 ESS 替代或补强 OAK-D 原生 depth。

任务：

- 安装 `isaac_ros_ess` 和 ESS engine。
- 提供 rectified stereo image + camera info。
- 接入 disparity-to-depth。
- 将 nvblox depth input 切换到 ESS depth。

验收：

- ESS depth 稠密度高于 OAK-D 原生 depth。
- 低纹理地面/墙面空洞减少。
- nvblox 地图稳定性提升。
- 端到端延迟仍满足避障要求。

### 阶段 8：MID360 局部安全层

目标：在 MPPI 和 nvblox 主闭环稳定后，将 MID360 作为局部安全障碍层接入，只影响 Nav2 local costmap，不参与主定位、不进入第一版 nvblox 主地图。

任务：

- 启动 `livox_ros_driver2`。
- 将 `/livox/lidar` 转换为 `/mid360/points`。
- 在 Nav2 `local_costmap` 中新增 MID360 obstacle/voxel layer。
- layer 顺序保持：
  - `nvblox_layer`
  - `mid360_obstacle_layer`
  - `inflation_layer`
- 只接入 local costmap，第一版不接入 global costmap。
- 不将 MID360 融入 Visual SLAM、ESS 或 nvblox 主链路。
- 配置 MID360 点云高度过滤和障碍保持时间。

验收：

- MID360 点云能进入 local costmap。
- MPPI 能基于 MID360 障碍绕行或减速。
- OAK-D 暂时漏检时，MID360 能提供周向障碍兜底。
- 不要求 OAK-D 与 MID360 做严格时间戳对齐。
- 关闭 MID360 后，OAK-D + nvblox + MPPI 主链路仍可运行。

### 阶段 9：可靠性测试

目标：验证新架构在目标场景内可用。

场景：

- 250 mm 低矮洞口。
- 弱纹理墙面/地面。
- 近距离小障碍。
- 原地旋转。
- 斜坡或轻微颠簸。
- 人从前方穿越。

验收：

- 无碰撞。
- 无持续 ghost obstacle。
- Visual SLAM 不频繁丢失。
- Nav2 不出现长时间卡死。
- MPPI 控制无明显振荡。
- MID360 safety layer 不造成长期假障碍。
- 断传感器或 tracking lost 时能停止。

## 4. 风险清单

| 风险 | 影响 | 缓解 |
|---|---|---|
| OAK-D 未发布 left/right CameraInfo | Visual SLAM/ESS 无法正确运行 | 阶段 2 补齐相机信息发布 |
| 低纹理导致 Visual SLAM 丢失 | 无底盘里程计时定位中断 | 降速、改善光照、后续加安全停机 |
| OAK-D 原生 depth 空洞 | nvblox 障碍漏检 | 阶段 7 接 ESS |
| ESS 引入额外延迟 | 近场避障反应变慢 | 先测延迟，再决定是否只用于建图 |
| nvblox 高度切片不合适 | 低矮障碍漏检或误检 | 针对 250 mm 通道调参 |
| MPPI 参数不合适 | 轨迹振荡、贴障、倒车异常 | 低速调参，先限制速度和加速度 |
| MID360 obstacle layer 保持时间过长 | 动态障碍残留 | 缩短 observation persistence 和 clearing 时间 |
| Isaac ROS 包版本不匹配 | launch 或参数不可用 | 固定 Isaac ROS 版本并记录 |
| 误用旧 VINS 入口 | 进入未维护链路，TF/内参/时间戳不可信 | 当前只使用 `oakd_visual_slam_rviz.launch.py` 和 `nvidia_3d_nav.launch.py`；恢复 VINS 需单独立项 |

## 5. 最小可行版本

最小可行版本不包含 ESS：

```text
OAK-D 原生深度 + Visual SLAM + nvblox + Nav2
```

最小可行版本成功标准：

- 不用底盘里程计。
- 不用 MID360。
- 能发布稳定 `odom -> base_link`。
- nvblox map slice 能进入 Nav2 costmap。
- Nav2 能输出 `/cmd_vel`。

## 6. 后续扩展

只有在最小可行版本跑稳后再考虑：

- ESS depth 替换。
- 语义分割 mask。
- robot_localization 融合额外定位源。
- MID360 进入 nvblox 多传感器融合。
- FAST-LIO / MID360 作为定位备选。
- VINS-Fusion 作为备选定位重新评估；必须先移除 `COLCON_IGNORE`、修复依赖、重新标定 OAK-D 内外参和时间戳。

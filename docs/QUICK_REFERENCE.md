# 快速参考（Quick Reference）

本文档提供统一节点的快速启动命令、常见参数、验证步骤与常见问题解决方案。

---

## 启动命令

### 完整系统（推荐）

```bash
./scripts/run_nav_stack.sh --odom-source vio --pointcloud-source oakd
```

### 仅硬件节点（统一节点）

```bash
./scripts/with_venv.sh ros2 launch oakd_perception oakd_unified.launch.py
src/oakd_perception/scripts/run_oakd_balance.sh
```

### 仅 IMU 融合 + TF 广播

```bash
./scripts/with_venv.sh ros2 launch imu_fusion imu_fusion.launch.py \
  launch_imu_node:=false \
  raw_topic_0:=/oakd/imu/raw \
  fused_topic_0:=/oakd/imu/fused \
  frame_id_0:=oakd_imu_link \
  parent_frame:=map
```

### 验证系统

```bash
./scripts/with_venv.sh ros2 topic list | grep -E "/oakd|/imu"
```

### OAK-D 视觉里程计 + RViz

```bash
env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_LOG_DIR=/tmp/ros_log \
./scripts/with_venv.sh ros2 launch src/omni_bringup/launch/oakd_visual_slam_rviz.launch.py
```

该入口只启动 OAK-D、Isaac ROS Visual SLAM、静态 TF 和 RViz，不启动 nvblox/Nav2/底盘桥。详细说明见 [OAKD_VISUAL_SLAM_RVIZ.md](./OAKD_VISUAL_SLAM_RVIZ.md)。

---

## 发布的主题

| 主题 | 类型 | 频率 | 说明 |
|------|------|------|------|
| `/oakd/imu/raw` | sensor_msgs/Imu | 400Hz | 原始 IMU 数据 |
| `/oakd/points` | sensor_msgs/PointCloud2 | 20Hz | 深度点云 |
| `/imu` | sensor_msgs/Imu | 100Hz | 融合后 IMU 数据 |
| `/tf` | tf2/TFMessage | 动态 | map → oakd_imu_link 变换 |

---

## 常用启动参数

```bash
# 启用主动立体（深度更好，但耗能）
./scripts/with_venv.sh ros2 launch oakd_perception oakd_unified.launch.py \
  enable_active_stereo:=true ir_intensity:=1000

# 提高点云频率（更流畅，更耗 CPU）
./scripts/with_venv.sh ros2 launch oakd_perception oakd_unified.launch.py \
  pointcloud_frequency:=30

# 降低采样率（减少数据，加快处理）
./scripts/with_venv.sh ros2 launch oakd_perception oakd_unified.launch.py \
  sampling_step:=4

# 调整深度范围（毫米）
./scripts/with_venv.sh ros2 launch oakd_perception oakd_unified.launch.py \
  min_depth:=100 max_depth:=8000
```

---

## 验证步骤

```bash
# 1. 列出所有话题
./scripts/with_venv.sh ros2 topic list

# 2. 查看 IMU 原始数据
./scripts/with_venv.sh ros2 topic echo /oakd/imu/raw

# 3. 查看点云数据
./scripts/with_venv.sh ros2 topic echo /oakd/points | head

# 4. 测量点云频率
./scripts/with_venv.sh ros2 topic hz /oakd/points

# 5. 查看计算图
./scripts/with_venv.sh rqt_graph
```

---

## 常见问题与解决

### "X_LINK_DEVICE_ALREADY_IN_USE"

**原因**：旧进程仍在运行。

**解决**：

```bash
pkill -9 -f "oakd_"
sleep 2
./scripts/with_venv.sh ros2 launch oakd_perception oakd_unified.launch.py
```

### "camera is not initialized"

**原因**：OAK-D 未插入或 depthai 驱动未安装。

**解决**：

```bash
# 检查 USB 连接
# 重装 depthai
./scripts/with_venv.sh pip install --upgrade depthai
```

### 点云为空或稀疏

**原因**：深度值超出范围或采样率过高。

**解决**：

```bash
./scripts/with_venv.sh ros2 launch oakd_perception oakd_unified.launch.py \
  min_depth:=100 max_depth:=10000 sampling_step:=2
```

### 帧率很低

**原因**：USB 带宽不足或 CPU 过载。

**解决**：

```bash
./scripts/with_venv.sh ros2 launch oakd_perception oakd_unified.launch.py \
  pointcloud_frequency:=10 sampling_step:=4
```

---

## RViz 配置

OAK-D VIO 验证优先使用：

```bash
./scripts/with_venv.sh ros2 launch src/omni_bringup/launch/oakd_visual_slam_rviz.launch.py
```

默认 RViz 预设：

```text
src/omni_bringup/rviz/visual_slam_check.rviz
```

该预设使用 `Fixed Frame = odom`，并突出显示 `base_link`、TF、`/visual_slam/tracking/odometry` 和 `/visual_slam/tracking/vo_path`。

传统点云查看方式：

1. **启动 RViz**：`./scripts/with_venv.sh rviz2`
2. **设置全局选项**：
  - Fixed Frame: `map`
3. **添加 PointCloud2**：
  - Topic: `/oakd/points`
  - Color Transformer: `Z` 或 `Intensity`
4. **添加 TF**（若使用 IMU 联动）：
  - 显示坐标系树 `map → oakd_imu_link`

---

## 系统架构简图

```
OAK-D 设备（单一硬件连接）
     ↓
oakd_unified_node（单一进程）
├─ IMU 采样器 (400Hz) → /oakd/imu/raw
└─ 深度处理器 (20Hz) → /oakd/points
     ↓
IMU 融合链路
├─ imu_fusion_node → /imu (融合后)
└─ imu_tf_broadcaster → TF (map → oakd_imu_link)
     ↓
应用层（RViz、导航等）
```

---

## 性能指标

| 指标 | 值 |
|------|-----|
| IMU 频率 | 400Hz ±1% |
| 点云频率 | 20Hz ±0.1% |
| CPU 占用 | 8–12% |
| 内存占用 | ~150MB |
| 端到端延迟 | ~40ms |

---

## 详细文档

- [INSTALLATION.md](./INSTALLATION.md) — 环境配置与构建
- [ARCHITECTURE.md](./ARCHITECTURE.md) — 系统架构详解
- [../README.md](../README.md) — 主文档与快速开始

---

## 源代码位置

```
workspace/
├── src/oakd_perception/
│   ├── oakd_perception/
│   │   └── oakd_unified_node.py      # 核心节点
│   └── launch/
│       └── oakd_unified.launch.py    # Launch 文件
├── scripts/
│   ├── run_nav_stack.sh              # 导航栈统一启动
│   └── with_venv.sh                  # 虚拟环境包裹
└── docs/
    ├── QUICK_REFERENCE.md            # 本文件
    └── ARCHITECTURE.md               # 详细架构
```

---

如有问题，参考 [../README.md](../README.md#10-故障排查) 的故障排查节。

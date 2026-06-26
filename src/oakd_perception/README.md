# oakd_perception 文档中心

本包负责 OAK-D 设备侧感知能力（IMU 原始数据 + 深度点云）。

**重要说明：**
- 实际使用（生产/日常运行）只使用统一节点 `oakd_unified_node`。
- 独立节点 `oakd_imu_node` 和 `oakd_depth_node` 仅用于测试、对比和问题定位。

---

## 📚 文档导航

| 文档 | 用途 | 适合人群 |
|------|------|---------|
| **[QUICK_START.md](QUICK_START.md)** | 快速参考，参数说明与预置模式 | 快速上手 |
| **[IMU_QUICK_START.md](IMU_QUICK_START.md)** | IMU 使用指南与完整示例 | 需要IMU数据 |
| **[DEPTH_MODE_CONFIG.md](DEPTH_MODE_CONFIG.md)** | 深度模式详细配置 | 需要调整深度参数 |
| **[FOV_FILTER_QUICK_REF.md](FOV_FILTER_QUICK_REF.md)** | FOV过滤快速参考 | 需要过滤点云 |
| **[FOV_FILTER_RULES.md](FOV_FILTER_RULES.md)** | FOV过滤详细原理 | 理解过滤机制 |
| **[CHANGELOG.md](CHANGELOG.md)** | 版本变更记录 | 了解更新内容 |

---

## 🚀 快速开始

### 1. 构建

```bash
./scripts/with_venv.sh colcon build --packages-select oakd_perception imu_fusion
```

### 2. 启动（选择一种）

#### 推荐：统一节点（生产模式）
```bash
./scripts/with_venv.sh ros2 launch oakd_perception oakd_unified.launch.py
```

#### 快速脚本（按场景）
```bash
# 户外低功耗
./scripts/run_oakd_outdoor.sh

# 室内混合高精度
./scripts/run_oakd_indoor.sh

# 平衡模式
./scripts/run_oakd_balance.sh

# 纯主动最强
./scripts/run_oakd_active_max.sh
```

#### 仅测试IMU
```bash
./scripts/with_venv.sh ros2 run oakd_perception oakd_imu_node
```

#### 仅测试深度
```bash
./scripts/with_venv.sh ros2 run oakd_perception oakd_depth_node
```

### 3. 验证运行

```bash
# 查看节点列表
./scripts/with_venv.sh ros2 node list

# 查看发布的话题
./scripts/with_venv.sh ros2 topic list

# 监控IMU数据
./scripts/with_venv.sh ros2 topic echo /oakd/imu/raw

# 监控点云发布频率
./scripts/with_venv.sh ros2 topic hz /oakd/points
```

---

## 1️⃣ 包内节点说明

### oakd_unified_node（主节点，推荐）

**文件位置：** `oakd_perception/oakd_unified_node.py`

**作用：**
- 单进程连接 OAK-D，统一采集 IMU 与深度。
- 避免独立节点并发启动时的设备冲突（常见为 `X_LINK_DEVICE_ALREADY_IN_USE`）。

**默认输出：**
- `/oakd/imu/raw`（`sensor_msgs/Imu`）
- `/oakd/points`（`sensor_msgs/PointCloud2`）
- `/oakd/points_filtered`（`sensor_msgs/PointCloud2`，边界裁剪后）

**常用参数（详见 [DEPTH_MODE_CONFIG.md](DEPTH_MODE_CONFIG.md)）：**
- `imu_frequency` — IMU采样频率（默认 400Hz）
- `pointcloud_frequency` — 点云发布频率（默认 20Hz）
- `enable_passive_stereo` — 被动立体（默认 true）
- `enable_active_stereo` — 主动立体（默认 false）
- `ir_intensity` — IR强度（范围 0-1600，默认 1600）
- `sampling_step` — 降采样步长（默认 2）
- `min_depth` — 最小深度（默认 200mm）
- `max_depth` — 最大深度（默认 15000mm）
- `enable_fov_boundary_filter` — 是否启用四棱锥边界过滤（默认 true）
- `auto_estimate_fov` — 是否根据相机内参自动估计 FOV（默认 true）
- `fov_h_deg` / `fov_v_deg` — 手动 FOV 兜底值（默认 72.0 / 53.0）
- `fov_boundary_margin_m` — 边界安全带宽（米，默认 0.15）

**说明：** 点云在发布前会先按四棱锥侧面平面做边界裁剪；如果启用 `auto_estimate_fov`，则会基于相机内参自动得到水平/垂直 FOV，不需要手工填角度。
**补充：** 原始点云仍发布到 `/oakd/points`，过滤结果发布到 `/oakd/points_filtered`，方便下游按需选择。

**启动命令：**
```bash
./scripts/with_venv.sh ros2 launch oakd_perception oakd_unified.launch.py
```

---

### oakd_imu_node（仅测试）

**文件位置：** `oakd_perception/oakd_imu_node.py`

**作用：** 单独验证 IMU 采集链路

**默认输出：** `/oakd/imu/raw`（`sensor_msgs/Imu`）

**注意：** 不建议与 `oakd_depth_node` 同时运行在同一个OAK-D上

**完整使用指南→** [IMU_QUICK_START.md](IMU_QUICK_START.md)

**启动命令：**
```bash
./scripts/with_venv.sh ros2 run oakd_perception oakd_imu_node
```

---

### oakd_depth_node（仅测试）

**文件位置：** `oakd_perception/oakd_depth_node.py`

**作用：** 单独验证深度点云链路和立体参数

**默认输出：** `/oakd/points`（`sensor_msgs/PointCloud2`）

**注意：** 不建议与 `oakd_imu_node` 同时运行在同一个OAK-D上

**详细使用指南→** [DEPTH_MODE_CONFIG.md](DEPTH_MODE_CONFIG.md)

**启动命令：**
```bash
# 默认参数
./scripts/with_venv.sh ros2 run oakd_perception oakd_depth_node

# 自定义参数
./scripts/with_venv.sh ros2 run oakd_perception oakd_depth_node \
  --ros-args \
  -p enable_active_stereo:=true \
  -p ir_intensity:=1000
```

---

### fov_boundary_filter（工具模块）

**文件位置：** `oakd_perception/fov_boundary_filter.py`

**作用：** 点云 FOV 边界过滤与几何约束工具（非独立ROS节点）

**使用文档→** [FOV_FILTER_QUICK_REF.md](FOV_FILTER_QUICK_REF.md) 和 [FOV_FILTER_RULES.md](FOV_FILTER_RULES.md)

**基础使用：**
```python
from oakd_perception.fov_boundary_filter import remove_fov_boundary_points

# 固定过滤
filtered_points = remove_fov_boundary_points(points, margin=2.0)

# 自适应过滤
from oakd_perception.fov_boundary_filter import AdaptiveFOVBoundaryFilter
filter = AdaptiveFOVBoundaryFilter()
filtered, stats = filter.filter_adaptive(points)
```

---

## 2️⃣ 深度模式配置（快速指南）

三种深度估计模式，可根据场景灵活选择：

| 模式 | 被动立体 | 主动立体 | 适用场景 | 功耗 |
|-----|---------|---------|--------|------|
| **纯被动** | ✅ | ❌ | 户外强光、低功耗 | 低 |
| **纯主动** | ❌ | ✅ | 室内、弱光、无纹理 | 高 |
| **混合** | ✅ | ✅ | 全场景、最优精度 | 中 |

**快速命令：**
```bash
# 户外低功耗
./scripts/run_oakd_outdoor.sh
# 配置: passive:ON, active:OFF

# 室内高精度
./scripts/run_oakd_indoor.sh
# 配置: passive:ON, active:ON, ir=1000

# 完全黑暗
./scripts/run_oakd_active_max.sh
# 配置: passive:OFF, active:ON, ir=1600
```

**详细配置→** [DEPTH_MODE_CONFIG.md](DEPTH_MODE_CONFIG.md)

---

## 3️⃣ 完整启动流程

### 方式1：仅启动感知节点（推荐）

```bash
# 终端1：启动OAK-D统一节点
./scripts/run_oakd_balance.sh

# 终端2：启动IMU融合 + TF广播（可选）
../../scripts/with_venv.sh ros2 launch imu_fusion imu_fusion.launch.py \
  launch_imu_node:=false \
  raw_topic_0:=/oakd/imu/raw \
  fused_topic_0:=/oakd/imu/fused \
  frame_id_0:=oakd_imu_link \
  parent_frame:=map

# 终端3：启动可视化RViz（可选）
../../scripts/with_venv.sh rviz2
```

> 说明：本包 `scripts/run_oakd_*.sh` 负责 OAK-D 场景预设；完整导航栈使用根目录 `./scripts/with_venv.sh ros2 launch omni_bringup nvidia_3d_nav.launch.py` 启动。

### 方式2：一键启动完整系统

```bash
../../scripts/with_venv.sh ros2 launch omni_bringup nvidia_3d_nav.launch.py
```

### 方式3：独立测试单节点

```bash
# 测试IMU
./scripts/with_venv.sh ros2 run oakd_perception oakd_imu_node

# 测试深度
./scripts/with_venv.sh ros2 run oakd_perception oakd_depth_node
```

---

## 4️⃣ 运行检查与验证

### 查看节点状态

```bash
# 列出所有运行中的节点
./scripts/with_venv.sh ros2 node list

# 查看特定节点的参数
ros2 param list /oakd_depth_node
```

### 查看话题发布

```bash
# 列出所有话题
./scripts/with_venv.sh ros2 topic list

# 查看话题信息
./scripts/with_venv.sh ros2 topic info /oakd/points

# 实时查看IMU数据
./scripts/with_venv.sh ros2 topic echo /oakd/imu/raw

# 查看消息发布频率
./scripts/with_venv.sh ros2 topic hz /oakd/imu/raw
./scripts/with_venv.sh ros2 topic hz /oakd/points
```

### 运行时参数修改

```bash
# 查看参数值
ros2 param get /oakd_depth_node enable_active_stereo

# 实时修改参数（无需重启）
ros2 param set /oakd_depth_node ir_intensity 1200
```

---

## 5️⃣ 常见问题排查

### ❌ 设备占用错误（X_LINK_DEVICE_ALREADY_IN_USE）

**原因：** 上一个进程未完全释放设备

**解决方案：**
```bash
# 清理所有OAK-D相关进程
pkill -9 -f oakd_ 2>/dev/null || true
sleep 1

# 重新启动节点
./scripts/with_venv.sh ros2 launch oakd_perception oakd_unified.launch.py
```

### ❌ 启动后无点云输出

**检查清单：**
1. OAK-D 硬件是否正常连接：
   ```bash
   lsusb | grep Movidius
   ```

2. `/oakd/points` 话题是否存在：
   ```bash
   ./scripts/with_venv.sh ros2 topic list | grep oakd
   ```

3. 深度参数是否过严：
   - 检查 `min_depth` 和 `max_depth` 设置
   - 参考 [DEPTH_MODE_CONFIG.md](DEPTH_MODE_CONFIG.md)

4. 立体模式是否适合环境：
   - 户外强光 → 关闭主动立体（`enable_active_stereo:=false`）
   - 弱光环境 → 启用主动立体（`enable_active_stereo:=true`）

### ❌ IMU 数据缺失或噪声大

**参考指南→** [IMU_QUICK_START.md](IMU_QUICK_START.md#故障排除)

### ❌ 点云有异常点或边界噪声

**使用FOV过滤：** [FOV_FILTER_QUICK_REF.md](FOV_FILTER_QUICK_REF.md)

---

## 6️⃣ 文件结构

```
src/oakd_perception/
├── README.md                           # 本文档（中心导航）
├── QUICK_START.md                      # 快速参考（参数和预置）
├── DEPTH_MODE_CONFIG.md                # 深度配置详细指南
├── IMU_QUICK_START.md                  # IMU使用完整指南
├── FOV_FILTER_QUICK_REF.md             # FOV过滤快速参考
├── FOV_FILTER_RULES.md                 # FOV过滤原理详解
├── CHANGELOG.md                        # 版本变更记录
├── oakd_perception/
│   ├── oakd_unified_node.py            # 统一采集节点（推荐）
│   ├── oakd_imu_node.py                # IMU单独采集节点
│   ├── oakd_depth_node.py              # 深度单独采集节点
│   └── fov_boundary_filter.py          # FOV过滤工具模块
├── launch/
│   ├── oakd_unified.launch.py          # 统一节点启动文件
│   └── oakd_depth.launch.py            # 深度节点启动文件
├── config/
│   ├── outdoor_low_power.yaml          # 户外低功耗配置
│   ├── indoor_high_precision.yaml      # 室内高精度配置
│   ├── balanced_mode.yaml              # 平衡配置
│   └── active_stereo_max.yaml          # 纯主动最强配置
└── scripts/
    ├── run_oakd_outdoor.sh             # 户外快速启动脚本
    ├── run_oakd_indoor.sh              # 室内快速启动脚本
    ├── run_oakd_balance.sh             # 平衡快速启动脚本
    └── run_oakd_active_max.sh          # 纯主动快速启动脚本
```

---

## 7️⃣ 性能指标

### IMU采集
- 采样率：200-400 Hz
- 延迟：~5-10ms
- 加速度量程：±2g 到 ±16g（可配）
- 陀螺仪量程：±250 dps 到 ±2000 dps（可配）

### 深度点云
- 发布频率：20 Hz（可配）
- 分辨率：160×100 点（已优化）
- FOV：水平 72°，竖直 53°
- 有效范围：200-5000 mm

### CPU/内存占用
- 统一节点：~40-60% CPU，~150-200 MB RAM
- 仅IMU：~10-15% CPU，~50 MB RAM
- 仅深度：~30-50% CPU，~150 MB RAM

---

## 8️⃣ 相关资源

- **OAK-D官方文档：** https://docs.luxonis.com/
- **DepthAI Python API：** https://docs.luxonis.com/projects/api/en/latest/references/python/
- **ROS 2 sensor_msgs：** https://github.com/ros2/common_interfaces

---

## 9️⃣ 地面导航扩展路线

这个包提供 OAK-D 感知输入。地面导航主线应优先对接
`isaac_ros_visual_slam`、`isaac_ros_nvblox` 和 Nav2。

---

## 🔗 版本信息

**最后更新：** 2026-05-13

**主要更新：**
- ✅ 深度模式独立开关实现
- ✅ 完整配置文档
- ✅ FOV过滤工具集
- ✅ 快速启动脚本

详见 [CHANGELOG.md](CHANGELOG.md)

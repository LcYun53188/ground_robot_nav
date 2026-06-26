# IMU 融合节点多IMU配置指南

## 概览

IMU融合系统已重构以支持多IMU配置，同时保持单IMU的完全兼容性。当前默认 frame 仍使用 `oakd_imu_link`，以便与 OAK-D 统一节点的 TF 链保持一致；如需通用命名，可在启动参数中自定义。

## 文件结构变化

### 新的通用文件（保持向后兼容）

| 位置 | 新文件 | 旧文件 | 用途 |
|------|--------|--------|------|
| imu_fusion/imu_fusion/ | `imu_fusion_node.py` | `oakd_imu_fusion_node.py` | 通用IMU融合节点 |
| imu_fusion/imu_fusion/ | `imu_tf_broadcaster.py` | `oakd_imu_tf_broadcaster.py` | 通用TF广播节点 |
| imu_fusion/launch/ | `imu_fusion.launch.py` | `oakd_imu_fusion.launch.py` | 多IMU支持的通用启动文件 |
| imu_fusion/launch/ | `oakd_imu_fusion.launch.py` | - | 向后兼容包装器 |

### 可执行程序入口点

```bash
# 新的命名（推荐）
imu_fusion_node          # 替代 oakd_imu_fusion_node
imu_tf_broadcaster       # 替代 oakd_imu_tf_broadcaster

# 旧的命名（仍可用，向后兼容）
oakd_imu_fusion_node     # 已弃用
oakd_imu_tf_broadcaster  # 已弃用
```

## 使用场景

### 场景1：单IMU（默认配置）

保持完全兼容，使用新的启动文件：

```bash
# 新推荐方式
ros2 launch imu_fusion imu_fusion.launch.py

# 推荐命令入口（与 OAK-D 统一节点配套）
./scripts/with_venv.sh ros2 launch imu_fusion imu_fusion.launch.py \
  launch_imu_node:=false \
  raw_topic_0:=/oakd/imu/raw \
  fused_topic_0:=/oakd/imu/fused \
  frame_id_0:=oakd_imu_link \
  parent_frame:=map

# 或使用旧启动文件（向后兼容）
ros2 launch imu_fusion oakd_imu_fusion.launch.py
```

**默认主题和框架：**
- 原始IMU: `/imu/raw`（新名称，原为 `/oakd/imu/raw` 可自定义）
- 融合IMU: `/imu`（新名称，原为 `/oakd/imu` 可自定义）
- 框架ID: `oakd_imu_link`（默认，推荐；也可自定义为 `imu_link` 或其他名称）

### 场景2：多IMU支持（新功能）

#### 配置2个IMU的完整示例

若要使用多IMU，需创建自定义启动文件 `launch/multi_imu_example.launch.py`（不存在的话需新建）：

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ============ IMU 0 配置 ============
        Node(
            package='oakd_perception',
            executable='oakd_imu_node',
            name='oakd_imu_node_0',
            parameters=[
                {'topic_name': '/imu_0/raw'},
                {'frame_id': 'imu_0_link'},
                {'imu_frequency': 400},
            ],
        ),
        Node(
            package='imu_fusion',
            executable='imu_fusion_node',
            name='imu_fusion_node_0',
            parameters=[
                {'input_topic': '/imu_0/raw'},
                {'output_topic': '/imu_0'},
                {'frame_id': 'imu_0_link'},
                {'complementary_alpha': 0.98},
            ],
        ),
        Node(
            package='imu_fusion',
            executable='imu_tf_broadcaster',
            name='imu_tf_broadcaster_0',
            parameters=[
                {'input_topic': '/imu_0'},
                {'parent_frame': 'map'},
                {'child_frame': 'imu_0_link'},
            ],
        ),
        
        # ============ IMU 1 配置 ============
        Node(
            package='oakd_perception',
            executable='oakd_imu_node',
            name='oakd_imu_node_1',
            parameters=[
                {'topic_name': '/imu_1/raw'},
                {'frame_id': 'imu_1_link'},
                {'imu_frequency': 400},
            ],
        ),
        Node(
            package='imu_fusion',
            executable='imu_fusion_node',
            name='imu_fusion_node_1',
            parameters=[
                {'input_topic': '/imu_1/raw'},
                {'output_topic': '/imu_1'},
                {'frame_id': 'imu_1_link'},
                {'complementary_alpha': 0.98},
            ],
        ),
        Node(
            package='imu_fusion',
            executable='imu_tf_broadcaster',
            name='imu_tf_broadcaster_1',
            parameters=[
                {'input_topic': '/imu_1'},
                {'parent_frame': 'map'},
                {'child_frame': 'imu_1_link'},
            ],
        ),
    ])
```

启动多IMU系统：

```bash
ros2 launch imu_fusion multi_imu_example.launch.py
```

**多IMU主题/框架命名约定：**
- IMU 0: `/imu_0/raw` → `/imu_0` → TF `imu_0_link`
- IMU 1: `/imu_1/raw` → `/imu_1` → TF `imu_1_link`
- IMU N: `/imu_N/raw` → `/imu_N` → TF `imu_N_link`

### 场景3：命令行参数自定义

单IMU使用自定义主题和框架：

```bash
ros2 launch imu_fusion imu_fusion.launch.py \
  raw_topic_0:=/my_imu/raw \
  fused_topic_0:=/my_imu \
  frame_id_0:=my_imu_link \
  parent_frame:=base_link
```

具体参数：
- `raw_topic_0`: 原始IMU主题（默认: `/imu/raw`）
- `fused_topic_0`: 融合后IMU主题（默认: `/imu`）
- `frame_id_0`: IMU框架ID（默认: `oakd_imu_link`）
- `parent_frame`: TF父框架（默认: `map`）
- `imu_frequency`: IMU采样频率Hz（默认: `400`）
- `num_imus`: IMU数量（默认: `1`，预留用）

## 融合参数配置

### complementary_alpha（互补滤波系数）

控制陀螺仪和加速度计的融合比例：

```python
# 更信任陀螺仪（快速响应但容易漂移）
complementary_alpha: 0.99

# 平衡（推荐，默认值）
complementary_alpha: 0.98

# 更信任加速度计（响应慢但稳定）
complementary_alpha: 0.95
```

**调整建议：**
- 如果系统漂移明显 → 降低alpha
- 如果融合结果噪声过大 → 提高alpha
- 不同IMU类型可能需要不同值

### 在启动文件中设置：

```python
Node(
    package='imu_fusion',
    executable='imu_fusion_node',
    name='imu_fusion_node_0',
    parameters=[
        {'input_topic': '/imu_0/raw'},
        {'output_topic': '/imu_0'},
        {'frame_id': 'imu_0_link'},
        {'complementary_alpha': 0.96},  # 自定义融合系数
    ],
),
```

## TF变换树

### 单IMU示例

```
map
└── oakd_imu_link (orientation only)
    └── (optional: IMU module attached to robot frame)
```

### 多IMU示例

```
map
├── imu_0_link (orientation only)
└── imu_1_link (orientation only)
```

## 数据流

### 单IMU管道

```
OAK-D Hardware
    ↓
oakd_imu_node (raw acquisition)
    ↓
/imu/raw (sensor_msgs/Imu: accel + gyro, no orientation)
    ↓
imu_fusion_node (complementary filtering)
    ↓
/imu (sensor_msgs/Imu: accel + gyro + orientation)
    ↓
imu_tf_broadcaster (TF publishing)
    ↓
map → oakd_imu_link (TF transform)
```

### 多IMU管道

```
OAK-D 0              OAK-D 1
  ↓                   ↓
imu_node_0          imu_node_1
  ↓                   ↓
/imu_0/raw          /imu_1/raw
  ↓                   ↓
fusion_node_0       fusion_node_1
  ↓                   ↓
/imu_0              /imu_1
  ↓                   ↓
tf_broadcaster_0    tf_broadcaster_1
  ↓                   ↓
TF: imu_0_link      TF: imu_1_link
```

## 预留配置点

启动文件 `imu_fusion.launch.py` 已声明了多IMU配置的参数（`raw_topic_0`, `raw_topic_1`, `raw_topic_2` 等），可用于扩展多IMU支持。

目前实现：
- ✅ IMU 0（主IMU）自动启动
- ⏳ IMU 1 和 IMU 2 参数已声明，需手动在启动文件中添加节点配置

如需启用多IMU，请创建自定义启动文件或直接编辑 `imu_fusion.launch.py`，在 `nodes` 列表中添加IMU 1和IMU 2的节点定义，参考场景2中的多IMU示例。

## 性能特性

| 特性 | 说明 |
|------|------|
| **融合算法** | 互补滤波 (Complementary Filter) |
| **计算复杂度** | O(1)，适合资源受限环境 |
| **延迟** | < 5ms（取决于硬件） |
| **精度** | ±5° 典型（补偿了漂移） |
| **max IMUs** | 理论无限（实际受CPU限制） |

## 故障排除

### 问题：TF不能发布
**检查步骤：**
```bash
# 1. 验证融合节点是否输出
ros2 topic echo /imu

# 2. 检查orientation_covariance是否有效
# （应该 >= 0，不是 -1）

# 3. 验证TF话题
ros2 tf2_monitor
```

### 问题：多IMU主题冲突
**解决方案：**
确保每个IMU使用唯一的主题和框架名：
```bash
raw_topic_0: /imu_0/raw   （IMU 0）
raw_topic_1: /imu_1/raw   （IMU 1）
frame_id_0: imu_0_link    （IMU 0）
frame_id_1: imu_1_link    （IMU 1）
```

### 问题：融合结果噪声过大或漂移
**微调建议：**
```bash
# 降低alpha来减少漂移
complementary_alpha: 0.94

# 或提高alpha来减少噪声
complementary_alpha: 0.99
```

## 向后兼容性

所有旧代码继续工作：

```bash
# 这两个命令等效
ros2 launch imu_fusion imu_fusion.launch.py
ros2 launch imu_fusion oakd_imu_fusion.launch.py  # 仍可用

# 这两个可执行程序等效
ros2 run imu_fusion imu_fusion_node
ros2 run imu_fusion oakd_imu_fusion_node  # 仍可用
```

## 迁移指南（从旧系统）

### 步骤1：更新主题监听

```python
# 旧代码
sub = self.create_subscription(Imu, '/oakd/imu', callback)

# 新推荐
sub = self.create_subscription(Imu, '/imu', callback)
```

### 步骤2：更新TF查询

```python
# 旧代码
tf_buffer.lookup_transform('map', 'oakd_imu_link', rclpy.time.Time())

# 新推荐
tf_buffer.lookup_transform('map', 'oakd_imu_link', rclpy.time.Time())
```

### 步骤3：更新启动文件

```xml
<!-- 旧方式 -->
<launch>
  <include file="$(find imu_fusion)/launch/oakd_imu_fusion.launch.py"/>
</launch>

<!-- 新推荐 -->
<launch>
  <include file="$(find imu_fusion)/launch/imu_fusion.launch.py"/>
</launch>
```

### 步骤4：切换到脚本入口

```bash
# OAK-D 统一节点
src/oakd_perception/scripts/run_oakd_balance.sh

# IMU 融合 + TF 广播
./scripts/with_venv.sh ros2 launch imu_fusion imu_fusion.launch.py \
  launch_imu_node:=false \
  raw_topic_0:=/oakd/imu/raw \
  fused_topic_0:=/oakd/imu/fused \
  frame_id_0:=oakd_imu_link \
  parent_frame:=map

# 当前地面导航主入口
./scripts/with_venv.sh ros2 launch omni_bringup nvidia_3d_nav.launch.py
```

## 总结

- ✅ **单IMU**：完全兼容，使用新的通用名称
- ✅ **多IMU**：预留配置框架，轻松扩展
- ✅ **向后兼容**：旧代码和启动文件仍可用
- ✅ **可扩展**：框架支持3个或更多IMU
- ✅ **性能**：complementary filter算法高效轻量

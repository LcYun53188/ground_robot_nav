# OAK-D 深度模式配置指南

📌 **回到主文档：** [README.md](README.md)

---

## 概述

节点支持三种深度估计模式：

| 模式 | 被动立体 | 主动立体 | 推荐场景 |
|------|--------|--------|--------|
| 纯被动 | ✅ ON | ❌ OFF | 户外强光、低功耗 |
| 纯主动 | ❌ OFF | ✅ ON | 室内、弱光、无纹理 |
| 混合 | ✅ ON | ✅ ON | 全场景、最优精度 |

## 参数说明

### 1. `enable_passive_stereo` (布尔值，默认: `True`)
- **值**: `true` 或 `false`
- **效果**:
  - `true`: 启用双目被动立体深度
  - `false`: 关闭被动立体（需启用主动立体）
- **配置**:
  - 启用时：使用更强的硬件滤波器（7x7中值滤波、空间滤波）
  - 关闭时：降低滤波强度节省计算

### 2. `enable_active_stereo` (布尔值，默认: `False`)
- **值**: `true` 或 `false`
- **效果**:
  - `true`: 启用 IR 投影仪主动立体
  - `false`: 关闭 IR 投影仪
- **配置**:
  - 启用时：自动使用 HIGH_DENSITY 预设（更密集的点云）
  - 关闭时：使用 FAST_DENSITY 预设（性能优先）

### 3. `ir_intensity` (整数，范围: 0-1600，默认: `1600`)
- **说明**: IR 投影仪发光强度
- **建议值**:
  - `0-400`: 弱光补足（低功耗）
  - `400-800`: 中等环境（平衡功耗和精度）
  - `800-1600`: 强光或远距离（最大精度，功耗高）

---

## 使用方法

> 生产/日常运行建议通过根目录 [run_nav_stack.sh](../../scripts/run_nav_stack.sh) 启动导航栈；单独调试 OAK-D 时使用本包 `scripts/` 下的场景预设。下面命令主要用于单独测试深度链路或验证参数。

### 方法 1: 命令行参数运行

#### 场景 A: 仅被动立体（默认，户外低功耗）
```bash
./scripts/with_venv.sh ros2 run oakd_perception oakd_depth_node
```

#### 场景 B: 启用主动立体（室内弱光）
```bash
./scripts/with_venv.sh ros2 run oakd_perception oakd_depth_node \
  --ros-args \
  -p enable_active_stereo:=true \
  -p ir_intensity:=1000
```

#### 场景 C: 禁用被动立体，仅主动立体（低延迟）
```bash
./scripts/with_venv.sh ros2 run oakd_perception oakd_depth_node \
  --ros-args \
  -p enable_passive_stereo:=false \
  -p enable_active_stereo:=true \
  -p ir_intensity:=1200
```

#### 场景 D: 混合模式（最优精度）
```bash
./scripts/with_venv.sh ros2 run oakd_perception oakd_depth_node \
  --ros-args \
  -p enable_passive_stereo:=true \
  -p enable_active_stereo:=true \
  -p ir_intensity:=800
```

### 方法 2: YAML 配置文件

创建 `config/depth_params.yaml`：

```yaml
oakd_depth_node:
  ros__parameters:
    enable_passive_stereo: true
    enable_active_stereo: false
    ir_intensity: 1600
```

运行：
```bash
./scripts/with_venv.sh ros2 run oakd_perception oakd_depth_node \
  --ros-args --params-file config/depth_params.yaml
```

### 方法 3: 启动文件（推荐）

创建 `launch/oakd_depth.launch.py`：

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='oakd_perception',
            executable='oakd_depth_node',
            name='oakd_depth_node',
            parameters=[
                {'enable_passive_stereo': True},
                {'enable_active_stereo': True},
                {'ir_intensity': 1000},
            ],
            output='screen'
        )
    ])
```

运行：
```bash
./scripts/with_venv.sh ros2 launch oakd_perception oakd_depth.launch.py
```

---

## 运行时监控

### 查看节点启动信息
```bash
./scripts/with_venv.sh ros2 launch oakd_perception oakd_depth.launch.py | grep -E "Passive|Active|IR Intensity"
```

*示例输出*:
```
[INFO] [oakd_depth_node]: Passive Stereo: ON
[INFO] [oakd_depth_node]: Active Stereo:  ON
[INFO] [oakd_depth_node]: IR Intensity: 1000
[INFO] [oakd_depth_node]: OAK-D 点云驱动节点已启动 [深度模式: 被动立体 + 主动立体(IR=1000)]，正在发布点云...
```

### 实时查看参数
```bash
ros2 param list /oakd_depth_node
ros2 param get /oakd_depth_node enable_active_stereo
```

### 实时修改参数（运行中）
```bash
ros2 param set /oakd_depth_node ir_intensity 800
```

---

## 性能对比

| 配置 | CPU 占用 | 内存 | 点云密度 | 噪声 | 功耗 |
|------|--------|------|--------|------|------|
| 仅被动 | 中/低 | 低 | 中 | 中等 | 低 |
| 仅主动 | 中 | 中 | 高 | 低 | 高 |
| 混合(IR=800) | 中/高 | 中 | 高 | 低 | 中 |
| 混合(IR=1600) | 高 | 中 | 很高 | 很低 | 高 |

---

## 故障排除

### 问题 1: 禁用被动立体但点云为空
**原因**: 未启用主动立体
**解决方案**:
```bash
ros2 param set /oakd_depth_node enable_passive_stereo true
# 或设置 enable_active_stereo:=true
```

### 问题 2: 室内点云很稀疏/噪声多
**原因**: 可能使用了仅被动立体或 IR 强度不足
**解决方案**:
```bash
ros2 param set /oakd_depth_node enable_active_stereo true
ros2 param set /oakd_depth_node ir_intensity 1200
```

### 问题 3: 室外深度不稳定/前景消失
**原因**: 阳光干扰了 IR 投影
**解决方案**:
```bash
ros2 param set /oakd_depth_node enable_active_stereo false
```

---

## 推荐配置

| 应用场景 | 参数组合 |
|--------|--------|
| **地面户外巡检** | `passive:true, active:false` (纯被动，低功耗) |
| **室内 SLAM** | `passive:true, active:true, ir:1000` (混合高精度) |
| **弱光环境** | `passive:false, active:true, ir:1600` (纯主动最强) |
| **通用高精度** | `passive:true, active:true, ir:800` (平衡注入) |
| **低延迟实时** | `passive:false, active:true, ir:600` (快速处理) |

---

## 相关文档

- **快速参考** → [QUICK_START.md](QUICK_START.md)
- **IMU使用指南** → [IMU_QUICK_START.md](IMU_QUICK_START.md)
- **FOV过滤参考** → [FOV_FILTER_QUICK_REF.md](FOV_FILTER_QUICK_REF.md)
- **FOV过滤原理** → [FOV_FILTER_RULES.md](FOV_FILTER_RULES.md)
- **版本更新记录** → [CHANGELOG.md](CHANGELOG.md)

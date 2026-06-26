# OAK-D IMU 使用指南

📌 **回到主文档：** [README.md](README.md)

---

## 概述

本文档说明如何从OAK-D深度摄像头获取IMU(惯性测量单元)数据。OAK-D配备6轴IMU传感器，提供：
- **加速度计**: 三轴线性加速度(m/s²)
- **陀螺仪**: 三轴角速度(rad/s)

## 快速开始

### 1. 编译项目

```bash
cd /home/nuc/Program/ground_robot_nav_ws
./scripts/with_venv.sh colcon build --packages-select oakd_perception
```

### 2. 运行IMU raw 节点

```bash
# 方式1: 直接运行 raw 采集节点
./scripts/with_venv.sh ros2 run oakd_perception oakd_imu_node

# 方式2: 使用配置文件启动 raw 节点
./scripts/with_venv.sh ros2 run oakd_perception oakd_imu_node \
  --ros-args \
  --params-file src/oakd_perception/config/imu_default.yaml

# 方式3: 使用当前地面导航主入口启动 OAK-D + VIO + 建图导航
./scripts/with_venv.sh ros2 launch omni_bringup nvidia_3d_nav.launch.py
```

### 2.1 运行 IMU 融合和 TF 广播器

如果只想单独启动融合和 TF 广播器，可以这样运行：

```bash
./scripts/with_venv.sh ros2 launch imu_fusion imu_fusion.launch.py \
  launch_imu_node:=false \
  raw_topic_0:=/oakd/imu/raw \
  fused_topic_0:=/oakd/imu/fused \
  frame_id_0:=oakd_imu_link \
  parent_frame:=map
```

### 3. 查看 IMU 数据

在另一个终端中：

```bash
# 查看原始 IMU 消息
./scripts/with_venv.sh ros2 topic echo /oakd/imu/raw

# 查看融合后的 IMU 消息
./scripts/with_venv.sh ros2 topic echo /oakd/imu

# 查看消息频率
./scripts/with_venv.sh ros2 topic hz /oakd/imu/raw

# 查看消息详情
./scripts/with_venv.sh ros2 topic info /oakd/imu
```

## 配置参数

在 `config/imu_default.yaml` 中配置以下参数：

| 参数 | 类型 | 范围 | 说明 |
|------|------|------|------|
| `imu_frequency` | int | 200-400 | IMU采样频率(Hz) |
| `gyro_full_scale` | string | - | 陀螺仪量程 |
| `accel_full_scale` | string | - | 加速度计量程 |

### 陀螺仪量程选项
- `gyroscope_250_dps`: ±250 deg/s
- `gyroscope_500_dps`: ±500 deg/s
- `gyroscope_1000_dps`: ±1000 deg/s
- `gyroscope_2000_dps`: ±2000 deg/s (默认)

### 加速度计量程选项
- `accelerometer_2g`: ±2g (0.19 m/s²)
- `accelerometer_4g`: ±4g (0.39 m/s²) (默认)
- `accelerometer_8g`: ±8g (0.78 m/s²)
- `accelerometer_16g`: ±16g (1.56 m/s²)

## ROS2 消息格式

发布主题: `/oakd/imu`  
消息类型: `sensor_msgs/Imu`

### 消息字段说明

```python
# 线性加速度 (m/s²)
imu.linear_acceleration.x   # X轴加速度
imu.linear_acceleration.y   # Y轴加速度
imu.linear_acceleration.z   # Z轴加速度

# 角速度 (rad/s)
imu.angular_velocity.x      # X轴旋转速度
imu.angular_velocity.y      # Y轴旋转速度
imu.angular_velocity.z      # Z轴旋转速度

# 协方差矩阵
imu.linear_acceleration_covariance    # 加速度计协方差 (3x3)
imu.angular_velocity_covariance       # 陀螺仪协方差 (3x3)
```

## 同时运行深度点云 + IMU

### 使用 Launch File（推荐）

然后运行：
```bash
./scripts/with_venv.sh ros2 launch omni_bringup nvidia_3d_nav.launch.py
```

### 使用两个终端

终端1：
```bash
./scripts/with_venv.sh ros2 run oakd_perception oakd_depth_node
```

终端2：
```bash
./scripts/with_venv.sh ros2 run oakd_perception oakd_imu_node
./scripts/with_venv.sh ros2 launch imu_fusion imu_fusion.launch.py \
  launch_imu_node:=false \
  raw_topic_0:=/oakd/imu/raw \
  fused_topic_0:=/oakd/imu/fused \
  frame_id_0:=oakd_imu_link \
  parent_frame:=map
```

## 故障排除

### 问题1: "找不到OAK-D设备"
```
DeviceError: X_LINK_ERROR
```

**解决方案:**
- 检查USB连接是否良好
- 检查OAK-D是否被识别: `lsusb | grep Movidius`
- 尝试重新插拔USB

### 问题2: IMU没有数据输出
- 检查频率设置(使用400Hz)
- 确保管道配置正确
- 查看日志输出: `ros2 topic echo /oakd/imu`

### 问题3: 数据噪音过大
- 增加陀螺仪/加速度计的最大值范围
- 调整协方差参数
- 放在稳定的平台上进行校准

## 应用示例

### 1. 机器人姿态估计
```python
#!/usr/bin/env python3
import rclpy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

def imu_callback(msg):
    # 从IMU数据计算机器人姿态
    ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
    wx, wy, wz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
    
    # 使用加速度和角速度估计姿态...
    pass

rclpy.init()
node = rclpy.create_node('robot_attitude')
node.create_subscription(Imu, '/oakd/imu', imu_callback, 10)
rclpy.spin(node)
```

### 2. 碰撞检测
```python
def imu_callback(msg):
    # 检测突然加速度变化(碰撞)
    accel_magnitude = (msg.linear_acceleration.x**2 + 
                       msg.linear_acceleration.y**2 + 
                       msg.linear_acceleration.z**2) ** 0.5
    
    if accel_magnitude > COLLISION_THRESHOLD:
        print("检测到碰撞!")
```

## 性能指标

| 指标 | 值 |
|------|-----|
| 采样率 | 200-400 Hz |
| 加速度计精度 | ±2-16g可选 |
| 陀螺仪精度 | ±250-2000 dps可选 |
| 延迟 | ~5-10ms |

## 参考资源

- [OAK-D官方文档](https://docs.luxonis.com/)
- [DepthAI Python API](https://docs.luxonis.com/projects/api/en/latest/references/python/)
- [ROS2 sensor_msgs](https://github.com/ros2/common_interfaces/tree/master/sensor_msgs)

---

## 相关文档

- **快速参考** → [QUICK_START.md](QUICK_START.md)
- **深度配置指南** → [DEPTH_MODE_CONFIG.md](DEPTH_MODE_CONFIG.md)
- **FOV过滤参考** → [FOV_FILTER_QUICK_REF.md](FOV_FILTER_QUICK_REF.md)
- **版本更新记录** → [CHANGELOG.md](CHANGELOG.md)

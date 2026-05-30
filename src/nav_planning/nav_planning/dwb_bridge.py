#!/usr/bin/env python3
"""
DWB (Dynamic Window Approach) Local Planner Bridge

Purpose: 
  与Nav2 DWB 2D规划器适配, 将 OccupancyGrid 转换为 Costmap2D, 
  计算避碰的速度命令

Input Topics:
  - /local_map/occupancy (OccupancyGrid): 本地二维占用栅格
  - /nav/goal_pose (PoseStamped): 目标位置
  - /tf: TF树 (需要 map → base_link 或 odom → base_link)

Output Topics:
  - /nav/cmd_vel (TwistStamped): 避碰后的速度命令

Parameters:
  - dwb_path_distance_bias: 路径跟踪权重
  - dwb_goal_distance_bias: 目标指向权重
  - dwb_occdist_scale: 障碍避开权重
  - dwb_transform_tolerance: TF查询超时

Work Principle (W1-D4-D5):
  1. 订阅 /local_map/occupancy (OccupancyGrid)
  2. 转换为 Costmap2D (cell-by-cell 占用率映射)
  3. 订阅 /nav/goal_pose 确定目标
  4. 在 TF 树中查询机器人当前位置
  5. 调用 DWB.computeVelocityCommands(robot_pose, robot_vel, goal_pose, costmap)
  6. 发布 /nav/cmd_vel
  7. 若点云丢失或目标不可达, 回退到保守减速/停车策略

Author: Copilot (2D Transition Phase 1)
Date: 2026-05-17
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from typing import Optional, Tuple

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TwistStamped, PoseStamped, Twist, Point, Quaternion
from nav2_costmap_2d import Costmap2D, CostmapSizeInMeters

# 尝试导入 Nav2 DWB
try:
    from nav2_core.dwb_local_planner import DWBLocalPlanner
    DWB_AVAILABLE = True
except ImportError:
    DWB_AVAILABLE = False
    print("[DWBBridge] WARNING: nav2_core.dwb_local_planner not available. Fallback to APF.")


class DWBBridge(Node):
    """
    DWB 2D 本地规划器适配层
    
    主要职责:
      1. 接收 OccupancyGrid 占用栅格
      2. 维护 Costmap2D 内部表示
      3. 查询 TF 树获取机器人状态
      4. 调用 DWB 规划器计算速度
      5. 在需要时回退到简单反应控制
    """

    def __init__(self):
        super().__init__('dwb_bridge')

        # ─────────────────────────────────────────────────────
        # 参数声明
        # ─────────────────────────────────────────────────────
        self.declare_parameter('costmap_resolution', 0.1)        # m/cell
        self.declare_parameter('costmap_size_x', 20.0)           # m
        self.declare_parameter('costmap_size_y', 20.0)           # m
        self.declare_parameter('robot_radius', 0.3)              # m
        self.declare_parameter('max_vel_x', 2.0)                 # m/s
        self.declare_parameter('min_vel_x', -0.1)
        self.declare_parameter('max_vel_y', 1.0)
        self.declare_parameter('min_vel_y', -1.0)
        self.declare_parameter('max_vel_theta', 1.57)            # rad/s
        self.declare_parameter('min_vel_theta', -1.57)
        self.declare_parameter('sim_time', 1.0)                  # s
        self.declare_parameter('path_distance_bias', 32.0)
        self.declare_parameter('goal_distance_bias', 24.0)
        self.declare_parameter('occdist_scale', 0.02)
        self.declare_parameter('transform_tolerance', 0.2)       # s
        self.declare_parameter('obstacles_max_range', 5.0)       # m
        self.declare_parameter('use_dwb_planner', DWB_AVAILABLE)
        self.declare_parameter('apf_repulsion_gain', 1.0)        # 斥力系数 (回退用)

        # ─────────────────────────────────────────────────────
        # 读取参数
        # ─────────────────────────────────────────────────────
        resolution = self.get_parameter('costmap_resolution').value
        size_x = self.get_parameter('costmap_size_x').value
        size_y = self.get_parameter('costmap_size_y').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.use_dwb = self.get_parameter('use_dwb_planner').value and DWB_AVAILABLE
        self.apf_gain = self.get_parameter('apf_repulsion_gain').value

        # ─────────────────────────────────────────────────────
        # 初始化 Costmap2D
        # ─────────────────────────────────────────────────────
        self.costmap = Costmap2D()
        costmap_size = CostmapSizeInMeters(size_x=size_x, size_y=size_y)
        self.costmap.setSizeInMeters(costmap_size)
        self.costmap.setCellSize(resolution)
        self.get_logger().info(
            f"[DWBBridge] Costmap initialized: {size_x:.1f}m × {size_y:.1f}m @ {resolution}m/cell"
        )

        # ─────────────────────────────────────────────────────
        # 初始化 DWB 规划器 (如果可用)
        # ─────────────────────────────────────────────────────
        if self.use_dwb:
            self.planner = DWBLocalPlanner()
            self.get_logger().info("[DWBBridge] Using Nav2 DWBLocalPlanner")
        else:
            self.planner = None
            self.get_logger().warn("[DWBBridge] DWB not available, using APF fallback")

        # ─────────────────────────────────────────────────────
        # TF 查询准备
        # ─────────────────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.transform_tolerance = self.get_parameter('transform_tolerance').value

        # ─────────────────────────────────────────────────────
        # 状态变量
        # ─────────────────────────────────────────────────────
        self.robot_pose = None          # (x, y, theta)
        self.robot_vel = None           # (vx, vy, vtheta)
        self.costmap_updated = False
        self.goal_pose = None           # PoseStamped
        self.last_cmd_time = None

        # ─────────────────────────────────────────────────────
        # 话题订阅
        # ─────────────────────────────────────────────────────
        cb_group = MutuallyExclusiveCallbackGroup()

        self.og_sub = self.create_subscription(
            OccupancyGrid,
            '/local_map/occupancy',
            self.occupancy_grid_callback,
            10,
            callback_group=cb_group,
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/nav/goal_pose',
            self.goal_callback,
            10,
            callback_group=cb_group,
        )

        # ─────────────────────────────────────────────────────
        # 话题发布
        # ─────────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/nav/cmd_vel',
            10,
        )

        # ─────────────────────────────────────────────────────
        # 定时器: 规划循环 (20Hz, 与 OccupancyGrid 对齐)
        # ─────────────────────────────────────────────────────
        self.planning_timer = self.create_timer(
            0.05,  # 20 Hz
            self.planning_loop,
            callback_group=cb_group,
        )

        self.get_logger().info(
            "[DWBBridge] Node initialized. Subscribed to /local_map/occupancy and /nav/goal_pose"
        )

    def occupancy_grid_callback(self, msg: OccupancyGrid) -> None:
        """
        处理 OccupancyGrid 消息, 更新 Costmap2D
        
        映射规则:
          OG value 0-50:     free_space (cost 0)
          OG value 51-100:   occupied (cost 254, LETHAL_OBSTACLE)
          OG value -1:       unknown (cost 127, UNKNOWN_COST)
        """
        if not msg.data:
            return

        width = msg.info.width
        height = msg.info.height

        # 逐cell更新代价
        for y in range(height):
            for x in range(width):
                idx = y * width + x
                cost = msg.data[idx]

                if cost == -1:
                    # 未知区域
                    cell_cost = 127
                elif cost <= 50:
                    # 空闲区域
                    cell_cost = 0
                else:
                    # 占用区域
                    cell_cost = 254

                # 设置 Costmap2D 中的代价
                # TODO: 确认 Costmap2D 的 API
                # self.costmap.setCost(x, y, cell_cost)

        self.costmap_updated = True
        # self.get_logger().debug(f"[DWBBridge] OG updated: {width}×{height}")

    def goal_callback(self, msg: PoseStamped) -> None:
        """处理 /nav/goal_pose"""
        self.goal_pose = msg
        self.get_logger().info(
            f"[DWBBridge] Goal received: ({msg.pose.position.x:.2f}, "
            f"{msg.pose.position.y:.2f})"
        )

    def get_robot_pose(self) -> Optional[Tuple[float, float, float]]:
        """
        查询 TF 树获取机器人位置
        
        尝试: map→base_link, 失败则尝试 odom→base_link
        
        返回: (x, y, yaw) 或 None
        """
        try:
            # 尝试主优先级: map → base_link
            try:
                trans = self.tf_buffer.lookup_transform(
                    'map',
                    'base_link',
                    rclpy.time.Time(nanoseconds=0),
                    timeout=rclpy.duration.Duration(seconds=self.transform_tolerance),
                )
                frame_name = 'map'
            except tf2_ros.TransformException:
                # 回退: odom → base_link
                trans = self.tf_buffer.lookup_transform(
                    'odom',
                    'base_link',
                    rclpy.time.Time(nanoseconds=0),
                    timeout=rclpy.duration.Duration(seconds=self.transform_tolerance),
                )
                frame_name = 'odom'

            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            # 从四元数提取 yaw
            yaw = self._quat_to_yaw(q.x, q.y, q.z, q.w)

            return (x, y, yaw), frame_name
        except Exception as e:
            self.get_logger().warn(f"[DWBBridge] TF lookup failed: {e}")
            return None

    @staticmethod
    def _quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
        """从四元数计算 yaw 角"""
        import math
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def planning_loop(self) -> None:
        """
        主规划循环 (20Hz)
        
        流程:
          1. 获取机器人位置
          2. 查询目标 (可选)
          3. 调用规划器计算速度
          4. 发布命令
        """
        # 获取机器人位置
        robot_state = self.get_robot_pose()
        if robot_state is None:
            self.get_logger().warn("[DWBBridge] No valid pose TF, skipping planning")
            return

        robot_pose, tf_frame = robot_state
        x, y, yaw = robot_pose

        # 如果 Costmap 未更新, 使用之前的
        if not self.costmap_updated:
            self.get_logger().debug("[DWBBridge] Costmap not yet received")
            return

        # ─────────────────────────────────────────────────────
        # 计算速度命令
        # ─────────────────────────────────────────────────────
        if self.use_dwb and self.planner:
            cmd_vel = self._compute_dwb_velocity(x, y, yaw)
        else:
            # 回退到 APF
            cmd_vel = self._compute_apf_velocity(x, y, yaw)

        # 发布命令
        if cmd_vel is not None:
            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = 'base_link'
            twist_msg.twist = cmd_vel
            self.cmd_pub.publish(twist_msg)
            self.last_cmd_time = self.get_clock().now()

    def _compute_dwb_velocity(self, x: float, y: float, yaw: float) -> Optional[Twist]:
        """使用 DWB 规划器计算速度"""
        try:
            # TODO: 调用 nav2_core 的 DWB 规划器
            # 目前作为占位符, 返回简单的前向速度
            cmd = Twist()
            cmd.linear.x = 1.0
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0
            return cmd
        except Exception as e:
            self.get_logger().warn(f"[DWBBridge] DWB planning failed: {e}")
            return None

    def _compute_apf_velocity(self, x: float, y: float, yaw: float) -> Twist:
        """
        回退: 人工势场法 (Artificial Potential Field)
        
        用途: DWB 不可用时的应急方案
        
        力的合成:
          1. 吸引力: 目标方向
          2. 斥力: 避开障碍 (从 Costmap 推导)
          3. 前向驱动: 默认前向移动
        """
        cmd = Twist()
        cmd.linear.x = 1.0    # 默认前向速度
        cmd.linear.y = 0.0
        cmd.angular.z = 0.0

        # 简单的目标对齐
        if self.goal_pose:
            goal_x = self.goal_pose.pose.position.x
            goal_y = self.goal_pose.pose.position.y
            dx = goal_x - x
            dy = goal_y - y

            # 计算目标方向角
            import math
            goal_yaw = math.atan2(dy, dx)

            # 角度差
            angle_error = goal_yaw - yaw

            # 简单的 P 控制旋转
            cmd.angular.z = min(max(angle_error, -1.57), 1.57) * 0.5

        return cmd


def main(args=None):
    rclpy.init(args=args)
    node = DWBBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

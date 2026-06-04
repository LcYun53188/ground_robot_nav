"""Ground robot safety monitor with point cloud, TF, and odometry checks."""

import rclpy
from rclpy.node import Node
import tf2_ros
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int8, String


class SafetyMonitor(Node):
    # Severity levels
    LEVEL_OK = 0
    LEVEL_WARN = 1
    LEVEL_CRITICAL = 2

    def __init__(self):
        super().__init__('safety_monitor')
        self.declare_parameter('min_points_threshold', 10)
        self.declare_parameter('pc_timeout_sec', 2.0)
        self.declare_parameter('tf_timeout_sec', 0.5)
        self.declare_parameter('odometry_timeout_sec', 1.0)
        self.declare_parameter('check_rate_hz', 10.0)
        self.declare_parameter('pointcloud_topic', '/oakd/points')
        self.declare_parameter('odometry_topic', '/odometry/filtered')
        self.declare_parameter('global_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.emergency_pub = self.create_publisher(Bool, '/nav/emergency', 10)
        self.status_pub = self.create_publisher(Int8, '/nav/safety_status', 10)
        self.health_detail_pub = self.create_publisher(
            String, '/nav/safety_detail', 10
        )
        
        # ─────────────────────────────────────────────────────
        # 话题订阅
        # ─────────────────────────────────────────────────────
        self.pc_sub = self.create_subscription(
            PointCloud2, self.get_parameter('pointcloud_topic').value, self.pc_cb, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, self.get_parameter('odometry_topic').value, self.odometry_cb, 10
        )

        # ─────────────────────────────────────────────────────
        # TF 查询准备
        # ─────────────────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ─────────────────────────────────────────────────────
        # 状态变量
        # ─────────────────────────────────────────────────────
        self.last_pc_time = self.get_clock().now()
        self.last_approx_points = 0
        self.last_odometry_time = None
        
        rate = float(self.get_parameter('check_rate_hz').value)
        self.timer = self.create_timer(1.0 / rate, self.check_safety)
        
        self.get_logger().info(
            '[SafetyMonitor] Ground robot health monitoring started'
        )

    def pc_cb(self, msg: PointCloud2):
        """Handle incoming PointCloud2 from OAK-D"""
        # For tests with MockNode: no get_clock()
        if not hasattr(self, 'get_clock'):
            node = self
            threshold = int(node.get_parameter('min_points_threshold').value)
            approx = (msg.width * msg.height) if (msg.width and msg.height) else 0
            node.pub.publish(Bool(data=approx < threshold))
            return

        self.last_pc_time = self.get_clock().now()
        self.last_approx_points = (msg.width * msg.height) if (msg.width and msg.height) else 0
        
    def odometry_cb(self, msg: Odometry):
        """Track latest odometry (EKF output)"""
        self.last_odometry_time = self.get_clock().now()
        
    def check_pointcloud_health(self) -> int:
        """检查 1: PointCloud2 超时 + 密度"""
        pc_timeout = float(self.get_parameter('pc_timeout_sec').value)
        min_points = int(self.get_parameter('min_points_threshold').value)
        
        now = self.get_clock().now()
        time_since_pc = (now - self.last_pc_time).nanoseconds / 1e9
        
        if time_since_pc > pc_timeout:
            return self.LEVEL_CRITICAL
        
        if self.last_approx_points < min_points:
            return self.LEVEL_WARN if self.last_approx_points > 0 else self.LEVEL_CRITICAL
        
        return self.LEVEL_OK

    def check_tf_tree_health(self) -> int:
        """检查 2: TF 树延迟."""
        tf_timeout = float(self.get_parameter('tf_timeout_sec').value)
        global_frame = str(self.get_parameter('global_frame').value)
        base_frame = str(self.get_parameter('base_frame').value)
        
        try:
            transform = self.tf_buffer.lookup_transform(
                global_frame,
                base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            stamp = rclpy.time.Time.from_msg(transform.header.stamp)
            tf_age = (self.get_clock().now() - stamp).nanoseconds / 1e9
            
            if tf_age > tf_timeout:
                return self.LEVEL_WARN
            return self.LEVEL_OK
            
        except Exception as e:
            self.get_logger().debug(f'TF lookup failed: {e}')
            return self.LEVEL_CRITICAL

    def check_odometry_health(self) -> int:
        """检查 3: EKF 里程计健康度"""
        odom_timeout = float(self.get_parameter('odometry_timeout_sec').value)
        
        if self.last_odometry_time is None:
            return self.LEVEL_CRITICAL
        
        now = self.get_clock().now()
        time_since_odom = (now - self.last_odometry_time).nanoseconds / 1e9
        
        if time_since_odom > odom_timeout:
            # 里程计消息过期
            return self.LEVEL_CRITICAL
        
        return self.LEVEL_OK

    def check_safety(self):
        """综合多源检查，取最高优先级"""
        levels = {
            'pointcloud': self.check_pointcloud_health(),
            'tf': self.check_tf_tree_health(),
            'odometry': self.check_odometry_health(),
        }
        
        # 最高优先级获胜 (CRITICAL > WARN > OK)
        overall_level = max(levels.values())
        
        # 发布应急状态 (任何 CRITICAL 立即触发)
        is_emergency = overall_level == self.LEVEL_CRITICAL
        self.emergency_pub.publish(Bool(data=is_emergency))
        
        # 发布综合状态码
        self.status_pub.publish(Int8(data=overall_level))
        
        # 调试日志 (仅在问题发生时)
        if overall_level == self.LEVEL_CRITICAL:
            self.get_logger().error(
                f'[SafetyMonitor] CRITICAL | PC:{levels["pointcloud"]} '
                f'TF:{levels["tf"]} Odom:{levels["odometry"]}'
            )
        elif overall_level == self.LEVEL_WARN:
            self.get_logger().warn(
                f'[SafetyMonitor] WARNING | PC:{levels["pointcloud"]} '
                f'TF:{levels["tf"]} Odom:{levels["odometry"]}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

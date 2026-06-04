"""Guard Visual SLAM odometry against implausible pose jumps."""

import copy
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def angle_delta(current, previous):
    return math.atan2(math.sin(current - previous), math.cos(current - previous))


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def pose_delta(current, previous):
    cp = current.pose.pose.position
    pp = previous.pose.pose.position
    dx = cp.x - pp.x
    dy = cp.y - pp.y
    dz = cp.z - pp.z
    dyaw = angle_delta(
        yaw_from_quaternion(current.pose.pose.orientation),
        yaw_from_quaternion(previous.pose.pose.orientation),
    )
    return dx, dy, dz, dyaw


class OdomJumpGuard(Node):
    """Reject or hold odometry samples that exceed configured motion limits."""

    def __init__(self):
        super().__init__("odom_jump_guard")
        self.declare_parameter("input_topic", "/visual_slam/tracking/odometry")
        self.declare_parameter("output_topic", "/visual_slam/guarded_odometry")
        self.declare_parameter("status_topic", "/visual_slam/odom_guard/status")
        self.declare_parameter("path_topic", "/visual_slam/guarded_path")
        self.declare_parameter("publish_path", True)
        self.declare_parameter("path_max_poses", 2000)
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("max_step_xy_m", 0.20)
        self.declare_parameter("max_step_z_m", 0.15)
        self.declare_parameter("max_yaw_step_deg", 20.0)
        self.declare_parameter("max_speed_xy_mps", 1.2)
        self.declare_parameter("max_yaw_rate_dps", 120.0)
        self.declare_parameter("min_dt_sec", 0.001)
        self.declare_parameter("publish_rejected_as_hold", True)
        self.declare_parameter("zero_twist_on_hold", True)
        self.declare_parameter("log_rejects", True)
        self.declare_parameter("log_reject_every_n", 25)
        self.declare_parameter("max_hold_sec_before_reseed", 2.0)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.status_topic = self.get_parameter("status_topic").value
        self.path_topic = self.get_parameter("path_topic").value
        self.publish_path = bool(self.get_parameter("publish_path").value)
        self.path_max_poses = int(self.get_parameter("path_max_poses").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.max_step_xy_m = float(self.get_parameter("max_step_xy_m").value)
        self.max_step_z_m = float(self.get_parameter("max_step_z_m").value)
        self.max_yaw_step = math.radians(
            float(self.get_parameter("max_yaw_step_deg").value)
        )
        self.max_speed_xy_mps = float(
            self.get_parameter("max_speed_xy_mps").value
        )
        self.max_yaw_rate = math.radians(
            float(self.get_parameter("max_yaw_rate_dps").value)
        )
        self.min_dt_sec = float(self.get_parameter("min_dt_sec").value)
        self.publish_rejected_as_hold = bool(
            self.get_parameter("publish_rejected_as_hold").value
        )
        self.zero_twist_on_hold = bool(self.get_parameter("zero_twist_on_hold").value)
        self.log_rejects = bool(self.get_parameter("log_rejects").value)
        self.log_reject_every_n = int(self.get_parameter("log_reject_every_n").value)
        self.max_hold_sec_before_reseed = float(
            self.get_parameter("max_hold_sec_before_reseed").value
        )

        self.last_accepted = None
        self.first_rejected_stamp = None
        self.accepted_count = 0
        self.rejected_count = 0
        self.held_count = 0
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.odom_frame

        self.odom_pub = self.create_publisher(Odometry, self.output_topic, 20)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.path_pub = (
            self.create_publisher(Path, self.path_topic, 10)
            if self.publish_path
            else None
        )
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.sub = self.create_subscription(Odometry, self.input_topic, self.odom_cb, 50)

        self.get_logger().info(
            "Odom jump guard started: "
            f"{self.input_topic} -> {self.output_topic}, "
            f"publish_tf={self.publish_tf}, publish_path={self.publish_path}"
        )

    def odom_cb(self, msg):
        if self.last_accepted is None:
            self.accept_sample(msg, "initial")
            return

        accepted, reason = self.should_accept(msg, self.last_accepted)
        if accepted:
            self.accept_sample(msg, "accepted")
            return

        self.rejected_count += 1
        if self.should_reseed(msg):
            self.get_logger().warn(
                "reseed guarded odom after sustained rejects: "
                f"held_for={self.rejected_duration(msg):.3f}s, "
                f"rejected={self.rejected_count}"
            )
            self.accept_sample(msg, "reseed")
            return

        if self.log_rejects and self.should_log_reject():
            self.get_logger().warn(reason)
        self.publish_status(reason)

        if self.publish_rejected_as_hold:
            self.publish_hold(msg.header.stamp)

    def should_accept(self, current, previous):
        dt = stamp_seconds(current.header.stamp) - stamp_seconds(previous.header.stamp)
        if dt < self.min_dt_sec:
            return False, f"reject non-increasing odom stamp: dt={dt:.6f}s"

        dx, dy, dz, dyaw = pose_delta(current, previous)
        step_xy = math.hypot(dx, dy)
        abs_dz = abs(dz)
        abs_dyaw = abs(dyaw)
        speed_xy = step_xy / dt
        yaw_rate = abs_dyaw / dt

        failures = []
        if step_xy > self.max_step_xy_m:
            failures.append(f"step_xy={step_xy:.3f}m>{self.max_step_xy_m:.3f}m")
        if abs_dz > self.max_step_z_m:
            failures.append(f"step_z={abs_dz:.3f}m>{self.max_step_z_m:.3f}m")
        if abs_dyaw > self.max_yaw_step:
            failures.append(
                f"step_yaw={math.degrees(abs_dyaw):.1f}deg>"
                f"{math.degrees(self.max_yaw_step):.1f}deg"
            )
        if speed_xy > self.max_speed_xy_mps:
            failures.append(
                f"speed_xy={speed_xy:.2f}m/s>{self.max_speed_xy_mps:.2f}m/s"
            )
        if yaw_rate > self.max_yaw_rate:
            failures.append(
                f"yaw_rate={math.degrees(yaw_rate):.1f}deg/s>"
                f"{math.degrees(self.max_yaw_rate):.1f}deg/s"
            )

        if failures:
            return (
                False,
                "reject odom jump: "
                + ", ".join(failures)
                + f", dt={dt:.3f}s, rejected={self.rejected_count + 1}",
            )
        return True, "accepted"

    def accept_sample(self, msg, status):
        guarded = copy.deepcopy(msg)
        guarded.header.frame_id = self.odom_frame
        guarded.child_frame_id = self.base_frame
        self.last_accepted = copy.deepcopy(guarded)
        self.first_rejected_stamp = None
        self.accepted_count += 1
        self.odom_pub.publish(guarded)
        self.publish_tf_msg(guarded)
        self.publish_path_msg(guarded)
        self.publish_status(
            f"{status}: accepted={self.accepted_count}, "
            f"rejected={self.rejected_count}, held={self.held_count}"
        )

    def publish_hold(self, stamp):
        held = copy.deepcopy(self.last_accepted)
        held.header.stamp = stamp
        if self.zero_twist_on_hold:
            held.twist.twist.linear.x = 0.0
            held.twist.twist.linear.y = 0.0
            held.twist.twist.linear.z = 0.0
            held.twist.twist.angular.x = 0.0
            held.twist.twist.angular.y = 0.0
            held.twist.twist.angular.z = 0.0
        self.held_count += 1
        self.odom_pub.publish(held)
        self.publish_tf_msg(held)
        self.publish_path_msg(held)

    def should_reseed(self, msg):
        if self.max_hold_sec_before_reseed <= 0.0:
            return False
        if self.first_rejected_stamp is None:
            self.first_rejected_stamp = msg.header.stamp
            return False
        return self.rejected_duration(msg) >= self.max_hold_sec_before_reseed

    def rejected_duration(self, msg):
        if self.first_rejected_stamp is None:
            return 0.0
        return stamp_seconds(msg.header.stamp) - stamp_seconds(self.first_rejected_stamp)

    def should_log_reject(self):
        if self.log_reject_every_n <= 1:
            return True
        return self.rejected_count == 1 or (
            self.rejected_count % self.log_reject_every_n == 0
        )

    def publish_tf_msg(self, odom):
        if self.tf_broadcaster is None:
            return

        transform = TransformStamped()
        transform.header.stamp = odom.header.stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = odom.pose.pose.position.x
        transform.transform.translation.y = odom.pose.pose.position.y
        transform.transform.translation.z = odom.pose.pose.position.z
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def publish_path_msg(self, odom):
        if self.path_pub is None:
            return

        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.path_msg.header.stamp = odom.header.stamp
        self.path_msg.header.frame_id = self.odom_frame
        self.path_msg.poses.append(pose)
        if self.path_max_poses > 0 and len(self.path_msg.poses) > self.path_max_poses:
            self.path_msg.poses = self.path_msg.poses[-self.path_max_poses :]
        self.path_pub.publish(self.path_msg)

    def publish_status(self, text):
        self.status_pub.publish(String(data=text))


def main(args=None):
    rclpy.init(args=args)
    node = OdomJumpGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

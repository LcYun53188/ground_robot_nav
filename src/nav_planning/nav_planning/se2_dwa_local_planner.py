import math
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener


class SE2DWALocalPlanner(Node):
    """Lightweight SE(2) DWA planner for ground robot navigation."""

    def __init__(self):
        super().__init__("se2_dwa_local_planner")

        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("map_timeout_sec", 1.0)
        self.declare_parameter("transform_timeout_sec", 0.1)
        self.declare_parameter("base_frame", "base_link")

        self.declare_parameter("max_vel_x", 0.8)
        self.declare_parameter("min_vel_x", -0.1)
        self.declare_parameter("max_vel_y", 0.5)
        self.declare_parameter("min_vel_y", -0.5)
        self.declare_parameter("max_yaw_rate", 0.8)

        self.declare_parameter("max_acc_x", 0.5)
        self.declare_parameter("max_acc_y", 0.5)
        self.declare_parameter("max_yaw_accel", 1.0)

        self.declare_parameter("sim_time", 1.2)
        self.declare_parameter("sim_dt", 0.1)
        self.declare_parameter("vx_samples", 9)
        self.declare_parameter("vy_samples", 9)
        self.declare_parameter("wz_samples", 11)

        self.declare_parameter("robot_radius", 0.35)
        self.declare_parameter("obstacle_clearance", 0.25)
        self.declare_parameter("slowdown_clearance", 0.8)
        self.declare_parameter("stop_clearance", 0.4)

        self.declare_parameter("forward_speed", 0.4)
        self.declare_parameter("goal_reached_xy", 0.4)
        self.declare_parameter("goal_distance_weight", 1.5)
        self.declare_parameter("heading_weight", 1.0)
        self.declare_parameter("obstacle_weight", 3.0)
        self.declare_parameter("velocity_weight", 0.2)
        self.declare_parameter("smoothness_weight", 0.5)
        self.declare_parameter("yaw_control_mode", "face_velocity")

        self.map_sub = self.create_subscription(
            OccupancyGrid, "/local_map/occupancy", self.map_cb, 10
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, "/nav/goal_pose", self.goal_cb, 10
        )
        self.emergency_sub = self.create_subscription(
            Bool, "/nav/emergency", self.emergency_cb, 10
        )
        self.cmd_pub = self.create_publisher(TwistStamped, "/nav/cmd_vel", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_map: Optional[OccupancyGrid] = None
        self.latest_map_time = None
        self.current_goal: Optional[PoseStamped] = None
        self.emergency_active = False
        self.last_cmd = (0.0, 0.0, 0.0)

        rate = float(self.get_parameter("control_rate_hz").value)
        self.timer = self.create_timer(max(0.01, 1.0 / rate), self.plan_once)

        self.get_logger().info("SE(2) DWA local planner initialized")

    def map_cb(self, msg: OccupancyGrid):
        self.latest_map = msg
        self.latest_map_time = self.get_clock().now()

    def goal_cb(self, msg: PoseStamped):
        self.current_goal = msg
        self.get_logger().info(
            f"New 2D goal: x={msg.pose.position.x:.2f}, "
            f"y={msg.pose.position.y:.2f}, frame={msg.header.frame_id}"
        )

    def emergency_cb(self, msg: Bool):
        self.emergency_active = bool(msg.data)
        if self.emergency_active:
            self.publish_halt("base_link")

    def plan_once(self):
        if self.emergency_active:
            self.publish_halt("base_link")
            return

        if self.latest_map is None or self.latest_map_time is None:
            self.publish_halt("base_link")
            return

        map_timeout = float(self.get_parameter("map_timeout_sec").value)
        map_age = (self.get_clock().now() - self.latest_map_time).nanoseconds / 1e9
        if map_age > map_timeout:
            self.get_logger().warn("Local map timeout; publishing halt")
            self.publish_halt(self.latest_map.header.frame_id or "map")
            return

        pose = self.get_robot_pose(self.latest_map.header.frame_id or "map")
        if pose is None:
            self.publish_halt(self.latest_map.header.frame_id or "map")
            return

        if self.goal_reached(pose):
            self.publish_halt(self.latest_map.header.frame_id or "map")
            return

        cmd = self.compute_best_command(pose, self.latest_map)
        if cmd is None:
            self.publish_halt(self.latest_map.header.frame_id or "map")
            return

        vx, vy, wz = cmd
        self.last_cmd = cmd
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.latest_map.header.frame_id or "map"
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = 0.0
        msg.twist.angular.z = wz
        self.cmd_pub.publish(msg)

    def get_robot_pose(self, frame_id: str) -> Optional[Tuple[float, float, float]]:
        base_frame = str(self.get_parameter("base_frame").value)
        timeout = float(self.get_parameter("transform_timeout_sec").value)
        try:
            transform = self.tf_buffer.lookup_transform(
                frame_id,
                base_frame,
                Time(),
                timeout=Duration(seconds=timeout),
            )
        except TransformException as exc:
            self.get_logger().warn(f"TF lookup {frame_id}->{base_frame} failed: {exc}")
            return None

        q = transform.transform.rotation
        yaw = self.quat_to_yaw(q.x, q.y, q.z, q.w)
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
            yaw,
        )

    def compute_best_command(
        self, pose: Tuple[float, float, float], grid: OccupancyGrid
    ) -> Optional[Tuple[float, float, float]]:
        occupied = self.occupied_cells(grid)
        clearance_now = self.nearest_obstacle_distance(pose[0], pose[1], grid, occupied)
        stop_clearance = float(self.get_parameter("stop_clearance").value)
        if clearance_now < stop_clearance:
            return self.brake_command()

        candidates = self.sample_velocity_window()
        best_cmd = None
        best_cost = math.inf

        for cmd in candidates:
            trajectory = self.rollout(pose, cmd)
            collision, min_clearance = self.trajectory_collision(
                trajectory, grid, occupied
            )
            if collision:
                continue

            cost = self.score_trajectory(trajectory, cmd, min_clearance)
            if cost < best_cost:
                best_cost = cost
                best_cmd = cmd

        if best_cmd is None:
            return self.brake_command()

        slowdown_clearance = float(self.get_parameter("slowdown_clearance").value)
        if clearance_now < slowdown_clearance:
            scale = max(0.2, clearance_now / slowdown_clearance)
            return (best_cmd[0] * scale, best_cmd[1] * scale, best_cmd[2] * scale)

        return best_cmd

    def sample_velocity_window(self) -> List[Tuple[float, float, float]]:
        vx0, vy0, wz0 = self.last_cmd
        rate = float(self.get_parameter("control_rate_hz").value)
        dynamic_window_dt = 1.0 / max(1.0, rate)

        min_vx = max(
            float(self.get_parameter("min_vel_x").value),
            vx0 - float(self.get_parameter("max_acc_x").value) * dynamic_window_dt,
        )
        max_vx = min(
            float(self.get_parameter("max_vel_x").value),
            vx0 + float(self.get_parameter("max_acc_x").value) * dynamic_window_dt,
        )
        min_vy = max(
            float(self.get_parameter("min_vel_y").value),
            vy0 - float(self.get_parameter("max_acc_y").value) * dynamic_window_dt,
        )
        max_vy = min(
            float(self.get_parameter("max_vel_y").value),
            vy0 + float(self.get_parameter("max_acc_y").value) * dynamic_window_dt,
        )
        max_wz_abs = float(self.get_parameter("max_yaw_rate").value)
        min_wz = max(
            -max_wz_abs,
            wz0 - float(self.get_parameter("max_yaw_accel").value) * dynamic_window_dt,
        )
        max_wz = min(
            max_wz_abs,
            wz0 + float(self.get_parameter("max_yaw_accel").value) * dynamic_window_dt,
        )

        return [
            (vx, vy, wz)
            for vx in self.linspace(min_vx, max_vx, int(self.get_parameter("vx_samples").value))
            for vy in self.linspace(min_vy, max_vy, int(self.get_parameter("vy_samples").value))
            for wz in self.linspace(min_wz, max_wz, int(self.get_parameter("wz_samples").value))
        ]

    def rollout(
        self, pose: Tuple[float, float, float], cmd: Tuple[float, float, float]
    ) -> List[Tuple[float, float, float]]:
        x, y, yaw = pose
        vx, vy, wz = cmd
        sim_time = float(self.get_parameter("sim_time").value)
        sim_dt = float(self.get_parameter("sim_dt").value)
        steps = max(1, int(math.ceil(sim_time / sim_dt)))
        trajectory = []

        for _ in range(steps):
            x += vx * sim_dt
            y += vy * sim_dt
            yaw = self.normalize_angle(yaw + wz * sim_dt)
            trajectory.append((x, y, yaw))
        return trajectory

    def score_trajectory(
        self,
        trajectory: List[Tuple[float, float, float]],
        cmd: Tuple[float, float, float],
        min_clearance: float,
    ) -> float:
        end_x, end_y, end_yaw = trajectory[-1]
        vx, vy, wz = cmd

        if self.current_goal is not None:
            goal_x = self.current_goal.pose.position.x
            goal_y = self.current_goal.pose.position.y
            goal_distance = math.hypot(goal_x - end_x, goal_y - end_y)
            desired_yaw = math.atan2(goal_y - end_y, goal_x - end_x)
        else:
            forward_speed = float(self.get_parameter("forward_speed").value)
            goal_distance = -((vx * math.cos(end_yaw) + vy * math.sin(end_yaw)) / max(0.1, forward_speed))
            desired_yaw = math.atan2(vy, vx) if math.hypot(vx, vy) > 0.05 else end_yaw

        yaw_mode = str(self.get_parameter("yaw_control_mode").value)
        if yaw_mode == "face_velocity" and math.hypot(vx, vy) > 0.05:
            desired_yaw = math.atan2(vy, vx)

        heading_error = abs(self.normalize_angle(desired_yaw - end_yaw))
        inv_clearance = 1.0 / max(min_clearance, 0.05)
        vx0, vy0, wz0 = self.last_cmd
        smoothness = math.hypot(vx - vx0, vy - vy0) + abs(wz - wz0)
        speed = math.hypot(vx, vy)

        return (
            float(self.get_parameter("goal_distance_weight").value) * goal_distance
            + float(self.get_parameter("heading_weight").value) * heading_error
            + float(self.get_parameter("obstacle_weight").value) * inv_clearance
            + float(self.get_parameter("smoothness_weight").value) * smoothness
            - float(self.get_parameter("velocity_weight").value) * speed
        )

    def trajectory_collision(
        self,
        trajectory: List[Tuple[float, float, float]],
        grid: OccupancyGrid,
        occupied: List[Tuple[int, int]],
    ) -> Tuple[bool, float]:
        min_clearance = math.inf
        collision_radius = float(self.get_parameter("robot_radius").value) + float(
            self.get_parameter("obstacle_clearance").value
        )

        for x, y, _yaw in trajectory:
            if not self.is_inside_grid(x, y, grid):
                return True, 0.0
            clearance = self.nearest_obstacle_distance(x, y, grid, occupied)
            min_clearance = min(min_clearance, clearance)
            if clearance < collision_radius:
                return True, clearance
        return False, min_clearance if math.isfinite(min_clearance) else 99.0

    def nearest_obstacle_distance(
        self,
        x: float,
        y: float,
        grid: OccupancyGrid,
        occupied: List[Tuple[int, int]],
    ) -> float:
        if not occupied:
            return 99.0
        res = grid.info.resolution
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y
        min_dist = math.inf
        for ix, iy in occupied:
            ox = origin_x + (ix + 0.5) * res
            oy = origin_y + (iy + 0.5) * res
            min_dist = min(min_dist, math.hypot(x - ox, y - oy))
        return min_dist if math.isfinite(min_dist) else 99.0

    @staticmethod
    def occupied_cells(grid: OccupancyGrid) -> List[Tuple[int, int]]:
        width = grid.info.width
        occupied = []
        for idx, value in enumerate(grid.data):
            if value >= 100:
                occupied.append((idx % width, idx // width))
        return occupied

    @staticmethod
    def is_inside_grid(x: float, y: float, grid: OccupancyGrid) -> bool:
        res = grid.info.resolution
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y
        ix = int((x - origin_x) / res)
        iy = int((y - origin_y) / res)
        return 0 <= ix < grid.info.width and 0 <= iy < grid.info.height

    def brake_command(self) -> Tuple[float, float, float]:
        vx0, vy0, wz0 = self.last_cmd
        sim_dt = float(self.get_parameter("sim_dt").value)
        ax = float(self.get_parameter("max_acc_x").value) * sim_dt
        ay = float(self.get_parameter("max_acc_y").value) * sim_dt
        aw = float(self.get_parameter("max_yaw_accel").value) * sim_dt
        return (
            self.approach_zero(vx0, ax),
            self.approach_zero(vy0, ay),
            self.approach_zero(wz0, aw),
        )

    def goal_reached(self, pose: Tuple[float, float, float]) -> bool:
        if self.current_goal is None:
            return False
        threshold = float(self.get_parameter("goal_reached_xy").value)
        dx = self.current_goal.pose.position.x - pose[0]
        dy = self.current_goal.pose.position.y - pose[1]
        if math.hypot(dx, dy) > threshold:
            return False
        self.current_goal = None
        self.get_logger().info("2D goal reached; hovering")
        return True

    def publish_halt(self, frame_id: str):
        self.last_cmd = (0.0, 0.0, 0.0)
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.z = 0.0
        self.cmd_pub.publish(msg)

    @staticmethod
    def linspace(start: float, stop: float, count: int) -> List[float]:
        if count <= 1:
            return [(start + stop) * 0.5]
        step = (stop - start) / float(count - 1)
        return [start + step * i for i in range(count)]

    @staticmethod
    def approach_zero(value: float, delta: float) -> float:
        if abs(value) <= delta:
            return 0.0
        return value - math.copysign(delta, value)

    @staticmethod
    def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = SE2DWALocalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

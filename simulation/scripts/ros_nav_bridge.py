#!/usr/bin/env python3
"""ROS2-side fallback bridge for Isaac Sim navigation smoke tests.

Isaac Sim and this workstation's ROS Jazzy environment can use incompatible
Python/rclpy ABIs. This helper runs under the workspace/ROS Python and
publishes the minimal topics expected by the Nav2/nvblox simulation launch.
"""

from __future__ import annotations

import argparse
import math
import socket
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster


def _load_yaml(path: Path) -> Dict[str, Any]:
    import yaml

    with path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def _deep_get(config: Dict[str, Any], dotted_key: str, default: Any) -> Any:
    value: Any = config
    for key in dotted_key.split("."):
        if not isinstance(value, dict) or key not in value:
            return default
        value = value[key]
    return value


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _quat_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


@dataclass
class RenderFrame:
    width: int
    height: int
    left_rgb: bytearray
    right_rgb: bytearray
    depth_32fc1: bytearray


@dataclass
class RobotState:
    x: float
    y: float
    z: float
    yaw: float
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


class IsaacRenderClient:
    """Small TCP client for Isaac-rendered RGB-D frames.

    ROS Jazzy and Isaac Sim use different Python ABIs on this host, so the
    Isaac process renders frames and this ROS-side process publishes them.
    """

    _HEADER = struct.Struct("<4sdIIIII")

    def __init__(self, config: Dict[str, Any]) -> None:
        self.enabled = str(_deep_get(config, "oakd.render_mode", "pattern")).lower() == "isaac"
        self.host = str(_deep_get(config, "ros.frame_server_host", "127.0.0.1"))
        self.port = int(_deep_get(config, "ros.frame_server_port", 47650))
        self.timeout = float(_deep_get(config, "ros.frame_server_timeout_sec", 0.10))
        self.sock: socket.socket | None = None
        self.last_frame: RenderFrame | None = None

    def request_frame(self, state: RobotState, sim_time: float) -> RenderFrame | None:
        if not self.enabled:
            return None
        if self.sock is None:
            self._connect()
        if self.sock is None:
            return self.last_frame
        try:
            self.sock.sendall(
                f"POSE {sim_time:.9f} {state.x:.9f} {state.y:.9f} {state.z:.9f} {state.yaw:.9f}\n".encode(
                    "ascii"
                )
            )
            header = self._read_exact(self._HEADER.size)
            if header is None:
                return self.last_frame
            magic, _frame_time, width, height, left_len, right_len, depth_len = self._HEADER.unpack(
                header
            )
            if magic != b"IRGB" or width <= 0 or height <= 0:
                self._disconnect()
                return self.last_frame
            payload = self._read_exact(left_len + right_len + depth_len)
            if payload is None:
                return self.last_frame
            left = bytearray(payload[:left_len])
            right = bytearray(payload[left_len : left_len + right_len])
            depth = bytearray(payload[left_len + right_len :])
            self.last_frame = RenderFrame(width, height, left, right, depth)
        except OSError:
            self._disconnect()
        return self.last_frame

    def _connect(self) -> None:
        try:
            self.sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
            self.sock.settimeout(self.timeout)
        except OSError:
            self.sock = None

    def _disconnect(self) -> None:
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None

    def _read_exact(self, size: int) -> bytes | None:
        chunks = bytearray()
        while len(chunks) < size:
            try:
                chunk = self.sock.recv(size - len(chunks)) if self.sock else b""
            except socket.timeout:
                return None
            if not chunk:
                self._disconnect()
                return None
            chunks.extend(chunk)
        return bytes(chunks)


class IsaacSimRosBridge(Node):
    def __init__(self, config: Dict[str, Any]) -> None:
        super().__init__("isaac_sim_ground_nav_bridge")
        self.config = config
        self.state = RobotState(
            x=float(_deep_get(config, "robot.initial_pose.x", 0.0)),
            y=float(_deep_get(config, "robot.initial_pose.y", 0.0)),
            z=float(_deep_get(
                config,
                "robot.initial_pose.z",
                float(_deep_get(config, "robot.dimensions.height_m", 0.16)) / 2.0,
            )),
            yaw=float(_deep_get(config, "robot.initial_pose.yaw", 0.0)),
        )
        self.cmd_vel = Twist()
        self.sim_time = 0.0
        self.dt = 1.0 / float(_deep_get(config, "oakd.frequency_hz", 30.0))
        self.max_v = float(_deep_get(config, "robot.max_linear_speed_mps", 0.6))
        self.max_w = float(_deep_get(config, "robot.max_angular_speed_radps", 1.0))

        self.base_frame = _deep_get(config, "robot.base_frame", "base_link")
        self.odom_frame = _deep_get(config, "robot.odom_frame", "odom")
        self.oakd_imu_frame = _deep_get(config, "oakd.imu_frame", "oakd_imu_link")
        self.oakd_optical_frame = _deep_get(
            config, "oakd.optical_frame", "oakd_camera_optical_frame"
        )
        self.left_frame = _deep_get(
            config, "oakd.left_frame", "oakd_left_camera_optical_frame"
        )
        self.right_frame = _deep_get(
            config, "oakd.right_frame", "oakd_right_camera_optical_frame"
        )
        self.width = int(_deep_get(config, "oakd.resolution.width", 640))
        self.height = int(_deep_get(config, "oakd.resolution.height", 400))
        self.baseline = float(_deep_get(config, "oakd.stereo_baseline_m", 0.075))
        self.oakd_pose = _deep_get(config, "oakd.pose", {})
        self.rgb_data = self._build_rgb_pattern()
        self.depth_data = self._build_depth_pattern()
        self.render_client = IsaacRenderClient(config)

        qos_depth = 10
        self.clock_pub = self.create_publisher(Clock, "/clock", qos_depth)
        self.odom_pub = self.create_publisher(
            Odometry,
            _deep_get(config, "robot.odometry_topic", "/visual_slam/tracking/odometry"),
            qos_depth,
        )
        self.left_image_pub = self.create_publisher(
            Image,
            _deep_get(config, "oakd.left_image_topic", "/oakd/left/image_raw"),
            qos_depth,
        )
        self.right_image_pub = self.create_publisher(
            Image,
            _deep_get(config, "oakd.right_image_topic", "/oakd/right/image_raw"),
            qos_depth,
        )
        self.depth_image_pub = self.create_publisher(
            Image,
            _deep_get(config, "oakd.depth_image_topic", "/oakd/depth/image"),
            qos_depth,
        )
        self.left_info_pub = self.create_publisher(
            CameraInfo,
            _deep_get(config, "oakd.left_camera_info_topic", "/oakd/left/camera_info"),
            qos_depth,
        )
        self.right_info_pub = self.create_publisher(
            CameraInfo,
            _deep_get(config, "oakd.right_camera_info_topic", "/oakd/right/camera_info"),
            qos_depth,
        )
        self.depth_info_pub = self.create_publisher(
            CameraInfo,
            _deep_get(config, "oakd.depth_camera_info_topic", "/oakd/depth/camera_info"),
            qos_depth,
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Twist,
            _deep_get(config, "robot.cmd_vel_topic", "/cmd_vel"),
            self._cmd_vel_callback,
            qos_depth,
        )
        self.create_timer(self.dt, self._tick)

    def _cmd_vel_callback(self, msg: Twist) -> None:
        self.cmd_vel = msg

    def _tick(self) -> None:
        if not rclpy.ok():
            return
        try:
            self.state.vx = _clamp(float(self.cmd_vel.linear.x), -self.max_v, self.max_v)
            self.state.vy = _clamp(float(self.cmd_vel.linear.y), -self.max_v, self.max_v)
            self.state.wz = _clamp(float(self.cmd_vel.angular.z), -self.max_w, self.max_w)

            cos_yaw = math.cos(self.state.yaw)
            sin_yaw = math.sin(self.state.yaw)
            self.state.x += (self.state.vx * cos_yaw - self.state.vy * sin_yaw) * self.dt
            self.state.y += (self.state.vx * sin_yaw + self.state.vy * cos_yaw) * self.dt
            self.state.yaw += self.state.wz * self.dt
            self.sim_time += self.dt

            stamp = self.get_clock().now().to_msg()
            stamp.sec = int(self.sim_time)
            stamp.nanosec = int((self.sim_time - int(self.sim_time)) * 1_000_000_000)
            self._publish_clock(stamp)
            self._publish_tf(stamp)
            self._publish_odom(stamp)
            frame = self.render_client.request_frame(self.state, self.sim_time)
            self._publish_camera(stamp, frame)
        except Exception as exc:
            if "context is invalid" not in str(exc):
                raise

    def _publish_clock(self, stamp: Any) -> None:
        msg = Clock()
        msg.clock = stamp
        self.clock_pub.publish(msg)

    def _publish_tf(self, stamp: Any) -> None:
        transforms = [
            self._transform(
                stamp,
                self.odom_frame,
                self.base_frame,
                self.state.x,
                self.state.y,
                self.state.z,
                self.state.yaw,
            ),
            self._transform(
                stamp,
                self.base_frame,
                self.oakd_imu_frame,
                float(self.oakd_pose.get("x", 0.18)),
                float(self.oakd_pose.get("y", 0.0)),
                float(self.oakd_pose.get("z", 0.16)),
                float(self.oakd_pose.get("yaw", 0.0)),
            ),
            self._transform(
                stamp, self.oakd_imu_frame, self.oakd_optical_frame, 0.0, 0.0, 0.0, 0.0
            ),
            self._transform(
                stamp, self.oakd_optical_frame, self.left_frame, 0.0, self.baseline * 0.5, 0.0, 0.0
            ),
            self._transform(
                stamp, self.oakd_optical_frame, self.right_frame, 0.0, -self.baseline * 0.5, 0.0, 0.0
            ),
        ]
        self.tf_broadcaster.sendTransform(transforms)

    def _transform(
        self, stamp: Any, parent: str, child: str, x: float, y: float, z: float, yaw: float
    ) -> TransformStamped:
        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = parent
        msg.child_frame_id = child
        msg.transform.translation.x = x
        msg.transform.translation.y = y
        msg.transform.translation.z = z
        qx, qy, qz, qw = _quat_from_yaw(yaw)
        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw
        return msg

    def _publish_odom(self, stamp: Any) -> None:
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = self.state.x
        msg.pose.pose.position.y = self.state.y
        msg.pose.pose.position.z = self.state.z
        qx, qy, qz, qw = _quat_from_yaw(self.state.yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = self.state.vx
        msg.twist.twist.linear.y = self.state.vy
        msg.twist.twist.angular.z = self.state.wz
        self.odom_pub.publish(msg)

    def _publish_camera(self, stamp: Any, frame: RenderFrame | None) -> None:
        width = frame.width if frame else self.width
        height = frame.height if frame else self.height
        left_data = frame.left_rgb if frame else self.rgb_data
        right_data = frame.right_rgb if frame else self.rgb_data
        depth_data = frame.depth_32fc1 if frame else self.depth_data
        left = self._image(stamp, self.left_frame, width, height, "rgb8", width * 3, left_data)
        right = self._image(stamp, self.right_frame, width, height, "rgb8", width * 3, right_data)
        depth = self._image(
            stamp, self.oakd_optical_frame, width, height, "32FC1", width * 4, depth_data
        )
        self.left_image_pub.publish(left)
        self.right_image_pub.publish(right)
        self.depth_image_pub.publish(depth)
        self.left_info_pub.publish(self._camera_info(stamp, self.left_frame, width, height))
        self.right_info_pub.publish(self._camera_info(stamp, self.right_frame, width, height))
        self.depth_info_pub.publish(
            self._camera_info(stamp, self.oakd_optical_frame, width, height)
        )

    def _image(
        self,
        stamp: Any,
        frame_id: str,
        width: int,
        height: int,
        encoding: str,
        step: int,
        data: bytearray,
    ) -> Image:
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = height
        msg.width = width
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = step
        msg.data = data
        return msg

    def _camera_info(self, stamp: Any, frame_id: str, width: int, height: int) -> CameraInfo:
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = height
        msg.width = width
        fx = fy = 420.0
        cx = width / 2.0
        cy = height / 2.0
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        return msg

    def _build_rgb_pattern(self) -> bytearray:
        data = bytearray(self.width * self.height * 3)
        for y in range(self.height):
            for x in range(self.width):
                offset = (y * self.width + x) * 3
                data[offset] = int(40 + 80 * x / max(1, self.width - 1))
                data[offset + 1] = int(60 + 60 * y / max(1, self.height - 1))
                data[offset + 2] = 120
        return data

    def _build_depth_pattern(self) -> bytearray:
        values = []
        obstacle_x_min = int(self.width * 0.42)
        obstacle_x_max = int(self.width * 0.58)
        obstacle_y_min = int(self.height * 0.36)
        obstacle_y_max = int(self.height * 0.68)
        for y in range(self.height):
            for x in range(self.width):
                depth_m = 3.5
                if obstacle_x_min <= x <= obstacle_x_max and obstacle_y_min <= y <= obstacle_y_max:
                    depth_m = 1.4
                values.append(depth_m)
        return bytearray(struct.pack(f"{len(values)}f", *values))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True, type=Path)
    args = parser.parse_args()

    config = _load_yaml(args.config)
    rclpy.init()
    node = IsaacSimRosBridge(config)
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

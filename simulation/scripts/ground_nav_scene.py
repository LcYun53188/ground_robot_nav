#!/usr/bin/env python3
"""Isaac Sim standalone scene for the ground robot Nav2 closed loop.

This script is intentionally self-contained so it can be launched by Isaac
Sim's python.sh. It publishes the ROS2 interface expected by
omni_bringup/isaac_sim_nav.launch.py and uses ground-truth odometry as the
simulation substitute for /visual_slam/tracking/odometry.
"""

from __future__ import annotations

import argparse
import math
import os
import socket
import struct
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict


def _load_yaml(path: Path) -> Dict[str, Any]:
    try:
        import yaml  # type: ignore
    except ImportError:
        return _load_simple_yaml(path)

    with path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def _load_simple_yaml(path: Path) -> Dict[str, Any]:
    root: Dict[str, Any] = {}
    stack: list[tuple[int, Dict[str, Any]]] = [(-1, root)]

    with path.open("r", encoding="utf-8") as stream:
        for raw_line in stream:
            line = raw_line.split("#", 1)[0].rstrip()
            if not line.strip():
                continue
            indent = len(raw_line) - len(raw_line.lstrip(" "))
            key_value = line.strip()
            while stack and indent <= stack[-1][0]:
                stack.pop()
            parent = stack[-1][1]
            if key_value.endswith(":"):
                key = key_value[:-1].strip()
                child: Dict[str, Any] = {}
                parent[key] = child
                stack.append((indent, child))
                continue
            if ":" not in key_value:
                continue
            key, value = key_value.split(":", 1)
            parent[key.strip()] = _parse_scalar(value.strip())
    return root


def _parse_scalar(value: str) -> Any:
    value = value.strip().strip('"').strip("'")
    if value.lower() in ("true", "false"):
        return value.lower() == "true"
    if value == "":
        return ""
    try:
        if "." in value:
            return float(value)
        return int(value)
    except ValueError:
        return value


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
class RobotState:
    x: float
    y: float
    z: float
    yaw: float
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


class RosInterface:
    def __init__(self, config: Dict[str, Any], state: RobotState) -> None:
        import rclpy
        from geometry_msgs.msg import TransformStamped, Twist
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import CameraInfo, Image
        from tf2_ros import TransformBroadcaster

        self.rclpy = rclpy
        self.TransformStamped = TransformStamped
        self.Twist = Twist
        self.Odometry = Odometry
        self.Clock = Clock
        self.Image = Image
        self.CameraInfo = CameraInfo

        self.state = state
        self.cmd_vel = Twist()
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
        self.depth_width = self.width
        self.depth_height = self.height
        self._rgb_data = self._build_rgb_pattern()
        self._depth_data = self._build_depth_pattern()
        self.oakd_pose = _deep_get(config, "oakd.pose", {})
        self.baseline = float(_deep_get(config, "oakd.stereo_baseline_m", 0.075))

        rclpy.init(args=None)
        self.node = rclpy.create_node("isaac_sim_ground_nav_bridge")
        qos_depth = 10
        self.clock_pub = self.node.create_publisher(Clock, "/clock", qos_depth)
        self.odom_pub = self.node.create_publisher(
            Odometry, _deep_get(config, "robot.odometry_topic", "/visual_slam/tracking/odometry"), qos_depth
        )
        self.left_image_pub = self.node.create_publisher(
            Image, _deep_get(config, "oakd.left_image_topic", "/oakd/left/image_raw"), qos_depth
        )
        self.right_image_pub = self.node.create_publisher(
            Image, _deep_get(config, "oakd.right_image_topic", "/oakd/right/image_raw"), qos_depth
        )
        self.depth_image_pub = self.node.create_publisher(
            Image, _deep_get(config, "oakd.depth_image_topic", "/oakd/depth/image"), qos_depth
        )
        self.left_info_pub = self.node.create_publisher(
            CameraInfo, _deep_get(config, "oakd.left_camera_info_topic", "/oakd/left/camera_info"), qos_depth
        )
        self.right_info_pub = self.node.create_publisher(
            CameraInfo, _deep_get(config, "oakd.right_camera_info_topic", "/oakd/right/camera_info"), qos_depth
        )
        self.depth_info_pub = self.node.create_publisher(
            CameraInfo, _deep_get(config, "oakd.depth_camera_info_topic", "/oakd/depth/camera_info"), qos_depth
        )
        self.tf_broadcaster = TransformBroadcaster(self.node)
        self.node.create_subscription(
            Twist,
            _deep_get(config, "robot.cmd_vel_topic", "/cmd_vel"),
            self._cmd_vel_callback,
            qos_depth,
        )

    def _cmd_vel_callback(self, msg: Any) -> None:
        self.cmd_vel = msg

    def spin_once(self) -> None:
        self.rclpy.spin_once(self.node, timeout_sec=0.0)

    def publish(self, sim_time: float) -> None:
        stamp = self.node.get_clock().now().to_msg()
        stamp.sec = int(sim_time)
        stamp.nanosec = int((sim_time - int(sim_time)) * 1_000_000_000)

        clock = self.Clock()
        clock.clock = stamp
        self.clock_pub.publish(clock)

        self._publish_tf(stamp)
        self._publish_odom(stamp)
        self._publish_camera(stamp)

    def _publish_tf(self, stamp: Any) -> None:
        transforms = []
        transforms.append(
            self._transform(
                stamp,
                self.odom_frame,
                self.base_frame,
                self.state.x,
                self.state.y,
                self.state.z,
                self.state.yaw,
            )
        )
        transforms.append(
            self._transform(
                stamp,
                self.base_frame,
                self.oakd_imu_frame,
                float(self.oakd_pose.get("x", 0.18)),
                float(self.oakd_pose.get("y", 0.0)),
                float(self.oakd_pose.get("z", 0.16)),
                float(self.oakd_pose.get("yaw", 0.0)),
            )
        )
        transforms.append(
            self._transform(stamp, self.oakd_imu_frame, self.oakd_optical_frame, 0.0, 0.0, 0.0, 0.0)
        )
        transforms.append(
            self._transform(stamp, self.oakd_optical_frame, self.left_frame, 0.0, self.baseline * 0.5, 0.0, 0.0)
        )
        transforms.append(
            self._transform(stamp, self.oakd_optical_frame, self.right_frame, 0.0, -self.baseline * 0.5, 0.0, 0.0)
        )
        self.tf_broadcaster.sendTransform(transforms)

    def _transform(
        self, stamp: Any, parent: str, child: str, x: float, y: float, z: float, yaw: float
    ) -> Any:
        msg = self.TransformStamped()
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
        msg = self.Odometry()
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

    def _publish_camera(self, stamp: Any) -> None:
        left = self._image(stamp, self.left_frame, "rgb8", self.width * 3)
        right = self._image(stamp, self.right_frame, "rgb8", self.width * 3)
        depth = self._image(stamp, self.oakd_optical_frame, "32FC1", self.depth_width * 4)
        left.data = self._rgb_data
        right.data = self._rgb_data
        depth.data = self._depth_data

        left_info = self._camera_info(stamp, self.left_frame)
        right_info = self._camera_info(stamp, self.right_frame)
        depth_info = self._camera_info(stamp, self.oakd_optical_frame)

        self.left_image_pub.publish(left)
        self.right_image_pub.publish(right)
        self.depth_image_pub.publish(depth)
        self.left_info_pub.publish(left_info)
        self.right_info_pub.publish(right_info)
        self.depth_info_pub.publish(depth_info)

    def _image(self, stamp: Any, frame_id: str, encoding: str, step: int) -> Any:
        msg = self.Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = self.height
        msg.width = self.width
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = step
        msg.data = bytearray(step * self.height)
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

    def _camera_info(self, stamp: Any, frame_id: str) -> Any:
        msg = self.CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = self.height
        msg.width = self.width
        fx = fy = 420.0
        cx = self.width / 2.0
        cy = self.height / 2.0
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        return msg

    def shutdown(self) -> None:
        self.node.destroy_node()
        self.rclpy.shutdown()


class NullRosInterface:
    def __init__(self) -> None:
        self.cmd_vel = type(
            "TwistLike",
            (),
            {
                "linear": type("Linear", (), {"x": 0.0, "y": 0.0})(),
                "angular": type("Angular", (), {"z": 0.0})(),
            },
        )()

    def spin_once(self) -> None:
        return

    def publish(self, sim_time: float) -> None:
        return

    def shutdown(self) -> None:
        return


class RenderFrameServer:
    _HEADER = struct.Struct("<4sdIIIII")

    def __init__(self, config: Dict[str, Any]) -> None:
        self.enabled = str(_deep_get(config, "oakd.render_mode", "pattern")).lower() == "isaac"
        self.host = str(_deep_get(config, "ros.frame_server_host", "127.0.0.1"))
        self.port = int(_deep_get(config, "ros.frame_server_port", 47650))
        self.server: socket.socket | None = None
        self.client: socket.socket | None = None
        self.buffer = bytearray()
        self.latest_pose: tuple[float, float, float, float, float] | None = None
        if self.enabled:
            self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server.bind((self.host, self.port))
            self.server.listen(1)
            self.server.setblocking(False)
            print(f"[isaac_sim_nav] RGB-D frame server listening on {self.host}:{self.port}")

    def poll_pose(self) -> tuple[float, float, float, float, float] | None:
        if not self.enabled:
            return None
        self._accept()
        if self.client is None:
            return self.latest_pose
        try:
            chunk = self.client.recv(4096)
            if chunk:
                self.buffer.extend(chunk)
            elif chunk == b"":
                self._drop_client()
        except BlockingIOError:
            pass
        except OSError:
            self._drop_client()

        while b"\n" in self.buffer:
            raw_line, _, remainder = self.buffer.partition(b"\n")
            self.buffer = bytearray(remainder)
            parts = raw_line.decode("ascii", errors="ignore").split()
            if len(parts) == 6 and parts[0] == "POSE":
                try:
                    self.latest_pose = tuple(float(value) for value in parts[1:])  # type: ignore[assignment]
                except ValueError:
                    continue
        return self.latest_pose

    def publish_frame(
        self,
        sim_time: float,
        width: int,
        height: int,
        left_rgb: bytes | bytearray,
        right_rgb: bytes | bytearray,
        depth_32fc1: bytes | bytearray,
    ) -> None:
        if self.client is None:
            return
        payload = bytes(left_rgb) + bytes(right_rgb) + bytes(depth_32fc1)
        header = self._HEADER.pack(
            b"IRGB",
            float(sim_time),
            int(width),
            int(height),
            len(left_rgb),
            len(right_rgb),
            len(depth_32fc1),
        )
        try:
            self.client.sendall(header + payload)
        except OSError:
            self._drop_client()

    def close(self) -> None:
        self._drop_client()
        if self.server is not None:
            self.server.close()
            self.server = None

    def _accept(self) -> None:
        if self.server is None or self.client is not None:
            return
        try:
            self.client, _address = self.server.accept()
            self.client.setblocking(False)
            self.buffer.clear()
            print("[isaac_sim_nav] ROS bridge connected to RGB-D frame server")
        except BlockingIOError:
            return

    def _drop_client(self) -> None:
        if self.client is not None:
            try:
                self.client.close()
            except OSError:
                pass
        self.client = None
        self.buffer.clear()


class IsaacRgbdCameras:
    def __init__(self, config: Dict[str, Any]) -> None:
        self.enabled = str(_deep_get(config, "oakd.render_mode", "pattern")).lower() == "isaac"
        self.width = int(_deep_get(config, "oakd.resolution.width", 320))
        self.height = int(_deep_get(config, "oakd.resolution.height", 240))
        self.baseline = float(_deep_get(config, "oakd.stereo_baseline_m", 0.075))
        self.oakd_pose = _deep_get(config, "oakd.pose", {})
        self.left = None
        self.right = None
        self._np = None
        self._fallback_rgb = self._build_rgb_pattern()
        self._fallback_depth = self._build_depth_pattern()
        if self.enabled:
            self._create_cameras(config)

    def update_pose(self, state: RobotState) -> None:
        if not self.enabled or self.left is None or self.right is None:
            return
        oakd_x = float(self.oakd_pose.get("x", 0.18))
        oakd_y = float(self.oakd_pose.get("y", 0.0))
        x = state.x + oakd_x * math.cos(state.yaw) - oakd_y * math.sin(state.yaw)
        y = state.y + oakd_x * math.sin(state.yaw) + oakd_y * math.cos(state.yaw)
        z = state.z + float(self.oakd_pose.get("z", 0.16))
        yaw = state.yaw + float(self.oakd_pose.get("yaw", 0.0))
        orientation = self._camera_orientation_wxyz(yaw)
        lateral_x = -math.sin(yaw)
        lateral_y = math.cos(yaw)
        left_position = [x + lateral_x * self.baseline * 0.5, y + lateral_y * self.baseline * 0.5, z]
        right_position = [x - lateral_x * self.baseline * 0.5, y - lateral_y * self.baseline * 0.5, z]
        for camera, position in ((self.left, left_position), (self.right, right_position)):
            if hasattr(camera, "set_world_pose"):
                camera.set_world_pose(position=position, orientation=orientation)

    def capture(self) -> tuple[int, int, bytearray, bytearray, bytearray]:
        left_rgb = self._camera_rgb(self.left) or self._fallback_rgb
        right_rgb = self._camera_rgb(self.right) or left_rgb
        depth = self._camera_depth(self.left) or self._fallback_depth
        return self.width, self.height, left_rgb, right_rgb, depth

    def _create_cameras(self, config: Dict[str, Any]) -> None:
        try:
            try:
                from isaacsim.sensors.camera import Camera  # type: ignore
            except ImportError:
                from omni.isaac.sensor import Camera  # type: ignore

            import numpy as np  # type: ignore

            self._np = np
            frequency = float(_deep_get(config, "oakd.frequency_hz", 30.0))
            self.left = Camera(
                prim_path=f"{_deep_get(config, 'oakd.prim_path', '/World/GroundRobot/OAKD')}/LeftCamera",
                resolution=(self.width, self.height),
                frequency=frequency,
            )
            self.right = Camera(
                prim_path=f"{_deep_get(config, 'oakd.prim_path', '/World/GroundRobot/OAKD')}/RightCamera",
                resolution=(self.width, self.height),
                frequency=frequency,
            )
            for camera in (self.left, self.right):
                if hasattr(camera, "initialize"):
                    camera.initialize()
                if hasattr(camera, "set_focal_length"):
                    camera.set_focal_length(float(_deep_get(config, "oakd.camera_focal_length_mm", 2.4)))
                if hasattr(camera, "set_horizontal_aperture"):
                    camera.set_horizontal_aperture(
                        float(_deep_get(config, "oakd.camera_horizontal_aperture_mm", 3.6))
                    )
                if hasattr(camera, "set_clipping_range"):
                    camera.set_clipping_range(
                        float(_deep_get(config, "oakd.near_clip_m", 0.05)),
                        float(_deep_get(config, "oakd.far_clip_m", 8.0)),
                    )
                if hasattr(camera, "add_distance_to_image_plane_to_frame"):
                    camera.add_distance_to_image_plane_to_frame()
            print("[isaac_sim_nav] Isaac RGB-D cameras created")
        except Exception as exc:
            self.enabled = False
            self.left = None
            self.right = None
            print(f"[isaac_sim_nav] Isaac camera API unavailable, using pattern frames: {exc}")

    def _camera_rgb(self, camera: Any) -> bytearray | None:
        if camera is None:
            return None
        try:
            rgb = camera.get_rgb() if hasattr(camera, "get_rgb") else None
            if rgb is None and hasattr(camera, "get_current_frame"):
                frame = camera.get_current_frame()
                rgb = frame.get("rgba")
                if rgb is None:
                    rgb = frame.get("rgb")
            return self._rgb_to_bytes(rgb)
        except Exception:
            return None

    def _camera_depth(self, camera: Any) -> bytearray | None:
        if camera is None:
            return None
        try:
            depth = camera.get_depth() if hasattr(camera, "get_depth") else None
            if depth is None and hasattr(camera, "get_current_frame"):
                frame = camera.get_current_frame()
                depth = frame.get("distance_to_image_plane")
                if depth is None:
                    depth = frame.get("depth")
            return self._depth_to_bytes(depth)
        except Exception:
            return None

    def _rgb_to_bytes(self, rgb: Any) -> bytearray | None:
        if rgb is None or self._np is None:
            return None
        array = self._np.asarray(rgb)
        if array.size == 0:
            return None
        array = array.reshape((self.height, self.width, -1))
        if array.shape[2] >= 3:
            array = array[:, :, :3]
        if array.dtype.kind == "f":
            array = self._np.clip(array * 255.0, 0, 255).astype(self._np.uint8)
        else:
            array = array.astype(self._np.uint8, copy=False)
        return bytearray(array.tobytes())

    def _depth_to_bytes(self, depth: Any) -> bytearray | None:
        if depth is None or self._np is None:
            return None
        array = self._np.asarray(depth)
        if array.size == 0:
            return None
        array = array.reshape((self.height, self.width)).astype("<f4", copy=False)
        return bytearray(array.tobytes())

    def _camera_orientation_wxyz(self, yaw: float) -> list[float]:
        forward = (math.cos(yaw), math.sin(yaw), 0.0)
        up = (0.0, 0.0, 1.0)
        right = (
            forward[1] * up[2] - forward[2] * up[1],
            forward[2] * up[0] - forward[0] * up[2],
            forward[0] * up[1] - forward[1] * up[0],
        )
        # USD cameras look along local -Z. Columns are local X, local Y, local Z.
        matrix = (
            (right[0], up[0], -forward[0]),
            (right[1], up[1], -forward[1]),
            (right[2], up[2], -forward[2]),
        )
        trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
        if trace > 0.0:
            scale = math.sqrt(trace + 1.0) * 2.0
            qw = 0.25 * scale
            qx = (matrix[2][1] - matrix[1][2]) / scale
            qy = (matrix[0][2] - matrix[2][0]) / scale
            qz = (matrix[1][0] - matrix[0][1]) / scale
        elif matrix[0][0] > matrix[1][1] and matrix[0][0] > matrix[2][2]:
            scale = math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0
            qw = (matrix[2][1] - matrix[1][2]) / scale
            qx = 0.25 * scale
            qy = (matrix[0][1] + matrix[1][0]) / scale
            qz = (matrix[0][2] + matrix[2][0]) / scale
        elif matrix[1][1] > matrix[2][2]:
            scale = math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0
            qw = (matrix[0][2] - matrix[2][0]) / scale
            qx = (matrix[0][1] + matrix[1][0]) / scale
            qy = 0.25 * scale
            qz = (matrix[1][2] + matrix[2][1]) / scale
        else:
            scale = math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0
            qw = (matrix[1][0] - matrix[0][1]) / scale
            qx = (matrix[0][2] + matrix[2][0]) / scale
            qy = (matrix[1][2] + matrix[2][1]) / scale
            qz = 0.25 * scale
        return [qw, qx, qy, qz]

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
        for y in range(self.height):
            for x in range(self.width):
                depth_m = 3.5
                if self.width * 0.42 <= x <= self.width * 0.58 and self.height * 0.36 <= y <= self.height * 0.68:
                    depth_m = 1.4
                values.append(depth_m)
        return bytearray(struct.pack(f"{len(values)}f", *values))


def _start_simulation_app(config: Dict[str, Any]):
    try:
        from isaacsim import SimulationApp  # type: ignore
    except ImportError:
        from omni.isaac.kit import SimulationApp  # type: ignore

    launch_config = {
        "headless": bool(_deep_get(config, "isaac_sim.headless", True)),
        "hide_ui": True,
        "renderer": str(_deep_get(config, "isaac_sim.renderer", "RaytracedLighting")),
        "anti_aliasing": int(_deep_get(config, "isaac_sim.anti_aliasing", 0)),
        "samples_per_pixel_per_frame": int(
            _deep_get(config, "isaac_sim.samples_per_pixel_per_frame", 1)
        ),
        "multi_gpu": bool(_deep_get(config, "isaac_sim.multi_gpu", False)),
        "max_gpu_count": int(_deep_get(config, "isaac_sim.max_gpu_count", 1)),
        "disable_viewport_updates": bool(
            _deep_get(config, "isaac_sim.disable_viewport_updates", True)
        ),
        "enable_crashreporter": bool(_deep_get(config, "isaac_sim.enable_crashreporter", False)),
    }
    return SimulationApp(launch_config)


def _build_stage(config: Dict[str, Any], state: RobotState) -> None:
    from pxr import Gf, UsdGeom

    import omni.usd

    world_usd_path = _deep_get(config, "scene.world_usd_path", "")
    if world_usd_path:
        world_path = Path(world_usd_path).expanduser()
        if world_path.exists():
            omni.usd.get_context().open_stage(str(world_path))
        else:
            print(f"[isaac_sim_nav] world_usd_path does not exist, using primitive scene: {world_path}")

    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, float(_deep_get(config, "scene.stage_units_in_meters", 1.0)))

    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    ground_size = float(_deep_get(config, "scene.ground_size_m", 12.0))
    ground = UsdGeom.Cube.Define(stage, "/World/Ground")
    ground.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.025))
    ground.AddScaleOp().Set(Gf.Vec3f(ground_size, ground_size, 0.05))

    robot_path = _deep_get(config, "robot.prim_path", "/World/GroundRobot")
    robot_radius = float(_deep_get(config, "robot.dimensions.radius_m", 0.18))
    robot_height = float(_deep_get(config, "robot.dimensions.height_m", 0.16))
    robot = UsdGeom.Xform.Define(stage, robot_path)
    robot.AddTranslateOp().Set(Gf.Vec3d(state.x, state.y, state.z))
    body = UsdGeom.Cylinder.Define(stage, f"{robot_path}/Body")
    body.CreateAxisAttr("Z")
    body.CreateRadiusAttr(robot_radius)
    body.CreateHeightAttr(robot_height)
    oakd_path = _deep_get(config, "oakd.prim_path", f"{robot_path}/OAKD")
    UsdGeom.Xform.Define(stage, oakd_path)

    for index in range(int(_deep_get(config, "scene.obstacle_count", 4))):
        obstacle = UsdGeom.Cube.Define(stage, f"/World/Obstacle_{index}")
        x = -2.0 + index * 1.25
        y = 1.6 if index % 2 else -1.4
        obstacle.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.3))
        obstacle.AddScaleOp().Set(Gf.Vec3f(0.35, 0.35, 0.6))

    _import_optional_asset(stage, _deep_get(config, "scene.scene_mesh_stl_path", ""), "/World/ImportedScene")
    _import_optional_asset(stage, _deep_get(config, "scene.robot_mesh_stl_path", ""), f"{robot_path}/ImportedRobotMesh")
    return str(robot_path)


def _update_robot_prim(robot_path: str, state: RobotState) -> None:
    try:
        from pxr import Gf, UsdGeom

        import omni.usd

        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(robot_path)
        if not prim:
            return
        xform = UsdGeom.Xformable(prim)
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(state.x, state.y, state.z))
        xform.AddRotateZOp().Set(math.degrees(state.yaw))
    except Exception:
        return


def _import_optional_asset(stage: Any, asset_path: str, prim_path: str) -> None:
    if not asset_path:
        return
    path = Path(asset_path).expanduser()
    if not path.exists():
        print(f"[isaac_sim_nav] Optional asset does not exist, skipping: {path}")
        return
    if path.suffix.lower() == ".usd":
        from pxr import Sdf

        prim = stage.DefinePrim(prim_path)
        prim.GetReferences().AddReference(str(path))
    else:
        print(
            "[isaac_sim_nav] STL/mesh path is reserved for local import workflows. "
            f"Convert to USD or import manually if this Isaac Sim build cannot load it directly: {path}"
        )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()

    config = _load_yaml(args.config)
    if args.headless:
        config.setdefault("isaac_sim", {})["headless"] = True

    os.environ.setdefault("ROS_DOMAIN_ID", str(_deep_get(config, "ros.domain_id", 0)))

    state = RobotState(
        x=float(_deep_get(config, "robot.initial_pose.x", 0.0)),
        y=float(_deep_get(config, "robot.initial_pose.y", 0.0)),
        z=float(_deep_get(
            config,
            "robot.initial_pose.z",
            float(_deep_get(config, "robot.dimensions.height_m", 0.16)) / 2.0,
        )),
        yaw=float(_deep_get(config, "robot.initial_pose.yaw", 0.0)),
    )

    app = _start_simulation_app(config)
    frame_server = RenderFrameServer(config)
    if bool(_deep_get(config, "ros.use_internal_python_bridge", False)):
        ros = RosInterface(config, state)
    else:
        ros = NullRosInterface()
    robot_path = _build_stage(config, state)
    app.update()
    cameras = IsaacRgbdCameras(config)
    cameras.update_pose(state)
    app.update()

    max_v = float(_deep_get(config, "robot.max_linear_speed_mps", 0.6))
    max_w = float(_deep_get(config, "robot.max_angular_speed_radps", 1.0))
    dt = 1.0 / float(_deep_get(config, "oakd.frequency_hz", 30.0))
    sim_time = 0.0

    try:
        while app.is_running():
            start = time.monotonic()
            ros.spin_once()
            bridge_pose = frame_server.poll_pose()
            if bridge_pose is not None:
                sim_time, state.x, state.y, state.z, state.yaw = bridge_pose
            else:
                cmd = ros.cmd_vel
                state.vx = _clamp(float(cmd.linear.x), -max_v, max_v)
                state.vy = _clamp(float(cmd.linear.y), -max_v, max_v)
                state.wz = _clamp(float(cmd.angular.z), -max_w, max_w)

                cos_yaw = math.cos(state.yaw)
                sin_yaw = math.sin(state.yaw)
                state.x += (state.vx * cos_yaw - state.vy * sin_yaw) * dt
                state.y += (state.vx * sin_yaw + state.vy * cos_yaw) * dt
                state.yaw += state.wz * dt
                sim_time += dt

            ros.publish(sim_time)
            _update_robot_prim(robot_path, state)
            cameras.update_pose(state)
            app.update()
            width, height, left_rgb, right_rgb, depth = cameras.capture()
            frame_server.publish_frame(sim_time, width, height, left_rgb, right_rgb, depth)
            elapsed = time.monotonic() - start
            if elapsed < dt:
                time.sleep(dt - elapsed)
    finally:
        ros.shutdown()
        frame_server.close()
        app.close()


if __name__ == "__main__":
    main()

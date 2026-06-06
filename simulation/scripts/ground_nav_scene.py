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
                float(self.oakd_pose.get("x", 0.12)),
                float(self.oakd_pose.get("y", 0.0)),
                float(self.oakd_pose.get("z", 0.28)),
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
    robot_height = float(_deep_get(config, "robot.dimensions.height_m", 0.20))
    robot = UsdGeom.Cube.Define(stage, robot_path)
    robot.CreateSizeAttr(1.0)
    robot.AddTranslateOp().Set(Gf.Vec3d(state.x, state.y, state.z))
    robot.AddScaleOp().Set(Gf.Vec3f(
        float(_deep_get(config, "robot.dimensions.length_m", 0.55)),
        float(_deep_get(config, "robot.dimensions.width_m", 0.45)),
        robot_height,
    ))

    for index in range(int(_deep_get(config, "scene.obstacle_count", 4))):
        obstacle = UsdGeom.Cube.Define(stage, f"/World/Obstacle_{index}")
        x = -2.0 + index * 1.25
        y = 1.6 if index % 2 else -1.4
        obstacle.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.3))
        obstacle.AddScaleOp().Set(Gf.Vec3f(0.35, 0.35, 0.6))

    _import_optional_asset(stage, _deep_get(config, "scene.scene_mesh_stl_path", ""), "/World/ImportedScene")
    _import_optional_asset(stage, _deep_get(config, "scene.robot_mesh_stl_path", ""), f"{robot_path}/ImportedRobotMesh")


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
            float(_deep_get(config, "robot.dimensions.height_m", 0.20)) / 2.0,
        )),
        yaw=float(_deep_get(config, "robot.initial_pose.yaw", 0.0)),
    )

    app = _start_simulation_app(config)
    if bool(_deep_get(config, "ros.use_internal_python_bridge", False)):
        ros = RosInterface(config, state)
    else:
        ros = NullRosInterface()
    _build_stage(config, state)

    max_v = float(_deep_get(config, "robot.max_linear_speed_mps", 0.6))
    max_w = float(_deep_get(config, "robot.max_angular_speed_radps", 1.0))
    dt = 1.0 / float(_deep_get(config, "oakd.frequency_hz", 30.0))
    sim_time = 0.0

    try:
        while app.is_running():
            start = time.monotonic()
            ros.spin_once()
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
            app.update()
            elapsed = time.monotonic() - start
            if elapsed < dt:
                time.sleep(dt - elapsed)
    finally:
        ros.shutdown()
        app.close()


if __name__ == "__main__":
    main()

import json
import math
import threading
import time
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, Int8, String
from tf2_ros import Buffer, TransformException, TransformListener

from .platform_state_machine import (
    PlatformInputs,
    PlatformState,
    PlatformStateMachine,
)
from .serial_protocol import BaseCommand, BaseStatus, FrameParser, clamp, encode_command

try:
    import serial
except Exception:
    serial = None


class GroundSerialBridge(Node):
    def __init__(self):
        super().__init__("ground_serial_bridge")

        self.declare_parameter("planner_cmd_topic", "/nav/cmd_vel")
        self.declare_parameter("planner_cmd_unstamped_topic", "/cmd_vel")
        self.declare_parameter("planner_emergency_topic", "/nav/emergency")
        self.declare_parameter("planner_safety_topic", "/nav/safety_status")
        self.declare_parameter("command_output_frame", "base_link")
        self.declare_parameter("transform_cmd_to_output_frame", True)
        self.declare_parameter("transform_timeout_sec", 0.05)
        self.declare_parameter("mcu_velocity_frame", "base_link")
        self.declare_parameter("mcu_yaw_sign", 1.0)
        self.declare_parameter("mcu_yaw_offset_rad", 0.0)
        self.declare_parameter("mcu_yaw_auto_zero", False)
        self.declare_parameter("mcu_yaw_reference_frame", "odom")
        self.declare_parameter("mcu_yaw_compensation_mode", "yaw_error")
        self.declare_parameter("require_mcu_yaw_for_mcu_world", True)
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("serial_timeout_sec", 0.02)
        self.declare_parameter("reconnect_interval_sec", 1.0)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("cmd_timeout_sec", 0.5)
        self.declare_parameter("require_feedback", False)
        self.declare_parameter("min_feedback_rate_hz", 10.0)
        self.declare_parameter("feedback_timeout_sec", 0.3)
        self.declare_parameter("max_vel_x", 0.6)
        self.declare_parameter("max_vel_y", 0.6)
        self.declare_parameter("max_yaw_rate", 1.0)
        self.declare_parameter("max_acc_x", 0.4)
        self.declare_parameter("max_acc_y", 0.4)
        self.declare_parameter("max_yaw_accel", 1.2)

        self.cmd_sub = self.create_subscription(
            TwistStamped,
            self.get_parameter("planner_cmd_topic").value,
            self.cmd_cb,
            10,
        )
        self.cmd_unstamped_sub = self.create_subscription(
            Twist,
            self.get_parameter("planner_cmd_unstamped_topic").value,
            self.cmd_unstamped_cb,
            10,
        )
        self.emergency_sub = self.create_subscription(
            Bool,
            self.get_parameter("planner_emergency_topic").value,
            self.emergency_cb,
            10,
        )
        self.safety_sub = self.create_subscription(
            Int8,
            self.get_parameter("planner_safety_topic").value,
            self.safety_cb,
            10,
        )

        self.state_pub = self.create_publisher(String, "/base/state", 10)
        self.status_pub = self.create_publisher(String, "/base/status", 10)
        self.diagnostics_pub = self.create_publisher(String, "/base/diagnostics", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.lock = threading.Lock()
        self.serial_handle = None
        self.parser = FrameParser()
        self.stop_event = threading.Event()

        self.latest_cmd: Optional[Tuple[float, float, float]] = None
        self.latest_cmd_frame = "base_link"
        self.last_cmd_time = self.get_clock().now()
        self.last_sent_cmd = (0.0, 0.0, 0.0)
        self.last_tf_warn_time = 0.0
        self.emergency_active = False
        self.safety_level = 0
        self.last_status: Optional[BaseStatus] = None
        self.last_feedback_monotonic: Optional[float] = None
        self.feedback_count = 0
        self.feedback_rate_hz = 0.0
        self.feedback_rate_window_valid = False
        self.feedback_window_start = time.monotonic()
        self.last_connect_attempt = 0.0
        self.mcu_yaw_zero: Optional[float] = None

        self.state_machine = PlatformStateMachine()
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()

        rate = float(self.get_parameter("control_rate_hz").value)
        self.control_timer = self.create_timer(max(0.01, 1.0 / rate), self.control_loop)
        self.status_timer = self.create_timer(1.0, self.publish_diagnostics)

        self.get_logger().info("ground_serial_bridge started")

    def cmd_cb(self, msg: TwistStamped):
        vx = float(msg.twist.linear.x)
        vy = float(msg.twist.linear.y)
        wz = float(msg.twist.angular.z)
        frame_id = msg.header.frame_id or str(self.get_parameter("command_output_frame").value)
        with self.lock:
            self.latest_cmd = (vx, vy, wz)
            self.latest_cmd_frame = frame_id
            self.last_cmd_time = self.get_clock().now()

    def cmd_unstamped_cb(self, msg: Twist):
        vx = float(msg.linear.x)
        vy = float(msg.linear.y)
        wz = float(msg.angular.z)
        frame_id = str(self.get_parameter("command_output_frame").value)
        with self.lock:
            self.latest_cmd = (vx, vy, wz)
            self.latest_cmd_frame = frame_id
            self.last_cmd_time = self.get_clock().now()

    def emergency_cb(self, msg: Bool):
        with self.lock:
            self.emergency_active = bool(msg.data)

    def safety_cb(self, msg: Int8):
        with self.lock:
            self.safety_level = int(msg.data)
            if self.safety_level >= 2:
                self.emergency_active = True

    def connect_serial_if_needed(self):
        if self.serial_handle is not None:
            return
        if serial is None:
            return

        now = time.monotonic()
        reconnect_interval = float(self.get_parameter("reconnect_interval_sec").value)
        if now - self.last_connect_attempt < reconnect_interval:
            return
        self.last_connect_attempt = now

        port = str(self.get_parameter("serial_port").value)
        baudrate = int(self.get_parameter("baudrate").value)
        timeout = float(self.get_parameter("serial_timeout_sec").value)
        try:
            self.serial_handle = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
            self.get_logger().info(f"Serial connected: {port} @ {baudrate}")
        except Exception as exc:
            self.serial_handle = None
            self.get_logger().warn(f"Serial connect failed: {exc}")

    def close_serial(self):
        handle = self.serial_handle
        self.serial_handle = None
        if handle is not None:
            try:
                handle.close()
            except Exception:
                pass

    def rx_loop(self):
        while not self.stop_event.is_set():
            self.connect_serial_if_needed()
            handle = self.serial_handle
            if handle is None:
                time.sleep(0.05)
                continue

            try:
                data = handle.read(64)
            except Exception as exc:
                self.get_logger().warn(f"Serial read failed: {exc}")
                self.close_serial()
                time.sleep(0.05)
                continue

            if not data:
                continue

            try:
                statuses = self.parser.feed(data)
            except Exception as exc:
                self.get_logger().warn(f"Serial frame parse failed: {exc}")
                continue

            for status in statuses:
                with self.lock:
                    self.last_status = status
                    self.last_feedback_monotonic = time.monotonic()
                    self.feedback_count += 1

    def serial_connected(self) -> bool:
        handle = self.serial_handle
        return bool(handle is not None and getattr(handle, "is_open", False))

    def command_timed_out(self) -> bool:
        cmd_timeout = float(self.get_parameter("cmd_timeout_sec").value)
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        return elapsed > cmd_timeout

    def feedback_timed_out(self) -> bool:
        if not bool(self.get_parameter("require_feedback").value):
            return False
        last_feedback = self.last_feedback_monotonic
        if last_feedback is None:
            return True
        timeout = float(self.get_parameter("feedback_timeout_sec").value)
        if (time.monotonic() - last_feedback) > timeout:
            return True
        min_feedback_rate = float(self.get_parameter("min_feedback_rate_hz").value)
        if self.feedback_rate_window_valid and self.feedback_rate_hz < min_feedback_rate:
            return True
        return False

    def limited_command(self, desired: Tuple[float, float, float]) -> Tuple[float, float, float]:
        rate = float(self.get_parameter("control_rate_hz").value)
        dt = 1.0 / max(rate, 1.0)
        max_vx = float(self.get_parameter("max_vel_x").value)
        max_vy = float(self.get_parameter("max_vel_y").value)
        max_wz = float(self.get_parameter("max_yaw_rate").value)
        max_ax = float(self.get_parameter("max_acc_x").value)
        max_ay = float(self.get_parameter("max_acc_y").value)
        max_awz = float(self.get_parameter("max_yaw_accel").value)

        vx = clamp(desired[0], -max_vx, max_vx)
        vy = clamp(desired[1], -max_vy, max_vy)
        wz = clamp(desired[2], -max_wz, max_wz)

        last_vx, last_vy, last_wz = self.last_sent_cmd
        vx = clamp(vx, last_vx - max_ax * dt, last_vx + max_ax * dt)
        vy = clamp(vy, last_vy - max_ay * dt, last_vy + max_ay * dt)
        wz = clamp(wz, last_wz - max_awz * dt, last_wz + max_awz * dt)
        return (vx, vy, wz)

    def transform_command_to_frame(
        self,
        command: Tuple[float, float, float],
        source_frame: str,
        target_frame: str,
    ) -> Optional[Tuple[float, float, float]]:
        if not source_frame or source_frame == target_frame:
            return command

        timeout_sec = float(self.get_parameter("transform_timeout_sec").value)
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=rclpy.duration.Duration(seconds=timeout_sec),
            )
        except TransformException as exc:
            now = time.monotonic()
            if now - self.last_tf_warn_time > 1.0:
                self.get_logger().warn(
                    f"Cannot transform cmd_vel {source_frame}->{target_frame}: {exc}"
                )
                self.last_tf_warn_time = now
            return None

        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        vx, vy, wz = command
        return (
            cos_yaw * vx - sin_yaw * vy,
            sin_yaw * vx + cos_yaw * vy,
            wz,
        )

    def transform_command_to_output_frame(
        self,
        command: Tuple[float, float, float],
        source_frame: str,
    ) -> Optional[Tuple[float, float, float]]:
        if not bool(self.get_parameter("transform_cmd_to_output_frame").value):
            return command
        return self.transform_command_to_frame(
            command,
            source_frame,
            str(self.get_parameter("command_output_frame").value),
        )

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def mcu_yaw_for_compensation(self, status: Optional[BaseStatus]) -> Optional[float]:
        if status is None or status.yaw_rad is None:
            return None

        raw_yaw = float(status.yaw_rad) * float(self.get_parameter("mcu_yaw_sign").value)
        if bool(self.get_parameter("mcu_yaw_auto_zero").value):
            if self.mcu_yaw_zero is None:
                self.mcu_yaw_zero = raw_yaw
            raw_yaw -= self.mcu_yaw_zero

        raw_yaw += float(self.get_parameter("mcu_yaw_offset_rad").value)
        return self.normalize_angle(raw_yaw)

    def get_base_yaw_in_reference_frame(self) -> Optional[float]:
        reference_frame = str(self.get_parameter("mcu_yaw_reference_frame").value)
        base_frame = str(self.get_parameter("command_output_frame").value)
        timeout_sec = float(self.get_parameter("transform_timeout_sec").value)

        try:
            transform = self.tf_buffer.lookup_transform(
                reference_frame,
                base_frame,
                Time(),
                timeout=rclpy.duration.Duration(seconds=timeout_sec),
            )
        except TransformException as exc:
            now = time.monotonic()
            if now - self.last_tf_warn_time > 1.0:
                self.get_logger().warn(
                    f"Cannot get base yaw {reference_frame}->{base_frame}: {exc}"
                )
                self.last_tf_warn_time = now
            return None

        q = transform.transform.rotation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def mcu_yaw_compensation_angle(self, status: Optional[BaseStatus]) -> Optional[float]:
        yaw_mcu = self.mcu_yaw_for_compensation(status)
        if yaw_mcu is None:
            return None

        mode = str(self.get_parameter("mcu_yaw_compensation_mode").value).lower()
        if mode in ("mcu_yaw", "absolute"):
            return yaw_mcu
        if mode not in ("yaw_error", "error"):
            self.get_logger().warn(
                f'Unknown mcu_yaw_compensation_mode "{mode}", using yaw_error'
            )

        yaw_base = self.get_base_yaw_in_reference_frame()
        if yaw_base is None:
            return None

        return self.normalize_angle(yaw_mcu - yaw_base)

    def transform_base_command_to_mcu_frame(
        self,
        command: Tuple[float, float, float],
        status: Optional[BaseStatus],
    ) -> Optional[Tuple[float, float, float]]:
        frame = str(self.get_parameter("mcu_velocity_frame").value).lower()
        if frame in ("base_link", "base", "body"):
            return command
        if frame not in ("mcu_world", "mcu", "lower_world"):
            self.get_logger().warn(f'Unknown mcu_velocity_frame "{frame}", using base_link')
            return command

        yaw = self.mcu_yaw_compensation_angle(status)
        if yaw is None:
            if bool(self.get_parameter("require_mcu_yaw_for_mcu_world").value):
                return None
            return command

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        vx, vy, wz = command
        return (
            cos_yaw * vx - sin_yaw * vy,
            sin_yaw * vx + cos_yaw * vy,
            wz,
        )

    def transform_command_to_mcu_frame(
        self,
        command: Tuple[float, float, float],
        source_frame: str,
        status: Optional[BaseStatus],
    ) -> Optional[Tuple[float, float, float]]:
        frame = str(self.get_parameter("mcu_velocity_frame").value).lower()
        if frame in ("base_link", "base", "body"):
            return self.transform_command_to_output_frame(command, source_frame)
        if frame not in ("mcu_world", "mcu", "lower_world"):
            self.get_logger().warn(f'Unknown mcu_velocity_frame "{frame}", using base_link')
            return self.transform_command_to_output_frame(command, source_frame)

        mode = str(self.get_parameter("mcu_yaw_compensation_mode").value).lower()
        if mode in ("mcu_yaw", "absolute"):
            base_command = self.transform_command_to_output_frame(command, source_frame)
            if base_command is None:
                return None
            return self.transform_base_command_to_mcu_frame(base_command, status)

        reference_frame = str(self.get_parameter("mcu_yaw_reference_frame").value)
        reference_command = self.transform_command_to_frame(
            command,
            source_frame,
            reference_frame,
        )
        if reference_command is None:
            return None

        yaw = self.mcu_yaw_compensation_angle(status)
        if yaw is None:
            if bool(self.get_parameter("require_mcu_yaw_for_mcu_world").value):
                return None
            return reference_command

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        vx, vy, wz = reference_command
        return (
            cos_yaw * vx - sin_yaw * vy,
            sin_yaw * vx + cos_yaw * vy,
            wz,
        )

    def control_loop(self):
        with self.lock:
            latest_cmd = self.latest_cmd
            latest_cmd_frame = self.latest_cmd_frame
            emergency_active = self.emergency_active
            safety_level = self.safety_level
            last_status = self.last_status

        cmd_timeout = self.command_timed_out()
        feedback_timeout = self.feedback_timed_out()
        fault_code = int(last_status.fault_code) if last_status is not None else 0
        base_enabled = bool(last_status.enabled) if last_status is not None else True

        inputs = PlatformInputs(
            nav_cmd_active=latest_cmd is not None and not cmd_timeout,
            emergency_active=emergency_active or safety_level >= 2,
            serial_connected=self.serial_connected(),
            base_enabled=base_enabled,
            cmd_timeout=cmd_timeout,
            feedback_timeout=feedback_timeout,
            fault_code=fault_code,
        )
        state = self.state_machine.update(inputs)

        if self.state_machine.should_send_motion and latest_cmd is not None:
            mcu_cmd = self.transform_command_to_mcu_frame(
                latest_cmd,
                latest_cmd_frame,
                last_status,
            )
            if mcu_cmd is None:
                vx, vy, wz = self.limited_command((0.0, 0.0, 0.0))
            else:
                vx, vy, wz = self.limited_command(mcu_cmd)
        else:
            vx, vy, wz = self.limited_command((0.0, 0.0, 0.0))

        command = BaseCommand(
            vx_mps=vx,
            vy_mps=vy,
            wz_radps=wz,
            enable=self.state_machine.should_enable_base,
            estop=self.state_machine.should_estop,
        )
        self.write_command(command)
        self.last_sent_cmd = (vx, vy, wz)
        self.publish_state(state, command, inputs)

    def write_command(self, command: BaseCommand):
        handle = self.serial_handle
        if handle is None:
            return
        try:
            handle.write(encode_command(command))
        except Exception as exc:
            self.get_logger().warn(f"Serial write failed: {exc}")
            self.close_serial()

    def publish_state(
        self,
        state: PlatformState,
        command: BaseCommand,
        inputs: PlatformInputs,
    ):
        self.state_pub.publish(String(data=state.name))
        payload = {
            "state": state.name,
            "vx_mps": command.vx_mps,
            "vy_mps": command.vy_mps,
            "wz_radps": command.wz_radps,
            "enable": command.enable,
            "estop": command.estop,
            "serial_connected": inputs.serial_connected,
            "cmd_timeout": inputs.cmd_timeout,
            "feedback_timeout": inputs.feedback_timeout,
            "fault_code": inputs.fault_code,
            "mcu_velocity_frame": str(self.get_parameter("mcu_velocity_frame").value),
            "mcu_yaw_compensation_mode": str(
                self.get_parameter("mcu_yaw_compensation_mode").value
            ),
        }
        self.status_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))

    def publish_diagnostics(self):
        now = time.monotonic()
        elapsed = max(1e-6, now - self.feedback_window_start)
        with self.lock:
            self.feedback_rate_hz = self.feedback_count / elapsed
            self.feedback_count = 0
            self.feedback_window_start = now
            self.feedback_rate_window_valid = True
            last_status = self.last_status

        min_feedback_rate = float(self.get_parameter("min_feedback_rate_hz").value)
        payload = {
            "serial_available": serial is not None,
            "serial_connected": self.serial_connected(),
            "require_feedback": bool(self.get_parameter("require_feedback").value),
            "feedback_rate_hz": round(self.feedback_rate_hz, 2),
            "min_feedback_rate_hz": min_feedback_rate,
            "feedback_rate_ok": self.feedback_rate_hz >= min_feedback_rate,
            "mcu_yaw_zero": self.mcu_yaw_zero,
            "last_status": None if last_status is None else last_status.__dict__,
        }
        self.diagnostics_pub.publish(
            String(data=json.dumps(payload, separators=(",", ":")))
        )

    def destroy_node(self):
        self.stop_event.set()
        if self.rx_thread.is_alive():
            self.rx_thread.join(timeout=1.0)
        self.close_serial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GroundSerialBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

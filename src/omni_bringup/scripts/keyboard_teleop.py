#!/usr/bin/env python3
"""GUI keyboard teleoperation for the simulated omni base."""

import argparse
import math
import sys
import tkinter as tk
from typing import Iterable, List, Set, Tuple

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


Velocity = Tuple[float, float, float]

FORWARD_KEYS = frozenset(('w', 'up'))
BACKWARD_KEYS = frozenset(('s', 'down'))
LEFT_KEYS = frozenset(('a', 'left'))
RIGHT_KEYS = frozenset(('d', 'right'))
TURN_LEFT_KEYS = frozenset(('q',))
TURN_RIGHT_KEYS = frozenset(('e',))
MOTION_KEYS = (
    FORWARD_KEYS
    | BACKWARD_KEYS
    | LEFT_KEYS
    | RIGHT_KEYS
    | TURN_LEFT_KEYS
    | TURN_RIGHT_KEYS
)


def normalize_key(keysym: str) -> str:
    """Normalize Tk key symbols for the control state."""
    key = keysym.lower()
    if key.startswith('kp_') and key[3:].isdigit():
        return key[3:]
    return key


def speed_scale_for_digit(key: str) -> float | None:
    """Map 1..9 to 10%..90%, and 0 to 100% speed."""
    if len(key) != 1 or not key.isdigit():
        return None
    return 1.0 if key == '0' else int(key) / 10.0


def velocity_from_pressed(
    pressed: Set[str],
    linear_speed: float,
    lateral_speed: float,
    angular_speed: float,
    speed_scale: float,
) -> Velocity:
    """Combine all held movement keys into one velocity command."""
    forward = int(bool(pressed & FORWARD_KEYS))
    backward = int(bool(pressed & BACKWARD_KEYS))
    left = int(bool(pressed & LEFT_KEYS))
    right = int(bool(pressed & RIGHT_KEYS))
    turn_left = int(bool(pressed & TURN_LEFT_KEYS))
    turn_right = int(bool(pressed & TURN_RIGHT_KEYS))

    x_axis = forward - backward
    y_axis = left - right
    translation_norm = math.hypot(x_axis, y_axis)
    if translation_norm > 0.0:
        x_axis /= translation_norm
        y_axis /= translation_norm

    return (
        x_axis * linear_speed * speed_scale,
        y_axis * lateral_speed * speed_scale,
        (turn_left - turn_right) * angular_speed * speed_scale,
    )


def make_twist(velocity: Velocity) -> Twist:
    """Build a Twist message from (forward, lateral, yaw) velocity."""
    message = Twist()
    message.linear.x, message.linear.y, message.angular.z = velocity
    return message


class KeyboardTeleopNode(Node):
    """Publish keyboard commands as geometry_msgs/Twist."""

    def __init__(self, topic: str) -> None:
        super().__init__('keyboard_teleop')
        self.publisher = self.create_publisher(Twist, topic, 10)
        self.get_logger().info(f'Publishing keyboard commands to {topic}')

    def publish_velocity(self, velocity: Velocity) -> None:
        self.publisher.publish(make_twist(velocity))


class KeyboardTeleopWindow:
    """Track real key press/release events and display controller state."""

    RELEASE_DELAY_MS = 30

    def __init__(self, node: KeyboardTeleopNode, args: argparse.Namespace):
        self.node = node
        self.linear_speed = args.linear
        self.lateral_speed = args.lateral
        self.angular_speed = args.angular
        self.speed_scale = speed_scale_for_digit(str(args.gear)) or 1.0
        self.period_ms = max(1, round(1000.0 / args.rate))
        self.pressed: Set[str] = set()
        self.pending_releases = {}
        self.closed = False

        self.root = tk.Tk()
        self.root.title('RMUC Keyboard Teleop')
        self.root.resizable(False, False)
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.root.bind_all('<KeyPress>', self._on_key_press)
        self.root.bind_all('<KeyRelease>', self._on_key_release)
        self.root.bind('<FocusOut>', self._on_focus_out)
        self.root.bind('<Control-c>', self.close)

        self._build_widgets(args.topic)
        self._update_status()
        self.root.after(self.period_ms, self._publish_tick)
        self.root.after(200, self.root.focus_force)

    def _build_widgets(self, topic: str) -> None:
        frame = tk.Frame(self.root, padx=18, pady=14)
        frame.pack()

        tk.Label(
            frame,
            text='全向底盘键盘控制',
            font=('Sans', 16, 'bold'),
        ).grid(row=0, column=0, columnspan=10, pady=(0, 10))
        tk.Label(
            frame,
            text=(
                'W / S：前后    A / D：左右平移    Q / E：左右旋转\n'
                '支持同时按键；松开全部按键、窗口失焦或关闭时立即停车'
            ),
            justify='center',
        ).grid(row=1, column=0, columnspan=10, pady=(0, 12))

        tk.Label(frame, text='速度档位：').grid(row=2, column=0, sticky='e')
        for column, digit in enumerate('1234567890', start=1):
            button = tk.Button(
                frame,
                text=digit,
                width=2,
                command=lambda value=digit: self._set_gear(value),
                takefocus=False,
            )
            button.grid(row=2, column=column, padx=1)

        self.gear_label = tk.Label(frame, width=20, anchor='w')
        self.gear_label.grid(row=3, column=0, columnspan=5, pady=(12, 0))
        self.keys_label = tk.Label(frame, width=25, anchor='w')
        self.keys_label.grid(row=3, column=5, columnspan=6, pady=(12, 0))
        self.velocity_label = tk.Label(
            frame,
            width=54,
            anchor='center',
            font=('Monospace', 11, 'bold'),
        )
        self.velocity_label.grid(
            row=4, column=0, columnspan=11, pady=(8, 10)
        )
        tk.Button(
            frame,
            text='停止',
            command=self.stop,
            bg='#d9534f',
            fg='white',
            activebackground='#c9302c',
            width=14,
            takefocus=False,
        ).grid(row=5, column=0, columnspan=11, pady=(0, 8))
        tk.Label(
            frame,
            text=f'发布话题：{topic}（点击此窗口后再按控制键）',
            fg='#555555',
        ).grid(row=6, column=0, columnspan=11)

    def current_velocity(self) -> Velocity:
        return velocity_from_pressed(
            self.pressed,
            self.linear_speed,
            self.lateral_speed,
            self.angular_speed,
            self.speed_scale,
        )

    def _set_gear(self, digit: str) -> None:
        scale = speed_scale_for_digit(digit)
        if scale is not None:
            self.speed_scale = scale
            self._publish_now()

    def _on_key_press(self, event) -> None:
        key = normalize_key(event.keysym)
        pending = self.pending_releases.pop(key, None)
        if pending is not None:
            self.root.after_cancel(pending)

        scale = speed_scale_for_digit(key)
        if scale is not None:
            self.speed_scale = scale
        elif key in MOTION_KEYS:
            self.pressed.add(key)
        elif key in ('space', 'x', 'escape'):
            self.stop()
            return
        else:
            return
        self._publish_now()

    def _on_key_release(self, event) -> None:
        key = normalize_key(event.keysym)
        if key not in MOTION_KEYS:
            return
        pending = self.pending_releases.pop(key, None)
        if pending is not None:
            self.root.after_cancel(pending)
        self.pending_releases[key] = self.root.after(
            self.RELEASE_DELAY_MS,
            lambda released_key=key: self._finish_key_release(released_key),
        )

    def _finish_key_release(self, key: str) -> None:
        self.pending_releases.pop(key, None)
        self.pressed.discard(key)
        self._publish_now()

    def _on_focus_out(self, _event) -> None:
        self.stop()

    def _publish_now(self) -> None:
        self.node.publish_velocity(self.current_velocity())
        self._update_status()

    def _publish_tick(self) -> None:
        if self.closed:
            return
        self.node.publish_velocity(self.current_velocity())
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self._update_status()
        self.root.after(self.period_ms, self._publish_tick)

    def _update_status(self) -> None:
        velocity = self.current_velocity()
        gear = 10 if self.speed_scale == 1.0 else round(self.speed_scale * 10)
        held = '+'.join(sorted(self.pressed)).upper() or '无（停车）'
        self.gear_label.config(
            text=f'档位 {gear}/10 ({self.speed_scale:.0%})'
        )
        self.keys_label.config(text=f'按键：{held}')
        self.velocity_label.config(
            text=(
                f'vx={velocity[0]:+.2f} m/s    '
                f'vy={velocity[1]:+.2f} m/s    '
                f'wz={velocity[2]:+.2f} rad/s'
            )
        )

    def stop(self) -> None:
        for callback in self.pending_releases.values():
            self.root.after_cancel(callback)
        self.pending_releases.clear()
        self.pressed.clear()
        self._publish_now()

    def close(self, _event=None) -> None:
        if self.closed:
            return
        self.stop()
        self.closed = True
        self.root.quit()

    def run(self) -> None:
        try:
            self.root.mainloop()
        finally:
            self.stop()
            self.root.destroy()


def parse_args(arguments: Iterable[str]) -> Tuple[argparse.Namespace, List[str]]:
    parser = argparse.ArgumentParser(
        description='Control the Gazebo omni robot from a keyboard window.'
    )
    parser.add_argument('--topic', default='/cmd_vel')
    parser.add_argument('--linear', type=float, default=0.25)
    parser.add_argument('--lateral', type=float, default=0.20)
    parser.add_argument('--angular', type=float, default=0.80)
    parser.add_argument('--gear', type=int, choices=range(0, 10), default=0)
    parser.add_argument('--rate', type=float, default=30.0)
    args, ros_args = parser.parse_known_args(list(arguments))
    if min(args.linear, args.lateral, args.angular) <= 0.0:
        parser.error('speed values must be greater than zero')
    if args.rate <= 0.0:
        parser.error('--rate must be greater than zero')
    return args, ros_args


def main() -> None:
    args, ros_args = parse_args(sys.argv[1:])
    rclpy.init(
        args=ros_args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = KeyboardTeleopNode(args.topic)
    window = None
    try:
        window = KeyboardTeleopWindow(node, args)
        window.run()
    except KeyboardInterrupt:
        # KeyboardTeleopWindow.run() stops and destroys the window in its
        # finally block before this exception reaches the outer scope.
        pass
    finally:
        if rclpy.ok():
            node.publish_velocity((0.0, 0.0, 0.0))
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

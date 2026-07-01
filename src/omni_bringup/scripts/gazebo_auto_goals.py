#!/usr/bin/env python3
"""Cycle Nav2 NavigateToPose goals for Gazebo validation."""

import json
import math
from typing import Dict, List

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


def yaw_to_quaternion(yaw: float):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class GazeboAutoGoals(Node):
    def __init__(self):
        super().__init__("gazebo_auto_goals")
        self.declare_parameter(
            "goals_json",
            json.dumps(
                [
                    {"x": 1.35, "y": -1.15, "yaw": 0.0},
                    {"x": -1.20, "y": 1.20, "yaw": 1.57},
                    {"x": 1.15, "y": 1.25, "yaw": 3.14},
                    {"x": -1.35, "y": -1.05, "yaw": -1.57},
                ]
            ),
        )
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("start_delay_sec", 8.0)
        self.declare_parameter("pause_between_goals_sec", 3.0)
        self.declare_parameter("action_name", "navigate_to_pose")

        self.frame_id = self.get_parameter("frame_id").value
        self.pause_between_goals_sec = float(
            self.get_parameter("pause_between_goals_sec").value
        )
        self.goals = self._load_goals()
        self.goal_index = 0
        self.goal_active = False

        action_name = self.get_parameter("action_name").value
        self.client = ActionClient(self, NavigateToPose, action_name)
        self.get_logger().info(
            f"Waiting for Nav2 action '{action_name}' with {len(self.goals)} goals"
        )

        start_delay = float(self.get_parameter("start_delay_sec").value)
        self.timer = self.create_timer(start_delay, self._try_send_next_goal)

    def _load_goals(self) -> List[Dict[str, float]]:
        raw = self.get_parameter("goals_json").value
        try:
            data = json.loads(raw)
        except json.JSONDecodeError as exc:
            raise RuntimeError(f"Invalid goals_json: {exc}") from exc
        if not isinstance(data, list) or not data:
            raise RuntimeError("goals_json must be a non-empty JSON list")
        goals = []
        for item in data:
            if not isinstance(item, dict):
                raise RuntimeError("Each goal must be a JSON object")
            goals.append(
                {
                    "x": float(item["x"]),
                    "y": float(item["y"]),
                    "yaw": float(item.get("yaw", 0.0)),
                }
            )
        return goals

    def _try_send_next_goal(self):
        if self.goal_active:
            return
        if not self.client.server_is_ready():
            self.get_logger().info("Nav2 action server not ready yet")
            return
        self.timer.cancel()
        self._send_next_goal()

    def _send_next_goal(self):
        goal = self.goals[self.goal_index]
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = goal["x"]
        pose.pose.position.y = goal["y"]
        qx, qy, qz, qw = yaw_to_quaternion(goal["yaw"])
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        request = NavigateToPose.Goal()
        request.pose = pose
        self.goal_active = True
        self.get_logger().info(
            f"Sending goal {self.goal_index + 1}/{len(self.goals)}: "
            f"x={goal['x']:.2f}, y={goal['y']:.2f}, yaw={goal['yaw']:.2f}"
        )
        future = self.client.send_goal_async(request)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("Goal rejected; trying the next goal after pause")
            self._schedule_next_goal()
            return
        self.get_logger().info("Goal accepted")
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        result = future.result()
        self.get_logger().info(f"Goal finished with status {result.status}")
        self._schedule_next_goal()

    def _schedule_next_goal(self):
        self.goal_active = False
        self.goal_index = (self.goal_index + 1) % len(self.goals)
        self.timer = self.create_timer(self.pause_between_goals_sec, self._timer_once)

    def _timer_once(self):
        self.timer.cancel()
        self._send_next_goal()


def main():
    rclpy.init()
    node = GazeboAutoGoals()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

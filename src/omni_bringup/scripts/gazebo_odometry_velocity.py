#!/usr/bin/env python3
"""Add finite-difference body velocity to Gazebo ground-truth odometry."""

import copy
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def _yaw(orientation):
    return math.atan2(
        2.0
        * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        ),
        1.0
        - 2.0
        * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        ),
    )


def _wrapped_angle_delta(current, previous):
    return math.atan2(
        math.sin(current - previous), math.cos(current - previous)
    )


class GazeboOdometryVelocity(Node):
    """Estimate the twist omitted by Gazebo's 3-D OdometryPublisher."""

    def __init__(self):
        super().__init__("gazebo_odometry_velocity")
        self.declare_parameter(
            "input_topic", "/visual_slam/tracking/odometry"
        )
        self.declare_parameter("output_topic", "/navigation/odometry")
        self.declare_parameter("smoothing_factor", 0.45)
        self.declare_parameter("max_sample_period_sec", 0.25)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self._alpha = float(self.get_parameter("smoothing_factor").value)
        self._max_dt = float(
            self.get_parameter("max_sample_period_sec").value
        )
        self._previous = None
        self._filtered_twist = None

        self._publisher = self.create_publisher(
            Odometry, output_topic, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry,
            input_topic,
            self._on_odometry,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"Estimating odometry twist: {input_topic} -> {output_topic}"
        )

    @staticmethod
    def _stamp_seconds(message):
        stamp = message.header.stamp
        return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

    def _on_odometry(self, message):
        output = copy.deepcopy(message)
        current_time = self._stamp_seconds(message)
        current_yaw = _yaw(message.pose.pose.orientation)

        if self._previous is not None:
            previous_time, previous_position, previous_yaw = self._previous
            dt = current_time - previous_time
            if 1.0e-4 < dt <= self._max_dt:
                world_vx = (
                    message.pose.pose.position.x - previous_position[0]
                ) / dt
                world_vy = (
                    message.pose.pose.position.y - previous_position[1]
                ) / dt
                world_vz = (
                    message.pose.pose.position.z - previous_position[2]
                ) / dt
                cos_yaw = math.cos(current_yaw)
                sin_yaw = math.sin(current_yaw)
                measured = [
                    cos_yaw * world_vx + sin_yaw * world_vy,
                    -sin_yaw * world_vx + cos_yaw * world_vy,
                    world_vz,
                    _wrapped_angle_delta(current_yaw, previous_yaw) / dt,
                ]
                if self._filtered_twist is None:
                    self._filtered_twist = measured
                else:
                    self._filtered_twist = [
                        self._alpha * new
                        + (1.0 - self._alpha) * old
                        for new, old in zip(
                            measured, self._filtered_twist
                        )
                    ]

        if self._filtered_twist is not None:
            twist = output.twist.twist
            twist.linear.x = self._filtered_twist[0]
            twist.linear.y = self._filtered_twist[1]
            twist.linear.z = self._filtered_twist[2]
            twist.angular.z = self._filtered_twist[3]

        position = message.pose.pose.position
        self._previous = (
            current_time,
            (position.x, position.y, position.z),
            current_yaw,
        )
        self._publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboOdometryVelocity()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

import math
import rclpy
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener, TransformException


class LocalPlanner(Node):
    def __init__(self):
        super().__init__('local_planner')
        
        # Declare Parameters with default values
        self.declare_parameter('forward_speed', 0.5)
        self.declare_parameter('repulsion_gain', 2.0)
        self.declare_parameter('influence_radius', 3.0)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('goal_reached_threshold', 0.5)

        # Publishers & Subscriptions
        self.pub = self.create_publisher(TwistStamped, '/nav/cmd_vel', 10)
        self.sub = self.create_subscription(OccupancyGrid, '/local_map/occupancy', self.map_cb, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/nav/goal_pose', self.goal_cb, 10)
        self.emergency_sub = self.create_subscription(Bool, '/nav/emergency', self.emergency_cb, 10)

        # TF2 Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Internal States
        self.current_goal = None
        self.emergency_active = False

        self.get_logger().info('APF Obstacle-Aware Local Planner initialized.')

    def emergency_cb(self, msg: Bool):
        self.emergency_active = msg.data
        if self.emergency_active:
            self.get_logger().warn('Emergency Signal received! Halting.')

    def goal_cb(self, msg: PoseStamped):
        self.current_goal = msg
        self.get_logger().info(
            f'New global goal received: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f} '
            f'in frame {msg.header.frame_id}'
        )

    def map_cb(self, msg: OccupancyGrid):
        # --- Decision Tree Check 1: Emergency Melt ---
        if self.emergency_active:
            self.publish_halt(msg.header)
            return

        # Fetch Parameters dynamically
        forward_speed = float(self.get_parameter('forward_speed').value)
        repulsion_gain = float(self.get_parameter('repulsion_gain').value)
        influence_radius = float(self.get_parameter('influence_radius').value)
        max_speed = float(self.get_parameter('max_speed').value)
        goal_reached_threshold = float(self.get_parameter('goal_reached_threshold').value)

        # Resolve robot coordinate within the map frame.
        frame_id = msg.header.frame_id
        if not frame_id:
            frame_id = 'map'

        robot_x = 0.0
        robot_y = 0.0
        yaw = 0.0

        if frame_id != 'base_link':
            try:
                # Retrieve latest transform from grid frame to robot center
                transform = self.tf_buffer.lookup_transform(
                    frame_id,
                    'base_link',
                    Time(),
                    timeout=Duration(seconds=0.1)
                )
                robot_x = transform.transform.translation.x
                robot_y = transform.transform.translation.y

                # Extract yaw heading from quaternion rotation
                q = transform.transform.rotation
                siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                yaw = math.atan2(siny_cosp, cosy_cosp)

            except TransformException as exc:
                self.get_logger().warning(
                    f'TF lookup from {frame_id} to base_link failed: {exc}. Stopping safely.'
                )
                self.publish_halt(msg.header)
                return

        # --- Decision Tree Check 2 & 3: Goal Navigation vs. Fallback Reactive Mode ---
        f_att_x = 0.0
        f_att_y = 0.0

        if self.current_goal is not None:
            # Active Goal Navigation Mode
            goal_x = self.current_goal.pose.position.x
            goal_y = self.current_goal.pose.position.y

            dist_to_goal = math.hypot(goal_x - robot_x, goal_y - robot_y)

            # Check if goal is reached
            if dist_to_goal < goal_reached_threshold:
                self.get_logger().info('Goal coordinate reached! Stopping.')
                self.current_goal = None
                self.publish_halt(msg.header)
                return

            # Apply Proportional deceleration in the final 2 meters
            speed = forward_speed
            if dist_to_goal < 2.0:
                speed = forward_speed * (dist_to_goal / 2.0)
                speed = max(0.1, speed)  # Maintain minimum control authority

            # Calculate Attractive Force vector
            f_att_x = ((goal_x - robot_x) / dist_to_goal) * speed
            f_att_y = ((goal_y - robot_y) / dist_to_goal) * speed

        else:
            # Fallback Reactive Mode: fly straight ahead while dodging obstacles
            f_att_x = math.cos(yaw) * forward_speed
            f_att_y = math.sin(yaw) * forward_speed

        # --- Decision Tree Check 4: Obstacle Repulsion ---
        f_rep_x = 0.0
        f_rep_y = 0.0

        res = msg.info.resolution
        width = msg.info.width
        height = msg.info.height
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        # Loop through occupied cells and sum repulsion vectors
        for iy in range(height):
            for ix in range(width):
                cell_val = msg.data[iy * width + ix]
                if cell_val >= 100:  # Obstacle present
                    # Transform cell center to physical map coordinates
                    obs_x = origin_x + (ix + 0.5) * res
                    obs_y = origin_y + (iy + 0.5) * res

                    dx = robot_x - obs_x
                    dy = robot_y - obs_y
                    dist_to_obs = math.hypot(dx, dy)

                    if dist_to_obs < influence_radius and dist_to_obs > 0.1:
                        # Classical APF repulsion formula
                        factor = repulsion_gain * (1.0 / dist_to_obs - 1.0 / influence_radius) * (1.0 / (dist_to_obs * dist_to_obs))
                        f_rep_x += (dx / dist_to_obs) * factor
                        f_rep_y += (dy / dist_to_obs) * factor

        # --- Decision Tree Check 5: Combined Control Synthesis ---
        v_x = f_att_x + f_rep_x
        v_y = f_att_y + f_rep_y

        # Scale velocity vector if exceeding max speed limits
        total_speed = math.hypot(v_x, v_y)
        if total_speed > max_speed:
            v_x = (v_x / total_speed) * max_speed
            v_y = (v_y / total_speed) * max_speed

        # Publish final control command
        twist = TwistStamped()
        twist.header = msg.header
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = frame_id
        twist.twist.linear.x = v_x
        twist.twist.linear.y = v_y
        twist.twist.linear.z = 0.0
        twist.twist.angular.z = 0.0
        self.pub.publish(twist)
        
        self.get_logger().debug(f'APF Output ENU: vx={v_x:.2f}, vy={v_y:.2f}')

    def publish_halt(self, header):
        twist = TwistStamped()
        twist.header = header
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.twist.linear.x = 0.0
        twist.twist.linear.y = 0.0
        twist.twist.linear.z = 0.0
        twist.twist.angular.z = 0.0
        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = LocalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

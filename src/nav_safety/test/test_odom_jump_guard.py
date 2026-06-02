import math
import unittest

from nav_msgs.msg import Odometry
from nav_safety.odom_jump_guard import angle_delta, pose_delta


def make_odom(x=0.0, y=0.0, z=0.0, yaw=0.0):
    msg = Odometry()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = z
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    return msg


class TestOdomJumpGuardMath(unittest.TestCase):
    def test_pose_delta_xy_z_yaw(self):
        previous = make_odom(x=1.0, y=2.0, z=0.1, yaw=0.1)
        current = make_odom(x=1.3, y=1.6, z=0.0, yaw=0.4)

        dx, dy, dz, dyaw = pose_delta(current, previous)

        self.assertAlmostEqual(dx, 0.3)
        self.assertAlmostEqual(dy, -0.4)
        self.assertAlmostEqual(dz, -0.1)
        self.assertAlmostEqual(dyaw, 0.3)

    def test_angle_delta_wraps_at_pi(self):
        current = math.radians(-179.0)
        previous = math.radians(179.0)

        self.assertAlmostEqual(math.degrees(angle_delta(current, previous)), 2.0)


if __name__ == "__main__":
    unittest.main()

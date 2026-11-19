"""Bounded motion references and controller interruption regressions."""

import math
import os
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "robosub_ws/src/robosub_control"))

from robosub_control.autonomy.motion_profile import MotionProfile  # noqa: E402

try:
    import rclpy
except ImportError:
    rclpy = None


class MotionProfileTests(unittest.TestCase):
    def test_one_metre_per_second_reference_reaches_cruise_and_stops(self):
        profile = MotionProfile((0.0, 0.0))
        profile.retarget((5.0, 0.0), 1.0, 0.5)
        state = profile.advance(2.0)
        self.assertAlmostEqual(state.position[0], 1.0)
        self.assertAlmostEqual(state.velocity[0], 1.0)
        state = profile.advance(3.0)
        self.assertAlmostEqual(state.position[0], 4.0)
        self.assertAlmostEqual(state.velocity[0], 1.0)
        state = profile.advance(2.0)
        self.assertEqual(state.position, (5.0, 0.0))
        self.assertEqual(state.velocity, (0.0, 0.0))
        self.assertTrue(profile.finished)


@unittest.skipIf(rclpy is None, "ROS Humble is not installed")
class ControllerProfileTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ["ROS_DOMAIN_ID"] = "198"
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        from robosub_control.autonomy.depth_heading_controller import (
            DepthHeadingController,
        )

        self.node = DepthHeadingController()
        self.node.pub = Mock()
        self.node.reference_pub = Mock()
        self.time = 10.0
        self.node.now = lambda: round(self.time * 1e9) * 1e-9
        self.node.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=self.stamp)
        )

    def tearDown(self):
        self.node.destroy_node()

    def stamp(self):
        from builtin_interfaces.msg import Time

        nanoseconds = round(self.time * 1e9)
        return Time(
            sec=nanoseconds // 1_000_000_000,
            nanosec=nanoseconds % 1_000_000_000,
        )

    def feed(self, x=0.0, y=0.0, yaw=0.0, valid=True, roll=0.0, pitch=0.0):
        from nav_msgs.msg import Odometry

        message = Odometry()
        message.header.stamp = self.stamp()
        message.header.frame_id = "world"
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        q = message.pose.pose.orientation
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        message.pose.covariance[0] = 0.0 if valid else -1.0
        self.node.on_state(message)
        return message

    def goal(self, x, y=0.0):
        from geometry_msgs.msg import Point

        self.node.on_pos_sp(Point(x=float(x), y=float(y)))

    def test_pitch_limit_cancels_targets_and_immediately_outputs_zero(self):
        self.node.fault_pub = Mock()
        self.feed(pitch=math.radians(35))
        self.goal(5)
        self.node.periodic()
        self.assertIsNone(self.node.pos_sp)
        command = self.node.pub.publish.call_args.args[0]
        self.assertEqual(command.linear.x, 0)
        self.assertEqual(command.linear.z, 0)
        self.node.fault_pub.publish.assert_called_once()
        self.assertIn(
            "pitch=+35.0", self.node.fault_pub.publish.call_args.args[0].data
        )

    def test_stale_state_zeros_and_recovery_replans(self):
        self.feed()
        self.goal(5)
        self.node.periodic()
        self.time += 0.1
        self.feed()
        self.node.periodic()
        for _ in range(6):
            self.time += 0.1
            self.node.periodic()
        command = self.node.pub.publish.call_args.args[0]
        self.assertEqual(command.linear.x, 0.0)
        self.assertTrue(self.node.position_needs_replan)
        self.time += 0.1
        self.feed(x=3.0)
        self.node.periodic()
        self.assertEqual(self.node.motion.state.position, (3.0, 0.0))

    def test_cancel_clears_targets_and_manual_cache_immediately(self):
        from geometry_msgs.msg import Twist
        from std_msgs.msg import Empty, Float64

        self.feed()
        self.goal(5)
        self.node.on_depth_sp(Float64(data=1.0))
        self.node.on_heading_sp(Float64(data=0.0))
        self.node.on_roll_sp(Float64(data=0.0))
        self.node.manual = Twist()
        self.node.manual.linear.x = 0.5
        self.node.manual_rx = self.time
        self.node.depth_pid.integral = 0.7
        self.node.next_output_time = self.time + 100
        self.node.on_cancel(Empty())
        self.assertIsNone(self.node.pos_sp)
        self.assertIsNone(self.node.depth_sp)
        self.assertIsNone(self.node.heading_sp)
        self.assertIsNone(self.node.roll_sp)
        self.assertIsNone(self.node.motion)
        self.assertIsNone(self.node.manual_rx)
        self.assertEqual(self.node.depth_pid.integral, 0.0)
        self.assertEqual(self.node.pub.publish.call_args.args[0].linear.x, 0.0)
        count = self.node.pub.publish.call_count
        self.time += 0.1
        self.feed()
        self.node.periodic()
        self.assertEqual(self.node.pub.publish.call_count, count)


if __name__ == "__main__":
    unittest.main()

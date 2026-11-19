"""Mission deadlines, input validity, arrival and cancellation checks."""

import math
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "robosub_ws/src/robosub_control"))

try:
    import rclpy
except ImportError:
    rclpy = None


@unittest.skipIf(rclpy is None, "ROS Humble is not installed")
class MissionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ["ROS_DOMAIN_ID"] = "198"
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def mission(self, steps):
        from robosub_control.autonomy.mission import Mission
        from robosub_control.config import constants

        with (
            patch.dict(
                constants.MISSION_DEFAULTS,
                {"mission": steps, "settle": 0.2, "step_timeout": 2.0},
            ),
            patch.object(Mission, "now", return_value=100.0),
        ):
            node = Mission()
        self.addCleanup(node.destroy_node)
        self.time = 100.0
        node.now = lambda: round(self.time * 1e9) * 1e-9
        for name in (
            "pub_depth",
            "pub_heading",
            "pub_pos",
            "pub_roll_turns",
            "pub_roll",
            "pub_cancel",
            "pub_status",
        ):
            setattr(node, name, Mock())
        return node

    def state(self, stamp=None, x=0.0, depth=1.0, dvl=True, speed=0.0):
        from nav_msgs.msg import Odometry

        msg = Odometry()
        stamp = self.time if stamp is None else stamp
        nanoseconds = round(stamp * 1e9)
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(
            nanoseconds, 1_000_000_000
        )
        msg.pose.pose.position.x = x
        msg.pose.pose.position.z = -depth
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance[0] = 0.0 if dvl else -1.0
        msg.twist.twist.linear.x = speed
        return msg

    def tick(self, node, time, **state):
        self.time = time
        node.on_state(self.state(**state))
        node.periodic()

    def test_first_step_deadline_runs_without_any_state(self):
        node = self.mission(["move 5"])
        self.time = 102.0
        node.periodic()
        node.periodic()
        self.assertTrue(node.finished)
        node.pub_cancel.publish.assert_called_once()
        node.pub_pos.publish.assert_not_called()

    def test_active_move_times_out_when_dvl_is_lost(self):
        node = self.mission(["move 5"])
        self.tick(node, 100.0)
        node.pub_pos.publish.assert_called_once()
        self.tick(node, 100.5, dvl=False)
        self.tick(node, 102.0, dvl=False)
        self.assertTrue(node.finished)
        node.pub_cancel.publish.assert_called_once()

    def test_stale_or_nonfinite_state_cannot_start_motion(self):
        for change in (
            {"stamp": 99.0},
            {"stamp": 101.0},
            {"x": math.nan},
            {"speed": math.inf},
        ):
            with self.subTest(change=change):
                node = self.mission(["move 5"])
                node.on_state(self.state(**change))
                node.periodic()
                self.assertIsNone(node.state)
                node.pub_pos.publish.assert_not_called()

    def test_operator_cancel_prevents_future_steps_without_republishing(self):
        from std_msgs.msg import Empty

        node = self.mission(["dive 1", "move 5"])
        self.tick(node, 100.0)
        node.on_cancel(Empty())
        self.tick(node, 100.1)
        self.tick(node, 100.4)
        self.assertTrue(node.finished)
        node.pub_pos.publish.assert_not_called()
        node.pub_cancel.publish.assert_not_called()

    def enter_depth_hold(self):
        node = self.mission(["dive 1", "hold 1"])
        for time in (100.0, 100.1, 100.4, 100.5):
            self.tick(node, time, dvl=False)
        self.assertEqual(node.parsed_steps[node.i][0], "hold")
        return node

    def enter_position_hold(self):
        node = self.mission(["dive 1", "heading 0", "move 5", "hold 1"])
        for time in (100.0, 100.1, 100.4, 100.5, 100.6, 100.9, 101.0):
            self.tick(node, time)
        for time in (101.1, 101.4, 101.5):
            self.tick(node, time, x=5.0)
        self.assertEqual(node.parsed_steps[node.i][0], "hold")
        return node

    def test_depth_only_hold_requires_continuous_duration_without_dvl(self):
        node = self.enter_depth_hold()
        for time in (100.9, 101.3, 101.4):
            self.tick(node, time, dvl=False)
        self.assertFalse(node.finished)
        self.tick(node, 101.6, dvl=False)
        self.assertTrue(node.finished)
        node.pub_depth.publish.assert_called_once()
        node.pub_pos.publish.assert_not_called()
        node.pub_cancel.publish.assert_not_called()


if __name__ == "__main__":
    unittest.main()

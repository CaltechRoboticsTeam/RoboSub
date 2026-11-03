"""Exercise Xbox tuning maneuvers with synthetic state and no hardware."""

import math
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "robosub_ws/src/robosub_control"))

try:
    import rclpy
except ImportError:
    rclpy = None


@unittest.skipIf(rclpy is None, "ROS Humble is not installed")
class AssistedGamepadTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ["ROS_DOMAIN_ID"] = "198"
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
        rclpy.init(
            args=[
                "--ros-args",
                "-r",
                "/sub/cmd_vel:=/test/assisted_cmd",
            ]
        )

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        from robosub_control.teleop.xbox_teleop import XboxTeleop

        self.node = XboxTeleop()
        self.clock = 100.0
        self.node.now = lambda: self.clock
        self.node.pub = Mock()
        self.node.target_pubs["depth"] = Mock()

    def tearDown(self):
        self.node.destroy_node()

    def tick(
        self,
        button=None,
        *,
        depth=1.0,
        yaw=0.0,
        roll=0.0,
        dvl=True,
        axes=None,
        buttons=None,
        x=2.0,
        y=3.0,
    ):
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Joy

        self.clock += 0.05
        state = Odometry()
        state.header.stamp.sec = int(self.clock)
        state.header.stamp.nanosec = int((self.clock % 1) * 1e9)
        state.pose.pose.position.x = x
        state.pose.pose.position.y = y
        state.pose.pose.position.z = -depth
        state.pose.pose.orientation.w = math.cos(yaw / 2) * math.cos(roll / 2)
        state.pose.pose.orientation.x = math.cos(yaw / 2) * math.sin(roll / 2)
        state.pose.pose.orientation.y = math.sin(yaw / 2) * math.sin(roll / 2)
        state.pose.pose.orientation.z = math.sin(yaw / 2) * math.cos(roll / 2)
        state.pose.covariance[0] = 0.0 if dvl else -1.0
        self.node.on_state(state)
        actions = buttons or [int(index == button) for index in range(8)]
        self.node.on_joy(Joy(axes=axes or [0.0] * 6, buttons=[1, *actions]))
        self.node.periodic()

    def assert_neutral(self):
        command = self.node.pub.publish.call_args.args[0]
        self.assertEqual(
            (
                command.linear.x,
                command.linear.y,
                command.linear.z,
                command.angular.x,
                command.angular.z,
            ),
            (0.0,) * 5,
        )

    def test_a_captures_one_endpoint_and_holds_until_release(self):
        self.tick(0, yaw=math.pi / 2)
        target = self.node.pos_sp
        self.assertAlmostEqual(target[0], 2.0)
        self.assertAlmostEqual(target[1], 4.0)
        for _ in range(70):
            self.tick(0, yaw=math.pi / 2)
            self.assertEqual(self.node.pos_sp, target)
        self.assertEqual(self.node.depth_sp, 1.0)
        self.tick()
        self.assertIsNone(self.node.active_button)
        self.assertIsNone(self.node.pos_sp)
        self.assertIsNone(self.node.depth_sp)
        self.assertTrue(self.node.control.enabled)
        self.assert_neutral()

    def test_b_and_x_hold_depth_without_requiring_dvl(self):
        self.tick(1, dvl=False)
        self.assertEqual(self.node.depth_sp, 1.0)
        self.assertIsNone(self.node.pos_sp)
        self.assertIsNone(self.node.heading_sp)
        self.assertEqual(self.node.roll_sp, 0.0)
        self.tick()
        self.tick(2, dvl=False)
        self.assertEqual(self.node.depth_sp, 1.5)
        self.assertEqual(
            self.node.target_pubs["depth"].publish.call_args.args[0].data, 1.5
        )
        self.tick(2, depth=1.2, dvl=False)
        self.assertEqual(self.node.depth_sp, 1.5)
        self.tick()
        self.tick(2, depth=1.2, dvl=False)
        self.assertAlmostEqual(self.node.depth_sp, 1.7)

    def test_y_is_clockwise_and_does_not_repeat_while_held(self):
        self.tick(3, dvl=False)
        self.assertAlmostEqual(self.node.heading_sp, -math.pi / 2)
        self.tick(3, yaw=-0.2, dvl=False)
        self.assertAlmostEqual(self.node.heading_sp, -math.pi / 2)
        self.tick()
        self.assertIsNone(self.node.heading_sp)
        self.assert_neutral()

    def test_sticks_take_over_without_rearming_a_held_button(self):
        self.tick(2)
        self.tick(2, axes=[0.0, -1.0, 0.0, 0.0, 0.0, 0.0])
        self.assertIsNone(self.node.active_button)
        self.assertEqual(self.node.pub.publish.call_args.args[0].linear.x, 1.0)
        self.tick(2)
        self.assertIsNone(self.node.active_button)
        self.assert_neutral()

    def test_dvl_loss_and_wall_time_staleness_cancel_assistance(self):
        self.tick(0)
        self.tick(0, dvl=False)
        self.assertIsNone(self.node.active_button)
        self.assert_neutral()
        self.tick()
        self.tick(1)
        self.node.state_wall_rx -= 1.0
        self.node.periodic()
        self.assertIsNone(self.node.active_button)
        self.assert_neutral()
        self.tick()
        self.tick(1)
        self.node.control.last_rx -= 1.0
        self.node.periodic()
        self.assertIsNone(self.node.active_button)
        self.assertTrue(self.node.control.enabled)
        self.assert_neutral()

    def test_hover_latches_steps_once_and_keeps_all_other_axes_off(self):
        from robosub_control.teleop.gamepad import LB, RB, SELECT

        self.tick(SELECT, dvl=False)
        self.assert_neutral()
        self.node.next_step_time = 0.0
        self.tick(LB, dvl=False)
        self.assertEqual(self.node.measurement_effort, -0.01)
        self.assertIsNone(self.node.depth_sp)
        self.assertIsNone(self.node.roll_sp)
        self.assertIsNone(self.node.heading_sp)
        self.tick(dvl=False)
        self.tick(LB, depth=1.2, roll=0.1, dvl=False)
        self.assertEqual(self.node.measurement_effort, -0.01)
        self.node.next_step_time = 0.0
        self.tick(LB, dvl=False)
        self.assertEqual(self.node.measurement_effort, -0.01)
        self.tick(dvl=False)
        self.tick(LB, dvl=False)
        command = self.node.pub.publish.call_args.args[0]
        self.assertEqual(command.linear.z, -0.02)
        self.assertEqual(
            (
                command.linear.x,
                command.linear.y,
                command.angular.x,
                command.angular.y,
                command.angular.z,
            ),
            (0.0,) * 5,
        )
        self.node.command_limit = 0.02
        self.node.next_step_time = 0.0
        self.tick()
        self.tick(LB)
        self.assertEqual(self.node.measurement_effort, -0.02)
        self.tick(SELECT)
        self.assertIsNone(self.node.measurement_button)
        self.assert_neutral()
        self.tick(SELECT)
        self.assertIsNone(self.node.measurement_button)
        self.tick()
        self.tick(SELECT)
        self.node.next_step_time = 0.0
        self.tick(RB)
        self.assertEqual(self.node.measurement_effort, 0.01)

    def test_drag_keeps_forward_effort_and_holds_depth_heading_and_track(self):
        from robosub_control.teleop.gamepad import RB, START

        self.node.depth_pid.kp = self.node.sway_pid.kp = 1.0
        self.node.sway_pid.kd = 0.0
        self.tick(START, yaw=math.pi / 2)
        self.assertIsNone(self.node.pos_sp)
        self.assertEqual(self.node.depth_sp, 1.0)
        self.assertEqual(self.node.roll_sp, 0.0)
        self.assertAlmostEqual(self.node.heading_sp, math.pi / 2)
        self.node.next_step_time = 0.0
        self.tick(RB, yaw=math.pi / 2)
        self.tick(depth=1.1, yaw=math.pi / 2, x=2.2, y=5.0)
        command = self.node.pub.publish.call_args.args[0]
        self.assertEqual(command.linear.x, 0.05)
        self.assertGreater(command.linear.y, 0.0)
        self.assertGreater(command.linear.z, 0.0)
        self.assertEqual(self.node.depth_sp, 1.0)
        self.tick(dvl=False)
        self.assertIsNone(self.node.measurement_button)
        self.assert_neutral()

    def test_measurements_cancel_on_manual_disable_stale_feedback_and_tilt(
        self,
    ):
        from robosub_control.teleop.gamepad import SELECT, START
        from sensor_msgs.msg import Joy

        self.tick(START)
        self.tick(axes=[0.0, -1.0, 0.0, 0.0, 0.0, 0.0])
        self.assertIsNone(self.node.measurement_button)
        self.assertEqual(self.node.pub.publish.call_args.args[0].linear.x, 1.0)
        self.tick(SELECT)
        self.node.state_wall_rx -= 1.0
        self.node.periodic()
        self.assertIsNone(self.node.measurement_button)
        self.assert_neutral()
        self.tick()
        self.tick(SELECT)
        self.tick(roll=math.radians(40))
        self.assertIsNone(self.node.measurement_button)
        self.assert_neutral()
        self.tick(START)
        self.node.control.last_rx -= 1.0
        self.node.periodic()
        self.assertIsNone(self.node.measurement_button)
        self.assertTrue(self.node.control.enabled)
        self.assert_neutral()
        self.tick()
        self.tick(SELECT)
        self.node.on_joy(Joy(axes=[0.0] * 6, buttons=[0]))
        self.assertIsNone(self.node.measurement_button)
        self.assert_neutral()


if __name__ == "__main__":
    unittest.main()

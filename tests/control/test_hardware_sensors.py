"""Exercise real-message adaptation without opening any hardware device."""

from copy import deepcopy
import math
import os
from pathlib import Path
import sys
import time
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "robosub_ws/src/robosub_control"))

try:
    import rclpy
    from mavros_msgs.msg import DebugValue
except ImportError:
    rclpy = None


@unittest.skipIf(rclpy is None, "ROS and mavros_msgs are required")
class HardwareSensorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ["ROS_DOMAIN_ID"] = "198"
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        from robosub_control.drivers.hardware_sensors import HardwareSensors

        self.node = HardwareSensors(
            cli_args=["--ros-args", "-r", "__node:=hardware_callback_test"]
        )
        self.addCleanup(self.node.destroy_node)
        self.now = 10.0
        self.node.now = lambda: self.now
        self.node.imu_pub = Mock()
        self.node.depth_pub = Mock()
        self.node.dvl_pub = Mock()

    def imu(self):
        from sensor_msgs.msg import Imu

        message = Imu()
        message.header.stamp.sec = 10
        message.header.frame_id = "base_link"
        message.angular_velocity.z = 0.1
        message.linear_acceleration.z = 9.81
        return message

    def dvl(self):
        from nav_msgs.msg import Odometry

        message = Odometry()
        message.header.stamp.sec = 10
        message.header.frame_id = "odom"
        message.child_frame_id = "base_link"
        message.pose.pose.position.x = math.nan
        message.pose.pose.position.y = math.nan
        message.pose.pose.position.z = math.nan
        message.twist.twist.linear.x = 0.75
        for index in (0, 7, 14):
            message.twist.covariance[index] = 0.02
        return message

    def test_invalid_or_repeated_imu_cannot_renew_freshness(self):
        valid = self.imu()
        self.node.on_imu(valid)
        self.now = 10.2
        invalids = []
        for attribute in (
            "angular_velocity_covariance",
            "linear_acceleration_covariance",
        ):
            message = deepcopy(valid)
            getattr(message, attribute)[0] = -1.0
            invalids.append(message)
        bad_frame = deepcopy(valid)
        bad_frame.header.frame_id = "unconverted_frd"
        invalids.append(bad_frame)
        bad_value = deepcopy(valid)
        bad_value.angular_velocity.x = math.nan
        invalids.append(bad_value)
        future = deepcopy(valid)
        future.header.stamp.sec = 11
        invalids.append(future)
        old = deepcopy(valid)
        old.header.stamp.sec = 9
        invalids.append(old)
        for message in (*invalids, valid):
            self.node.on_imu(message)
        self.node.imu_pub.publish.assert_called_once()
        self.assertEqual(self.node.imu_samples.received, 10.0)
        self.assertFalse(self.node.imu_samples.fresh(10.5))

    def test_dvl_rejects_stale_repeated_and_invalid_velocity(self):
        valid = self.dvl()
        self.node.on_dvl(valid)
        self.now = 10.2
        for kind in ("old", "future", "nan", "covariance", "frame", "repeat"):
            message = deepcopy(valid)
            if kind == "old":
                message.header.stamp.sec = 8
            elif kind == "future":
                message.header.stamp.sec = 11
            elif kind == "nan":
                message.twist.twist.linear.y = math.nan
            elif kind == "covariance":
                message.twist.covariance[7] = -1.0
            elif kind == "frame":
                message.child_frame_id = "world"
            self.node.on_dvl(message)
        self.node.dvl_pub.publish.assert_called_once()
        self.assertEqual(self.node.dvl_samples.received, 10.0)

    def test_pressure_prefers_usb_and_falls_back_only_on_new_mavros_input(self):
        from std_msgs.msg import Float64

        self.node.on_usb_depth(Float64(data=1.5))
        self.now = 10.1
        self.node.on_depth(DebugValue(name="DEPTH", value_float=2.0))
        self.node.on_usb_depth(Float64(data=math.nan))
        self.node.depth_pub.publish.assert_called_once()
        self.now = 10.6
        self.assertFalse(self.node.usb_depth_samples.fresh(self.now))
        self.node.depth_pub.publish.assert_called_once()
        self.node.on_depth(DebugValue(name="DEPTH", value_float=2.0))
        self.assertEqual(self.node.active_depth_source, "mavros")
        self.assertEqual(self.node.depth_pub.publish.call_count, 2)
        self.assertEqual(
            self.node.depth_pub.publish.call_args.args[0].data, 2.0
        )
        self.now = 10.7
        self.node.on_usb_depth(Float64(data=1.6))
        self.assertEqual(self.node.active_depth_source, "usb")

    def test_teensy_depth_callback_drops_low_adc_and_malformed_values(self):
        from robosub_control.drivers.teensy_actuators import TeensyActuators

        with patch(
            "robosub_control.drivers.teensy_actuators.TeensyUsb"
        ) as bridge:
            bridge.return_value.connect.return_value = {
                "rs485_serial": 1,
                "max_delta_us": 500,
            }
            driver = TeensyActuators()
        try:
            self.assertEqual(
                bridge.call_args.kwargs["on_depth"], driver.on_depth
            )
            driver.depth_pub = Mock()
            for fields in (
                ["D1", "0", "1.0", "0", "0"],
                ["D1", "1", "1.0", "0", "0"],
                ["D1", "400", "nan", "0", "0"],
                ["D1", "400", "1.0", "bad", "0"],
            ):
                driver.on_depth(fields)
            driver.depth_pub.publish.assert_not_called()
            driver.on_depth(["D1", "410", "1.25", "0", "0"])
            self.assertEqual(
                driver.depth_pub.publish.call_args.args[0].data, 1.25
            )
        finally:
            driver.destroy_node()

    def test_best_effort_hardware_messages_reach_reliable_estimator(self):
        from nav_msgs.msg import Odometry
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import qos_profile_sensor_data
        from robosub_control.autonomy.state_estimator import StateEstimator
        from robosub_control.config import current_m5
        from robosub_control.drivers.hardware_sensors import HardwareSensors
        from sensor_msgs.msg import Imu

        hardware = Node("fake_existing_hardware_readers")
        adapter = HardwareSensors()
        estimator = StateEstimator(
            parameter_overrides=[
                Parameter(
                    "dvl_offset",
                    value=list(current_m5.CONTROL_BOARD_POSITION_BODY_M),
                )
            ]
        )
        imu_pub = hardware.create_publisher(
            Imu, "/mavros/imu/data", qos_profile_sensor_data
        )
        depth_pub = hardware.create_publisher(
            DebugValue, "/mavros/debug_value/named_value_float", 10
        )
        dvl_pub = hardware.create_publisher(
            Odometry, "/mavros/odometry/out", 10
        )
        states = []
        hardware.create_subscription(Odometry, "/sub/state", states.append, 10)
        executor = SingleThreadedExecutor()
        for node in (hardware, adapter, estimator):
            executor.add_node(node)
        try:
            deadline = time.monotonic() + 4.0
            while time.monotonic() < deadline:
                imu = self.imu()
                imu.header.stamp = hardware.get_clock().now().to_msg()
                imu.angular_velocity.z = 1.0
                dvl = self.dvl()
                dvl.header.stamp = imu.header.stamp
                # Pure yaw at rest: legacy reader reports Pixhawk velocity.
                rx, ry, _ = current_m5.CONTROL_BOARD_POSITION_BODY_M
                dvl.twist.twist.linear.x = -ry
                dvl.twist.twist.linear.y = rx
                imu_pub.publish(imu)
                depth_pub.publish(DebugValue(name="DEPTH", value_float=1.25))
                if estimator.rates[2] == 1.0:
                    dvl_pub.publish(dvl)
                until = time.monotonic() + 0.02
                while time.monotonic() < until:
                    executor.spin_once(timeout_sec=0.002)
                if (
                    len(states) >= 3
                    and states[-1].twist.twist.angular.z == 1.0
                    and abs(states[-1].twist.twist.linear.x) < 1e-9
                    and states[-1].pose.covariance[0] == 0.0
                ):
                    break
            else:
                self.fail("Hardware messages did not produce a valid state")
            self.assertAlmostEqual(states[-1].pose.pose.position.z, -1.25)
            self.assertAlmostEqual(states[-1].pose.pose.position.x, 0.0)
            self.assertAlmostEqual(states[-1].pose.pose.position.y, 0.0)
            count = len(states)
            until = time.monotonic() + 0.65
            while time.monotonic() < until:
                imu = self.imu()
                imu.header.stamp = hardware.get_clock().now().to_msg()
                imu_pub.publish(imu)
                executor.spin_once(timeout_sec=0.01)
            after_expiry = len(states)
            self.assertGreaterEqual(after_expiry, count)
            until = time.monotonic() + 0.15
            while time.monotonic() < until:
                imu.header.stamp = hardware.get_clock().now().to_msg()
                imu_pub.publish(imu)
                executor.spin_once(timeout_sec=0.005)
            self.assertEqual(len(states), after_expiry)
        finally:
            executor.shutdown()
            for node in (hardware, adapter, estimator):
                node.destroy_node()


if __name__ == "__main__":
    unittest.main()

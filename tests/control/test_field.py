"""Field-test limits, passive checks and isolated fake-Pixhawk execution."""

import copy
import json
import os
from pathlib import Path
import struct
import sys
import threading
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "robosub_ws/src/robosub_sim"))
sys.path.insert(0, str(ROOT / "robosub_ws/src/robosub_bringup"))

from robosub_bringup.field_checks import (  # noqa: E402
    check_outputs,
    json_finite,
    manual_values,
    preflight,
    pulse_effort,
    pulse_plan,
    receiver_power,
    servo_packet,
)

try:
    import mavros_msgs  # noqa: F401
    import rclpy
except ImportError:
    rclpy = None


def valid_report():
    return {
        "last_messages": {
            "state": {"connected": True, "armed": False, "mode": "MANUAL"},
            "rc_out": {"channels": [1500] * 8},
        },
        "topics": {
            "state": {"age_s": 0.1},
            "rc_out": {"age_s": 0.1, "rate_hz": 10.0},
        },
        "vehicle_info": [{"autopilot": 12, "type": 12}],
        "cached_parameters": {"SYS_AUTOSTART": 60002, "UUV_MGM_THRTL": 0.5},
        "servo_ports_on_jetson_link": {
            "0": {"channels": [1500] * 8, "age_s": 0.1}
        },
        "manual_control_publishers": 0,
        "rc_override_publishers": 0,
    }


class FieldChecksTests(unittest.TestCase):
    def test_pulse_limits_and_ramp(self):
        plan = pulse_plan("throttle", 0.05, 0.5, 0.25)
        self.assertEqual(pulse_effort(-1, plan), 0)
        self.assertEqual(pulse_effort(0, plan), 0)
        self.assertAlmostEqual(pulse_effort(0.125, plan), 0.025)
        self.assertEqual(pulse_effort(0.5, plan), 0.05)
        self.assertAlmostEqual(pulse_effort(0.875, plan), 0.025)
        self.assertEqual(pulse_effort(1.0, plan), 0)
        for args in (
            ("throttle", 1.0, 0.5, 0.25),
            ("throttle", 0.05, 5.0, 0.25),
            ("throttle", float("nan"), 0.5, 0.25),
            ("throttle", 0.05, 0.5, 0),
            ("roll", 0.05, 0.5, 0.25),
        ):
            with self.subTest(args=args), self.assertRaises(ValueError):
                pulse_plan(*args)

    def test_native_manual_units_and_neutral(self):
        self.assertEqual(manual_values("throttle", 0), (0.0, 0.0, 500.0, 0.0))
        self.assertEqual(
            manual_values("throttle", 0.05), (0.0, 0.0, 525.0, 0.0)
        )
        self.assertEqual(manual_values("yaw", -0.05), (0.0, 0.0, 500.0, -50.0))
        with self.assertRaises(ValueError):
            manual_values("throttle", 1.0)

    def test_zero_servo_is_rejected_and_output_deviation_is_bounded(self):
        self.assertEqual(receiver_power([1500] * 8), [0.0] * 8)
        self.assertAlmostEqual(receiver_power([1525] * 8)[0], 0.05)
        for channels in ([0] * 8, [1500] * 7, [2001] * 8):
            with self.assertRaises(ValueError):
                receiver_power(channels)
        check_outputs([1550] * 8)
        with self.assertRaises(ValueError):
            check_outputs([1551] * 8)

    def test_raw_servo_decode_and_unknown_sensor_values(self):
        payload = struct.pack("<I8HB", 123, *([1500] * 8), 0).ljust(24, b"\x00")
        message = SimpleNamespace(
            msgid=36,
            len=21,
            sysid=1,
            compid=1,
            payload64=struct.unpack("<3Q", payload),
        )
        self.assertEqual(servo_packet(message)["channels"], [1500] * 8)
        self.assertEqual(servo_packet(message)["port"], 0)
        message.msgid = 0
        self.assertIsNone(servo_packet(message))
        self.assertEqual(json_finite({"depth": float("nan")}), {"depth": None})
        self.assertEqual(
            json_finite({"version": bytes([214, 241, 42])}),
            {"version": "d6f12a"},
        )

    def test_preflight_rejects_stale_armed_wrong_firmware_and_competing_input(
        self,
    ):
        report = valid_report()
        preflight(report)
        for section, key, value in (
            ("state", "armed", True),
            ("state", "mode", "OFFBOARD"),
            ("state", "connected", False),
            ("vehicle", "autopilot", 3),
            ("params", "SYS_AUTOSTART", 60001),
            ("root", "manual_control_publishers", 1),
            ("root", "rc_override_publishers", 1),
        ):
            changed = copy.deepcopy(report)
            target = {
                "state": changed["last_messages"]["state"],
                "vehicle": changed["vehicle_info"][0],
                "params": changed["cached_parameters"],
                "root": changed,
            }[section]
            target[key] = value
            with (
                self.subTest(section=section, key=key),
                self.assertRaises(ValueError),
            ):
                preflight(changed)
        slow = copy.deepcopy(report)
        slow["topics"]["rc_out"]["rate_hz"] = 1.0
        with self.assertRaisesRegex(ValueError, "below 5 Hz"):
            preflight(slow)
        report["topics"]["state"]["age_s"] = 2.0
        with self.assertRaises(ValueError):
            preflight(report)


@unittest.skipIf(rclpy is None, "ROS/MAVROS messages are not installed")
class FieldRosTests(unittest.TestCase):
    def setUp(self):
        from mavros_msgs.msg import (
            ManualControl,
            Mavlink,
            RCOut,
            State,
            VehicleInfo,
        )
        from mavros_msgs.srv import CommandBool, VehicleInfoGet
        from rcl_interfaces.msg import ParameterValue
        from rcl_interfaces.srv import GetParameters
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from robosub_bringup.field_monitor import FieldMonitor

        os.environ["ROS_DOMAIN_ID"] = "198"
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
        rclpy.init()
        self.fake = Node("fake_field_pixhawk")
        self.armed = False
        self.bad_output = False
        self.arm_calls = []
        self.manual = []
        self.fake.create_subscription(
            ManualControl, "/mavros/manual_control/send", self.manual.append, 1
        )
        state_pub = self.fake.create_publisher(State, "/mavros/state", 10)
        rc_pub = self.fake.create_publisher(RCOut, "/mavros/rc/out", 10)
        raw_pub = self.fake.create_publisher(
            Mavlink, "/uas1/mavlink_source", 10
        )

        def publish():
            state_pub.publish(
                State(connected=True, armed=self.armed, mode="MANUAL")
            )
            delta = 0
            if self.armed and self.manual:
                delta = round(self.manual[-1].z - 500 + self.manual[-1].r / 2)
            if self.armed and self.bad_output:
                delta = 60
            channels = [1500 + delta] * 8
            rc_pub.publish(RCOut(channels=channels))
            payload = struct.pack("<I8HB", 123, *channels, 0).ljust(24, b"\x00")
            raw_pub.publish(
                Mavlink(
                    msgid=36,
                    len=21,
                    sysid=1,
                    compid=1,
                    payload64=list(struct.unpack("<3Q", payload)),
                )
            )

        def arm(request, response):
            self.arm_calls.append(request.value)
            self.armed = request.value
            response.success = True
            response.result = 0
            return response

        def info(_request, response):
            response.success = True
            response.vehicles = [
                VehicleInfo(sysid=1, compid=1, autopilot=12, type=12)
            ]
            return response

        def parameters(request, response):
            response.values = [
                ParameterValue(type=2, integer_value=60002)
                if name == "SYS_AUTOSTART"
                else ParameterValue(type=3, double_value=0.5)
                if name == "UUV_MGM_THRTL"
                else ParameterValue()
                for name in request.names
            ]
            return response

        self.fake.create_service(CommandBool, "/mavros/cmd/arming", arm)
        self.fake.create_service(
            VehicleInfoGet, "/mavros/vehicle_info_get", info
        )
        self.fake.create_service(
            GetParameters, "/mavros/param/get_parameters", parameters
        )
        self.fake.create_timer(0.05, publish)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.fake)
        self.worker = threading.Thread(target=self.executor.spin, daemon=True)
        self.worker.start()
        self.monitor = FieldMonitor()
        self.monitor.read_cached_configuration()
        self.monitor.collect(0.25)

    def tearDown(self):
        self.monitor.destroy_node()
        self.executor.shutdown()
        self.worker.join(timeout=3)
        self.fake.destroy_node()
        rclpy.shutdown()

    def test_passive_check_sends_no_manual_or_arm_commands(self):
        preflight(self.monitor.snapshot())
        self.assertFalse(self.manual)
        self.assertFalse(self.arm_calls)

    def test_status_report_is_strict_json(self):
        report = self.monitor.snapshot()
        report["binary_version"] = bytes([214, 241, 42])
        encoded = json.dumps(json_finite(report), allow_nan=False)
        self.assertEqual(json.loads(encoded)["binary_version"], "d6f12a")
        self.assertFalse(self.arm_calls)

    def test_short_executed_pulse_returns_neutral_and_confirms_disarm(self):
        from robosub_bringup.field_test import PulseTest

        pulse = PulseTest(self.monitor, pulse_plan("throttle", 0.05, 0.1, 0.1))
        pulse.run()
        self.assertEqual(self.arm_calls, [True, False])
        self.assertFalse(self.armed)
        self.assertGreater(max(message.z for message in self.manual), 500.0)
        self.assertLessEqual(max(message.z for message in self.manual), 525.0)
        self.assertEqual(self.manual[-1].z, 500.0)
        self.assertEqual(pulse.events[-1], "disarming_confirmed")

    def test_output_excursion_aborts_and_disarms(self):
        from robosub_bringup.field_test import PulseTest

        self.bad_output = True
        pulse = PulseTest(self.monitor, pulse_plan("throttle", 0.05, 0.1, 0.1))
        with self.assertRaisesRegex(RuntimeError, "exceeded"):
            pulse.run()
        self.assertFalse(self.armed)
        self.assertEqual(self.arm_calls[-1], False)
        self.assertEqual(self.manual[-1].z, 500.0)

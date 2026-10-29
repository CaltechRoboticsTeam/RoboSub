"""Numeric tests plus isolated ROS integration when Humble is available."""

from http.server import ThreadingHTTPServer
import json
import os
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "robosub_ws/src/robosub_control"))
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "tools/diagnostics"))
sys.path.insert(0, str(ROOT / "robosub_ws/src/robosub_sim"))
sys.path.insert(0, str(ROOT / "robosub_ws/src/robosub_bringup"))

from robosub_control.utils.tunable_number import TunableNumber  # noqa: E402

try:
    import rclpy
except ImportError:
    rclpy = None


class NumberTests(unittest.TestCase):
    def test_cached_read_and_bound_attribute(self):
        number = TunableNumber(1.0, minimum=0.0, maximum=5.0)
        pid = SimpleNamespace(kp=0.0)
        number.bind_to(pid, "kp")
        self.assertEqual(pid.kp, 1.0)
        number._commit(2.0)
        self.assertEqual(number.get(), 2.0)
        self.assertEqual(pid.kp, 2.0)

    def test_validation_has_no_side_effects(self):
        number = TunableNumber(1.0, minimum=0.0, maximum=5.0)
        for value in (float("nan"), float("inf"), -1.0, 6.0, True, 2):
            with self.subTest(value=value), self.assertRaises(ValueError):
                number.validate(value)
            self.assertEqual(number.get(), 1.0)

    def test_integer_type_is_preserved(self):
        number = TunableNumber(3, minimum=1, maximum=10)
        number._commit(4)
        self.assertIs(type(number.get()), int)
        with self.assertRaises(ValueError):
            number._commit(4.5)


@unittest.skipIf(rclpy is None, "ROS Humble is not installed")
class RosTuningTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # No hardware topics are reachable from this isolated DDS domain.
        os.environ["ROS_DOMAIN_ID"] = "198"
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
        rclpy.init(
            args=[
                "--ros-args",
                "-p",
                "tuning_enabled:=true",
                "-r",
                "/sub/cmd_vel:=/test/cmd_vel",
            ]
        )

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from robosub_bringup.tuning_menu import ParameterBrowser
        from robosub_control.utils.tuning import TunableNumbers

        self.node = Node("tuning_test")
        self.registry = TunableNumbers(self.node)
        self.kp = self.registry.number("kp", 1.0, minimum=0.0, maximum=5.0)
        self.ki = self.registry.number("ki", 0.0, minimum=0.0, maximum=5.0)
        self.browser = ParameterBrowser()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.add_node(self.browser)
        self.worker = threading.Thread(target=self.executor.spin, daemon=True)
        self.worker.start()

    def tearDown(self):
        self.executor.shutdown()
        self.worker.join(timeout=3)
        self.node.destroy_node()
        self.browser.destroy_node()

    def wait_for(self, condition):
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            if condition():
                return
            time.sleep(0.01)
        self.fail("Committed parameter event was not applied")

    def test_accepted_update_changes_cache_and_bound_field(self):
        target = SimpleNamespace(kp=0.0)
        self.kp.bind_to(target, "kp")
        self.browser.update("/tuning_test", {"kp": 2.5})
        self.wait_for(lambda: target.kp == 2.5)
        self.assertEqual(self.kp.get(), 2.5)

    def test_invalid_atomic_batch_changes_nothing(self):
        with self.assertRaises(ValueError):
            self.browser.update("/tuning_test", {"kp": 2.0, "ki": -1.0})
        self.assertEqual(self.node.get_parameter("kp").value, 1.0)
        self.assertEqual(self.kp.get(), 1.0)

    def test_later_validator_rejection_does_not_change_cache(self):
        from rcl_interfaces.msg import SetParametersResult

        self.node.add_on_set_parameters_callback(
            lambda _parameters: SetParametersResult(
                successful=False, reason="Another component rejected this"
            )
        )
        with self.assertRaises(ValueError):
            self.browser.update("/tuning_test", {"kp": 2.0})
        self.assertEqual(self.kp.get(), 1.0)

    def test_hard_lock_ignores_overrides_and_rejects_updates(self):
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from robosub_control.config import constants
        from robosub_control.utils.tuning import TunableNumbers

        locked = Node(
            "locked_test",
            parameter_overrides=[
                Parameter("kp", value=4.0),
            ],
        )
        try:
            with patch.object(constants, "LOCK_TUNING_DEFAULTS", True):
                registry = TunableNumbers(locked)
                number = registry.number("kp", 1.0)
            self.assertFalse(registry.enabled)
            self.assertFalse(hasattr(registry, "subscription"))
            self.assertEqual(number.get(), 1.0)
            result = locked.set_parameters_atomically(
                [
                    Parameter("kp", value=2.0),
                ]
            )
            self.assertFalse(result.successful)
            self.assertEqual(number.get(), 1.0)
        finally:
            locked.destroy_node()

    def test_real_controller_gain_updates_without_reconstruction(self):
        from robosub_control.autonomy.depth_heading_controller import (
            DepthHeadingController,
        )

        controller = DepthHeadingController()
        self.executor.add_node(controller)
        try:
            original_pid = controller.depth_pid
            self.browser.update("/depth_heading_controller", {"depth_kp": 2.0})
            self.wait_for(lambda: original_pid.kp == 2.0)
            self.assertIs(controller.depth_pid, original_pid)
            self.assertAlmostEqual(original_pid.step(0.1, 0.0, 0.01), 0.2002)
            with self.assertRaises(ValueError):
                self.browser.update("/depth_heading_controller", {"rate": 500})
        finally:
            self.executor.remove_node(controller)
            controller.destroy_node()

    def test_default_output_rate_publishes_each_tick_despite_jitter(self):
        from nav_msgs.msg import Odometry
        from robosub_control.autonomy.depth_heading_controller import (
            DepthHeadingController,
        )

        controller = DepthHeadingController()
        try:
            controller.state = Odometry()
            controller.state.pose.pose.orientation.w = 1.0
            controller.depth_sp = 1.0
            controller.pub = Mock()
            for tick in range(20):
                now = 1.0 + tick * 0.05 + (tick % 2) * 1e-5
                controller.now = lambda value=now: value
                controller.state_rx = now
                controller.periodic()
            self.assertEqual(controller.pub.publish.call_count, 20)
        finally:
            controller.destroy_node()

    def test_tuning_enabled_accepts_startup_numeric_overrides(self):
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from robosub_control.utils.tuning import TunableNumbers

        node = Node(
            "override_test",
            parameter_overrides=[
                Parameter("kp", value=2.5),
            ],
        )
        try:
            registry = TunableNumbers(node)
            number = registry.number("kp", 1.0)
            self.assertEqual(number.get(), 2.5)
        finally:
            node.destroy_node()

    def test_http_requires_token_and_uses_real_parameter_services(self):
        from robosub_bringup.tuning_menu import make_handler

        server = ThreadingHTTPServer(
            ("127.0.0.1", 0), make_handler(self.browser, "__TOKEN__", "test")
        )
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        url = f"http://127.0.0.1:{server.server_port}/api/values"
        body = json.dumps(
            {
                "node": "/tuning_test",
                "parameters": {"kp": 3.0},
            }
        ).encode()
        try:
            with self.assertRaises(HTTPError) as error:
                urlopen(Request(url, data=body), timeout=5)
            self.assertEqual(error.exception.code, 403)
            request = Request(
                url,
                data=body,
                headers={
                    "Content-Type": "application/json",
                    "X-Tuning-Token": "test",
                },
            )
            with urlopen(request, timeout=5) as response:
                self.assertTrue(json.load(response)["successful"])
            self.wait_for(lambda: self.kp.get() == 3.0)
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=3)


if __name__ == "__main__":
    unittest.main()

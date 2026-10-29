"""Gamepad mapping, loss of input, HTTP bridge and ROS ownership checks."""

from http.server import ThreadingHTTPServer
import json
import math
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

from robosub_control.teleop.gamepad import ZERO, GamepadControl  # noqa: E402
from robosub_bringup.gamepad_bridge import BrowserGamepad  # noqa: E402

try:
    import rclpy
except ImportError:
    rclpy = None


class GamepadTests(unittest.TestCase):
    def buttons(self, source, pressed):
        index = 5 if source == "browser" else 10
        values = [0] * (index + 1)
        values[index] = int(pressed)
        return values

    def enabled_control(self, source="browser", scale=0.5):
        control = GamepadControl(source, scale)
        control.accept([0.0] * 6, self.buttons(source, False), 0.0)
        control.accept([0.0] * 6, self.buttons(source, True), 0.01)
        self.assertTrue(control.enabled)
        return control

    def test_stick_directions_and_layouts_match(self):
        for source, sign in (("browser", -1), ("local", 1)):
            for axis, output in ((1, 0), (0, 1), (3, 2), (2, 4)):
                for direction in (-1, 1):
                    control = self.enabled_control(source)
                    axes = [0.0] * 6
                    axes[axis] = sign * direction
                    control.accept(axes, self.buttons(source, False), 0.05)
                    expected = [0.0] * 5
                    expected[output] = 0.5 * direction
                    self.assertEqual(control.sample(0.1), tuple(expected))

    def test_press_edges_toggle_and_holding_does_not_repeat(self):
        control = self.enabled_control()
        pressed = self.buttons("browser", True)
        released = self.buttons("browser", False)
        moving = [0, -1, 0, 0, 0, 0]
        control.accept(moving, pressed, 0.05)
        control.accept(moving, pressed, 0.1)
        control.accept(moving, released, 0.15)
        self.assertGreater(control.sample(0.16)[0], 0)
        control.accept(moving, pressed, 0.2)
        self.assertFalse(control.enabled)
        self.assertEqual(control.sample(0.21), ZERO)
        control.accept(moving, pressed, 0.22)
        self.assertFalse(control.enabled)
        control.accept(moving, released, 0.23)
        control.accept(moving, pressed, 0.24)
        self.assertFalse(control.enabled)  # Enabling requires neutral.

    def test_triggers_roll_right_left_and_cancel(self):
        for source, sign in (("browser", 1), ("local", -1)):
            control = self.enabled_control(source)
            for lt, rt, expected in ((0, 1, 0.5), (1, 0, -0.5), (1, 1, 0)):
                control.accept(
                    [0, 0, 0, 0, sign * lt, sign * rt],
                    self.buttons(source, False),
                    0.1,
                )
                self.assertEqual(control.sample(0.11), (0, 0, 0, expected, 0))

    def test_timeout_and_stop_require_neutral_and_a_new_press(self):
        control = self.enabled_control()
        pressed = self.buttons("browser", True)
        released = self.buttons("browser", False)
        control.accept([0, -1, 0, 0, 0, 0], released, 0.1)
        self.assertEqual(control.sample(0.5), ZERO)
        control.accept([0.0] * 6, pressed, 0.51)
        self.assertFalse(control.enabled)
        control.accept([0.0] * 6, released, 0.52)
        control.accept([0.0] * 6, pressed, 0.53)
        self.assertTrue(control.enabled)
        control.stop()
        control.accept([0.0] * 6, pressed, 0.54)
        self.assertFalse(control.enabled)
        self.assertEqual(control.sample(0.1), ZERO)

    def test_trigger_roll_reaches_rd_and_ld_usb_slots(self):
        from robosub_control.drivers.teensy_usb import m5_power_slots
        from robosub_control.thrusters.thrust_allocation import ThrustAllocator

        allocator = ThrustAllocator()
        control = self.enabled_control()
        for lt, rt, direction in ((0, 1, 1), (1, 0, -1)):
            control.accept(
                [0, 0, 0, 0, lt, rt], self.buttons("browser", False), 0.1
            )
            targets = allocator.allocate(control.sample(0.11)).targets
            slots = m5_power_slots(allocator.commands(targets))
            self.assertEqual(slots[:4], [1500] * 4)
            self.assertGreater((slots[4] - 1500) * direction, 0)
            self.assertLess((slots[6] - 1500) * direction, 0)
            self.assertEqual([slots[5], slots[7]], [1500, 1500])

    def test_deadzone_and_diagonal_bound(self):
        control = self.enabled_control("local", 0.7)
        control.accept([0.01] * 4 + [0, 0], self.buttons("local", False), 0.05)
        self.assertEqual(control.sample(0.1), ZERO)
        control.accept([1.0] * 4 + [0, -1], self.buttons("local", False), 0.15)
        surge, sway, heave, roll, yaw = control.sample(0.2)
        self.assertAlmostEqual(math.hypot(surge, sway), 0.7)
        self.assertEqual((heave, roll, yaw), (0.7, 0.7, 0.7))

    def test_invalid_inputs_clear_enable(self):
        for axes in (
            [0] * 4,
            [float("nan")] * 6,
            [True] * 6,
            [1.1] * 6,
            [0, 0, 0, 0, -1, 0],
        ):
            control = self.enabled_control()
            with self.assertRaises(ValueError):
                control.accept(axes, self.buttons("browser", False), 0.1)
            self.assertEqual(control.sample(0.11), ZERO)
            self.assertFalse(control.enabled)
        for scale in (0, 1.1, float("nan"), True):
            with self.assertRaises(ValueError):
                GamepadControl("browser", scale)


class BrowserBridgeTests(unittest.TestCase):
    def frame(self, **changes):
        return {
            "session": "one",
            "sequence": 0,
            "axes": [0.0] * 6,
            "held": False,
            **changes,
        }

    def test_late_packets_cannot_override_stop_or_second_operator(self):
        publish = Mock()
        bridge = BrowserGamepad(publish)
        with patch(
            "robosub_bringup.gamepad_bridge.time.monotonic", return_value=1
        ):
            bridge.update(self.frame())
            bridge.update(self.frame(sequence=2))
            with self.assertRaises(ValueError):
                bridge.update(self.frame(sequence=1, held=True))
            with self.assertRaises(ValueError):
                bridge.update(self.frame(session="two"))
        self.assertEqual(publish.call_count, 2)
        with patch(
            "robosub_bringup.gamepad_bridge.time.monotonic", return_value=2
        ):
            with self.assertRaises(ValueError):
                bridge.update(self.frame(sequence=3, held=True))
            bridge.update(self.frame(session="two"))
        self.assertEqual(publish.call_count, 3)

    def test_explicit_page_stop_is_a_reset_not_a_bumper_release(self):
        publish = Mock()
        bridge = BrowserGamepad(publish)
        bridge.update(self.frame())
        bridge.update(self.frame(sequence=1, stop=True))
        publish.assert_called_with([], False)
        with self.assertRaises(ValueError):
            bridge.update(self.frame(sequence=0, held=True))

    def test_rejects_unmapped_or_nonfinite_browser_frames(self):
        publish = Mock()
        bridge = BrowserGamepad(publish)
        for changes in (
            {"axes": [float("nan")] * 6},
            {"axes": [0.0] * 5},
            {"session": ""},
            {"sequence": True},
            {"held": 1},
            {"held": True},
            {"axes": [1.0] * 6},
        ):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                bridge.update(self.frame(**changes))
        publish.assert_not_called()


@unittest.skipIf(rclpy is None, "ROS Humble is not installed")
class GamepadRosTests(unittest.TestCase):
    def test_joystick_mixer_usb_mapping_limit_and_loss_of_input(self):
        from rclpy.executors import SingleThreadedExecutor
        from robosub_control.drivers.teensy_actuators import TeensyActuators
        from robosub_control.teleop.xbox_teleop import XboxTeleop
        from robosub_control.thrusters.thruster_mixer import ThrusterMixer
        from sensor_msgs.msg import Joy

        os.environ["ROS_DOMAIN_ID"] = "198"
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
        rclpy.init(args=["--ros-args", "-p", "max_power:=0.2"])
        received = []
        bridge = SimpleNamespace(
            connect=lambda: {"rs485_serial": 1, "max_delta_us": 500},
            command=lambda values, enabled=False: received.append(
                (list(values), enabled)
            ),
            close=Mock(),
        )
        with patch(
            "robosub_control.drivers.teensy_actuators.TeensyUsb",
            return_value=bridge,
        ):
            driver = TeensyActuators()
        teleop = XboxTeleop()
        mixer = ThrusterMixer()
        executor = SingleThreadedExecutor()
        for node in (driver, teleop, mixer):
            executor.add_node(node)

        def spin_until(condition):
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.01)
                if condition():
                    return
            self.fail("Expected USB command did not arrive")

        try:
            spin_until(lambda: driver.ready)
            teleop.on_joy(Joy(axes=[0.0] * 6, buttons=[0] * 6))
            teleop.on_joy(Joy(axes=[0.0] * 6, buttons=[0] * 5 + [1]))
            teleop.on_joy(
                Joy(
                    axes=[0.0, -1.0, 0.0, -1.0, 0.0, 0.0], buttons=[0] * 5 + [1]
                )
            )
            spin_until(lambda: received and received[-1][1])
            values, _ = received[-1]
            self.assertTrue(all(abs(value - 1500) <= 100 for value in values))
            self.assertGreater(values[0], 1500)  # FR forward.
            self.assertGreater(values[1], 1500)  # FL forward.
            self.assertLess(values[2], 1500)  # BR reverse.
            self.assertLess(values[3], 1500)  # BL reverse.
            self.assertLess(values[4], 1500)  # RD upward.
            self.assertLess(values[6], 1500)  # LD upward, motor ID 6.
            self.assertEqual([values[5], values[7]], [1500, 1500])
            executor.remove_node(teleop)
            executor.remove_node(mixer)
            spin_until(lambda: received[-1] == ([1500] * 8, False))
            self.assertFalse(driver.ready)
            # Delayed nonneutral targets cannot re-enable a timed-out driver.
            from std_msgs.msg import Float64MultiArray, MultiArrayDimension

            target = Float64MultiArray(data=[0.1] * 6)
            target.layout.dim = [
                MultiArrayDimension(label="FR,FL,BR,BL,RD,LD", size=6, stride=6)
            ]
            driver.on_targets(target)
            driver.periodic()
            self.assertEqual(received[-1], ([1500] * 8, False))
            target.data = [0.0] * 6
            driver.on_targets(target)
            self.assertTrue(driver.ready)
            target.layout.dim[0].label = "FL,FR,BR,BL,LD,RD"
            driver.on_targets(target)
            driver.periodic()
            self.assertFalse(driver.ready)
            self.assertEqual(received[-1], ([1500] * 8, False))
        finally:
            executor.shutdown()
            for node in (driver, teleop, mixer):
                node.destroy_node()
            rclpy.shutdown()

    def test_http_to_m5_preview_watchdog_and_manual_ownership(self):
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from robosub_bringup.tuning_menu import ParameterBrowser, make_handler
        from robosub_control.teleop.xbox_teleop import XboxTeleop
        from robosub_control.thrusters.actuator_commands import ActuatorCommands
        from robosub_control.autonomy.depth_heading_controller import (
            DepthHeadingController,
        )
        from robosub_control.thrusters.thruster_mixer import ThrusterMixer

        os.environ["ROS_DOMAIN_ID"] = "198"
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
        rclpy.init(
            args=[
                "--ros-args",
                "-p",
                "teleop_source:=browser",
                "-p",
                "manual_control:=true",
                "-r",
                "/sub/cmd_vel:=/test/xbox_cmd",
                "-r",
                "/operator/joy:=/test/xbox_joy",
            ]
        )
        nodes = [
            ParameterBrowser(),
            XboxTeleop(),
            ThrusterMixer(),
            ActuatorCommands(),
            DepthHeadingController(),
            Node("xbox_test"),
        ]
        browser, teleop, mixer, actuator, controller, observer = nodes
        executor = SingleThreadedExecutor()
        for node in nodes:
            executor.add_node(node)
        commands = []
        observer.create_subscription(
            Twist, "/test/xbox_cmd", lambda value: commands.append(value), 10
        )
        worker = threading.Thread(target=executor.spin, daemon=True)
        worker.start()
        server = ThreadingHTTPServer(
            ("127.0.0.1", 0),
            make_handler(browser, "__TOKEN__", "test", "// gamepad script"),
        )
        server_worker = threading.Thread(
            target=server.serve_forever, daemon=True
        )
        server_worker.start()
        address = f"http://127.0.0.1:{server.server_port}"

        def wait_for(condition):
            deadline = time.monotonic() + 4
            while time.monotonic() < deadline:
                if condition():
                    return
                time.sleep(0.01)
            self.fail("Gamepad ROS condition did not arrive")

        def post(sequence, held=False, axes=None, token="test", stop=False):
            body = {
                "session": "test",
                "sequence": sequence,
                "held": held,
                "axes": axes or [0.0] * 6,
                "stop": stop,
            }
            request = Request(
                address + "/api/gamepad",
                data=json.dumps(body).encode(),
                headers={"X-Tuning-Token": token},
            )
            with urlopen(request, timeout=3) as response:
                return json.load(response)

        try:
            wait_for(lambda: commands)
            with self.assertRaises(HTTPError) as error:
                post(0, token="wrong")
            self.assertEqual(error.exception.code, 403)
            post(0)
            wait_for(lambda: teleop.control.ready)
            post(1, held=True)
            wait_for(lambda: teleop.control.enabled)
            post(2, axes=[0.0, -1.0, 0.5, -0.5, 0.0, 0.0])
            wait_for(lambda: commands[-1].linear.x > 0.4)
            wait_for(
                lambda: (
                    browser.operator.snapshot()["preview"]
                    and any(
                        abs(value) > 0.1
                        for value in browser.operator.snapshot()["preview"]
                    )
                )
            )
            self.assertLess(commands[-1].angular.z, 0.0)
            self.assertGreater(commands[-1].linear.z, 0.0)
            post(3, stop=True)
            wait_for(lambda: not teleop.control.enabled)
            wait_for(lambda: commands[-1].linear.x == 0.0)
            wait_for(
                lambda: browser.operator.snapshot()["preview"] == [0.0] * 6
            )
            self.assertFalse(teleop.control.ready)
            self.assertFalse(actuator.force_pubs)
            controller.pub = Mock()
            controller.state = Odometry()
            controller.state.pose.pose.orientation.w = 1.0
            controller.state_rx = controller.now()
            controller.depth_sp = 1.0
            controller.periodic()
            controller.pub.publish.assert_not_called()
            with urlopen(address + "/gamepad.js", timeout=3) as response:
                self.assertIn(b"gamepad script", response.read())
        finally:
            server.shutdown()
            server.server_close()
            server_worker.join(timeout=3)
            executor.shutdown()
            worker.join(timeout=3)
            for node in nodes:
                node.destroy_node()
            rclpy.shutdown()

    def test_input_watchdog_uses_wall_time_without_sim_clock(self):
        from geometry_msgs.msg import Twist
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from robosub_control.teleop.xbox_teleop import XboxTeleop
        from sensor_msgs.msg import Joy

        os.environ["ROS_DOMAIN_ID"] = "198"
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
        rclpy.init(
            args=[
                "--ros-args",
                "-p",
                "use_sim_time:=true",
                "-r",
                "/sub/cmd_vel:=/test/paused_cmd",
            ]
        )
        teleop = XboxTeleop()
        observer = Node("paused_gamepad_test")
        executor = SingleThreadedExecutor()
        executor.add_node(teleop)
        executor.add_node(observer)
        commands = []
        observer.create_subscription(
            Twist, "/test/paused_cmd", lambda value: commands.append(value), 10
        )
        try:
            teleop.on_joy(Joy(axes=[0.0] * 6, buttons=[0] * 6))
            teleop.on_joy(Joy(axes=[0.0] * 6, buttons=[0] * 5 + [1]))
            teleop.on_joy(
                Joy(axes=[0.0, -1.0, 0.0, 0.0, 0.0, 0.0], buttons=[0] * 5 + [1])
            )
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.02)
                if (
                    any(command.linear.x > 0 for command in commands)
                    and commands[-1].linear.x == 0
                ):
                    break
            else:
                self.fail("Wall-time watchdog did not stop with /clock absent")
        finally:
            executor.shutdown()
            teleop.destroy_node()
            observer.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()

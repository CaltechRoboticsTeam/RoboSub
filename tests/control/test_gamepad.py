"""Gamepad mapping, loss of input, HTTP bridge and ROS ownership checks."""

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

from robosub_control.teleop.gamepad import (  # noqa: E402
    ZERO,
    GamepadControl,
    validate_buttons,
)

try:
    import rclpy
except ImportError:
    rclpy = None


class GamepadTests(unittest.TestCase):
    def test_enable_accepts_moving_sticks_and_only_disable_turns_it_off(self):
        control = GamepadControl("browser")
        moving = [0, -1, 0, 0, 0, 0]
        control.accept(moving, [1], 0.0)
        self.assertTrue(control.enabled)
        self.assertGreater(control.sample(0.01)[0], 0)
        control.accept(moving, [1], 0.05)
        self.assertGreater(control.sample(0.06)[0], 0)
        control.accept(moving, [0], 0.1)
        self.assertFalse(control.enabled)
        self.assertEqual(control.sample(0.11), ZERO)

        for browser_index, local_index in enumerate((0, 1, 2, 3, 9, 10, 4, 6)):
            actions = [int(index == browser_index) for index in range(8)]
            control.accept([0] * 6, [1, *actions], 1.0)
            self.assertEqual(control.buttons, tuple(actions))
            local = GamepadControl("local")
            local.set_enabled(True)
            sdl = [int(index == local_index) for index in range(15)]
            local.accept([0] * 6, sdl, 1.0)
            self.assertEqual(local.buttons, tuple(actions))
        self.assertEqual(validate_buttons([0] * 4), (0,) * 8)


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
            teleop.on_joy(
                Joy(axes=[0.0, -1.0, 0.0, -1.0, 0.0, 0.0], buttons=[1])
            )
            spin_until(lambda: received and received[-1][1])
            values, _ = received[-1]
            self.assertTrue(all(abs(value - 1500) <= 100 for value in values))
            self.assertLess(values[0], 1500)  # FR forward, inverted polarity.
            self.assertLess(values[1], 1500)  # FL forward, inverted polarity.
            self.assertLess(values[2], 1500)  # BR reverse.
            self.assertLess(values[3], 1500)  # BL reverse.
            self.assertLess(values[4], 1500)  # RD upward.
            self.assertLess(values[6], 1500)  # LD upward, motor ID 6.
            self.assertEqual([values[5], values[7]], [1500, 1500])
            executor.remove_node(teleop)
            executor.remove_node(mixer)
            spin_until(lambda: received[-1] == ([1500] * 8, False))
            # Fresh targets resume directly after a stale-input neutral output.
            from std_msgs.msg import Float64MultiArray, MultiArrayDimension

            target = Float64MultiArray(data=[0.1] * 6)
            target.layout.dim = [
                MultiArrayDimension(label="FR,FL,BR,BL,RD,LD", size=6, stride=6)
            ]
            driver.on_targets(target)
            driver.periodic()
            self.assertTrue(received[-1][1])
            target.data = [0.0] * 6
            driver.on_targets(target)
            target.layout.dim[0].label = "FL,FR,BR,BL,LD,RD"
            driver.on_targets(target)
            driver.periodic()
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
        from robosub_bringup.driver_station import DriverStation, make_handler
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
                "-r",
                "/sub/cmd_vel:=/test/xbox_cmd",
                "-r",
                "/operator/joy:=/test/xbox_joy",
            ]
        )
        nodes = [
            DriverStation(),
            XboxTeleop(),
            ThrusterMixer(),
            ActuatorCommands(),
            Node("xbox_test"),
        ]
        browser, teleop, mixer, actuator, observer = nodes
        executor = SingleThreadedExecutor()
        for node in nodes:
            executor.add_node(node)
        commands = []
        observer.create_subscription(
            Twist, "/test/xbox_cmd", lambda value: commands.append(value), 10
        )
        state_pub = observer.create_publisher(Odometry, "/sub/state", 10)

        def publish_state():
            state = Odometry()
            state.header.stamp = observer.get_clock().now().to_msg()
            state.pose.pose.orientation.w = 1.0
            state.pose.pose.position.z = -1.0
            state_pub.publish(state)

        observer.create_timer(0.02, publish_state)
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

        def post(sequence, enabled=True, axes=None, token="test", buttons=None):
            body = {
                "session": "test",
                "sequence": sequence,
                "enabled": enabled,
                "axes": axes or [0.0] * 6,
                "buttons": buttons or [0] * 4,
            }
            request = Request(
                address + "/api/gamepad",
                data=json.dumps(body).encode(),
                headers={"X-Driver-Station-Token": token},
            )
            with urlopen(request, timeout=3) as response:
                return json.load(response)

        try:
            wait_for(lambda: commands)
            with self.assertRaises(HTTPError) as error:
                post(0, token="wrong")
            self.assertEqual(error.exception.code, 403)
            post(0)
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
            post(3, enabled=False)
            wait_for(lambda: not teleop.control.enabled)
            wait_for(lambda: commands[-1].linear.x == 0.0)
            wait_for(
                lambda: browser.operator.snapshot()["preview"] == [0.0] * 6
            )
            self.assertFalse(actuator.force_pubs)
            self.assertIsInstance(teleop, DepthHeadingController)
            self.assertEqual(observer.count_publishers("/test/xbox_cmd"), 1)
            wait_for(lambda: teleop.state is not None)
            post(4, buttons=[0, 0, 1, 0])
            wait_for(lambda: teleop.depth_sp == 1.5)
            post(5)
            wait_for(lambda: teleop.depth_sp is None)
            wait_for(lambda: commands[-1].linear.z == 0.0)
            self.assertTrue(teleop.control.enabled)
            post(6, buttons=[0, 0, 0, 0, 0, 0, 1, 0])
            wait_for(lambda: teleop.measurement_button == 6)
            post(7)
            teleop.next_step_time = 0.0
            post(8, buttons=[0, 0, 0, 0, 0, 1, 0, 0])
            wait_for(lambda: commands[-1].linear.z == 0.01)
            self.assertEqual(commands[-1].linear.x, 0.0)
            self.assertEqual(commands[-1].angular.x, 0.0)
            post(9, enabled=False)
            wait_for(lambda: not teleop.control.enabled)
            wait_for(lambda: commands[-1].linear.z == 0.0)
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
            teleop.on_joy(
                Joy(axes=[0.0, -1.0, 0.0, 0.0, 0.0, 0.0], buttons=[1])
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

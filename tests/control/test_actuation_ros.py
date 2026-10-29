"""ROS-only actuator pipeline checks, isolated from physical outputs."""

import os
from pathlib import Path
import sys
import threading
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "robosub_ws/src/robosub_control"))
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "tools/diagnostics"))
sys.path.insert(0, str(ROOT / "robosub_ws/src/robosub_sim"))

try:
    import rclpy
except ImportError:
    rclpy = None


@unittest.skipIf(rclpy is None, "ROS Humble is not installed")
class ActuationRosTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ["ROS_DOMAIN_ID"] = "198"
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
        rclpy.init(
            args=[
                "--ros-args",
                "-p",
                "simulation:=true",
                "-p",
                "vehicle_profile:=future_t200",
                "-r",
                "/sub/cmd_vel:=/test/cmd_vel",
                "-r",
                "/sub/thruster_forces:=/test/thruster_forces",
                "-r",
                "/sub/esc/pwm_preview:=/test/pwm_preview",
            ]
        )

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        from geometry_msgs.msg import Twist
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from robosub_control.config import future_t200 as constants
        from robosub_control.thrusters.actuator_commands import ActuatorCommands
        from robosub_control.thrusters.thruster_mixer import ThrusterMixer
        from std_msgs.msg import Float64, UInt16MultiArray

        self.initialization = patch.object(constants, "ESC_NEUTRAL_INIT_S", 0.0)
        self.initialization.start()
        self.mixer = ThrusterMixer()
        self.esc = ActuatorCommands()
        self.observer = Node("actuator_test_observer")
        self.pulses = None
        self.applied = {}
        self.target = Twist()
        self.publisher = self.observer.create_publisher(
            Twist, "/test/cmd_vel", 1
        )
        self.observer.create_subscription(
            UInt16MultiArray,
            "/test/pwm_preview",
            lambda message: setattr(self, "pulses", list(message.data)),
            10,
        )
        for name, topic in constants.THRUSTER_TOPIC_IDS.items():
            self.observer.create_subscription(
                Float64,
                f"/sim/thrust/{topic}",
                lambda message, key=name: self.applied.__setitem__(
                    key, message.data
                ),
                10,
            )
        self.sender = self.observer.create_timer(
            0.02, lambda: self.publisher.publish(self.target)
        )
        self.executor = SingleThreadedExecutor()
        for node in (self.mixer, self.esc, self.observer):
            self.executor.add_node(node)
        self.worker = threading.Thread(target=self.executor.spin, daemon=True)
        self.worker.start()

    def tearDown(self):
        self.executor.shutdown()
        self.worker.join(timeout=3)
        for node in (self.mixer, self.esc, self.observer):
            node.destroy_node()
        self.initialization.stop()

    def wait_for(self, condition, timeout=3):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if condition():
                return
            time.sleep(0.01)
        self.fail("Actuator topic condition did not arrive")

    def test_forward_command_and_independent_esc_watchdog(self):
        self.target.linear.x = 0.5
        self.wait_for(
            lambda: (
                self.pulses is not None
                and self.pulses[0] > 1500
                and self.pulses[2] < 1500
            )
        )
        self.wait_for(lambda: len(self.applied) == 6)
        self.assertGreater(self.applied["FL"], 0)
        self.assertLess(self.applied["BR"], 0)
        self.assertEqual(self.applied["LD"], 0)
        # Remove the mixer too, so neutral cannot come from its watchdog.
        self.sender.cancel()
        self.executor.remove_node(self.mixer)
        self.wait_for(lambda: self.pulses == [1500] * 6, timeout=2)
        self.wait_for(
            lambda: all(value == 0 for value in self.applied.values())
        )

    def test_upward_heave_uses_reverse_branch_on_downward_mounts(self):
        self.target.linear.z = 0.4
        self.wait_for(
            lambda: (
                self.pulses is not None
                and self.pulses[4] < 1500
                and self.pulses[5] < 1500
            )
        )
        self.assertEqual(self.pulses[:4], [1500] * 4)

    def test_invalid_command_stops_every_channel(self):
        self.target.linear.x = 0.5
        self.wait_for(lambda: self.pulses is not None and self.pulses[0] > 1500)
        self.target.linear.x = float("nan")
        self.wait_for(lambda: self.pulses == [1500] * 6)

    def test_initialization_policy_holds_neutral(self):
        from robosub_control.config import future_t200 as constants

        self.target.linear.x = 0.5
        with patch.object(constants, "ESC_NEUTRAL_INIT_S", 60.0):
            self.wait_for(lambda: self.pulses == [1500] * 6)
            self.assertEqual(self.pulses, [1500] * 6)
        self.wait_for(lambda: self.pulses is not None and self.pulses[0] > 1500)


@unittest.skipIf(rclpy is None, "ROS Humble is not installed")
class M5ActuationRosTests(unittest.TestCase):
    def test_normalized_output_and_watchdog(self):
        from geometry_msgs.msg import Twist
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from robosub_control.thrusters.actuator_commands import ActuatorCommands
        from robosub_control.thrusters.thruster_mixer import ThrusterMixer
        from std_msgs.msg import Float64MultiArray

        os.environ["ROS_DOMAIN_ID"] = "198"
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
        rclpy.init(args=["--ros-args", "-r", "/sub/cmd_vel:=/test/m5_cmd"])
        mixer = ThrusterMixer()
        output = ActuatorCommands()
        observer = Node("m5_test_observer")
        executor = SingleThreadedExecutor()
        received = []
        publisher = observer.create_publisher(Twist, "/test/m5_cmd", 1)
        observer.create_subscription(
            Float64MultiArray,
            "/sub/m5/power_preview",
            lambda message: received.append(message),
            10,
        )
        for node in (mixer, output, observer):
            executor.add_node(node)

        def wait_for(condition):
            deadline = time.monotonic() + 4
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.02)
                if condition():
                    return
            self.fail("M5 preview condition did not arrive")

        try:
            self.assertTrue(output.allocator.is_m5)
            self.assertFalse(output.force_pubs)
            wait_for(lambda: received and list(received[-1].data) == [0.0] * 6)
            target = Twist()
            target.linear.x = 0.5
            sender = observer.create_timer(
                0.02, lambda: publisher.publish(target)
            )
            wait_for(lambda: received and received[-1].data[0] > 0.1)
            message = received[-1]
            self.assertEqual(message.layout.dim[0].label, "FR,FL,BR,BL,RD,LD")
            self.assertTrue(all(-1 <= value <= 1 for value in message.data))
            self.assertLess(message.data[2], 0)
            sender.cancel()
            executor.remove_node(mixer)
            wait_for(lambda: received and list(received[-1].data) == [0.0] * 6)
            bad = Float64MultiArray(data=[float("nan")] * 6)
            output.on_targets(bad)
            self.assertIsNone(output.last_rx)
        finally:
            executor.shutdown()
            for node in (mixer, output, observer):
                node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()

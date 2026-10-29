"""Browser input and read-only command previews for the developer menu."""

import threading
import time

from geometry_msgs.msg import Twist
from robosub_sim import constants
from robosub_sim.tuning import fixed_parameter
from robosub_sim.vehicle_profiles import profile_from_node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float64MultiArray, UInt16MultiArray

from robosub_bringup.gamepad_bridge import BrowserGamepad


class OperatorPanel:
    def __init__(self, node):
        self.node = node
        self.source = fixed_parameter(node, "teleop_source", "off")
        if self.source not in ("off", "browser", "local"):
            raise ValueError("teleop_source must be off, browser or local")
        self.profile = profile_from_node(node)
        self.simulation = fixed_parameter(node, "simulation", False)
        self.transport = fixed_parameter(node, "actuator_transport", "preview")
        self.max_effort = fixed_parameter(
            node, "max_effort", constants.GAMEPAD_MAX_EFFORT
        )
        self.lock = threading.Lock()
        self.values = {}
        self.bridge = None
        if self.source == "off":
            return
        node.create_subscription(Twist, "/sub/cmd_vel", self.on_command, 1)
        node.create_subscription(
            Bool,
            "/operator/enabled",
            lambda message: self.store("enabled", message.data),
            1,
        )
        node.create_subscription(
            Float64MultiArray
            if self.profile.ACTUATOR_KIND == "m5"
            else UInt16MultiArray,
            self.profile.SENT_TOPIC
            if self.transport == "teensy_usb"
            else self.profile.PREVIEW_TOPIC,
            lambda message: self.store("preview", list(message.data)),
            1,
        )
        if self.source == "browser":
            self.publisher = node.create_publisher(Joy, "/operator/joy", 1)
            self.bridge = BrowserGamepad(self.publish_joy)

    def on_command(self, message):
        self.store(
            "command",
            [
                message.linear.x,
                message.linear.y,
                message.linear.z,
                message.angular.z,
            ],
        )

    def store(self, key, value):
        with self.lock:
            self.values[key] = (time.monotonic(), value)

    def publish_joy(self, axes, held):
        message = Joy()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.axes = [float(value) for value in axes]
        index = constants.GAMEPAD_ENABLE_BUTTON["browser"]
        message.buttons = [0] * (index + 1)
        message.buttons[index] = int(held)
        self.publisher.publish(message)

    def update(self, data):
        if self.bridge is None:
            raise ValueError("Browser controller input is disabled")
        return self.bridge.update(data)

    def snapshot(self):
        is_m5 = self.profile.ACTUATOR_KIND == "m5"
        now = time.monotonic()
        with self.lock:
            values = {
                key: value if now - received <= 1.0 else None
                for key, (received, value) in self.values.items()
            }
        return {
            "source": self.source,
            "vehicle": self.profile.NAME,
            "simulation": self.simulation,
            "actuator_transport": self.transport,
            "max_effort": self.max_effort,
            "deadzone": constants.GAMEPAD_DEADZONE,
            "enabled": values.get("enabled", False),
            "command": values.get("command"),
            "preview": values.get("preview"),
            "preview_ids": [
                self.profile.M5_MOTOR_IDS[name]
                for name in self.profile.THRUSTER_NAMES
            ]
            if is_m5
            else list(range(len(self.profile.THRUSTER_NAMES))),
            "preview_names": list(self.profile.THRUSTER_NAMES)
            if is_m5
            else [
                name
                for name, _ in sorted(
                    self.profile.THRUSTER_CHANNELS.items(),
                    key=lambda item: item[1],
                )
            ],
            "preview_units": "power fraction" if is_m5 else "PWM us",
        }

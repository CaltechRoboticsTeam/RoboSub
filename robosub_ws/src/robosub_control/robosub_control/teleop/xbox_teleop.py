"""Read selected Xbox input periodically and publish manual body efforts."""

import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from robosub_control.config import constants
from robosub_control.utils.tuning import fixed_parameter
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

from robosub_control.teleop.gamepad import GamepadControl


class XboxTeleop(Node):
    def __init__(self):
        super().__init__("xbox_teleop")
        source = fixed_parameter(self, "source", "browser")
        scale = fixed_parameter(
            self, "max_effort", constants.GAMEPAD_MAX_EFFORT
        )
        self.control = GamepadControl(source, scale)
        self.pub = self.create_publisher(Twist, "/sub/cmd_vel", 1)
        self.enabled_pub = self.create_publisher(Bool, "/operator/enabled", 1)
        self.create_subscription(Joy, "/operator/joy", self.on_joy, 1)
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(
            1.0 / constants.GAMEPAD_PUBLISH_RATE_HZ,
            self.periodic,
            clock=self.wall_clock,
        )
        self.get_logger().info(
            f"Xbox {source} input: center sticks and release controls; "
            "press RB to toggle movement. "
            f"Maximum normalized effort: {scale:.0%}."
        )

    def on_joy(self, message):
        if not message.axes and not message.buttons:
            self.control.stop()
            return
        try:
            self.control.accept(message.axes, message.buttons, time.monotonic())
        except ValueError as error:
            self.get_logger().error(str(error), throttle_duration_sec=2.0)

    def periodic(self):
        surge, sway, heave, roll, yaw = self.control.sample(time.monotonic())
        command = Twist()
        command.linear.x = surge
        command.linear.y = sway
        command.linear.z = heave
        command.angular.x = roll
        command.angular.z = yaw
        self.pub.publish(command)
        self.enabled_pub.publish(Bool(data=self.control.enabled))


def main():
    rclpy.init()
    node = XboxTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.control.stop()
        if rclpy.ok():
            node.periodic()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

"""Manual Xbox driving and momentary tuning moves share one command owner."""

import math
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.clock import Clock, ClockType
from robosub_control.autonomy.depth_heading_controller import (
    DepthHeadingController,
    wrap_pi,
)
from robosub_control.autonomy.state_estimator import q_to_rpy
from robosub_control.config import constants
from robosub_control.utils.tuning import fixed_parameter
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Empty, Float64, String

from robosub_control.teleop.gamepad import GamepadControl, NO_BUTTONS


class XboxTeleop(DepthHeadingController):
    def __init__(self):
        super().__init__()
        self.destroy_timer(self.control_timer)
        source = fixed_parameter(self, "source", "browser")
        self.source = source
        scale = fixed_parameter(
            self, "max_effort", constants.GAMEPAD_MAX_EFFORT
        )
        self.control = GamepadControl(source, scale)
        self.command_limit = scale
        self.active_button = None
        self.previous_buttons = NO_BUTTONS
        self.state_wall_rx = None
        self.assist_message = "Manual input"
        self.enabled_pub = self.create_publisher(Bool, "/operator/enabled", 1)
        self.maneuver_pub = self.create_publisher(
            String, "/operator/maneuver", 1
        )
        self.create_subscription(Joy, "/operator/joy", self.on_joy, 1)
        if source == "local":
            self.create_subscription(
                Bool, "/operator/input_enabled", self.on_enabled, 1
            )
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(
            1.0 / self.rate,
            self.periodic,
            clock=self.wall_clock,
        )
        self.get_logger().info(
            f"Xbox {source} input: use the page Enable/Disable buttons. "
            f"Maximum normalized effort: {scale:.0%}. "
            "Hold A/B/X/Y for assisted tuning; release ends the maneuver."
        )

    def on_enabled(self, message):
        self.control.set_enabled(message.data)
        if not message.data:
            self.periodic()

    def on_joy(self, message):
        try:
            self.control.accept(message.axes, message.buttons, time.monotonic())
        except ValueError as error:
            self.get_logger().error(str(error), throttle_duration_sec=2.0)
            self.periodic()
        else:
            if not self.control.enabled:
                self.periodic()

    def on_state(self, message):
        previous = self.state_freshness.stamp
        super().on_state(message)
        if self.state_freshness.stamp != previous:
            self.state_wall_rx = time.monotonic()

    def on_cancel(self, message):
        super().on_cancel(message)
        self.active_button = None
        self.assist_message = "Manual input: assistance cancelled"
        self.last_control_time = None

    def feedback_problem(self, button, wall_now):
        if (
            self.state is None
            or not self.state_freshness.fresh(self.now())
            or self.state_wall_rx is None
            or not 0 <= wall_now - self.state_wall_rx < self.state_timeout
        ):
            return "No fresh depth/IMU estimate"
        if button == 0 and self.state.pose.covariance[0] < 0.0:
            return "A needs fresh DVL position"
        return None

    def start_assistance(self, button, wall_now):
        problem = self.feedback_problem(button, wall_now)
        if problem:
            self.assist_message = problem + "; release and press to retry"
            self.get_logger().warn(self.assist_message)
            return
        self.on_cancel(Empty())
        depth = -self.state.pose.pose.position.z
        q = self.state.pose.pose.orientation
        _, _, yaw = q_to_rpy((q.w, q.x, q.y, q.z))
        self.on_depth_sp(Float64(data=depth + (0.5 if button == 2 else 0.0)))
        self.on_roll_sp(Float64(data=0.0))
        if button in (0, 3):
            target = wrap_pi(yaw - math.pi / 2) if button == 3 else yaw
            self.on_heading_sp(Float64(data=target))
        if button == 0:
            self.on_move(Float64(data=1.0))
        self.active_button = button
        self.assist_message = (
            "A: move forward 1 m, then hold position/depth",
            "B: hold the depth captured on press",
            "X: hold 0.5 m below the depth captured on press",
            "Y: hold depth and a heading 90 degrees clockwise from press",
        )[button]
        self.get_logger().info(self.assist_message)

    def cancel_assistance(self, reason):
        if self.active_button is not None or any(
            value is not None
            for value in (
                self.depth_sp,
                self.heading_sp,
                self.roll_sp,
                self.pos_sp,
            )
        ):
            self.on_cancel(Empty())
        self.assist_message = reason

    def drive(self, manual, fresh, wall_now):
        if not self.control.enabled or not fresh:
            self.cancel_assistance(
                "Input disabled"
                if not self.control.enabled
                else "Input stale; assistance cancelled"
            )
            self.pub.publish(Twist())
            return
        buttons = self.control.buttons
        rising = tuple(
            pressed and not previous
            for pressed, previous in zip(
                buttons, self.previous_buttons, strict=True
            )
        )
        self.previous_buttons = buttons
        if self.active_button is not None and not buttons[self.active_button]:
            self.cancel_assistance("Manual input: button released")
        if any(manual):
            self.cancel_assistance("Manual input: sticks/triggers take over")
        elif sum(buttons) > 1:
            self.cancel_assistance("Press one maneuver button at a time")
        elif sum(buttons) == 1:
            button = buttons.index(1)
            if rising[button]:
                self.cancel_assistance("Manual input")
                self.start_assistance(button, wall_now)
        if self.active_button is not None:
            problem = self.feedback_problem(self.active_button, wall_now)
            if problem:
                self.cancel_assistance(problem + "; release and press to retry")
            else:
                super().periodic()
                return
        command = Twist()
        command.linear.x, command.linear.y, command.linear.z = manual[:3]
        command.angular.x, command.angular.z = manual[3:]
        self.pub.publish(command)

    def periodic(self):
        wall_now = time.monotonic()
        fresh = self.control.last_rx is not None and (
            0 <= wall_now - self.control.last_rx <= self.control.timeout
        )
        manual = self.control.sample(wall_now)
        self.drive(manual, fresh, wall_now)
        self.enabled_pub.publish(Bool(data=self.control.enabled))
        self.maneuver_pub.publish(String(data=self.assist_message))


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

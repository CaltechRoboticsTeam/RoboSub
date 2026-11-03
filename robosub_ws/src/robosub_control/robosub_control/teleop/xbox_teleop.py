"""Manual driving, held maneuvers, and steady-effort measurements."""

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

from robosub_control.teleop.gamepad import (
    LB,
    RB,
    SELECT,
    START,
    GamepadControl,
    NO_BUTTONS,
)

STEP_INTERVAL_S = 2.0


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
        self.measurement_button = None
        self.measurement_effort = 0.0
        self.measurement_origin = None
        self.next_step_time = 0.0
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
            "Hold A/B/X/Y for assisted tuning. Select: hover measurement; "
            "Start: drag measurement; LB/RB: decrease/increase effort."
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
        self.measurement_button = None
        self.measurement_effort = 0.0
        self.measurement_origin = None
        self.next_step_time = 0.0
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
        if button in (0, START) and self.state.pose.covariance[0] < 0.0:
            return "A/drag measurement needs fresh DVL velocity/position"
        if button in (SELECT, START):
            q = self.state.pose.pose.orientation
            roll, pitch, _ = q_to_rpy((q.w, q.x, q.y, q.z))
            if max(abs(roll), abs(pitch)) > math.radians(self.max_tilt_deg):
                return "Measurement cancelled: attitude limit"
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
        if (
            self.active_button is not None
            or self.measurement_button is not None
            or any(
                value is not None
                for value in (
                    self.depth_sp,
                    self.heading_sp,
                    self.roll_sp,
                    self.pos_sp,
                )
            )
        ):
            self.on_cancel(Empty())
        self.assist_message = reason

    def start_measurement(self, button, wall_now):
        problem = self.feedback_problem(button, wall_now)
        if problem:
            self.assist_message = problem + "; release and press to retry"
            self.get_logger().warn(self.assist_message)
            return
        self.on_cancel(Empty())
        if button == START:
            self.on_depth_sp(Float64(data=-self.state.pose.pose.position.z))
            self.on_roll_sp(Float64(data=0.0))
            q = self.state.pose.pose.orientation
            _, _, yaw = q_to_rpy((q.w, q.x, q.y, q.z))
            self.on_heading_sp(Float64(data=yaw))
            position = self.state.pose.pose.position
            self.measurement_origin = (position.x, position.y, yaw)
        self.measurement_button = button
        self.next_step_time = wall_now + STEP_INTERVAL_S
        self.assist_message = self.measurement_status(wall_now)
        self.get_logger().info(self.assist_message)

    def measurement_status(self, wall_now):
        hover = self.measurement_button == SELECT
        axis = "vertical (+up)" if hover else "forward (+ahead)"
        step = "1%" if hover else "5%"
        wait = max(0.0, self.next_step_time - wall_now)
        return (
            f"{'Hover' if hover else 'Drag'} measurement: "
            f"{axis} effort {self.measurement_effort:+.0%}; "
            f"LB -{step}, RB +{step}; "
            + (f"wait {wait:.1f}s, then tap again" if wait else "ready")
        )

    def measurement_periodic(self, rising, wall_now):
        problem = self.feedback_problem(self.measurement_button, wall_now)
        if problem:
            self.cancel_assistance(problem + "; release and press to retry")
            self.pub.publish(Twist())
            return
        buttons = self.control.buttons
        if (
            buttons[LB] != buttons[RB]
            and (rising[LB] or rising[RB])
            and wall_now >= self.next_step_time
        ):
            step = 0.01 if self.measurement_button == SELECT else 0.05
            effort = self.measurement_effort + (step if buttons[RB] else -step)
            effort = round(
                max(-self.command_limit, min(self.command_limit, effort)), 10
            )
            if effort != self.measurement_effort:
                self.measurement_effort = effort
                self.next_step_time = wall_now + STEP_INTERVAL_S
                self.get_logger().info(self.measurement_status(wall_now))
        self.assist_message = self.measurement_status(wall_now)
        command = Twist()
        if self.measurement_button == SELECT:
            command.linear.z = self.measurement_effort
            self.pub.publish(command)
            return
        command.linear.x = self.measurement_effort
        x, y, heading = self.measurement_origin
        position = self.state.pose.pose.position
        q = self.state.pose.pose.orientation
        _, _, yaw = q_to_rpy((q.w, q.x, q.y, q.z))
        error = -(x - position.x) * math.sin(heading)
        error += (y - position.y) * math.cos(heading)
        velocity = self.state.twist.twist.linear
        difference = yaw - heading
        side_rate = velocity.x * math.sin(difference) + velocity.y * math.cos(
            difference
        )
        dt = (
            1.0 / self.rate
            if self.last_control_time is None
            else self.now() - self.last_control_time
        )
        if 0 < dt <= self.state_timeout:
            command.linear.y = self.sway_pid.step(error, side_rate, dt)
            command.linear.y *= math.cos(difference)
        self.on_manual(command)
        super().periodic()

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
        else:
            requests = [i for i in (0, 1, 2, 3, SELECT, START) if buttons[i]]
            if len(requests) > 1:
                self.cancel_assistance("Press one maneuver button at a time")
            elif requests and rising[requests[0]]:
                button = requests[0]
                if button == self.measurement_button:
                    self.cancel_assistance("Manual input: measurement ended")
                else:
                    self.cancel_assistance("Manual input")
                    if button in (SELECT, START):
                        self.start_measurement(button, wall_now)
                    else:
                        self.start_assistance(button, wall_now)
        if self.measurement_button is not None:
            self.measurement_periodic(rising, wall_now)
            return
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

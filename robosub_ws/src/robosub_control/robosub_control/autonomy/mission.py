#!/usr/bin/env python3
"""Execute mission steps with fresh-state deadlines and cancellation."""

import json
import math

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, Float64, String

from robosub_control.autonomy.depth_heading_controller import wrap_pi
from robosub_control.autonomy.sensor_freshness import (
    SensorFreshness,
    stamp_to_sec,
)
from robosub_control.autonomy.state_estimator import q_to_rpy
from robosub_control.config import constants
from robosub_control.utils.tuning import fixed_parameter


def parse_steps(steps):
    """Validate the complete command list before publishing any setpoints."""
    counts = {
        "dive": 1,
        "heading": 1,
        "move": 1,
        "goto": 2,
        "roll": 1,
        "wait": 1,
        "hold": 1,
        "surface": 0,
    }
    if not steps:
        raise ValueError("mission must contain at least one step")
    parsed = []
    depth_commanded = False
    for step in steps:
        words = step.split()
        if not words or words[0].lower() not in counts:
            raise ValueError(f"unknown mission step: {step!r}")
        cmd = words[0].lower()
        if len(words) - 1 != counts[cmd]:
            raise ValueError(f"{cmd} requires {counts[cmd]} arguments")
        try:
            args = tuple(float(word) for word in words[1:])
        except ValueError as error:
            raise ValueError(f"non-numeric argument in {step!r}") from error
        if not all(math.isfinite(value) for value in args):
            raise ValueError(f"non-finite argument in {step!r}")
        if cmd in ("dive", "wait", "hold") and args[0] < 0.0:
            raise ValueError(f"{cmd} requires a non-negative argument")
        if cmd == "hold" and not depth_commanded:
            raise ValueError("hold requires a preceding dive or surface")
        if cmd in ("dive", "surface"):
            depth_commanded = True
        parsed.append((cmd, args))
    return parsed


class Mission(Node):
    def __init__(self):
        super().__init__("mission")
        for name, default in constants.MISSION_DEFAULTS.items():
            fixed_parameter(self, name, default)
        fixed_parameter(self, "state_timeout", 0.5)
        fixed_parameter(self, "speed_tol", 0.05)
        fixed_parameter(self, "max_tilt_deg", 30.0)
        fixed_parameter(self, "hold_tilt_deg", 10.0)

        def g(name):
            return self.get_parameter(name).value

        self.steps = [str(s) for s in g("mission")]
        self.pos_tol, self.depth_tol = (
            float(g("pos_tol")),
            float(g("depth_tol")),
        )
        self.heading_tol = math.radians(float(g("heading_tol")))
        self.settle, self.step_timeout = (
            float(g("settle")),
            float(g("step_timeout")),
        )
        self.state_timeout = float(g("state_timeout"))
        self.speed_tol = float(g("speed_tol"))
        self.max_tilt_deg = float(g("max_tilt_deg"))
        self.hold_tilt_deg = float(g("hold_tilt_deg"))
        self.startup_error = None
        try:
            self.parsed_steps = parse_steps(self.steps)
            for name, value in (
                ("pos_tol", self.pos_tol),
                ("depth_tol", self.depth_tol),
                ("heading_tol", self.heading_tol),
                ("step_timeout", self.step_timeout),
                ("state_timeout", self.state_timeout),
                ("speed_tol", self.speed_tol),
                ("max_tilt_deg", self.max_tilt_deg),
            ):
                if not math.isfinite(value) or value <= 0.0:
                    raise ValueError(f"{name} must be finite and positive")
            if not math.isfinite(self.settle) or self.settle < 0.0:
                raise ValueError("settle must be finite and non-negative")
            if not 0 < self.hold_tilt_deg < self.max_tilt_deg < 60:
                raise ValueError(
                    "Require 0 < hold_tilt_deg < max_tilt_deg < 60"
                )
        except ValueError as error:
            self.startup_error = str(error)
        self.state_freshness = SensorFreshness(
            self.state_timeout if self.startup_error is None else 0.5
        )

        def p(topic, message_type):
            return self.create_publisher(message_type, topic, 10)

        self.pub_depth = p("/sub/setpoint/depth", Float64)
        self.pub_heading = p("/sub/setpoint/heading", Float64)
        self.pub_pos = p("/sub/setpoint/position", Point)
        self.pub_roll_turns = p("/sub/setpoint/roll_turns", Float64)
        self.pub_roll = p("/sub/setpoint/roll", Float64)
        self.pub_cancel = p("/sub/setpoint/cancel", Empty)
        self.pub_status = self.create_publisher(
            String,
            "/sub/mission/status",
            QoSProfile(
                depth=10,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )
        self.create_subscription(Odometry, "/sub/state", self.on_state, 10)
        self.create_subscription(
            Empty, "/sub/setpoint/cancel", self.on_cancel, 10
        )
        self.create_subscription(
            String, "/sub/control/fault", self.on_fault, 10
        )

        self.state = None
        self.depth_cmd = None
        self.pos_cmd = None
        self.heading_cmd = None  # yaw we are holding, rad
        self.i = -1  # current step index
        self.check = None  # function -> True when the step is done
        self.t_step = self.now()
        self.last_tick = self.t_step
        self.step_started = False
        self.t_inside = None
        self.t_log = 0.0
        self.finished = False
        self.create_timer(constants.MISSION_PERIOD_S, self.periodic)
        self.get_logger().info(f"mission: {self.steps}")

    # ------------------------------------------------------------ helpers
    def on_state(self, msg):
        t = self.now()
        stamp = stamp_to_sec(msg.header.stamp)
        position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular
        values = (
            position.x,
            position.y,
            position.z,
            q.w,
            q.x,
            q.y,
            q.z,
            linear.x,
            linear.y,
            linear.z,
            angular.x,
            angular.y,
            angular.z,
            *msg.pose.covariance,
            *msg.twist.covariance,
        )
        norm = sum(value * value for value in (q.w, q.x, q.y, q.z))
        if not self.state_freshness.fresh(t):
            self.t_inside = None
        if (
            not math.isfinite(norm)
            or abs(norm - 1.0) > 0.01
            or not self.state_freshness.accept(stamp, t, values)
        ):
            self.state = None
            self.t_inside = None
            self.get_logger().warn(
                "invalid or stale mission state; waiting",
                throttle_duration_sec=5.0,
            )
            return
        self.state = msg
        if (
            self.startup_error is None
            and self.i < len(self.steps)
            and self.needs_dvl()
            and msg.pose.covariance[0] < 0.0
        ):
            self.t_inside = None
        if (
            self.startup_error is None
            and 0 <= self.i < len(self.steps)
            and self.parsed_steps[self.i][0] == "hold"
            and not self.holding_targets()
        ):
            self.t_inside = None

    def fail(self, reason):
        """Cancel automatic outputs once; zero effort does not mean parking."""
        self.pub_cancel.publish(Empty())
        self.t_inside = None
        self.finished = True
        self.publish_status("failed", reason=reason)
        self.get_logger().error(f"{reason}; automatic setpoints cancelled")

    def on_fault(self, msg):
        if self.finished:
            return
        try:
            reason = json.loads(msg.data)["reason"]
        except (ValueError, KeyError, TypeError):
            reason = "Controller reported a malformed fault"
        self.fail(reason)

    def on_cancel(self, _msg):
        """An operator cancellation also stops future mission setpoints."""
        if self.finished:
            return
        self.t_inside = None
        self.finished = True
        self.publish_status("cancelled")

    def publish_status(self, status, **details):
        """Publish source-clock events for passive mission observation."""
        event = {
            "status": status,
            "index": self.i,
            "stamp": self.now(),
            **details,
        }
        if 0 <= self.i < len(self.steps):
            event["step"] = self.parsed_steps[self.i][0]
        if self.depth_cmd is not None:
            event["target_depth"] = self.depth_cmd
        if self.heading_cmd is not None:
            event["target_heading"] = self.heading_cmd
        if self.pos_cmd is not None:
            event["target_position"] = list(self.pos_cmd)
        self.pub_status.publish(String(data=json.dumps(event, allow_nan=False)))

    def ready(self, t):
        if self.state is None or not self.state_freshness.fresh(t):
            return False
        return not self.needs_dvl() or self.state.pose.covariance[0] >= 0.0

    def needs_dvl(self):
        cmd, _ = self.parsed_steps[max(0, self.i)]
        return cmd in ("move", "goto") or (
            cmd == "hold" and self.pos_cmd is not None
        )

    def holding_targets(self):
        """Check previously commanded targets without moving their setpoints."""
        x, y, depth, yaw = self.pose()
        q = self.state.pose.pose.orientation
        roll, pitch, _ = q_to_rpy((q.w, q.x, q.y, q.z))
        if (
            max(abs(math.degrees(roll)), abs(math.degrees(pitch)))
            >= self.hold_tilt_deg
        ):
            return False
        if (
            self.depth_cmd is None
            or abs(depth - self.depth_cmd) >= self.depth_tol
        ):
            return False
        if (
            self.heading_cmd is not None
            and abs(wrap_pi(yaw - self.heading_cmd)) >= self.heading_tol
        ):
            return False
        if (
            self.pos_cmd is not None
            and math.hypot(x - self.pos_cmd[0], y - self.pos_cmd[1])
            >= self.pos_tol
        ):
            return False
        velocity = self.state.twist.twist.linear
        return math.hypot(velocity.x, velocity.y, velocity.z) < self.speed_tol

    def pose(self):
        s = self.state
        q = s.pose.pose.orientation
        _, _, yaw = q_to_rpy((q.w, q.x, q.y, q.z))
        return (
            s.pose.pose.position.x,
            s.pose.pose.position.y,
            -s.pose.pose.position.z,
            yaw,
        )

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------ steps
    def start_step(self):
        cmd, args = self.parsed_steps[self.i]
        x, y, depth, yaw = self.pose()
        label = f"step {self.i + 1}/{len(self.steps)}: {self.steps[self.i]}"
        if cmd != "roll":
            self.pub_roll.publish(Float64(data=0.0))

        if cmd == "dive":
            target = args[0]
            self.depth_cmd = target
            self.pub_depth.publish(Float64(data=target))
            self.check = lambda: abs(self.pose()[2] - target) < self.depth_tol
        elif cmd == "heading":
            self.heading_cmd = math.radians(args[0])
            self.pub_heading.publish(Float64(data=self.heading_cmd))
            self.check = lambda: (
                abs(wrap_pi(self.pose()[3] - self.heading_cmd))
                < self.heading_tol
            )
        elif cmd in ("move", "goto"):
            if (
                self.heading_cmd is None
            ):  # hold the current heading while moving
                self.heading_cmd = yaw
                self.pub_heading.publish(Float64(data=yaw))
            if cmd == "move":
                tx = x + args[0] * math.cos(self.heading_cmd)
                ty = y + args[0] * math.sin(self.heading_cmd)
            else:
                tx, ty = args[0], args[1]
            if not math.isfinite(tx) or not math.isfinite(ty):
                self.fail("position target exceeds finite range")
                return
            self.pub_pos.publish(Point(x=tx, y=ty))
            self.pos_cmd = (tx, ty)
            label += f" -> target ({tx:+.2f}, {ty:+.2f})"
            self.check = lambda: (
                math.hypot(self.pose()[0] - tx, self.pose()[1] - ty)
                < self.pos_tol
                and math.hypot(
                    self.state.twist.twist.linear.x,
                    self.state.twist.twist.linear.y,
                )
                < self.speed_tol
            )
        elif cmd == "roll":
            turns = args[0]
            t_done = (
                self.now()
                + abs(turns) * 2 * math.pi / constants.ROLL_SPIN_RATE_RAD_S
                + 3.0
            )
            if not math.isfinite(t_done):
                self.fail("roll duration exceeds finite range")
                return
            self.pub_roll_turns.publish(Float64(data=turns))
            self.check = lambda: (
                self.now() > t_done and abs(self.roll()) < math.radians(10)
            )
        elif cmd == "wait":
            t_done = self.now() + args[0]
            self.check = lambda: self.now() > t_done
        elif cmd == "hold":
            self.check = self.holding_targets
        elif cmd == "surface":
            self.depth_cmd = 0.3
            self.pub_depth.publish(Float64(data=0.3))
            self.check = lambda: self.pose()[2] < 0.5
        self.step_started = True
        self.t_inside = self.now() if cmd == "hold" and self.check() else None
        self.publish_status("step_started")
        self.get_logger().info(label)

    def roll(self):
        q = self.state.pose.pose.orientation
        return q_to_rpy((q.w, q.x, q.y, q.z))[0]

    def periodic(self):
        """Check mission progress on each mission timer tick."""
        if self.finished:
            return
        if self.startup_error is not None:
            self.fail(self.startup_error)
            return
        t = self.now()
        if t < self.last_tick:
            self.fail("mission clock moved backwards")
            return
        self.last_tick = t
        if t - self.t_step >= self.step_timeout:
            index = max(0, self.i)
            self.fail(
                f'step "{self.steps[index]}" timed out after '
                f"{self.step_timeout:.0f} s"
            )
            return
        if not self.ready(t):
            self.t_inside = None
            self.get_logger().warn(
                "waiting for fresh state and required DVL",
                throttle_duration_sec=5.0,
            )
            return
        q = self.state.pose.pose.orientation
        roll, pitch, yaw = q_to_rpy((q.w, q.x, q.y, q.z))
        command = self.parsed_steps[max(0, self.i)][0]
        if abs(math.degrees(pitch)) > self.max_tilt_deg or (
            command != "roll" and abs(math.degrees(roll)) > self.max_tilt_deg
        ):
            self.fail(
                f"Attitude limit at t={t:.3f}s: "
                f"roll={math.degrees(roll):+.1f} deg, "
                f"pitch={math.degrees(pitch):+.1f} deg "
                f"(limit {self.max_tilt_deg:.1f} deg)"
            )
            return
        if self.i < 0:
            self.i = 0
        if not self.step_started:
            self.start_step()
            return

        if t - self.t_log > 2.0:
            self.t_log = t
            x, y, depth, yaw = self.pose()
            self.get_logger().info(
                f"t={t:.2f}s step={self.i + 1}/{len(self.steps)} "
                f"xy=({x:+.2f},{y:+.2f})m depth={depth:.2f}m "
                f"rpy=({math.degrees(roll):+.1f},"
                f"{math.degrees(pitch):+.1f},{math.degrees(yaw):+.1f})deg "
                f"DVL_valid={self.state.pose.covariance[0] >= 0}"
            )

        if self.check():
            cmd, args = self.parsed_steps[self.i]
            duration = args[0] if cmd == "hold" else self.settle
            if self.t_inside is None:
                self.t_inside = t
            if t - self.t_inside >= duration:
                self.get_logger().info(f"  done in {t - self.t_step:.1f} s")
                self.i += 1
                if self.i >= len(self.steps):
                    self.finished = True
                    self.publish_status("succeeded")
                    self.get_logger().info("MISSION COMPLETE; holding")
                else:
                    self.t_step = t
                    self.t_inside = None
                    self.step_started = False
        else:
            self.t_inside = None


def main():
    rclpy.init()
    node = Mission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():  # a real error; a SIGTERM mid-spin is not
            node.fail("mission failed")
            raise
    finally:
        if rclpy.ok() and not node.finished:
            node.fail("mission interrupted")
        node.destroy_node()
        if rclpy.ok():  # a SIGTERM already shut the context down
            rclpy.shutdown()


if __name__ == "__main__":
    main()

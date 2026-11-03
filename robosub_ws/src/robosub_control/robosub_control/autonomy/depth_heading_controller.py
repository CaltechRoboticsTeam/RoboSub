#!/usr/bin/env python3
"""PID depth, heading, roll and profiled position control."""

import json
import math

from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
import rclpy
from std_msgs.msg import Empty, Float64, String

from robosub_control.autonomy.motion_profile import MotionProfile
from robosub_control.autonomy.sensor_freshness import (
    SensorFreshness,
    stamp_to_sec,
)
from robosub_control.autonomy.state_estimator import q_to_rpy, rotate
from robosub_control.config import constants
from robosub_control.config.vehicle_profiles import profile_from_node
from robosub_control.utils.tuning import TunableNode, fixed_parameter


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class PID:
    """u = kp*e + ki*int(e) - kd*rate.  Derivative acts on the MEASURED rate
    (which the estimator already gives us), not on d(error)/dt, so a step
    in the setpoint does not produce a spike."""

    def __init__(self, kp, ki, kd, i_limit, out_limit=1.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_limit, self.out_limit = i_limit, out_limit
        self.integral = 0.0
        self.terms = {}

    def step(self, error, rate, dt, integrate=True):
        if integrate:
            self.integral = clamp(
                self.integral + error * dt, -self.i_limit, self.i_limit
            )
        u = self.kp * error + self.ki * self.integral - self.kd * rate
        output = clamp(u, -self.out_limit, self.out_limit)
        self.terms = {
            "error": error,
            "measured_rate": rate,
            "p": self.kp * error,
            "i": self.ki * self.integral,
            "d": -self.kd * rate,
            "unsaturated": u,
            "output": output,
            "saturated": bool(output != u),
            "integrating": integrate,
            "kp": self.kp,
            "ki": self.ki,
            "kd": self.kd,
        }
        return output

    def reset(self):
        self.integral = 0.0
        self.terms = {}


class DepthHeadingController(TunableNode):
    def __init__(self):
        super().__init__("depth_heading_controller")
        profile = profile_from_node(self)
        self.manual_control = fixed_parameter(self, "manual_control", False)
        self.command_limit = 1.0
        self.rate = float(
            fixed_parameter(self, "rate", constants.CONTROLLER_DEFAULTS["rate"])
        )
        if not (
            constants.MIN_CONTROL_RATE_HZ
            <= self.rate
            <= constants.MAX_CONTROL_RATE_HZ
        ):
            raise ValueError("Controller rate must be between 1 and 1000 Hz")
        output_rate = float(
            fixed_parameter(
                self,
                "output_rate",
                constants.CONTROLLER_DEFAULTS["output_rate"],
            )
        )
        if not (
            constants.MIN_CONTROL_RATE_HZ
            <= output_rate
            <= profile.MAX_OUTPUT_RATE_HZ
        ):
            raise ValueError(
                f"Output rate must be 1-{profile.MAX_OUTPUT_RATE_HZ:g} Hz"
            )
        self.output_rate = min(output_rate, self.rate)
        self.next_output_time = None
        self.state_timeout = float(
            fixed_parameter(
                self,
                "state_timeout",
                constants.CONTROLLER_DEFAULTS["state_timeout"],
            )
        )
        if not math.isfinite(self.state_timeout) or self.state_timeout <= 0:
            raise ValueError("state_timeout must be finite and positive")
        self.state_freshness = SensorFreshness(self.state_timeout)
        self.max_tilt_deg = float(
            fixed_parameter(
                self,
                "max_tilt_deg",
                constants.CONTROLLER_DEFAULTS["max_tilt_deg"],
            )
        )
        if (
            not math.isfinite(self.max_tilt_deg)
            or not 0 < self.max_tilt_deg < 60
        ):
            raise ValueError("max_tilt_deg must be between 0 and 60")
        for name in ("pos_max_speed", "pos_max_acceleration"):
            value = float(
                fixed_parameter(self, name, constants.CONTROLLER_DEFAULTS[name])
            )
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
            setattr(self, name, value)
        for name, default in constants.CONTROLLER_DEFAULTS.items():
            if name in (
                "rate",
                "output_rate",
                "state_timeout",
                "pos_max_speed",
                "pos_max_acceleration",
                "max_tilt_deg",
            ):
                continue
            self.tuning.number(
                name,
                default,
                minimum=1e-6 if name == "roll_spin_rate" else 0.0,
                maximum=1.0 if name == "pos_limit" else None,
                description=name.replace("_", " "),
            )

        def g(name):
            return self.tuning.numbers[name].get()

        self.depth_pid = PID(
            g("depth_kp"), g("depth_ki"), g("depth_kd"), g("i_limit")
        )
        self.heading_pid = PID(
            g("heading_kp"), g("heading_ki"), g("heading_kd"), g("i_limit")
        )
        self.roll_pid = PID(
            g("roll_kp"), g("roll_ki"), g("roll_kd"), g("i_limit")
        )
        self.roll_ff = g("roll_ff")
        self.spin_rate = g("roll_spin_rate")
        self.surge_pid = PID(g("pos_kp"), 0.0, g("pos_kd"), 0.0, g("pos_limit"))
        self.sway_pid = PID(g("pos_kp"), 0.0, g("pos_kd"), 0.0, g("pos_limit"))

        for prefix, pid in (
            ("depth", self.depth_pid),
            ("heading", self.heading_pid),
            ("roll", self.roll_pid),
        ):
            for gain in ("kp", "ki", "kd"):
                self.tuning.numbers[f"{prefix}_{gain}"].bind_to(pid, gain)
            self.tuning.numbers["i_limit"].bind_to(pid, "i_limit")
        for pid in (self.surge_pid, self.sway_pid):
            self.tuning.numbers["pos_kp"].bind_to(pid, "kp")
            self.tuning.numbers["pos_kd"].bind_to(pid, "kd")
            self.tuning.numbers["pos_limit"].bind_to(pid, "out_limit")
        self.tuning.numbers["roll_ff"].bind_to(self, "roll_ff")
        self.tuning.numbers["roll_spin_rate"].bind_to(self, "spin_rate")
        self.last_control_time = None

        self.depth_sp = None
        self.heading_sp = None
        self.pos_sp = None  # (x, y) or None
        self.motion = None
        self.position_needs_replan = True
        self.roll_sp = None
        self.prev_roll = None
        self.roll_unwrapped = 0.0  # continuous roll angle, for multi-turn moves
        self.spin_sp = None  # ramping unwrapped setpoint during a roll_turns
        self.spin_end = None  # where the ramp stops (unwrapped)
        self.state = None
        self.state_rx = None  # node-clock time of the last state message
        self.manual = Twist()
        self.manual_rx = None

        self.pub = self.create_publisher(Twist, "/sub/cmd_vel", 10)
        self.status_pub = self.create_publisher(
            String, "/sub/control/status", 10
        )
        self.fault_pub = self.create_publisher(String, "/sub/control/fault", 10)
        self.reference_pub = self.create_publisher(
            Odometry, "/sub/reference", 10
        )
        self.target_pubs = {
            name: self.create_publisher(Float64, f"/sub/reference/{name}", 10)
            for name in ("depth", "heading", "roll")
        }
        self.create_subscription(Odometry, "/sub/state", self.on_state, 10)
        self.create_subscription(
            Float64, "/sub/setpoint/depth", self.on_depth_sp, 10
        )
        self.create_subscription(
            Float64, "/sub/setpoint/heading", self.on_heading_sp, 10
        )
        self.create_subscription(
            Float64, "/sub/setpoint/roll", self.on_roll_sp, 10
        )
        self.create_subscription(
            Float64, "/sub/setpoint/roll_turns", self.on_roll_turns, 10
        )
        self.create_subscription(
            Point, "/sub/setpoint/position", self.on_pos_sp, 10
        )
        self.create_subscription(
            Float64, "/sub/setpoint/move", self.on_move, 10
        )
        self.create_subscription(
            Twist, "/sub/cmd_vel_manual", self.on_manual, 10
        )
        self.create_subscription(
            Empty, "/sub/setpoint/cancel", self.on_cancel, 10
        )
        self.control_timer = self.create_timer(1.0 / self.rate, self.periodic)

        self.get_logger().info(
            "depth_heading_controller: Xbox owns commands; hold loops disabled"
            if self.manual_control
            else "depth_heading_controller up, IDLE until a setpoint on "
            "/sub/setpoint/depth or /sub/setpoint/heading"
        )

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_state(self, msg):
        now = self.now()
        position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        linear, angular = msg.twist.twist.linear, msg.twist.twist.angular
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
            msg.pose.covariance[0],
        )
        if abs(math.hypot(q.w, q.x, q.y, q.z) - 1.0) > 0.01:
            return
        was_fresh = self.state_freshness.fresh(now)
        if not self.state_freshness.accept(
            stamp_to_sec(msg.header.stamp), now, values
        ):
            return
        if not was_fresh or msg.pose.covariance[0] < 0.0:
            self.position_needs_replan = True
        self.state = msg
        self.state_rx = now

    def on_depth_sp(self, msg):
        if not math.isfinite(msg.data):
            return
        if self.depth_sp is None:
            self.get_logger().info("depth loop armed")
        self.depth_sp = msg.data

    def on_heading_sp(self, msg):
        if not math.isfinite(msg.data):
            return
        if self.heading_sp is None:
            self.get_logger().info("heading loop armed")
        self.heading_sp = msg.data

    def on_roll_sp(self, msg):
        if not math.isfinite(msg.data):
            return
        if self.roll_sp is None:
            self.get_logger().info("roll loop armed")
        self.roll_sp = msg.data
        self.spin_end = None  # a plain setpoint cancels a spin

    def on_roll_turns(self, msg):
        if not math.isfinite(msg.data):
            return
        if not self.state_freshness.fresh(self.now()) or self.prev_roll is None:
            self.get_logger().warn("roll_turns ignored: no /sub/state yet")
            return
        if self.roll_sp is None:
            self.get_logger().info("roll loop armed")
            self.roll_sp = 0.0
        self.spin_sp = self.roll_unwrapped
        self.spin_end = self.roll_unwrapped + 2 * math.pi * msg.data
        self.roll_pid.reset()
        self.get_logger().info(
            f"rolling {msg.data:+.2f} turns at {self.spin_rate} rad/s"
        )

    def on_pos_sp(self, msg):
        if not all(math.isfinite(value) for value in (msg.x, msg.y)):
            return
        if self.pos_sp == (msg.x, msg.y):
            return
        if self.pos_sp is None:
            self.get_logger().info("position loop armed")
        self.pos_sp = (msg.x, msg.y)
        if self.motion is not None and not self.position_needs_replan:
            self.motion.retarget(
                self.pos_sp, self.pos_max_speed, self.pos_max_acceleration
            )
        else:
            self.position_needs_replan = True
        self.get_logger().info(f"go to ({msg.x:.2f}, {msg.y:.2f})")

    def on_move(self, msg):
        """Forward `data` metres along the heading setpoint (or current yaw)."""
        if not math.isfinite(msg.data):
            return
        if (
            not self.state_freshness.fresh(self.now())
            or self.state.pose.covariance[0] < 0.0
        ):
            self.get_logger().warn("move ignored: no /sub/state yet")
            return
        q = self.state.pose.pose.orientation
        _, _, yaw = q_to_rpy((q.w, q.x, q.y, q.z))
        h = self.heading_sp if self.heading_sp is not None else yaw
        x0 = self.state.pose.pose.position.x
        y0 = self.state.pose.pose.position.y
        target = Point(
            x=x0 + msg.data * math.cos(h), y=y0 + msg.data * math.sin(h)
        )
        self.on_pos_sp(target)

    def on_manual(self, msg):
        if not all(
            math.isfinite(value) for value in (msg.linear.x, msg.linear.y)
        ):
            return
        self.manual = msg
        self.manual_rx = self.now()
        if self.pos_sp is not None and (
            msg.linear.x != 0.0 or msg.linear.y != 0.0
        ):
            self.get_logger().info(
                "manual surge/sway received: position mode cancelled"
            )
            self.pos_sp = None
            self.motion = None
            self.position_needs_replan = True

    def reset_pids(self):
        for pid in (
            self.depth_pid,
            self.heading_pid,
            self.roll_pid,
            self.surge_pid,
            self.sway_pid,
        ):
            pid.reset()

    def on_cancel(self, _msg):
        """Clear automatic targets while preserving operator enable state."""
        self.depth_sp = self.heading_sp = self.roll_sp = self.pos_sp = None
        self.spin_sp = self.spin_end = self.prev_roll = None
        self.roll_unwrapped = 0.0
        self.motion = None
        self.position_needs_replan = True
        self.manual = Twist()
        self.manual_rx = None
        self.next_output_time = None
        self.reset_pids()
        if not self.manual_control:
            self.pub.publish(Twist())

    def position_reference(self, dt):
        """Reacquire measured motion after an interruption, then advance."""
        if self.position_needs_replan or self.motion is None:
            p, q = (
                self.state.pose.pose.position,
                self.state.pose.pose.orientation,
            )
            v = self.state.twist.twist.linear
            world_velocity = rotate((q.w, q.x, q.y, q.z), (v.x, v.y, v.z))
            self.motion = MotionProfile((p.x, p.y), world_velocity[:2])
            self.motion.retarget(
                self.pos_sp, self.pos_max_speed, self.pos_max_acceleration
            )
            self.position_needs_replan = False
            self.surge_pid.reset()
            self.sway_pid.reset()
            dt = 0.0
        reference = self.motion.advance(dt)
        message = Odometry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.state.header.frame_id
        # Reference velocities, like reference positions, use the world frame.
        message.child_frame_id = message.header.frame_id
        message.pose.pose.position.x, message.pose.pose.position.y = (
            reference.position
        )
        message.pose.pose.position.z = self.state.pose.pose.position.z
        message.pose.pose.orientation.w = 1.0
        message.twist.twist.linear.x, message.twist.twist.linear.y = (
            reference.velocity
        )
        self.reference_pub.publish(message)
        return reference

    def periodic(self):
        """Run the active hold loops on each controller timer tick."""
        if self.manual_control:
            return
        now = self.now()
        previous = self.last_control_time
        self.last_control_time = now
        dt = 1.0 / self.rate if previous is None else now - previous
        if dt <= 0.0 or dt > self.state_timeout:
            self.next_output_time = None
            self.position_needs_replan = True
            self.reset_pids()
            self.pub.publish(Twist())
            return
        if (
            self.depth_sp is None
            and self.heading_sp is None
            and self.pos_sp is None
            and self.roll_sp is None
        ):
            return  # idle: leave /sub/cmd_vel alone
        if self.state is None or not self.state_freshness.fresh(now):
            self.get_logger().warn(
                "no fresh /sub/state; not commanding", throttle_duration_sec=2.0
            )
            self.position_needs_replan = True
            self.reset_pids()
            self.pub.publish(Twist())
            return

        cmd = Twist()

        depth = -self.state.pose.pose.position.z
        heave_up = self.state.twist.twist.linear.z
        q = self.state.pose.pose.orientation
        quaternion = (q.w, q.x, q.y, q.z)
        roll, pitch, yaw = q_to_rpy(quaternion)
        level_command = self.spin_end is None and (
            self.roll_sp is None or abs(wrap_pi(self.roll_sp)) < 1e-6
        )
        if abs(math.degrees(pitch)) > self.max_tilt_deg or (
            level_command and abs(math.degrees(roll)) > self.max_tilt_deg
        ):
            reason = (
                f"Attitude limit at t={now:.3f}s: "
                f"roll={math.degrees(roll):+.1f} deg, "
                f"pitch={math.degrees(pitch):+.1f} deg; "
                f"limit={self.max_tilt_deg:.1f} deg"
            )
            self.fault_pub.publish(
                String(
                    data=json.dumps(
                        {
                            "stamp": now,
                            "reason": reason,
                            "roll_deg": math.degrees(roll),
                            "pitch_deg": math.degrees(pitch),
                        }
                    )
                )
            )
            self.get_logger().error(reason)
            self.on_cancel(Empty())
            return
        rates = self.state.twist.twist.angular
        yaw_rate = (
            math.sin(roll) * rates.y + math.cos(roll) * rates.z
        ) / math.cos(pitch)
        roll_rate = rates.x + math.tan(pitch) * (
            math.sin(roll) * rates.y + math.cos(roll) * rates.z
        )

        # Keep a continuous roll angle so a 360 is not a no-op.
        if self.prev_roll is not None:
            self.roll_unwrapped += wrap_pi(roll - self.prev_roll)
        self.prev_roll = roll
        upright = math.cos(roll) * math.cos(pitch)

        if self.spin_end is not None:
            # Ramp the unwrapped setpoint toward spin_end at spin_rate.
            direction = 1.0 if self.spin_end >= self.spin_sp else -1.0
            self.spin_sp += direction * self.spin_rate * dt
            done_ramp = (direction > 0 and self.spin_sp >= self.spin_end) or (
                direction < 0 and self.spin_sp <= self.spin_end
            )
            if done_ramp:
                self.spin_sp = self.spin_end
            rate_sp = 0.0 if done_ramp else direction * self.spin_rate
            err = self.spin_sp - self.roll_unwrapped
            u = self.roll_pid.step(err, roll_rate - rate_sp, dt)
            cmd.angular.x = clamp(u + self.roll_ff * math.sin(roll), -1.0, 1.0)
            if done_ramp and abs(err) < 0.15:
                self.get_logger().info("roll manoeuvre complete")
                self.roll_sp = wrap_pi(self.spin_end)
                self.spin_end = None
                self.roll_pid.reset()
        elif self.roll_sp is not None:
            err = wrap_pi(self.roll_sp - roll)
            u = self.roll_pid.step(err, roll_rate, dt)
            cmd.angular.x = clamp(
                u + self.roll_ff * math.sin(self.roll_sp), -1.0, 1.0
            )

        if self.pos_sp is not None:
            if self.state.pose.covariance[0] < 0.0:
                self.position_needs_replan = True
                self.surge_pid.reset()
                self.sway_pid.reset()
                self.get_logger().warn(
                    "position mode but no DVL: holding still",
                    throttle_duration_sec=2.0,
                )
            else:
                reference = self.position_reference(dt)
                ex = reference.position[0] - self.state.pose.pose.position.x
                ey = reference.position[1] - self.state.pose.pose.position.y
                inverse = (q.w, -q.x, -q.y, -q.z)
                ex_b, ey_b, _ = rotate(inverse, (ex, ey, 0.0))
                vx_b = self.state.twist.twist.linear.x
                vy_b = self.state.twist.twist.linear.y
                vx_ref, vy_ref, _ = rotate(inverse, (*reference.velocity, 0.0))
                cmd.linear.x = self.surge_pid.step(ex_b, vx_b - vx_ref, dt)
                cmd.linear.y = self.sway_pid.step(ey_b, vy_b - vy_ref, dt)
        elif (
            self.manual_rx is not None
            and self.now() - self.manual_rx < constants.MANUAL_COMMAND_TIMEOUT_S
        ):
            # Surge / sway from the operator, if recent.
            cmd.linear.x = self.manual.linear.x
            cmd.linear.y = self.manual.linear.y

        if self.depth_sp is not None:
            err_down = self.depth_sp - depth  # +: too shallow
            u_down = self.depth_pid.step(
                err_down, -heave_up, dt, integrate=abs(upright) > 0.5
            )
            # Compensate the vertical projection inside the operating range.
            # During a deliberate roll, suppress heave near horizontal.
            cmd.linear.z = (
                clamp(-u_down / upright, -1.0, 1.0)
                if abs(upright) > 0.5
                else 0.0
            )

        if self.heading_sp is not None:
            err = wrap_pi(self.heading_sp - yaw)
            cmd.angular.z = self.heading_pid.step(err, yaw_rate, dt) * upright

        if (
            self.output_rate == self.rate
            or self.next_output_time is None
            or now >= self.next_output_time
        ):
            for axis in (cmd.linear, cmd.angular):
                axis.x = clamp(axis.x, -self.command_limit, self.command_limit)
                axis.y = clamp(axis.y, -self.command_limit, self.command_limit)
                axis.z = clamp(axis.z, -self.command_limit, self.command_limit)
            self.pub.publish(cmd)
            for name, publisher in self.target_pubs.items():
                target = getattr(self, f"{name}_sp")
                if target is not None:
                    publisher.publish(Float64(data=target))
            self.status_pub.publish(
                String(
                    data=json.dumps(
                        {
                            "stamp": now,
                            "rpy_deg": [
                                math.degrees(v) for v in (roll, pitch, yaw)
                            ],
                            "body_z_vertical_projection": upright,
                            "depth_target_m": self.depth_sp,
                            "heading_target_rad": self.heading_sp,
                            "roll_target_rad": self.roll_sp,
                            "position_target_m": self.pos_sp,
                            "dvl_valid": bool(
                                self.state.pose.covariance[0] >= 0
                            ),
                            "settings": {
                                "rate_hz": self.rate,
                                "max_speed_m_s": self.pos_max_speed,
                                "max_accel_m_s2": self.pos_max_acceleration,
                                "max_tilt_deg": self.max_tilt_deg,
                            },
                            "pid": {
                                name: pid.terms
                                for name, pid in (
                                    ("depth", self.depth_pid),
                                    ("heading", self.heading_pid),
                                    ("roll", self.roll_pid),
                                    ("surge", self.surge_pid),
                                    ("sway", self.sway_pid),
                                )
                            },
                        },
                        allow_nan=False,
                    )
                )
            )
            period = 1.0 / self.output_rate
            if self.next_output_time is None or self.output_rate == self.rate:
                self.next_output_time = now + period
            else:
                # Preserve the publication phase without sending old outputs
                # in a burst after a delayed callback.
                skipped = math.floor((now - self.next_output_time) / period)
                self.next_output_time += (skipped + 1) * period


def main():
    rclpy.init()
    node = DepthHeadingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():  # a real error; a SIGTERM mid-spin is not
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():  # a SIGTERM already shut the context down
            rclpy.shutdown()


if __name__ == "__main__":
    main()

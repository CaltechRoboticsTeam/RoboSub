#!/usr/bin/env python3
"""Hold depth, heading, roll and horizontal position using /sub/state.

Inputs are /sub/setpoint/depth (+down metres), heading/roll (right-hand ROS
radians), position (world x/y metres), move (forward metres), roll_turns,
and optional /sub/cmd_vel_manual surge/sway efforts. Setpoints persist.
Startup manual_control=True disables all automatic command publication;
the Xbox program then owns /sub/cmd_vel without depending on state input.

periodic() runs at the configured calculation rate. The Twist published on
/sub/cmd_vel carries normalized body efforts, not velocity setpoints. The
geometry allocator uses normalized M5 power, or T200 force/torque limits
at the configured reference voltage, according to the vehicle profile.

The controller starts idle. Missing/stale state stops command publication;
the mixer and command stage each have a separate command timeout. Position
hold additionally requires a fresh DVL estimate. Nonzero manual surge/sway
cancels position hold. The layout has no independent pitch authority.

During multi-turn rolls, an unwrapped target is ramped at roll_spin_rate.
Depth and yaw efforts are scaled by cos(roll), and depth integration pauses
when well off level. Gains/feedforward remain tuning defaults, not measured
physical vehicle dynamics. Recheck them after geometry/voltage changes.
"""
import math

from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from robosub_sim import constants
from robosub_sim.state_estimator import q_to_rpy
from robosub_sim.tuning import TunableNumbers, fixed_parameter
from robosub_sim.vehicle_profiles import profile_from_node


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def wrap_pi(a):
    return (a + math.pi) % (2*math.pi) - math.pi


class PID:
    """u = kp*e + ki*int(e) - kd*rate.  Derivative acts on the MEASURED rate
    (which the estimator already gives us), not on d(error)/dt, so a step
    in the setpoint does not produce a spike."""

    def __init__(self, kp, ki, kd, i_limit, out_limit=1.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_limit, self.out_limit = i_limit, out_limit
        self.integral = 0.0

    def step(self, error, rate, dt, integrate=True):
        if integrate:
            self.integral = clamp(self.integral + error*dt,
                                  -self.i_limit, self.i_limit)
        u = self.kp*error + self.ki*self.integral - self.kd*rate
        return clamp(u, -self.out_limit, self.out_limit)

    def reset(self):
        self.integral = 0.0


class DepthHeadingController(Node):
    def __init__(self):
        super().__init__('depth_heading_controller')
        profile = profile_from_node(self)
        self.manual_control = fixed_parameter(self, 'manual_control', False)
        self.rate = float(fixed_parameter(
            self, 'rate', constants.CONTROLLER_DEFAULTS['rate']))
        if not (constants.MIN_CONTROL_RATE_HZ <= self.rate
                <= constants.MAX_CONTROL_RATE_HZ):
            raise ValueError('Controller rate must be between 1 and 1000 Hz')
        output_rate = float(fixed_parameter(
            self, 'output_rate', constants.CONTROLLER_DEFAULTS['output_rate']))
        if not (constants.MIN_CONTROL_RATE_HZ <= output_rate
                <= profile.MAX_OUTPUT_RATE_HZ):
            raise ValueError(
                f'Output rate must be 1-{profile.MAX_OUTPUT_RATE_HZ:g} Hz')
        self.output_rate = min(output_rate, self.rate)
        self.next_output_time = None
        self.state_timeout = float(fixed_parameter(
            self, 'state_timeout',
            constants.CONTROLLER_DEFAULTS['state_timeout']))
        if not math.isfinite(self.state_timeout) or self.state_timeout <= 0:
            raise ValueError('state_timeout must be finite and positive')
        self.tuning = TunableNumbers(self)
        for name, default in constants.CONTROLLER_DEFAULTS.items():
            if name in ('rate', 'output_rate', 'state_timeout'):
                continue
            self.tuning.number(
                name, default,
                minimum=1e-6 if name == 'roll_spin_rate' else 0.0,
                maximum=1.0 if name == 'pos_limit' else None,
                description=name.replace('_', ' '),
            )
        g = lambda n: self.tuning.numbers[n].get()
        self.depth_pid = PID(g('depth_kp'), g('depth_ki'), g('depth_kd'), g('i_limit'))
        self.heading_pid = PID(g('heading_kp'), g('heading_ki'), g('heading_kd'), g('i_limit'))
        self.roll_pid = PID(g('roll_kp'), g('roll_ki'), g('roll_kd'), g('i_limit'))
        self.roll_ff = g('roll_ff')
        self.spin_rate = g('roll_spin_rate')
        self.surge_pid = PID(g('pos_kp'), 0.0, g('pos_kd'), 0.0, g('pos_limit'))
        self.sway_pid = PID(g('pos_kp'), 0.0, g('pos_kd'), 0.0, g('pos_limit'))

        for prefix, pid in (
            ('depth', self.depth_pid),
            ('heading', self.heading_pid),
            ('roll', self.roll_pid),
        ):
            for gain in ('kp', 'ki', 'kd'):
                self.tuning.numbers[f'{prefix}_{gain}'].bind_to(pid, gain)
            self.tuning.numbers['i_limit'].bind_to(pid, 'i_limit')
        for pid in (self.surge_pid, self.sway_pid):
            self.tuning.numbers['pos_kp'].bind_to(pid, 'kp')
            self.tuning.numbers['pos_kd'].bind_to(pid, 'kd')
            self.tuning.numbers['pos_limit'].bind_to(pid, 'out_limit')
        self.tuning.numbers['roll_ff'].bind_to(self, 'roll_ff')
        self.tuning.numbers['roll_spin_rate'].bind_to(self, 'spin_rate')
        self.last_control_time = None

        self.depth_sp = None
        self.heading_sp = None
        self.pos_sp = None          # (x, y) or None
        self.roll_sp = None
        self.prev_roll = None
        self.roll_unwrapped = 0.0   # continuous roll angle, for multi-turn moves
        self.spin_sp = None         # ramping unwrapped setpoint during a roll_turns
        self.spin_end = None        # where the ramp stops (unwrapped)
        self.state = None
        self.state_rx = None        # node-clock time of the last state message
        self.manual = Twist()
        self.manual_rx = None

        self.pub = self.create_publisher(Twist, '/sub/cmd_vel', 10)
        self.create_subscription(Odometry, '/sub/state', self.on_state, 10)
        self.create_subscription(Float64, '/sub/setpoint/depth', self.on_depth_sp, 10)
        self.create_subscription(Float64, '/sub/setpoint/heading', self.on_heading_sp, 10)
        self.create_subscription(Float64, '/sub/setpoint/roll', self.on_roll_sp, 10)
        self.create_subscription(Float64, '/sub/setpoint/roll_turns', self.on_roll_turns, 10)
        self.create_subscription(Point, '/sub/setpoint/position', self.on_pos_sp, 10)
        self.create_subscription(Float64, '/sub/setpoint/move', self.on_move, 10)
        self.create_subscription(Twist, '/sub/cmd_vel_manual', self.on_manual, 10)
        self.control_timer = self.create_timer(1.0/self.rate, self.periodic)

        self.get_logger().info(
            'depth_heading_controller: Xbox owns commands; hold loops disabled'
            if self.manual_control else
            'depth_heading_controller up, IDLE until a setpoint arrives on '
            '/sub/setpoint/depth or /sub/setpoint/heading')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_state(self, msg):
        self.state = msg
        self.state_rx = self.now()

    def on_depth_sp(self, msg):
        if self.depth_sp is None:
            self.get_logger().info('depth loop armed')
        self.depth_sp = msg.data

    def on_heading_sp(self, msg):
        if self.heading_sp is None:
            self.get_logger().info('heading loop armed')
        self.heading_sp = msg.data

    def on_roll_sp(self, msg):
        if self.roll_sp is None:
            self.get_logger().info('roll loop armed')
        self.roll_sp = msg.data
        self.spin_end = None            # a plain setpoint cancels a spin

    def on_roll_turns(self, msg):
        if self.state is None or self.prev_roll is None:
            self.get_logger().warn('roll_turns ignored: no /sub/state yet')
            return
        if self.roll_sp is None:
            self.get_logger().info('roll loop armed')
            self.roll_sp = 0.0
        self.spin_sp = self.roll_unwrapped
        self.spin_end = self.roll_unwrapped + 2*math.pi*msg.data
        self.roll_pid.reset()
        self.get_logger().info(f'rolling {msg.data:+.2f} turns at {self.spin_rate} rad/s')

    def on_pos_sp(self, msg):
        if self.pos_sp is None:
            self.get_logger().info('position loop armed')
        self.pos_sp = (msg.x, msg.y)
        self.get_logger().info(f'go to ({msg.x:.2f}, {msg.y:.2f})')

    def on_move(self, msg):
        """Forward `data` metres along the heading setpoint (or current yaw)."""
        if self.state is None:
            self.get_logger().warn('move ignored: no /sub/state yet')
            return
        q = self.state.pose.pose.orientation
        _, _, yaw = q_to_rpy((q.w, q.x, q.y, q.z))
        h = self.heading_sp if self.heading_sp is not None else yaw
        x0 = self.state.pose.pose.position.x
        y0 = self.state.pose.pose.position.y
        target = Point(x=x0 + msg.data*math.cos(h), y=y0 + msg.data*math.sin(h))
        self.on_pos_sp(target)

    def on_manual(self, msg):
        self.manual = msg
        self.manual_rx = self.now()
        if self.pos_sp is not None and (msg.linear.x != 0.0 or msg.linear.y != 0.0):
            self.get_logger().info('manual surge/sway received: position mode cancelled')
            self.pos_sp = None

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
            for pid in (self.depth_pid, self.heading_pid, self.roll_pid,
                        self.surge_pid, self.sway_pid):
                pid.reset()
            return
        if (self.depth_sp is None and self.heading_sp is None
                and self.pos_sp is None and self.roll_sp is None):
            return                              # idle: leave /sub/cmd_vel alone
        if self.state is None or self.now() - self.state_rx > self.state_timeout:
            self.get_logger().warn('no fresh /sub/state; not commanding',
                                   throttle_duration_sec=2.0)
            self.depth_pid.reset()
            self.heading_pid.reset()
            self.roll_pid.reset()
            return

        cmd = Twist()

        depth = -self.state.pose.pose.position.z
        heave_up = self.state.twist.twist.linear.z
        q = self.state.pose.pose.orientation
        roll, _, yaw = q_to_rpy((q.w, q.x, q.y, q.z))
        yaw_rate = self.state.twist.twist.angular.z
        roll_rate = self.state.twist.twist.angular.x

        # Keep a continuous roll angle so a 360 is not a no-op.
        if self.prev_roll is not None:
            self.roll_unwrapped += wrap_pi(roll - self.prev_roll)
        self.prev_roll = roll
        upright = math.cos(roll)             # +1 level, 0 on its side, -1 inverted

        if self.spin_end is not None:
            # Ramp the unwrapped setpoint toward spin_end at spin_rate.
            direction = 1.0 if self.spin_end >= self.spin_sp else -1.0
            self.spin_sp += direction * self.spin_rate * dt
            done_ramp = (direction > 0 and self.spin_sp >= self.spin_end) or \
                        (direction < 0 and self.spin_sp <= self.spin_end)
            if done_ramp:
                self.spin_sp = self.spin_end
            rate_sp = 0.0 if done_ramp else direction * self.spin_rate
            err = self.spin_sp - self.roll_unwrapped
            u = self.roll_pid.step(err, roll_rate - rate_sp, dt)
            cmd.angular.x = clamp(u + self.roll_ff * math.sin(roll), -1.0, 1.0)
            if done_ramp and abs(err) < 0.15:
                self.get_logger().info('roll manoeuvre complete')
                self.roll_sp = wrap_pi(self.spin_end)
                self.spin_end = None
                self.roll_pid.reset()
        elif self.roll_sp is not None:
            err = wrap_pi(self.roll_sp - roll)
            u = self.roll_pid.step(err, roll_rate, dt)
            cmd.angular.x = clamp(u + self.roll_ff * math.sin(roll), -1.0, 1.0)

        if self.pos_sp is not None:
            if self.state.pose.covariance[0] < 0.0:
                self.get_logger().warn('position mode but no DVL: holding still',
                                       throttle_duration_sec=2.0)
            else:
                ex = self.pos_sp[0] - self.state.pose.pose.position.x
                ey = self.pos_sp[1] - self.state.pose.pose.position.y
                c, s_ = math.cos(yaw), math.sin(yaw)
                ex_b = c*ex + s_*ey            # error in the body frame
                ey_b = -s_*ex + c*ey
                vx_b = self.state.twist.twist.linear.x
                vy_b = self.state.twist.twist.linear.y
                cmd.linear.x = self.surge_pid.step(ex_b, vx_b, dt)
                cmd.linear.y = self.sway_pid.step(ey_b, vy_b, dt)
        elif (
            self.manual_rx is not None
            and self.now() - self.manual_rx
            < constants.MANUAL_COMMAND_TIMEOUT_S
        ):
            # Surge / sway from the operator, if recent.
            cmd.linear.x = self.manual.linear.x
            cmd.linear.y = self.manual.linear.y

        if self.depth_sp is not None:
            err_down = self.depth_sp - depth            # +: too shallow
            u_down = self.depth_pid.step(err_down, -heave_up, dt,
                                         integrate=abs(upright) > 0.7)
            # Body "up" is only  cos(roll)  of world up: scale so an inverted
            # sub pushes the right way and one on its side does not fight.
            cmd.linear.z = -u_down * upright           # mixer heave is +up

        if self.heading_sp is not None:
            err = wrap_pi(self.heading_sp - yaw)
            cmd.angular.z = self.heading_pid.step(err, yaw_rate, dt) * upright

        if (self.output_rate == self.rate or self.next_output_time is None
                or now >= self.next_output_time):
            self.pub.publish(cmd)
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
        if rclpy.ok():          # a real error; a SIGTERM mid-spin is not
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():          # a SIGTERM already shut the context down
            rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Run a scripted mission: a list of moves, each one waited for before the next.

    ros2 run robosub_sim mission
    ros2 run robosub_sim mission --ros-args -p "mission:=['dive 1.5','move 3','goto 0 0','surface']"

Everything happens by publishing setpoints to the controller and watching
/sub/state until the sub is there. The mission does not access hardware
directly; physical operation still needs verified drivers and configuration.

STEPS (one string each, metres / degrees / seconds)
    dive D          go to depth D (+down), hold it from now on
    heading A       turn to yaw A degrees (0 = where the estimator started,
                    + = left), hold it from now on
    move M          M metres forward along the held heading (or the current
                    heading if none was set); negative = backwards
    goto X Y        go to (X, Y) in the start frame; (0, 0) = return to origin
    roll N          N full barrel rolls (+ = port side down first)
    wait S          hold everything for S seconds
    surface         depth 0.3 (thrusters off would be simpler, but the
                    controller keeps heading meanwhile)

A step is "done" when the sub has been inside tolerance for `settle`
seconds. If a step is not done after `step_timeout` seconds the mission
STOPS and the controller keeps holding whatever it was last told: the sub
parks rather than blundering on. Position and heading are logged every 2 s
so you can watch progress in the terminal.

Parameters
    mission         list of step strings   (default: dive 1.5, move 3, goto 0 0)
    pos_tol         m                      (0.20)
    depth_tol       m                      (0.10)
    heading_tol     deg                    (5.0)
    settle          s inside tolerance     (1.0)
    step_timeout    s                      (60.0)
"""
import math

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from robosub_sim import constants
from robosub_sim.depth_heading_controller import wrap_pi
from robosub_sim.state_estimator import q_to_rpy


class Mission(Node):
    def __init__(self):
        super().__init__('mission')
        for name, default in constants.MISSION_DEFAULTS.items():
            self.declare_parameter(name, default)
        g = lambda n: self.get_parameter(n).value
        self.steps = [str(s) for s in g('mission')]
        self.pos_tol, self.depth_tol = float(g('pos_tol')), float(g('depth_tol'))
        self.heading_tol = math.radians(float(g('heading_tol')))
        self.settle, self.step_timeout = float(g('settle')), float(g('step_timeout'))

        p = lambda t, T: self.create_publisher(T, t, 10)
        self.pub_depth = p('/sub/setpoint/depth', Float64)
        self.pub_heading = p('/sub/setpoint/heading', Float64)
        self.pub_pos = p('/sub/setpoint/position', Point)
        self.pub_roll_turns = p('/sub/setpoint/roll_turns', Float64)
        self.create_subscription(Odometry, '/sub/state', self.on_state, 10)

        self.state = None
        self.heading_cmd = None      # yaw we are holding, rad
        self.i = -1                  # current step index
        self.check = None            # function -> True when the step is done
        self.t_step = None
        self.t_inside = None
        self.t_log = 0.0
        self.finished = False
        self.create_timer(constants.MISSION_PERIOD_S, self.periodic)
        self.get_logger().info(f'mission: {self.steps}')

    # ------------------------------------------------------------ helpers
    def on_state(self, msg):
        self.state = msg

    def pose(self):
        s = self.state
        q = s.pose.pose.orientation
        _, _, yaw = q_to_rpy((q.w, q.x, q.y, q.z))
        return (s.pose.pose.position.x, s.pose.pose.position.y,
                -s.pose.pose.position.z, yaw)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------ steps
    def start_step(self):
        self.i += 1
        if self.i >= len(self.steps):
            self.finished = True
            x, y, depth, yaw = self.pose()
            self.get_logger().info(
                f'MISSION COMPLETE at ({x:+.2f}, {y:+.2f}) depth {depth:.2f} m '
                f'yaw {math.degrees(yaw):+.1f} deg; holding')
            return
        words = self.steps[self.i].split()
        cmd, args = words[0].lower(), [float(a) for a in words[1:]]
        x, y, depth, yaw = self.pose()
        label = f'step {self.i+1}/{len(self.steps)}: {self.steps[self.i]}'

        if cmd == 'dive':
            target = args[0]
            self.pub_depth.publish(Float64(data=target))
            self.check = lambda: abs(self.pose()[2] - target) < self.depth_tol
        elif cmd == 'heading':
            self.heading_cmd = math.radians(args[0])
            self.pub_heading.publish(Float64(data=self.heading_cmd))
            self.check = lambda: abs(wrap_pi(self.pose()[3] - self.heading_cmd)) < self.heading_tol
        elif cmd in ('move', 'goto'):
            if self.heading_cmd is None:          # hold the current heading while moving
                self.heading_cmd = yaw
                self.pub_heading.publish(Float64(data=yaw))
            if cmd == 'move':
                tx = x + args[0] * math.cos(self.heading_cmd)
                ty = y + args[0] * math.sin(self.heading_cmd)
            else:
                tx, ty = args[0], args[1]
            self.pub_pos.publish(Point(x=tx, y=ty))
            label += f' -> target ({tx:+.2f}, {ty:+.2f})'
            self.check = lambda: math.hypot(self.pose()[0] - tx, self.pose()[1] - ty) < self.pos_tol
        elif cmd == 'roll':
            turns = args[0]
            self.pub_roll_turns.publish(Float64(data=turns))
            t_done = (
                self.now() + abs(turns) * 2*math.pi
                / constants.ROLL_SPIN_RATE_RAD_S + 3.0
            )
            self.check = lambda: self.now() > t_done and abs(self.roll()) < math.radians(10)
        elif cmd == 'wait':
            t_done = self.now() + args[0]
            self.check = lambda: self.now() > t_done
        elif cmd == 'surface':
            self.pub_depth.publish(Float64(data=0.3))
            self.check = lambda: self.pose()[2] < 0.5
        else:
            self.get_logger().error(f'unknown step "{self.steps[self.i]}"; stopping')
            self.finished = True
            return

        self.t_step = self.now()
        self.t_inside = None
        self.get_logger().info(label)

    def roll(self):
        q = self.state.pose.pose.orientation
        return q_to_rpy((q.w, q.x, q.y, q.z))[0]

    def periodic(self):
        """Check mission progress on each mission timer tick."""
        if self.finished or self.state is None:
            return
        if self.state.pose.covariance[0] < 0.0:
            self.get_logger().warn('no DVL: position unknown, waiting', throttle_duration_sec=5.0)
            return
        if self.i < 0:
            self.start_step()
            return

        t = self.now()
        if t - self.t_log > 2.0:
            self.t_log = t
            x, y, depth, yaw = self.pose()
            self.get_logger().info(
                f'  at ({x:+.2f}, {y:+.2f}) depth {depth:.2f} yaw {math.degrees(yaw):+.1f} deg')

        if self.check():
            if self.t_inside is None:
                self.t_inside = t
            elif t - self.t_inside >= self.settle:
                self.get_logger().info(f'  done in {t - self.t_step:.1f} s')
                self.start_step()
        else:
            self.t_inside = None
            if t - self.t_step > self.step_timeout:
                self.get_logger().error(
                    f'step "{self.steps[self.i]}" timed out after {self.step_timeout:.0f} s; '
                    'mission stopped, holding position')
                self.finished = True


def main():
    rclpy.init()
    node = Mission()
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

#!/usr/bin/env python3
"""Live terminal view of everything the sub senses, estimates and commands.

    ros2 run robosub_sim dashboard          # Ctrl-C to quit

Left column: the raw sensor streams the estimator consumes, with their
message rates (a stream that dies shows 0.0 Hz). Right column: what the
estimator makes of them, then what the controller is commanding and what
each thruster is being told. Drive the sub from another terminal and watch
the numbers move. Subscribes only; changes nothing.
"""
from collections import deque
import math
import sys
import time

from geometry_msgs.msg import (
    Point,
    Twist,
    TwistWithCovarianceStamped,
    Vector3Stamped,
)
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

from robosub_sim.vehicle_profiles import profile_from_node

W = 40   # column width


class Rate:
    """Messages per second over a sliding 2 s window."""
    def __init__(self):
        self.t = deque()

    def tick(self):
        now = time.time()
        self.t.append(now)
        while self.t and now - self.t[0] > 2.0:
            self.t.popleft()

    def hz(self):
        while self.t and time.time() - self.t[0] > 2.0:
            self.t.popleft()
        return len(self.t) / 2.0


class Dashboard(Node):
    def __init__(self):
        super().__init__('dashboard')
        self.profile = profile_from_node(self)
        self.imu = None; self.depth = None; self.dvl = None; self.alt = None
        self.state = None; self.rpy = None; self.cmd = None
        self.thrust = {topic: None for topic in self.profile.THRUSTERS}
        self.sp = {'depth': None, 'heading': None, 'roll': None, 'position': None}
        self.rates = {k: Rate() for k in ('imu', 'depth', 'dvl', 'state', 'cmd')}

        def sub(T, topic, attr, rate=None):
            def cb(m):
                setattr(self, attr, m)
                if rate:
                    self.rates[rate].tick()
            self.create_subscription(T, topic, cb, 10)

        sub(Imu, '/imu/data_raw', 'imu', 'imu')
        sub(Float64, '/depth', 'depth', 'depth')
        sub(TwistWithCovarianceStamped, '/dvl/velocity', 'dvl', 'dvl')
        sub(Float64, '/dvl/altitude', 'alt')
        sub(Odometry, '/sub/state', 'state', 'state')
        sub(Vector3Stamped, '/sub/state/rpy', 'rpy')
        sub(Twist, '/sub/cmd_vel', 'cmd', 'cmd')
        for k in self.thrust:
            self.create_subscription(
                Float64, f'{self.profile.INDIVIDUAL_TOPIC_PREFIX}/{k}',
                lambda m, k=k: self.thrust.__setitem__(k, m.data), 10)
        for k in ('depth', 'heading', 'roll'):
            self.create_subscription(Float64, f'/sub/setpoint/{k}',
                                     lambda m, k=k: self.sp.__setitem__(k, m.data), 10)
        self.create_subscription(Point, '/sub/setpoint/position',
                                 lambda m: self.sp.__setitem__('position', (m.x, m.y)), 10)
        self.create_subscription(Twist, '/sub/cmd_vel_manual', lambda m: None, 10)
        self.t0 = time.time()
        self.create_timer(0.2, self.draw)

    # ------------------------------------------------------------ columns
    def left(self):
        L = []
        r = self.rates
        L.append(f'IMU  /imu/data_raw        {r["imu"].hz():6.1f} Hz')
        if self.imu:
            a, g = self.imu.linear_acceleration, self.imu.angular_velocity
            L.append(f'  accel m/s2 {a.x:+6.2f} {a.y:+6.2f} {a.z:+6.2f}')
            L.append(f'  gyro  rad/s {g.x:+6.3f} {g.y:+6.3f} {g.z:+6.3f}')
        else:
            L += ['  (waiting)', '']
        L.append('')
        L.append(f'DEPTH /depth              {r["depth"].hz():6.1f} Hz')
        L.append(f'  {self.depth.data:6.3f} m  (+down)' if self.depth else '  (waiting)')
        L.append('')
        L.append(f'DVL  /dvl/velocity        {r["dvl"].hz():6.1f} Hz')
        if self.dvl:
            v = self.dvl.twist.twist.linear
            L.append(f'  vel m/s     {v.x:+6.3f} {v.y:+6.3f} {v.z:+6.3f}')
        else:
            L.append('  (no bottom lock / waiting)')
        L.append(f'  altitude    {self.alt.data:6.2f} m' if self.alt else '  altitude    -')
        L.append('')
        L.append('body frame: x fwd, y port, z up')
        return L

    def right(self):
        R = []
        r = self.rates
        R.append(f'ESTIMATOR /sub/state      {r["state"].hz():6.1f} Hz')
        if self.state:
            p = self.state.pose.pose.position
            v = self.state.twist.twist.linear
            w = self.state.twist.twist.angular
            lock = 'DVL ok' if self.state.pose.covariance[0] >= 0 else 'NO DVL: x/y frozen'
            R.append(f'  pos m       {p.x:+6.2f} {p.y:+6.2f}   depth {-p.z:5.2f}')
            if self.rpy:
                e = self.rpy.vector
                R.append(f'  r/p/y deg   {math.degrees(e.x):+6.1f} {math.degrees(e.y):+6.1f} {math.degrees(e.z):+7.1f}')
            R.append(f'  body vel    {v.x:+6.2f} {v.y:+6.2f} {v.z:+6.2f}')
            R.append(f'  rates       {w.x:+6.2f} {w.y:+6.2f} {w.z:+6.2f}')
            R.append(f'  {lock}')
        else:
            R += ['  (waiting for IMU + depth)', '', '', '', '']
        R.append('')
        sp = self.sp
        f = lambda v, u: '-' if v is None else f'{v:.2f}{u}'
        pos = '-' if sp['position'] is None else f'({sp["position"][0]:.1f},{sp["position"][1]:.1f})'
        R.append('SETPOINTS')
        deg = lambda v: '-' if v is None else f'{math.degrees(v):.0f}deg'
        R.append(f'  depth {f(sp["depth"], "m")}  hdg {deg(sp["heading"])}'
                 f'  roll {deg(sp["roll"])}  pos {pos}')
        R.append('')
        R.append(f'CMD_VEL (controller/manual) {r["cmd"].hz():5.1f} Hz')
        if self.cmd:
            c = self.cmd
            R.append(f'  surge {c.linear.x:+5.2f} sway {c.linear.y:+5.2f} heave {c.linear.z:+5.2f}')
            R.append(f'  roll  {c.angular.x:+5.2f} yaw  {c.angular.z:+5.2f}')
        else:
            R += ['  (idle)', '']
        R.append('')
        t = self.thrust
        fmt = lambda k: '   -  ' if t[k] is None else f'{t[k]:+6.2f}'
        unit = 'POWER' if self.profile.ACTUATOR_KIND == 'm5' else 'THRUST N'
        labels = ' '.join(
            f'{index}:{name:2}'
            for index, name in enumerate(self.profile.THRUSTER_NAMES)
        )
        R.append(f'{unit}  {labels}')
        R.append('        ' + ' '.join(
            fmt(topic) for topic in self.profile.THRUSTERS
        ))
        return R

    def draw(self):
        L, R = self.left(), self.right()
        n = max(len(L), len(R))
        L += [''] * (n - len(L)); R += [''] * (n - len(R))
        out = ['\033[H\033[J' + f'ROBOSUB LIVE   {time.time() - self.t0:6.1f} s   Ctrl-C to quit',
               '=' * (2 * W + 3)]
        for a, b in zip(L, R):
            out.append(f'{a:<{W}} | {b}')
        sys.stdout.write('\n'.join(out) + '\n')
        sys.stdout.flush()


def main():
    rclpy.init()
    node = Dashboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

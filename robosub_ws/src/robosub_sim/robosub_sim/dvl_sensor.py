#!/usr/bin/env python3
"""Stand-in for the Teledyne Pathfinder DVL driver: /dvl/velocity, /dvl/altitude.

WHY THIS NODE EXISTS. The vehicle has a Pathfinder OEM DVL (Doppler
velocity log). It pings the pool floor and reports the sub's velocity over
the ground in its own frame, plus its altitude above the floor. Gazebo has
no such sensor here, so this node manufactures the same two topics from
ground truth, with the Pathfinder's character bolted on:

    - pings at a fixed rate (~7 Hz), not 100 Hz
    - a few mm/s of white noise per ping and a small scale error
    - NO velocity when it has no bottom lock (floor too close / too far,
      or a random dropout) -- real drivers go quiet, they do not send zeros
    - lever arm: the DVL is not at the centre of rotation, so while the sub
      yaws the DVL sees an extra  omega x r  that the estimator must remove

On the vehicle, replace this node with the Pathfinder serial driver and
publish the same two topics; nothing above changes.

    /dvl/velocity   geometry_msgs/TwistWithCovarianceStamped
                    linear = velocity over ground in the DVL frame (m/s),
                    published velocity frame == body frame (x fwd, y port, z up) here;
                    mounting yaw belongs in the real driver's config
    /dvl/altitude   std_msgs/Float64   metres above the floor (only when valid)

Parameters
    rate            ping rate, Hz                          (7.0)
    noise_std       per-ping velocity noise, m/s           (0.004)
    scale_error     fractional scale error                 (0.002)
    dvl_offset      DVL position in body frame, m          ([-0.153, 0.007, -0.016])
    floor_z         world z of the pool floor              (-9.5, from pool.sdf)
    min_altitude    below this: no bottom lock             (0.2)
    max_altitude    above this: no bottom lock             (80.0)
    dropout_prob    chance a ping is lost                  (0.01)
"""
import random

from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from robosub_sim import constants
from robosub_sim.vehicle_profiles import profile_from_node


class DvlSensor(Node):
    def __init__(self):
        super().__init__('dvl_sensor')
        profile = profile_from_node(self)
        for name, default in constants.DVL_DEFAULTS.items():
            if name == "dvl_offset":
                default = list(profile.DVL_POSITION_BODY_M)
            self.declare_parameter(name, default)
        g = lambda n: self.get_parameter(n).value
        self.noise = float(g('noise_std')); self.scale = 1.0 + float(g('scale_error'))
        self.r = [float(v) for v in g('dvl_offset')]
        self.floor_z = float(g('floor_z'))
        self.min_alt, self.max_alt = float(g('min_altitude')), float(g('max_altitude'))
        self.dropout = float(g('dropout_prob')); self.frame_id = str(g('frame_id'))

        self.odom = None
        self.create_subscription(Odometry, '/sub/odom', self.on_odom, 10)
        self.pub_vel = self.create_publisher(TwistWithCovarianceStamped, '/dvl/velocity', 10)
        self.pub_alt = self.create_publisher(Float64, '/dvl/altitude', 10)
        self.create_timer(1.0 / float(g('rate')), self.periodic)
        self.locked = None
        self.misses = 0          # consecutive pings without bottom lock
        self.get_logger().info(
            f'dvl_sensor up: {g("rate")} Hz, noise {self.noise} m/s, offset {self.r}')

    def on_odom(self, msg):
        self.odom = msg

    def periodic(self):
        """Generate one simulated DVL ping on each sensor timer tick."""
        if self.odom is None:
            return
        o = self.odom
        altitude = o.pose.pose.position.z - self.floor_z
        locked = (self.min_alt <= altitude <= self.max_alt
                  and random.random() >= self.dropout)
        # Log only sustained changes: a single dropped ping is normal.
        self.misses = 0 if locked else self.misses + 1
        if locked and self.locked is not True:
            self.locked = True
            self.get_logger().info(f'bottom lock acquired (altitude {altitude:.2f} m)')
        elif self.misses == 5:
            self.locked = False
            self.get_logger().warn(f'bottom lock LOST (altitude {altitude:.2f} m)')
        if not locked:
            return

        self.pub_alt.publish(Float64(data=altitude))

        v = o.twist.twist.linear          # body frame (OdometryPublisher, 3D)
        w = o.twist.twist.angular         # body frame
        rx, ry, rz = self.r
        # velocity seen at the DVL = v + omega x r
        vx = v.x + (w.y * rz - w.z * ry)
        vy = v.y + (w.z * rx - w.x * rz)
        vz = v.z + (w.x * ry - w.y * rx)

        m = TwistWithCovarianceStamped()
        m.header.stamp = o.header.stamp
        m.header.frame_id = self.frame_id
        m.twist.twist.linear.x = vx * self.scale + random.gauss(0, self.noise)
        m.twist.twist.linear.y = vy * self.scale + random.gauss(0, self.noise)
        m.twist.twist.linear.z = vz * self.scale + random.gauss(0, self.noise)
        var = self.noise ** 2
        m.twist.covariance[0] = var; m.twist.covariance[7] = var; m.twist.covariance[14] = var
        self.pub_vel.publish(m)


def main():
    rclpy.init()
    node = DvlSensor()
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

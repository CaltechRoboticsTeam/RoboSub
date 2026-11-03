#!/usr/bin/env python3
"""Synthesize body-frame DVL velocity and bottom-lock loss from Gazebo."""

import random

from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from robosub_control.autonomy.sensor_freshness import (
    SensorFreshness,
    stamp_to_sec,
)
from robosub_control.config import constants
from robosub_control.config.vehicle_profiles import profile_from_node
from robosub_control.utils.tuning import fixed_parameter
from std_msgs.msg import Float64


class DvlSensor(Node):
    def __init__(self):
        super().__init__("dvl_sensor")
        profile = profile_from_node(self)
        for name, default in constants.DVL_DEFAULTS.items():
            if name == "dvl_offset":
                default = list(profile.DVL_POSITION_BODY_M)
            fixed_parameter(self, name, default)

        def g(name):
            return self.get_parameter(name).value

        self.noise = float(g("noise_std"))
        self.scale = 1.0 + float(g("scale_error"))
        self.r = [float(v) for v in g("dvl_offset")]
        self.floor_z = float(g("floor_z"))
        self.min_alt, self.max_alt = (
            float(g("min_altitude")),
            float(g("max_altitude")),
        )
        self.dropout = float(g("dropout_prob"))
        self.frame_id = str(g("frame_id"))
        fixed_parameter(
            self, "odom_timeout", constants.CONTROLLER_DEFAULTS["state_timeout"]
        )
        self.odom_samples = SensorFreshness(float(g("odom_timeout")))

        self.odom = None
        self.last_clock_t = None
        self.last_ping_stamp = None
        self.create_subscription(Odometry, "/sub/odom", self.on_odom, 10)
        self.pub_vel = self.create_publisher(
            TwistWithCovarianceStamped, "/dvl/velocity", 10
        )
        self.pub_alt = self.create_publisher(Float64, "/dvl/altitude", 10)
        self.create_timer(1.0 / float(g("rate")), self.periodic)
        self.locked = None
        self.misses = 0  # consecutive pings without bottom lock
        self.get_logger().info(
            f"dvl_sensor up: {g('rate')} Hz, noise {self.noise} m/s, "
            f"offset {self.r}"
        )

    def on_odom(self, msg):
        now = self.now()
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular
        if not self.odom_samples.accept(
            stamp_to_sec(msg.header.stamp),
            now,
            (msg.pose.pose.position.z, v.x, v.y, v.z, w.x, w.y, w.z),
        ):
            return
        self.odom = msg

    def now(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_clock_t is not None and now < self.last_clock_t:
            self.odom_samples.reset()
            self.odom = None
            self.last_ping_stamp = None
            self.locked = None
            self.misses = 0
        self.last_clock_t = now
        return now

    def periodic(self):
        """Generate one simulated DVL ping on each sensor timer tick."""
        now = self.now()
        if self.odom is None or not self.odom_samples.fresh(now):
            return
        if self.odom_samples.stamp == self.last_ping_stamp:
            return
        self.last_ping_stamp = self.odom_samples.stamp
        o = self.odom
        altitude = o.pose.pose.position.z - self.floor_z
        locked = (
            self.min_alt <= altitude <= self.max_alt
            and random.random() >= self.dropout
        )
        # Log only sustained changes: a single dropped ping is normal.
        self.misses = 0 if locked else self.misses + 1
        if locked and self.locked is not True:
            self.locked = True
            self.get_logger().info(
                f"bottom lock acquired (altitude {altitude:.2f} m)"
            )
        elif self.misses == 5:
            self.locked = False
            self.get_logger().warn(
                f"bottom lock LOST (altitude {altitude:.2f} m)"
            )
        if not locked:
            return

        self.pub_alt.publish(Float64(data=altitude))

        v = o.twist.twist.linear  # body frame (OdometryPublisher, 3D)
        w = o.twist.twist.angular  # body frame
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
        var = self.noise**2
        m.twist.covariance[0] = var
        m.twist.covariance[7] = var
        m.twist.covariance[14] = var
        self.pub_vel.publish(m)


def main():
    rclpy.init()
    node = DvlSensor()
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

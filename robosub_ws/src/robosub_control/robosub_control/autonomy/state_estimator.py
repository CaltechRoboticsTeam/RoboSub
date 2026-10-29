#!/usr/bin/env python3
"""Fuse the IMU and the depth sensor into one state estimate on /sub/state.

WHY THIS NODE EXISTS. Controllers should never read sensors directly: a
depth PID that subscribes to /depth and a heading PID that subscribes to
/imu/data_raw each end up with their own filtering, their own sign
conventions and their own bugs. This node is the single place that turns
raw sensors into "where is the sub and how is it moving". Everything above
it consumes /sub/state and nothing else.

It consumes ONLY the topics the real vehicle will have:

    /imu/data_raw   sensor_msgs/Imu   body rates + specific force, 100 Hz
    /depth          std_msgs/Float64  metres below surface, +down, 50 Hz
    /dvl/velocity   geometry_msgs/TwistWithCovarianceStamped
                                      velocity over ground, DVL frame, ~7 Hz

The Imu message's `orientation` field is deliberately IGNORED. Gazebo fills
it with ground truth; a raw IMU driver on the vehicle may not fill it at
all. Attitude is computed here from rates + accelerometer so the node
behaves the same on both.

WHAT IT PUBLISHES

    /sub/state      nav_msgs/Odometry
        pose.position.x/y    metres from where this node started, in the
                             frame whose x axis is the sub's heading at
                             start (yaw 0). Integrated DVL velocity.
        pose.position.z      world z, metres, surface = 0 (so z = -depth)
        pose.orientation     body -> world quaternion
        pose.covariance[0]   -1.0 while the DVL is silent (no bottom lock):
                             x/y are frozen and must not be trusted
        twist.linear.x/y     body-frame velocity from the DVL (m/s), 0 if stale
        twist.angular        bias-corrected body rates (rad/s)
        twist.linear.z       heave rate, +up (m/s)
    /sub/state/rpy  geometry_msgs/Vector3Stamped   roll, pitch, yaw in rad,
                    for humans: `ros2 topic echo /sub/state/rpy`

FRAMES. Body is x forward, y port, z up (same as thruster_mixer). Yaw is
positive counter-clockwise seen from above. Yaw is RELATIVE: it starts at
0 wherever the sub points when this node starts, because a gyro can only
integrate change. Without a magnetometer or DVL that is all an IMU knows.

HOW THE ATTITUDE FILTER WORKS (Mahony complementary filter)

    1. Integrate the gyro: q <- q + 0.5 * q (x) omega * dt.  Fast and
       accurate short-term, but every bias and every bit of noise
       accumulates, so roll/pitch drift within minutes.
    2. The accelerometer at rest measures the reaction to gravity, which
       points "up" in the body frame. Predict where "up" should be from the
       current q, compare with the measurement, and nudge the gyro rate by
       kp * (measured x predicted). This slowly pulls roll and pitch back
       to gravity. ki integrates the same error into a gyro bias estimate.
    3. Yaw gets no correction (gravity says nothing about heading), so it
       integrates the z gyro. In sim the bias is ~1e-5 rad/s: negligible.

    kp sets how much you trust the accelerometer. Too high and thruster
    surges show up as fake tilt; too low and roll/pitch drift. 1-3 is the
    usual range for a slow vehicle.

HOW POSITION WORKS. The DVL measures velocity at ITS location, so while
the sub turns it also sees omega x r (r = DVL offset from the centre of
rotation). Remove that, rotate the body velocity into the world frame with
the current attitude, integrate. Drift is ~0.2-1 % of distance travelled
plus whatever yaw error you have, which is why yaw matters so much.

Parameters
    dvl_offset     DVL position in body frame, m          (from vehicle profile)
    dvl_timeout    s without a DVL ping -> velocity = 0   (default 1.0)
    kp             accelerometer correction gain          (default 2.0)
    ki             gyro bias learning gain                (default 0.02)
    depth_tau      depth low-pass time constant, s        (default 0.08)
    heave_tau      heave-rate low-pass time constant, s   (default 0.25)
"""

import math

from geometry_msgs.msg import TwistWithCovarianceStamped, Vector3Stamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

from robosub_control.config import constants
from robosub_control.utils.tuning import TunableNumbers, fixed_parameter
from robosub_control.config.vehicle_profiles import profile_from_node

G = 9.81


# --- tiny quaternion toolkit, q = (w, x, y, z), body -> world -------------
def q_mult(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def q_normalize(q):
    n = math.sqrt(sum(c * c for c in q))
    return tuple(c / n for c in q)


def world_up_in_body(q):
    """Rotate the world's +z into the body frame: R(q)^T * (0, 0, 1)."""
    w, x, y, z = q
    return (
        2 * (x * z - w * y),
        2 * (y * z + w * x),
        w * w - x * x - y * y + z * z,
    )


def rotate(q, v):
    """Rotate a body-frame vector into the world frame: q * v * q^-1."""
    w, x, y, z = q
    vx, vy, vz = v
    # t = 2 * (q_vec x v)
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def q_to_rpy(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    s = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(s)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class StateEstimator(Node):
    def __init__(self):
        super().__init__("state_estimator")
        profile = profile_from_node(self)
        self.r_dvl = [
            float(value)
            for value in fixed_parameter(
                self, "dvl_offset", list(profile.DVL_POSITION_BODY_M)
            )
        ]
        self.dvl_timeout = float(
            fixed_parameter(
                self, "dvl_timeout", constants.ESTIMATOR_DEFAULTS["dvl_timeout"]
            )
        )
        self.tuning = TunableNumbers(self)
        for name in ("kp", "ki", "depth_tau", "heave_tau"):
            number = self.tuning.number(
                name,
                constants.ESTIMATOR_DEFAULTS[name],
                minimum=1e-6 if name.endswith("_tau") else 0.0,
                description=name.replace("_", " "),
            )
            setattr(self, name, number.get())
            number.bind_to(self, name)

        # Attitude state.
        self.q = (1.0, 0.0, 0.0, 0.0)
        self.bias = [0.0, 0.0, 0.0]
        self.rates = (0.0, 0.0, 0.0)
        self.last_imu_t = None

        # Depth state.
        self.depth = None  # filtered, m, +down
        self.heave_up = 0.0  # m/s, +up
        self.last_depth_t = None
        self.last_depth_raw_t = None

        # Position state (DVL).
        self.x = 0.0
        self.y = 0.0
        self.v_body = (
            0.0,
            0.0,
            0.0,
        )  # last DVL body velocity, lever arm removed
        self.v_world_prev = None
        self.last_dvl_t = None  # message stamp, for integration dt
        self.last_dvl_rx = None  # node clock, for the timeout

        self.pub_state = self.create_publisher(Odometry, "/sub/state", 10)
        self.pub_rpy = self.create_publisher(
            Vector3Stamped, "/sub/state/rpy", 10
        )
        self.create_subscription(Imu, "/imu/data_raw", self.on_imu, 50)
        self.create_subscription(Float64, "/depth", self.on_depth, 50)
        self.create_subscription(
            TwistWithCovarianceStamped, "/dvl/velocity", self.on_dvl, 20
        )

        self.get_logger().info(
            f"state_estimator up: /imu/data_raw + /depth -> /sub/state "
            f"(kp={self.kp}, ki={self.ki}); yaw starts at 0 here"
        )

    # ---------------------------------------------------------------- depth
    def on_depth(self, msg: Float64):
        t = self.get_clock().now().nanoseconds * 1e-9
        raw = msg.data
        if self.depth is None:
            self.depth = raw
            self.last_depth_t = t
            return
        dt = t - self.last_depth_t
        self.last_depth_t = t
        if not (0.0 < dt < 0.5):
            return
        prev = self.depth
        a = dt / (self.depth_tau + dt)
        self.depth += a * (raw - self.depth)
        # Heave from the filtered depth, then smoothed again: differentiating
        # a 1 cm-noise signal at 50 Hz is otherwise 0.5 m/s of garbage.
        raw_rate_down = (self.depth - prev) / dt
        b = dt / (self.heave_tau + dt)
        self.heave_up += b * (-raw_rate_down - self.heave_up)

    # ------------------------------------------------------------------ dvl
    def on_dvl(self, msg: TwistWithCovarianceStamped):
        t = stamp_to_sec(msg.header.stamp)
        self.last_dvl_rx = self.get_clock().now().nanoseconds * 1e-9
        v = msg.twist.twist.linear
        wx, wy, wz = self.rates
        rx, ry, rz = self.r_dvl
        # The DVL saw v_dvl = v_body + omega x r. Undo the lever arm.
        vb = (
            v.x - (wy * rz - wz * ry),
            v.y - (wz * rx - wx * rz),
            v.z - (wx * ry - wy * rx),
        )
        self.v_body = vb
        vw = rotate(self.q, vb)
        if self.last_dvl_t is not None:
            dt = t - self.last_dvl_t
            if 0.0 < dt < self.dvl_timeout and self.v_world_prev is not None:
                # trapezoid: average of the last two samples
                self.x += 0.5 * (vw[0] + self.v_world_prev[0]) * dt
                self.y += 0.5 * (vw[1] + self.v_world_prev[1]) * dt
        self.last_dvl_t = t
        self.v_world_prev = vw

    def dvl_fresh(self):
        return (
            self.last_dvl_rx is not None
            and self.get_clock().now().nanoseconds * 1e-9 - self.last_dvl_rx
            < self.dvl_timeout
        )

    # ------------------------------------------------------------------ imu
    def on_imu(self, msg: Imu):
        t = stamp_to_sec(msg.header.stamp)
        if self.last_imu_t is None:
            self.last_imu_t = t
            return
        dt = t - self.last_imu_t
        self.last_imu_t = t
        if not (0.0 < dt < 0.1):
            return  # first message, a pause, or a sim reset: skip the step

        w = msg.angular_velocity
        a = msg.linear_acceleration
        gx, gy, gz = w.x - self.bias[0], w.y - self.bias[1], w.z - self.bias[2]

        # Accelerometer correction. Only trust it when the magnitude looks
        # like gravity alone (the sub is slow, so this is nearly always).
        norm = math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)
        if 0.5 * G < norm < 1.5 * G:
            ax, ay, az = a.x / norm, a.y / norm, a.z / norm
            vx, vy, vz = world_up_in_body(self.q)
            ex = ay * vz - az * vy
            ey = az * vx - ax * vz
            ez = ax * vy - ay * vx
            self.bias[0] -= self.ki * ex * dt
            self.bias[1] -= self.ki * ey * dt
            self.bias[2] -= self.ki * ez * dt
            gx += self.kp * ex
            gy += self.kp * ey
            gz += self.kp * ez

        # Integrate: q_dot = 0.5 * q (x) (0, omega_body)
        dq = q_mult(self.q, (0.0, gx, gy, gz))
        self.q = q_normalize(
            tuple(c + 0.5 * d * dt for c, d in zip(self.q, dq))
        )
        self.rates = (
            w.x - self.bias[0],
            w.y - self.bias[1],
            w.z - self.bias[2],
        )

        self.publish(msg.header.stamp)

    # -------------------------------------------------------------- publish
    def publish(self, stamp):
        if self.depth is None:
            return  # no depth yet: a state with a made-up z is worse than none

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "world"
        odom.child_frame_id = "base_link"
        fresh = self.dvl_fresh()
        if not fresh:
            self.v_body = (0.0, 0.0, 0.0)
            self.v_world_prev = None
            self.get_logger().warn(
                "no DVL: position frozen", throttle_duration_sec=5.0
            )
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = -self.depth
        odom.pose.covariance[0] = 0.0 if fresh else -1.0
        odom.twist.twist.linear.x = self.v_body[0]
        odom.twist.twist.linear.y = self.v_body[1]
        qw, qx, qy, qz = self.q
        odom.pose.pose.orientation.w = qw
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.twist.twist.angular.x = self.rates[0]
        odom.twist.twist.angular.y = self.rates[1]
        odom.twist.twist.angular.z = self.rates[2]
        odom.twist.twist.linear.z = self.heave_up
        self.pub_state.publish(odom)

        r, p, y = q_to_rpy(self.q)
        rpy = Vector3Stamped()
        rpy.header = odom.header
        rpy.vector.x, rpy.vector.y, rpy.vector.z = r, p, y
        self.pub_rpy.publish(rpy)


def main():
    rclpy.init()
    node = StateEstimator()
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

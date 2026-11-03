"""Adapt existing MAVROS and Teensy sensors to the shared estimator."""

from copy import deepcopy
import math

from geometry_msgs.msg import TwistWithCovarianceStamped
from mavros_msgs.msg import DebugValue
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

from robosub_control.autonomy.sensor_freshness import (
    SensorFreshness,
    stamp_to_sec,
)
from robosub_control.utils.tuning import fixed_parameter


class HardwareSensors(Node):
    def __init__(self, **kwargs):
        super().__init__("hardware_sensors", **kwargs)
        self.depth_source = fixed_parameter(self, "depth_source", "auto")
        if self.depth_source not in ("auto", "usb", "mavros"):
            raise ValueError("depth_source must be auto, usb or mavros")
        self.imu_frame = fixed_parameter(self, "imu_frame", "base_link")
        self.dvl_frame = fixed_parameter(self, "dvl_frame", "base_link")
        self.imu_samples = SensorFreshness(
            fixed_parameter(self, "imu_timeout", 0.5)
        )
        self.dvl_samples = SensorFreshness(
            fixed_parameter(self, "dvl_timeout", 1.0)
        )
        self.usb_depth_samples = SensorFreshness(
            fixed_parameter(self, "depth_timeout", 0.5)
        )
        self.active_depth_source = None
        # Reliable outputs also serve the estimator's reliable subscriptions.
        self.imu_pub = self.create_publisher(Imu, "/imu/data_raw", 10)
        self.depth_pub = self.create_publisher(Float64, "/depth", 10)
        self.dvl_pub = self.create_publisher(
            TwistWithCovarianceStamped, "/dvl/velocity", 10
        )
        for message_type, topic, callback in (
            (Imu, "/mavros/imu/data", self.on_imu),
            (
                DebugValue,
                "/mavros/debug_value/named_value_float",
                self.on_depth,
            ),
            (Float64, "/sub/sensors/depth_usb", self.on_usb_depth),
            (Odometry, "/mavros/odometry/out", self.on_dvl),
        ):
            self.create_subscription(
                message_type, topic, callback, qos_profile_sensor_data
            )
        self.get_logger().info(
            "Hardware sensor interface: MAVROS IMU, USB/MAVROS DEPTH "
            f"(source={self.depth_source}), legacy DVL odometry twist."
        )

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def rejected(self, sensor):
        self.get_logger().warning(
            f"Rejected {sensor}: check source stamp, frame and values.",
            throttle_duration_sec=2.0,
        )

    def on_imu(self, message):
        angular = message.angular_velocity
        acceleration = message.linear_acceleration
        covariances = (
            *message.angular_velocity_covariance,
            *message.linear_acceleration_covariance,
        )
        if (
            message.header.frame_id != self.imu_frame
            or message.angular_velocity_covariance[0] < 0
            or message.linear_acceleration_covariance[0] < 0
            or not self.imu_samples.accept(
                stamp_to_sec(message.header.stamp),
                self.now(),
                (
                    angular.x,
                    angular.y,
                    angular.z,
                    acceleration.x,
                    acceleration.y,
                    acceleration.z,
                    *covariances,
                ),
            )
        ):
            self.rejected("IMU")
            return
        self.imu_pub.publish(message)

    def publish_depth(self, value, source):
        if self.active_depth_source != source:
            self.get_logger().info(f"Pressure depth source: {source}")
            self.active_depth_source = source
        self.depth_pub.publish(Float64(data=value))

    def on_usb_depth(self, message):
        if self.depth_source == "mavros":
            return
        now = self.now()
        if not self.usb_depth_samples.accept(now, now, (message.data,)):
            self.rejected("USB depth")
            return
        self.publish_depth(message.data, "usb")

    def on_depth(self, message):
        if message.name != "DEPTH" or self.depth_source == "usb":
            return
        if not math.isfinite(message.value_float):
            self.rejected("MAVROS depth")
            return
        if self.depth_source == "auto" and self.usb_depth_samples.fresh(
            self.now()
        ):
            return
        # Teensy uptime in NAMED_VALUE_FLOAT is not the Pixhawk's clock.
        # The headerless /depth contract uses receipt time on both paths.
        self.publish_depth(float(message.value_float), "mavros")

    def on_dvl(self, message):
        velocity = message.twist.twist.linear
        covariance = message.twist.covariance
        if (
            message.child_frame_id != self.dvl_frame
            or any(covariance[index] < 0 for index in (0, 7, 14))
            or not self.dvl_samples.accept(
                stamp_to_sec(message.header.stamp),
                self.now(),
                (velocity.x, velocity.y, velocity.z, *covariance),
            )
        ):
            self.rejected("DVL")
            return
        output = TwistWithCovarianceStamped()
        output.header = deepcopy(message.header)
        output.header.frame_id = "pixhawk_link"
        output.twist = deepcopy(message.twist)
        self.dvl_pub.publish(output)


def main():
    rclpy.init()
    node = HardwareSensors()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

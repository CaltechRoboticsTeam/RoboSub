"""Feed Pixhawk IMU, Teensy depth and DVL velocity to the estimator."""

from copy import deepcopy

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
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
        self.imu_frame = fixed_parameter(self, "imu_frame", "base_link")
        self.dvl_frame = fixed_parameter(self, "dvl_frame", "base_link")
        self.imu_samples = SensorFreshness(
            fixed_parameter(self, "imu_timeout", 0.5)
        )
        self.dvl_samples = SensorFreshness(
            fixed_parameter(self, "dvl_timeout", 1.0)
        )
        self.depth_samples = SensorFreshness(
            fixed_parameter(self, "depth_timeout", 0.5)
        )
        # Reliable outputs also serve the estimator's reliable subscriptions.
        self.imu_pub = self.create_publisher(Imu, "/imu/data_raw", 10)
        self.depth_pub = self.create_publisher(Float64, "/depth", 10)
        self.dvl_pub = self.create_publisher(
            TwistWithCovarianceStamped, "/dvl/velocity", 10
        )
        for message_type, topic, callback in (
            (Imu, "/mavros/imu/data", self.on_imu),
            (Float64, "/sub/sensors/depth_usb", self.on_depth),
            (Odometry, "/sub/sensors/dvl/odometry", self.on_dvl),
        ):
            self.create_subscription(
                message_type, topic, callback, qos_profile_sensor_data
            )
        self.get_logger().info(
            "Sensors: Pixhawk IMU, Teensy depth and Teledyne DVL."
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

    def on_depth(self, message):
        now = self.now()
        if not self.depth_samples.accept(now, now, (message.data,)):
            self.rejected("depth")
            return
        self.depth_pub.publish(message)

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
        output.header.frame_id = "dvl_link"
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

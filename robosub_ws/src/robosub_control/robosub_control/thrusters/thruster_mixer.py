"""Allocate normalized body efforts using the selected vehicle profile.

T200 targets are newtons. M5 targets are normalized power fractions using an
uncalibrated equal-authority approximation. Pitch is not independently
controllable. Named motor order is FR,FL,BR,BL,RD,LD; no bus is written.
"""

import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray, MultiArrayDimension

from robosub_control.config import constants
from robosub_control.thrusters.thrust_allocation import AXES, ThrustAllocator
from robosub_control.config.vehicle_profiles import profile_from_node


class ThrusterMixer(Node):
    def __init__(self):
        super().__init__("thruster_mixer")
        self.profile = profile = profile_from_node(self)
        self.allocator = ThrustAllocator(vehicle=profile.NAME)
        self.pubs = {
            name: self.create_publisher(
                Float64,
                f"{profile.INDIVIDUAL_TOPIC_PREFIX}/{profile.THRUSTER_TOPIC_IDS[name]}",
                1,
            )
            for name in self.profile.THRUSTER_NAMES
        }
        self.batch_pub = self.create_publisher(
            Float64MultiArray, profile.TARGET_TOPIC, 1
        )
        self.create_subscription(Twist, "/sub/cmd_vel", self.on_cmd, 1)
        self.last_cmd = self.get_clock().now()
        self.create_timer(constants.THRUSTER_WATCHDOG_PERIOD_S, self.periodic)
        if self.allocator.is_m5:
            self.get_logger().warn(
                "current_m5: normalized power allocation; power-to-force "
                "calibration is unknown. Simulation uses a force proxy."
            )
        else:
            scales = ", ".join(
                f"{axis}={scale:.3f} {'N' if index < 3 else 'N.m'}"
                for index, (axis, scale) in enumerate(
                    zip(AXES, self.allocator.axis_scales, strict=True)
                )
            )
            self.get_logger().info(
                f"future_t200: {self.allocator.model.voltage_v} V reference; "
                f"per unit effort: {scales}. Mounts are a planning copy."
            )

    def on_cmd(self, msg):
        self.last_cmd = self.get_clock().now()
        try:
            if not math.isfinite(msg.angular.y) or msg.angular.y != 0.0:
                raise ValueError("Independent pitch control is unavailable")
            allocation = self.allocator.allocate(
                (
                    msg.linear.x,
                    msg.linear.y,
                    msg.linear.z,
                    msg.angular.x,
                    msg.angular.z,
                )
            )
            self.publish_targets(allocation.targets)
        except ValueError as error:
            self.get_logger().error(str(error), throttle_duration_sec=2.0)
            self.publish_targets((0.0,) * len(self.profile.THRUSTER_NAMES))

    def publish_targets(self, forces):
        for name, force in zip(
            self.profile.THRUSTER_NAMES, forces, strict=True
        ):
            self.pubs[name].publish(Float64(data=float(force)))
        batch = Float64MultiArray(data=list(forces))
        batch.layout.dim = [
            MultiArrayDimension(
                label=",".join(self.profile.THRUSTER_NAMES),
                size=len(forces),
                stride=len(forces),
            )
        ]
        self.batch_pub.publish(batch)

    def periodic(self):
        """Zero targets if commands stop or the clock jumps backward."""
        age = (self.get_clock().now() - self.last_cmd).nanoseconds * 1e-9
        if age < 0 or age > constants.THRUSTER_COMMAND_TIMEOUT_S:
            self.publish_targets((0.0,) * len(self.profile.THRUSTER_NAMES))


def main():
    rclpy.init()
    node = ThrusterMixer()
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


if __name__ == "__main__":
    main()

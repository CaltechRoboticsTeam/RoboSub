"""Publish actuator previews and simulated thruster forces."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import (
    Float64,
    Float64MultiArray,
    MultiArrayDimension,
    UInt16MultiArray,
)

from robosub_control.config import constants
from robosub_control.config.vehicle_profiles import profile_from_node
from robosub_control.thrusters.thrust_allocation import ThrustAllocator
from robosub_control.utils.tuning import fixed_parameter


class ActuatorCommands(Node):
    def __init__(self):
        super().__init__("actuator_commands")
        self.profile = profile = profile_from_node(self)
        self.simulation = fixed_parameter(self, "simulation", False)
        rate = fixed_parameter(
            self, "rate", constants.CONTROLLER_DEFAULTS["rate"]
        )
        output_rate = fixed_parameter(
            self, "output_rate", constants.CONTROLLER_DEFAULTS["output_rate"]
        )
        self.rate = min(rate, output_rate)
        self.allocator = ThrustAllocator(vehicle=profile.NAME)
        self.max_power = fixed_parameter(self, "max_power", 1.0)
        self.applied_commands = [0.0] * len(profile.THRUSTER_NAMES)
        self.last_force_time = None
        if self.simulation and self.allocator.is_m5:
            self.get_logger().info(
                f"M5: measured {self.allocator.model.voltage_v:g} V force "
                "curve with directional asymmetry and an estimated ramp."
            )
        self.started = None
        self.last_rx = None
        self.pending_commands = (self.profile.NEUTRAL_COMMAND,)
        self.pending_commands *= len(self.profile.THRUSTER_NAMES)
        self.preview_pub = self.create_publisher(
            Float64MultiArray if self.allocator.is_m5 else UInt16MultiArray,
            profile.PREVIEW_TOPIC,
            1,
        )
        self.force_pubs = (
            {
                name: self.create_publisher(
                    Float64,
                    f"/sim/thrust/{profile.THRUSTER_TOPIC_IDS[name]}",
                    1,
                )
                for name in self.profile.THRUSTER_NAMES
            }
            if self.simulation
            else {}
        )
        self.create_subscription(
            Float64MultiArray, profile.TARGET_TOPIC, self.on_targets, 1
        )
        self.create_timer(1.0 / self.rate, self.periodic)
        self.get_logger().info(
            f"{profile.NAME}: preview on {profile.PREVIEW_TOPIC}; "
            "physical output is handled separately."
        )

    def on_targets(self, message):
        try:
            dimensions = message.layout.dim
            if (
                len(dimensions) != 1
                or dimensions[0].label != ",".join(self.profile.THRUSTER_NAMES)
                or dimensions[0].size != len(self.profile.THRUSTER_NAMES)
                or dimensions[0].stride != len(self.profile.THRUSTER_NAMES)
                or message.layout.data_offset != 0
            ):
                raise ValueError(
                    "Thruster target array order is missing/invalid"
                )
            commands = self.allocator.commands(
                message.data, max_power=self.max_power
            )
        except ValueError as error:
            self.last_rx = None
            self.get_logger().error(str(error), throttle_duration_sec=2.0)
            return
        self.last_rx = self.get_clock().now()
        self.pending_commands = commands

    def publish_commands(self, commands):
        profile = self.profile
        if self.allocator.is_m5:
            message = Float64MultiArray(data=list(commands))
            label = ",".join(profile.THRUSTER_NAMES)
        else:
            message = UInt16MultiArray(data=list(commands))
            label = "board_channels_0_to_5_pwm_us"
        message.layout.dim = [
            MultiArrayDimension(
                label=label, size=len(commands), stride=len(commands)
            )
        ]
        self.preview_pub.publish(message)
        applied = commands
        if self.simulation and self.allocator.is_m5:
            now = self.get_clock().now().nanoseconds * 1e-9
            dt = (
                max(0.0, now - self.last_force_time)
                if self.last_force_time is not None
                else 0.0
            )
            delta = self.allocator.model.command_slew_rate * dt
            self.applied_commands = [
                current + max(-delta, min(delta, target - current))
                for current, target in zip(
                    self.applied_commands, commands, strict=True
                )
            ]
            self.last_force_time = now
            applied = self.applied_commands
        for index, (name, publisher) in enumerate(self.force_pubs.items()):
            if self.allocator.is_m5:
                force = self.allocator.model.force_from_command(
                    applied[index], profile.THRUSTER_REVERSED[name]
                )
            else:
                channel = profile.THRUSTER_CHANNELS[name]
                force = self.allocator.model.force_from_pwm(
                    commands[channel], profile.THRUSTER_REVERSED[name]
                )
            publisher.publish(Float64(data=force))

    def publish_neutral(self):
        self.applied_commands = [0.0] * len(self.profile.THRUSTER_NAMES)
        self.last_force_time = None
        self.publish_commands(
            (self.profile.NEUTRAL_COMMAND,) * len(self.profile.THRUSTER_NAMES)
        )

    def periodic(self):
        now = self.get_clock().now()
        if self.started is None:
            self.started = now
        running = (now - self.started).nanoseconds * 1e-9
        if running < 0:
            self.started = now
            self.last_rx = None
        hold = (
            self.profile.INITIAL_NEUTRAL_S
            if self.allocator.is_m5
            else self.profile.ESC_NEUTRAL_INIT_S
        )
        if running < hold:
            self.publish_neutral()
            return
        if self.last_rx is None:
            self.publish_neutral()
            return
        age = (now - self.last_rx).nanoseconds * 1e-9
        if age < 0 or age > constants.THRUSTER_COMMAND_TIMEOUT_S:
            self.publish_neutral()
        else:
            self.publish_commands(self.pending_commands)


def main():
    rclpy.init()
    node = ActuatorCommands()
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

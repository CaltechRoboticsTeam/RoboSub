"""Send fresh M5 mixer targets to the Teensy USB bridge."""

import signal
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from robosub_control.config import constants, current_m5
from robosub_control.thrusters.thrust_allocation import ThrustAllocator
from robosub_control.utils.tuning import fixed_parameter
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

from robosub_control.drivers.teensy_usb import (
    DEVICE,
    MAX_DELTA_US,
    NEUTRAL_US,
    SLOTS,
    TeensyUsb,
    m5_power_slots,
)


class TeensyActuators(Node):
    def __init__(self):
        super().__init__("teensy_actuators")
        device = fixed_parameter(self, "device", DEVICE)
        self.max_power = fixed_parameter(
            self, "max_power", constants.GAMEPAD_MAX_EFFORT
        )
        m5_power_slots([0.0] * 6, self.max_power)
        self.allocator = ThrustAllocator(vehicle="current_m5")
        self.last_rx = None
        self.pending = [NEUTRAL_US] * SLOTS
        self.publisher = self.create_publisher(
            Float64MultiArray, current_m5.SENT_TOPIC, 1
        )
        self.create_subscription(
            Float64MultiArray, current_m5.TARGET_TOPIC, self.on_targets, 1
        )
        self.bridge = TeensyUsb(device)
        try:
            info = self.bridge.connect()
            if info["rs485_serial"] != 1:
                raise ValueError("M5 joystick output requires Serial1 firmware")
            if self.max_power * MAX_DELTA_US > info["max_delta_us"]:
                raise ValueError("Power limit exceeds the installed firmware")
        except BaseException:
            self.bridge.close()
            raise
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(
            1.0 / constants.GAMEPAD_PUBLISH_RATE_HZ,
            self.periodic,
            clock=self.wall_clock,
        )
        mapping = " ".join(
            f"{slot}={name}" for name, slot in current_m5.M5_MOTOR_IDS.items()
        )
        self.get_logger().info(
            f"Teensy Serial1 output; motor power cap {self.max_power:.0%}; "
            f"motor IDs {mapping}."
        )

    def stop(self):
        self.last_rx = None
        self.pending = [NEUTRAL_US] * SLOTS

    def on_targets(self, message):
        now = time.monotonic()
        try:
            dimensions = message.layout.dim
            if (
                len(dimensions) != 1
                or dimensions[0].label != ",".join(current_m5.THRUSTER_NAMES)
                or dimensions[0].size != 6
                or dimensions[0].stride != 6
                or message.layout.data_offset != 0
            ):
                raise ValueError("Invalid M5 target array order")
            commands = self.allocator.commands(message.data)
            values = m5_power_slots(commands, self.max_power)
        except ValueError as error:
            self.stop()
            self.get_logger().error(str(error), throttle_duration_sec=2.0)
            return
        self.last_rx = now
        self.pending = values

    def periodic(self):
        if self.last_rx is None or not (
            0
            <= time.monotonic() - self.last_rx
            <= constants.GAMEPAD_INPUT_TIMEOUT_S
        ):
            self.stop()
        enabled = any(value != NEUTRAL_US for value in self.pending)
        self.bridge.command(self.pending, enabled=enabled)
        sent = Float64MultiArray(
            data=[
                (self.pending[current_m5.M5_MOTOR_IDS[name]] - NEUTRAL_US)
                / MAX_DELTA_US
                for name in current_m5.THRUSTER_NAMES
            ]
        )
        sent.layout.dim = [
            MultiArrayDimension(
                label=",".join(current_m5.THRUSTER_NAMES), size=6, stride=6
            )
        ]
        self.publisher.publish(sent)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    def interrupted(_signum, _frame):
        raise KeyboardInterrupt

    previous = {
        signum: signal.signal(signum, interrupted)
        for signum in (signal.SIGINT, signal.SIGTERM)
    }
    node = None
    try:
        node = TeensyActuators()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node is not None:
                try:
                    node.bridge.close()
                finally:
                    node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
            for signum, handler in previous.items():
                signal.signal(signum, handler)


if __name__ == "__main__":
    main()

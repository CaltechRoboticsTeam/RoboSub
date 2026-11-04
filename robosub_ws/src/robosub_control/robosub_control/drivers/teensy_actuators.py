"""Send fresh M5 mixer targets to the Teensy USB bridge."""

import json
import signal
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import (
    Float64,
    Float64MultiArray,
    Int32,
    MultiArrayDimension,
    String,
)

from robosub_control.config import constants, current_m5
from robosub_control.drivers.teensy_usb import (
    DEVICE,
    MAX_DELTA_US,
    NEUTRAL_US,
    SLOTS,
    TeensyUsb,
    depth_sample,
    m5_power_slots,
)
from robosub_control.thrusters.thrust_allocation import ThrustAllocator
from robosub_control.utils.tuning import fixed_parameter


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
        self.target_error = None
        self.last_command = None
        self.identity_attempts = {}
        self.publisher = self.create_publisher(
            Float64MultiArray, current_m5.SENT_TOPIC, 1
        )
        self.create_subscription(
            Float64MultiArray, current_m5.TARGET_TOPIC, self.on_targets, 1
        )
        self.depth_pub = self.create_publisher(
            Float64, "/sub/sensors/depth_usb", 10
        )
        self.pressure_adc_pub = self.create_publisher(
            Int32, "/sub/sensors/pressure_adc", 10
        )
        self.bridge = TeensyUsb(device, on_depth=self.on_depth)
        self.status_pub = self.create_publisher(
            String, "/sub/teensy/status", 10
        )
        try:
            info = self.bridge.connect()
            self.bridge_info = info
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
        self.create_timer(0.5, self.publish_status, clock=self.wall_clock)
        self.create_timer(0.025, self.query_identity, clock=self.wall_clock)
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

    def on_depth(self, fields):
        try:
            sample = depth_sample(fields)
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=2.0)
            return
        self.pressure_adc_pub.publish(Int32(data=sample["adc"]))
        if sample["depth_m"] is None:
            self.get_logger().warning(
                f"Pressure ADC={sample['adc']}; depth unavailable.",
                throttle_duration_sec=2.0,
            )
            return
        self.depth_pub.publish(Float64(data=sample["depth_m"]))

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
            commands = self.allocator.commands(
                message.data, max_power=self.max_power
            )
            values = m5_power_slots(commands, self.max_power)
        except ValueError as error:
            self.stop()
            self.target_error = str(error)
            self.get_logger().error(str(error), throttle_duration_sec=2.0)
            return
        self.last_rx = now
        self.target_error = None
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
        self.last_command = time.monotonic()
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

    def query_identity(self):
        now = time.monotonic()
        if (
            self.bridge_info.get("protocol_version", 1) < 3
            or self.last_command is None
            or not 0.01 <= now - self.last_command <= 0.04
        ):
            return
        # Query between power packets, after the preceding reply has finished.
        for status in self.bridge.bus.motor_statuses():
            node = status["node_id"]
            if status["age_s"] <= 15 and (
                node not in self.bridge.bus.motor_ids
                or now - self.identity_attempts.get(node, -float("inf")) > 30
            ):
                if now - self.identity_attempts.get(node, -float("inf")) < 2:
                    continue
                self.bridge.query_motor_id(node)
                self.identity_attempts[node] = now
                break

    def publish_status(self):
        now = time.monotonic()
        self.status_pub.publish(
            String(
                data=json.dumps(
                    {
                        "firmware": self.bridge_info,
                        "ack": self.bridge.last_ack,
                        "ack_age_s": (
                            now - self.last_command
                            if self.last_command is not None
                            else None
                        ),
                        "target_age_s": (
                            now - self.last_rx
                            if self.last_rx is not None
                            else None
                        ),
                        "target_error": self.target_error,
                        "bus": self.bridge.bus.snapshot(),
                        "motors": self.bridge.bus.motor_statuses(),
                    },
                    allow_nan=False,
                )
            )
        )


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

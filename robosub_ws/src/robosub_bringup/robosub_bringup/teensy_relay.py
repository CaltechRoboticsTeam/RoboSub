"""Relay fresh PX4 port-0 servo telemetry to the USB M5 bridge."""

import argparse
import signal
import time

from mavros_msgs.msg import DebugValue, Mavlink, State
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float64

from robosub_bringup.field_checks import servo_packet
from robosub_bringup.teensy_usb import (
    DEVICE,
    NEUTRAL_US,
    SLOTS,
    TeensyUsb,
    depth_sample,
    validate_pwm,
)

RELAY_MAX_DELTA_US = 25


def relay_values(state, state_age, channels, output_age):
    """Never refresh a nonzero USB command from stale PX4 telemetry."""
    if state is None or not state.connected or state_age > 1.5:
        raise ValueError("No fresh connected PX4 state")
    if state.mode != "MANUAL":
        raise ValueError("USB field relay requires MANUAL mode")
    if not state.armed:
        return [NEUTRAL_US] * SLOTS, False
    if channels is None or output_age > 0.2:
        raise ValueError("No port-0 servo output newer than 0.2 seconds")
    return validate_pwm(
        channels, enabled=True, max_delta=RELAY_MAX_DELTA_US
    ), True


class TeensyRelay(Node):
    def __init__(self, device, legacy_depth):
        super().__init__("teensy_usb_relay")
        self.state = None
        self.state_time = 0.0
        self.channels = None
        self.output_time = 0.0
        self.startup_deadline = time.monotonic() + 15.0
        self.neutral_since = None
        self.ready = False
        self.pressure_unavailable = False
        self.depth_pub = self.create_publisher(Float64, "/depth", 10)
        self.legacy_pub = None
        if legacy_depth:
            self.legacy_pub = self.create_publisher(
                DebugValue, "/mavros/debug_value/named_value_float", 10
            )
        self.create_subscription(
            State, "/mavros/state", self.on_state, qos_profile_sensor_data
        )
        self.create_subscription(
            Mavlink,
            "/uas1/mavlink_source",
            self.on_mavlink,
            qos_profile_sensor_data,
        )
        self.bridge = TeensyUsb(device, on_depth=self.on_depth)
        try:
            info = self.bridge.connect()
            if info["rs485_serial"] == 0:
                raise ValueError("RS-485 disabled; use teensy_test neutral")
        except BaseException:
            self.bridge.close()
            raise
        self.create_timer(0.05, self.periodic)
        self.get_logger().info(
            "USB relay sends neutral until disarmed MANUAL startup is seen. "
            f"It does not arm PX4. Limit: +/-{RELAY_MAX_DELTA_US} us; "
            "stale PX4 telemetry aborts forwarding."
        )

    def on_state(self, message):
        self.state = message
        self.state_time = time.monotonic()

    def on_mavlink(self, message):
        packet = servo_packet(message)
        if packet is not None and (
            packet["sysid"],
            packet["compid"],
            packet["port"],
        ) == (1, 1, 0):
            self.channels = packet["channels"]
            self.output_time = time.monotonic()

    def on_depth(self, fields):
        value = depth_sample(fields)["depth_m"]
        if value is None:
            if not self.pressure_unavailable:
                self.get_logger().warning(
                    "Pressure ADC is at the low rail; depth unavailable"
                )
            self.pressure_unavailable = True
            return
        self.pressure_unavailable = False
        self.depth_pub.publish(Float64(data=value))
        if self.legacy_pub is not None:
            message = DebugValue(
                name="DEPTH",
                value_float=value,
                type=DebugValue.TYPE_NAMED_VALUE_FLOAT,
                index=-1,
                array_id=-1,
            )
            message.header.stamp = self.get_clock().now().to_msg()
            self.legacy_pub.publish(message)

    def periodic(self):
        now = time.monotonic()
        if not self.ready:
            self.bridge.command([NEUTRAL_US] * SLOTS)
            if now > self.startup_deadline:
                raise RuntimeError("PX4 discovery/neutral startup timed out")
            if (
                self.state is None
                or not self.state.connected
                or now - self.state_time > 1.5
            ):
                self.neutral_since = None
                return
            if self.state.armed:
                raise RuntimeError("Start the USB relay with PX4 disarmed")
            relay_values(
                self.state,
                now - self.state_time,
                self.channels,
                now - self.output_time,
            )
            if self.channels is None or now - self.output_time > 0.2:
                self.neutral_since = None
                return
            if any(value != NEUTRAL_US for value in self.channels):
                raise RuntimeError("PX4 outputs must start exactly neutral")
            if self.neutral_since is None:
                self.neutral_since = now
            self.ready = now - self.neutral_since >= 1.0
            return
        values, enabled = relay_values(
            self.state,
            now - self.state_time,
            self.channels,
            now - self.output_time,
        )
        self.bridge.command(values, enabled)


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default=DEVICE)
    parser.add_argument("--legacy-depth", action="store_true")
    parser.add_argument("--execute", action="store_true")
    options = parser.parse_args(args)
    if not options.execute:
        print("Dry run; USB was not opened. --execute starts the live relay.")
        return 0

    def interrupted(_signum, _frame):
        raise KeyboardInterrupt

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    previous = {
        signum: signal.signal(signum, interrupted)
        for signum in (signal.SIGINT, signal.SIGTERM)
    }
    node = None
    success = False
    try:
        node = TeensyRelay(options.device, options.legacy_depth)
        rclpy.spin(node)
        success = True
    except KeyboardInterrupt:
        success = True
    except (OSError, ValueError, RuntimeError) as exc:
        print("USB relay stopped:", exc)
    finally:
        if node is not None:
            try:
                node.bridge.close()
            except (OSError, ValueError, RuntimeError) as exc:
                success = False
                print("Neutral ACK missing; remove propulsion power:", exc)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for signum, handler in previous.items():
            signal.signal(signum, handler)
    return 0 if success else 1

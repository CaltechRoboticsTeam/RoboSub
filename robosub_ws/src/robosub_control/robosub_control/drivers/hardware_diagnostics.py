"""Publish device and command status on /diagnostics."""

import json
import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from mavros_msgs.msg import State
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import Bool, Float64, Int32, String

from robosub_control.config import current_m5


class HardwareDiagnostics(Node):
    def __init__(self):
        super().__init__("hardware_diagnostics")
        self.samples = {}
        self.publisher = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )
        for key, topic, kind in (
            ("depth", "/depth", Float64),
            ("pressure", "/sub/sensors/pressure_adc", Int32),
            ("dvl_driver", "/sub/sensors/dvl/status", String),
            ("pixhawk", "/mavros/state", State),
            ("imu", "/imu/data_raw", Imu),
            ("teensy", "/sub/teensy/status", String),
            ("joy", "/operator/joy", Joy),
            ("enabled", "/operator/enabled", Bool),
            ("command", "/sub/cmd_vel", Twist),
        ):
            self.create_subscription(
                kind,
                topic,
                lambda message, name=key: self.on_sample(name, message),
                qos_profile_sensor_data,
            )
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.5, self.periodic, clock=self.wall_clock)

    def on_sample(self, key, message):
        if key in ("teensy", "dvl_driver"):
            try:
                value = json.loads(message.data)
                if not isinstance(value, dict):
                    return
            except ValueError:
                return
        else:
            value = message
        self.samples[key] = (time.monotonic(), value)

    def sample(self, key, maximum_age=1.5):
        received, value = self.samples.get(key, (None, None))
        if received is None or time.monotonic() - received > maximum_age:
            return None
        return value

    def status(self, name, level, message, **values):
        return DiagnosticStatus(
            name=f"sub/Diagnostics/{name}",
            level=bytes([level]),
            message=message,
            hardware_id="current_m5",
            values=[
                KeyValue(key=key, value=str(value))
                for key, value in values.items()
            ],
        )

    def sensor_statuses(self):
        rows = []
        for key, name in (("depth", "Depth"), ("pressure", "Pressure")):
            message = self.sample(key)
            if message is None:
                rows.append(self.status(name, 3, "No fresh data"))
                continue
            value = message.data
            if not math.isfinite(value) or (key == "pressure" and value <= 1):
                rows.append(
                    self.status(
                        name, 1, "Invalid / low-rail reading", value=value
                    )
                )
            else:
                rows.append(
                    self.status(
                        name,
                        1,
                        "Data received; calibration unverified",
                        **{"depth_m" if key == "depth" else "adc": value},
                    )
                )
        driver = self.sample("dvl_driver")
        if driver is None:
            rows.append(self.status("DVL", 3, "No fresh DVL driver status"))
        else:
            packet_age = driver.get("packet_age_s")
            fresh = packet_age is not None and 0 <= packet_age <= 2.5
            if not driver.get("serial_open"):
                level, message = 2, driver.get("error") or "Serial port closed"
            elif not fresh:
                level = 1
                message = "Serial open; no fresh checksum-valid PD0"
            elif driver.get("error"):
                level, message = 1, driver["error"]
            else:
                level, message = 0, "Checksum-valid PD0 bottom track received"
            rows.append(self.status("DVL", level, message, **driver))
        pixhawk = self.sample("pixhawk")
        rows.append(
            self.status(
                "Pixhawk",
                3 if pixhawk is None else (0 if pixhawk.connected else 2),
                "No fresh MAVROS state"
                if pixhawk is None
                else (
                    "MAVROS connected"
                    if pixhawk.connected
                    else "MAVROS disconnected"
                ),
                imu_received=self.sample("imu") is not None,
            )
        )
        return rows

    def motor_statuses(self, bridge):
        rows = []
        motors = bridge.get("motors", [])
        for name in current_m5.THRUSTER_NAMES:
            motor_id = current_m5.M5_MOTOR_IDS[name]
            matches = [m for m in motors if m.get("motor_id") == motor_id]
            fresh = [m for m in matches if m.get("age_s", math.inf) <= 15]
            values = {"motor_id": motor_id}
            if not fresh:
                text = "No identified reply; power/wiring/address unconfirmed"
                rows.append(self.status(f"Thruster {name}", 3, text, **values))
                continue
            if len(fresh) > 1:
                rows.append(
                    self.status(
                        f"Thruster {name}",
                        2,
                        "Duplicate stored motor assignment",
                        nodes=",".join(str(m["node_id"]) for m in fresh),
                        **values,
                    )
                )
                continue
            motor = fresh[0]
            fault = motor["fault"]
            values.update(motor)
            rows.append(
                self.status(
                    f"Thruster {name}",
                    2 if fault else 0,
                    f"M5 fault 0x{fault:02x}"
                    if fault
                    else "CRC-valid M5 reply; motion not verified",
                    **values,
                )
            )
        for motor in motors:
            if motor.get("motor_id") not in current_m5.M5_MOTOR_IDS.values():
                rows.append(
                    self.status(
                        f"Unassigned node {motor['node_id']}",
                        1,
                        "M5 replied; motor assignment unknown or unexpected",
                        **motor,
                    )
                )
        return rows

    def periodic(self):
        rows = self.sensor_statuses()
        bridge = self.sample("teensy")
        if bridge is None:
            rows.append(self.status("Teensy", 3, "No fresh driver status"))
            rows.extend(
                self.status(
                    f"Thruster {name}",
                    3,
                    "No fresh Teensy driver status",
                    motor_id=current_m5.M5_MOTOR_IDS[name],
                )
                for name in current_m5.THRUSTER_NAMES
            )
        else:
            ack = bridge.get("ack") or {}
            ack_age = bridge.get("ack_age_s")
            fresh = (
                ack.get("sequence", 0) > 0
                and ack_age is not None
                and 0 <= ack_age <= 0.5
            )
            rows.append(
                self.status(
                    "Teensy",
                    0 if fresh else 3,
                    "USB commands acknowledged; M5 reception is separate"
                    if fresh
                    else "No fresh USB acknowledgment",
                    **bridge.get("firmware", {}),
                    **ack,
                )
            )
            bus = bridge.get("bus", {})
            rows.append(
                self.status(
                    "RS485",
                    0
                    if any(m["age_s"] <= 15 for m in bridge.get("motors", []))
                    else 1,
                    "See individual motor reply ages; counters are cumulative",
                    **bus,
                )
            )
            rows.extend(self.motor_statuses(bridge))
        enabled = self.sample("enabled")
        error = bridge.get("target_error") if bridge else None
        target_age = bridge.get("target_age_s") if bridge else None
        active = enabled is not None and enabled.data
        joy = self.sample("joy", 0.25)
        if error:
            command_status = error
        elif not active:
            command_status = "Operator disabled or status unavailable"
        elif joy is None:
            command_status = "Operator enabled; controller input stale"
        elif self.sample("command") is None:
            command_status = "Enabled, but no fresh body commands"
        elif target_age is None or target_age > 0.25:
            command_status = "Body commands received; M5 targets missing/stale"
        else:
            command_status = "Operator enabled; commands reach the USB driver"
        rows.append(
            self.status(
                "Command path",
                2
                if error
                else (
                    0
                    if active
                    and joy is not None
                    and target_age is not None
                    and target_age <= 0.25
                    and self.sample("command")
                    else 1
                ),
                command_status,
                joy_received=joy is not None,
                joy_axes=list(joy.axes) if joy is not None else [],
                command_received=self.sample("command") is not None,
                target_age_s=target_age,
            )
        )
        output = DiagnosticArray()
        output.header.stamp = self.get_clock().now().to_msg()
        output.status = rows
        self.publisher.publish(output)


def main():
    rclpy.init()
    node = HardwareDiagnostics()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

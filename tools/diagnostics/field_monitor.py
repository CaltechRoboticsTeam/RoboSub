"""Passive observations of existing MAVROS; no hardware ports or publishers."""

from collections import Counter
from datetime import datetime, timezone
import time

from diagnostic_msgs.msg import DiagnosticArray
from mavros_msgs.msg import DebugValue, Mavlink, RCOut, State, StatusText
from mavros_msgs.srv import VehicleInfoGet
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import BatteryState, Imu

from field_checks import (
    json_finite,
    receiver_power,
    servo_packet,
)

PARAMETERS = (
    "SYS_AUTOSTART",
    "UUV_MGM_THRTL",
    "UUV_MGM_YAW",
    "UUV_THRUST_SAT",
    "UUV_TORQUE_SAT",
    "UUV_STAB_MODE",
    "COM_RC_IN_MODE",
    "COM_RC_LOSS_T",
    "MAV_0_CONFIG",
    "MAV_0_MODE",
    "MAV_0_RATE",
    "MAV_1_CONFIG",
    "MAV_1_MODE",
    "MAV_2_CONFIG",
    "MAV_2_MODE",
    "MAV_2_RATE",
    "SER_TEL1_BAUD",
    "SER_TEL2_BAUD",
    *(
        f"PWM_MAIN_{kind}{index}"
        for kind in ("FUNC", "MIN", "MAX", "DIS")
        for index in range(1, 9)
    ),
)


class FieldMonitor(Node):
    def __init__(self):
        super().__init__("field_test")
        self.started = time.monotonic()
        self.counts = Counter()
        self.last_rx = {}
        self.messages = {}
        self.servo_ports = {}
        self.texts = []
        self.pwm_min = [2001] * 8
        self.pwm_max = [-1] * 8
        self.vehicle_info = []
        self.parameters = {}
        for key, topic, kind in (
            ("state", "/mavros/state", State),
            ("rc_out", "/mavros/rc/out", RCOut),
            ("imu", "/mavros/imu/data", Imu),
            ("battery", "/mavros/battery", BatteryState),
            ("depth", "/mavros/debug_value/named_value_float", DebugValue),
            ("dvl_odom", "/mavros/odometry/out", Odometry),
            ("diagnostics", "/diagnostics", DiagnosticArray),
            ("statustext", "/mavros/statustext/recv", StatusText),
            ("raw_mavlink", "/uas1/mavlink_source", Mavlink),
        ):
            self.create_subscription(
                kind,
                topic,
                lambda message, name=key: self.on_message(name, message),
                qos_profile_sensor_data,
            )

    def on_message(self, key, message):
        if key == "depth" and message.name != "DEPTH":
            return
        self.counts[key] += 1
        self.last_rx[key] = time.monotonic()
        self.messages[key] = message
        if key == "statustext":
            self.texts.append(
                {"severity": message.severity, "text": message.text}
            )
            self.texts = self.texts[-30:]
        if key == "raw_mavlink":
            packet = servo_packet(message)
            if packet is not None:
                port = packet["port"]
                previous = self.servo_ports.get(port, {})
                self.servo_ports[port] = {
                    **packet,
                    "count": previous.get("count", 0) + 1,
                    "received": time.monotonic(),
                }
        if key == "rc_out":
            for index, value in enumerate(message.channels[:8]):
                self.pwm_min[index] = min(self.pwm_min[index], value)
                self.pwm_max[index] = max(self.pwm_max[index], value)

    def age(self, key):
        received = self.last_rx.get(key)
        return None if received is None else time.monotonic() - received

    def collect(self, seconds):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(
                self,
                timeout_sec=max(0.0, min(0.1, deadline - time.monotonic())),
            )

    def cached_request(self, kind, endpoint, request, timeout=3.0):
        client = self.create_client(kind, endpoint)
        try:
            if not client.wait_for_service(timeout_sec=0.5):
                return None
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            if not future.done():
                future.cancel()
                return None
            return future.result()
        finally:
            self.destroy_client(client)

    def read_cached_configuration(self):
        # Both services read MAVROS's cache. No parameter pull, parameter set,
        # message-rate request, mode change or arming command is sent to FCU.
        info = self.cached_request(
            VehicleInfoGet, "/mavros/vehicle_info_get", VehicleInfoGet.Request()
        )
        if info is not None and info.success:
            self.vehicle_info = [
                dict(message_to_ordereddict(vehicle))
                for vehicle in info.vehicles
            ]
        result = self.cached_request(
            GetParameters,
            "/mavros/param/get_parameters",
            GetParameters.Request(names=list(PARAMETERS)),
        )
        if result is not None:
            fields = {
                1: "bool_value",
                2: "integer_value",
                3: "double_value",
                4: "string_value",
            }
            self.parameters = {
                name: getattr(value, fields[value.type])
                if value.type in fields
                else None
                for name, value in zip(PARAMETERS, result.values, strict=True)
            }

    def snapshot(self):
        elapsed = time.monotonic() - self.started
        messages = {
            key: dict(message_to_ordereddict(message))
            for key, message in self.messages.items()
            if key not in ("raw_mavlink", "diagnostics", "statustext")
        }
        ports = {
            str(port): {
                **packet,
                "age_s": time.monotonic() - packet["received"],
            }
            for port, packet in self.servo_ports.items()
        }
        for packet in ports.values():
            packet.pop("received")
            try:
                packet["legacy_normalized_power"] = receiver_power(
                    packet["channels"]
                )
            except ValueError as error:
                packet["receiver_input_error"] = str(error)
        diagnostics = self.messages.get("diagnostics")
        return json_finite(
            {
                "time_utc": datetime.now(timezone.utc).isoformat(),
                "passive": True,
                "observation_s": elapsed,
                "vehicle_info": self.vehicle_info,
                "cached_parameters": self.parameters,
                "topics": {
                    key: {
                        "count": count,
                        "rate_hz": count / elapsed,
                        "age_s": self.age(key),
                    }
                    for key, count in self.counts.items()
                },
                "last_messages": messages,
                "servo_ports_on_jetson_link": ports,
                "servo_output_min": self.pwm_min
                if self.counts["rc_out"]
                else None,
                "servo_output_max": self.pwm_max
                if self.counts["rc_out"]
                else None,
                "status_text": self.texts,
                "diagnostics": []
                if diagnostics is None
                else [
                    {
                        "name": status.name,
                        "level": status.level,
                        "message": status.message,
                    }
                    for status in diagnostics.status
                ],
                "manual_control_publishers": self.count_publishers(
                    "/mavros/manual_control/send"
                ),
                "rc_override_publishers": self.count_publishers(
                    "/mavros/rc/override"
                ),
                "limitations": [
                    "Jetson-link servo data does not verify the Teensy UART.",
                    "No Teensy or M5 acknowledgment is available.",
                    "Topic/command success does not prove motor movement.",
                ],
            }
        )

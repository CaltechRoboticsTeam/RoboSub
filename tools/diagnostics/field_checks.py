"""Bounds and decoding for the current Pixhawk/legacy receiver field tests."""

import math
import struct

NEUTRAL_US = 1500
RECEIVER_OUTPUTS = 8
MAX_EFFORT = 0.10
MAX_HOLD_S = 1.0
MAX_DELTA_US = 50


def pulse_plan(axis, effort, hold_s, ramp_s):
    if axis not in ("throttle", "yaw"):
        raise ValueError("Only the legacy throttle or yaw input is supported")
    for name, value, low, high in (
        ("effort", effort, -MAX_EFFORT, MAX_EFFORT),
        ("hold_s", hold_s, 0.1, MAX_HOLD_S),
        ("ramp_s", ramp_s, 0.1, 0.5),
    ):
        if (
            type(value) not in (int, float)
            or not math.isfinite(value)
            or not low <= value <= high
        ):
            raise ValueError(
                f"{name} must be finite and within [{low}, {high}]"
            )
    if effort == 0:
        raise ValueError(
            "Use status for a passive check; pulse effort is nonzero"
        )
    return {
        "axis": axis,
        "effort": float(effort),
        "hold_s": float(hold_s),
        "ramp_s": float(ramp_s),
        "max_output_delta_us": MAX_DELTA_US,
        "input_units": "normalized manual stick, not physical speed",
        "automatic_mode_change": False,
    }


def pulse_effort(elapsed, plan):
    ramp, hold = plan["ramp_s"], plan["hold_s"]
    total = 2 * ramp + hold
    if elapsed < 0 or elapsed >= total:
        return 0.0
    fraction = min(1.0, elapsed / ramp, (total - elapsed) / ramp)
    return plan["effort"] * fraction


def manual_values(axis, effort):
    """Use MAVROS send's native MAVLink units, matching the old PX4 program."""
    if not math.isfinite(effort) or abs(effort) > MAX_EFFORT:
        raise ValueError("Manual effort exceeds the field test limit")
    if axis == "throttle":
        return (0.0, 0.0, 500.0 + 500.0 * effort, 0.0)
    if axis == "yaw":
        return (0.0, 0.0, 500.0, 1000.0 * effort)
    raise ValueError("Unknown manual axis")


def receiver_power(channels):
    """Normalize valid legacy 1000..2000 values; zero is invalid."""
    if len(channels) < RECEIVER_OUTPUTS:
        raise ValueError("Expected the receiver's eight servo output values")
    values = channels[:RECEIVER_OUTPUTS]
    if any(
        type(value) is not int or not 1000 <= value <= 2000 for value in values
    ):
        raise ValueError("Invalid receiver input; a zero value is not neutral")
    return [(value - NEUTRAL_US) / 500.0 for value in values]


def check_outputs(channels, max_delta=MAX_DELTA_US):
    receiver_power(channels)
    if any(
        abs(value - NEUTRAL_US) > max_delta
        for value in channels[:RECEIVER_OUTPUTS]
    ):
        raise ValueError(
            f"Reported servo output exceeded {max_delta} us from neutral"
        )


def servo_packet(message):
    """Decode MAVLink SERVO_OUTPUT_RAW base fields without opening a port."""
    if message.msgid != 36:
        return None
    payload = b"".join(struct.pack("<Q", value) for value in message.payload64)
    # MAVLink 2 may trim trailing zeros. Base fields are 4-byte time, eight
    # uint16 outputs, then the uint8 port; missing tail bytes represent zero.
    payload = payload[: message.len].ljust(21, b"\x00")
    channels = list(struct.unpack_from("<8H", payload, 4))
    return {
        "port": payload[20],
        "channels": channels,
        "sysid": message.sysid,
        "compid": message.compid,
    }


def json_finite(value):
    """Represent unknown sensor NaNs as JSON null rather than invented zeros."""
    if isinstance(value, (bytes, bytearray, memoryview)):
        return bytes(value).hex()
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: json_finite(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_finite(item) for item in value]
    return value


def preflight(report):
    """Refuse actuation unless passive observations match this legacy path."""
    state = report.get("last_messages", {}).get("state", {})
    if not state.get("connected") or state.get("armed"):
        raise ValueError("Start connected and disarmed; test owns its arming")
    if state.get("mode") != "MANUAL":
        raise ValueError("Select MANUAL locally; the test never changes modes")
    for key, maximum in (("state", 1.5), ("rc_out", 0.75)):
        age = report.get("topics", {}).get(key, {}).get("age_s")
        if age is None or not 0 <= age <= maximum:
            raise ValueError(f"No fresh {key} telemetry")
    rate = report["topics"]["rc_out"].get("rate_hz", 0.0)
    if rate < 5.0:
        raise ValueError(
            "Servo telemetry is below 5 Hz; request message 36 at 10 Hz first"
        )
    vehicles = report.get("vehicle_info", [])
    if len(vehicles) != 1 or vehicles[0].get("autopilot") != 12:
        raise ValueError("This test is specific to the observed PX4 firmware")
    if vehicles[0].get("type") != 12:
        raise ValueError("Vehicle must report the submarine MAVLink type")
    params = report.get("cached_parameters", {})
    if params.get("SYS_AUTOSTART") != 60002:
        raise ValueError("Airframe changed; review the test mapping first")
    gain = params.get("UUV_MGM_THRTL")
    if (
        type(gain) not in (int, float)
        or not math.isfinite(gain)
        or not 0 < gain <= 1
    ):
        raise ValueError("No usable cached UUV_MGM_THRTL setting")
    if report.get("manual_control_publishers", 0) != 0:
        raise ValueError("Another MANUAL_CONTROL publisher is active")
    if report.get("rc_override_publishers", 0) != 0:
        raise ValueError("An RC override publisher is active")
    channels = report["last_messages"]["rc_out"]["channels"]
    check_outputs(channels, max_delta=5)
    packet = report.get("servo_ports_on_jetson_link", {}).get("0")
    if packet is None or packet["age_s"] > 0.75:
        raise ValueError("No fresh port-0 SERVO_OUTPUT_RAW on the Jetson link")
    check_outputs(packet["channels"], max_delta=5)

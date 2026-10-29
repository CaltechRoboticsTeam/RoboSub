"""Bounded USB transport for the Teensy M5 bridge; no ROS dependency."""

import binascii
import fcntl
import math
import os
import secrets
import select
import struct
import termios
import time

from robosub_control.config import current_m5

from robosub_control.drivers.videoray import BusDiagnostics

DEVICE = "/dev/serial/by-id/usb-Teensyduino_USB_Serial_19620450-if00"
SLOTS = 8
NEUTRAL_US = 1500
MAX_DELTA_US = 500
WATCHDOG_MS = 250
ACK_TIMEOUT_S = 0.15
MAX_LINE = 160


def m5_power_slots(commands, max_power=1.0):
    """Map polarity-adjusted named powers into the eight-slot USB packet.

    Scale the whole vector to preserve allocation when enforcing the motor
    power cap. Unassigned protocol slots always remain neutral.
    """
    if not math.isfinite(max_power) or not 0 < max_power <= 1:
        raise ValueError("M5 power limit must be greater than 0 and at most 1")
    powers = tuple(commands)
    if len(powers) != len(current_m5.THRUSTER_NAMES) or any(
        not math.isfinite(value) or abs(value) > 1 for value in powers
    ):
        raise ValueError("Expected six finite M5 powers between -1 and 1")
    if set(current_m5.M5_MOTOR_IDS) != set(current_m5.THRUSTER_NAMES):
        raise ValueError("Every M5 thruster needs a motor ID")
    slots = tuple(
        current_m5.M5_MOTOR_IDS[name] for name in current_m5.THRUSTER_NAMES
    )
    if any(
        type(slot) is not int or not 0 <= slot < SLOTS for slot in slots
    ) or (len(set(slots)) != len(slots)):
        raise ValueError("M5 motor IDs must uniquely select slots 0-7")
    scale = max(1.0, max(abs(value) for value in powers) / max_power)
    cap = math.floor(MAX_DELTA_US * max_power)
    values = [NEUTRAL_US] * SLOTS
    for slot, power in zip(slots, powers, strict=True):
        delta = round(MAX_DELTA_US * power / scale)
        values[slot] += max(-cap, min(cap, delta))
    return values


def encode_frame(*fields):
    """Protect every USB command and response with the same CRC32."""
    body = ",".join(str(value) for value in fields).encode("ascii")
    if len(body) + 10 >= MAX_LINE or any(c < 32 or c > 126 for c in body):
        raise ValueError("USB frame exceeds its bounds")
    return body + f"*{binascii.crc32(body):08X}\n".encode("ascii")


def decode_frame(line):
    if len(line) >= MAX_LINE:
        raise ValueError("USB response is too long")
    body, separator, checksum = line.rstrip(b"\n").rpartition(b"*")
    if not separator or len(checksum) != 8:
        raise ValueError("Not USB M5 firmware: missing framed response")
    try:
        expected = int(checksum, 16)
        fields = body.decode("ascii").split(",")
    except (ValueError, UnicodeError) as exc:
        raise ValueError("Invalid USB response encoding") from exc
    if binascii.crc32(body) != expected:
        raise ValueError("USB response CRC mismatch")
    lengths = {"I1": 7, "A1": 6, "F1": 3, "D1": 5, "R1": 4}
    if len(fields) != lengths.get(fields[0]):
        raise ValueError("USB response fields do not match the protocol")
    return fields


def validate_pwm(values, enabled=False, max_delta=MAX_DELTA_US):
    values = tuple(values)
    if len(values) != SLOTS:
        raise ValueError("Exactly eight legacy wire slots are required")
    for value in values:
        if type(value) is not int or abs(value - NEUTRAL_US) > max_delta:
            raise ValueError(
                f"USB outputs must be integers within 1500 +/-{max_delta}"
            )
        if not enabled and value != NEUTRAL_US:
            raise ValueError("Disabled commands must contain only neutral")
    return values


def depth_sample(fields):
    """Keep low-rail ADC readings visible without publishing bogus depth."""
    if len(fields) != 5 or fields[0] != "D1":
        raise ValueError("Invalid USB depth response")
    adc = int(fields[1])
    value = float(fields[2])
    if adc < 0 or not math.isfinite(value):
        raise ValueError("Invalid pressure sensor value")
    return {
        "adc": adc,
        "depth_m": value if adc > 1 else None,
        "serial1_rx_bytes": int(fields[3]),
        "serial2_rx_bytes": int(fields[4]),
    }


class TeensyUsb:
    """Exclusive port owner; accepted commands must be acknowledged promptly.

    An acknowledgment establishes USB acceptance. With the bus enabled, its
    transmit counter records RS-485 writes. Neither proves motor movement.
    """

    def __init__(self, device=DEVICE, on_depth=None):
        self.fd = None
        self.session = secrets.randbelow(0xFFFFFFFE) + 1
        self.sequence = 0
        self.buffer = bytearray()
        self.on_depth = on_depth
        self.last_ack = None
        self.info = None
        self.bus = BusDiagnostics()
        descriptor = os.open(
            device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK | os.O_CLOEXEC
        )
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.ioctl(descriptor, termios.TIOCEXCL)
            attributes = termios.tcgetattr(descriptor)
            attributes[0] = 0
            attributes[1] = 0
            attributes[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            attributes[3] = 0
            attributes[4] = termios.B115200
            attributes[5] = termios.B115200
            attributes[6][termios.VMIN] = 0
            attributes[6][termios.VTIME] = 0
            termios.tcsetattr(descriptor, termios.TCSANOW, attributes)
            termios.tcflush(descriptor, termios.TCIOFLUSH)
            # CDC-ACM opens normally assert DTR. Set it explicitly on hardware.
            if os.path.basename(os.path.realpath(device)).startswith("ttyACM"):
                fcntl.ioctl(
                    descriptor,
                    termios.TIOCMBIS,
                    struct.pack("I", termios.TIOCM_DTR),
                )
            self.fd = descriptor
        except BaseException:
            os.close(descriptor)
            raise

    def _write(self, data):
        deadline = time.monotonic() + ACK_TIMEOUT_S
        while data:
            remaining = deadline - time.monotonic()
            if (
                remaining <= 0
                or not select.select([], [self.fd], [], max(0, remaining))[1]
            ):
                raise TimeoutError("Teensy USB write timed out")
            try:
                written = os.write(self.fd, data)
            except BlockingIOError:
                continue
            if written == 0:
                raise ConnectionError("Teensy USB write returned no progress")
            data = data[written:]

    def _wait(self, kind, sequence=None, timeout=ACK_TIMEOUT_S):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if b"\n" in self.buffer:
                line, _, remaining = self.buffer.partition(b"\n")
                self.buffer = bytearray(remaining)
                fields = decode_frame(line)
                if fields[0] == "D1":
                    if self.on_depth is not None:
                        self.on_depth(fields)
                    continue
                if fields[0] == "R1":
                    self.bus.capture(fields)
                    continue
                if fields[0] == "F1":
                    raise RuntimeError("Teensy stopped: " + ",".join(fields))
                if fields[0] == kind and int(fields[1]) == self.session:
                    if sequence is None or int(fields[2]) == sequence:
                        return fields
                continue
            if len(self.buffer) >= MAX_LINE:
                raise ValueError("Teensy USB input exceeded line bounds")
            remaining = max(0, deadline - time.monotonic())
            if not select.select([self.fd], [], [], remaining)[0]:
                continue
            try:
                data = os.read(self.fd, 256)
            except BlockingIOError:
                continue
            if not data:
                raise ConnectionError("Teensy USB disconnected")
            self.buffer.extend(data)
        raise TimeoutError(f"Teensy did not acknowledge {kind}")

    def connect(self):
        self._write(encode_frame("H1", self.session))
        fields = self._wait("I1", timeout=2.0)
        if int(fields[2]) not in (1, 2) or tuple(map(int, fields[3:5])) != (
            SLOTS,
            WATCHDOG_MS,
        ):
            raise ValueError("Teensy firmware version/limits do not match")
        limit = int(fields[5])
        if not 1 <= limit <= MAX_DELTA_US:
            raise ValueError("Teensy firmware output limit is unsupported")
        self.info = {
            "protocol_version": int(fields[2]),
            "session": self.session,
            "slots": SLOTS,
            "watchdog_ms": WATCHDOG_MS,
            "max_delta_us": limit,
            "rs485_serial": int(fields[6]),
        }
        if self.info["rs485_serial"] not in (0, 1, 2):
            raise ValueError("Unknown firmware UART selection")
        self.command([NEUTRAL_US] * SLOTS)
        return self.info

    def command(self, values, enabled=False):
        if self.info is None:
            raise RuntimeError("USB handshake is required before commands")
        if enabled and self.info["rs485_serial"] == 0:
            raise ValueError("RS-485 is disabled in this diagnostic firmware")
        values = validate_pwm(values, enabled, self.info["max_delta_us"])
        self.sequence += 1
        if self.sequence > 0xFFFFFFFF:
            raise OverflowError("USB sequence exhausted; reconnect at neutral")
        self._write(
            encode_frame(
                "C1", self.session, self.sequence, int(enabled), *values
            )
        )
        fields = self._wait("A1", self.sequence)
        if len(fields) != 6 or int(fields[3]) != int(enabled):
            raise ValueError("Teensy acknowledgment did not match command")
        self.last_ack = {
            "sequence": self.sequence,
            "enabled": bool(int(fields[3])),
            "rs485_packets_sent": int(fields[4]),
            "rs485_response_bytes": int(fields[5]),
        }
        return self.last_ack

    def close(self):
        if self.fd is not None:
            try:
                if self.info is not None:
                    self.command([NEUTRAL_US] * SLOTS)
            finally:
                os.close(self.fd)
                self.fd = None

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        self.close()

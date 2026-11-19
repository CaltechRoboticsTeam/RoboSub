"""Decode PD0 per the Teledyne manual; bottom velocities start at byte 24."""

import math
import struct

MAX_ENSEMBLE = 16384


class Pd0Decoder:
    def __init__(self):
        self.buffer = bytearray()
        self.bytes_received = 0
        self.valid_packets = 0
        self.bad_checksums = 0
        self.malformed_packets = 0

    def feed(self, data):
        self.bytes_received += len(data)
        results = []
        for offset in range(0, len(data), 1024):
            self.buffer.extend(data[offset : offset + 1024])
            while len(self.buffer) >= 6:
                start = self.buffer.find(b"\x7f\x7f")
                if start < 0:
                    self.buffer[:] = self.buffer[-1:]
                    break
                del self.buffer[:start]
                if len(self.buffer) < 6:
                    break
                length = struct.unpack_from("<H", self.buffer, 2)[0]
                count = self.buffer[5]
                if (
                    not 1 <= count <= 64
                    or not 6 + 2 * count <= length <= MAX_ENSEMBLE
                ):
                    del self.buffer[0]
                    self.malformed_packets += 1
                    continue
                if len(self.buffer) < length + 2:
                    break
                checksum = struct.unpack_from("<H", self.buffer, length)[0]
                if sum(self.buffer[:length]) & 0xFFFF != checksum:
                    self.bad_checksums += 1
                    del self.buffer[0]
                    continue
                packet = bytes(self.buffer[:length])
                del self.buffer[: length + 2]
                self.valid_packets += 1
                try:
                    results.append(decode_bottom_track(packet))
                except ValueError as error:
                    self.malformed_packets += 1
                    results.append({"error": str(error), "velocity": None})
        return results


def decode_bottom_track(packet):
    if len(packet) < 6 or not 1 <= packet[5] <= 64:
        raise ValueError("PD0 header is too short or has no blocks")
    count = packet[5]
    if len(packet) < 6 + 2 * count:
        raise ValueError("PD0 offset table is truncated")
    offsets = struct.unpack_from(f"<{count}H", packet, 6)
    if len(set(offsets)) != count or any(
        not 6 + 2 * count <= item <= len(packet) - 2 for item in offsets
    ):
        raise ValueError("PD0 block offsets are invalid")
    starts = sorted(offsets)
    blocks = {}
    for start, end in zip(starts, [*starts[1:], len(packet)], strict=True):
        block_id = struct.unpack_from("<H", packet, start)[0]
        if block_id in blocks:
            raise ValueError("Duplicate PD0 block ID")
        blocks[block_id] = packet[start:end]
    fixed = blocks.get(0x0000, b"")
    bottom = blocks.get(0x0600, b"")
    variable = blocks.get(0x0080, b"")
    if len(fixed) < 26 or len(bottom) < 44:
        raise ValueError("PD0 fixed leader or bottom-track block missing/short")
    coordinate = (fixed[25] >> 3) & 3
    raw = struct.unpack_from("<4h", bottom, 24)
    ranges = struct.unpack_from("<4H", bottom, 16)
    result = {
        "firmware": f"{fixed[2]}.{fixed[3]}",
        "coordinate_system": ("beam", "instrument", "ship", "earth")[
            coordinate
        ],
        "raw_velocity_mm_s": list(raw),
        "ranges_cm": list(ranges),
        "percent_good": list(bottom[40:44]),
        "ensemble": (
            struct.unpack_from("<H", variable, 2)[0] | variable[11] << 16
        )
        if len(variable) >= 12
        else None,
        "velocity": None,
        "error": None,
    }
    if (
        struct.unpack_from("<H", bottom, 2)[0] == 0
        or any(value == -32768 for value in raw)
        or not all(ranges)
        or any(value < max(1, bottom[8]) for value in bottom[40:44])
        or any(value < bottom[6] for value in bottom[32:36])
        or any(value < bottom[7] for value in bottom[36:40])
    ):
        result["error"] = "No valid four-beam bottom track"
        return result
    if coordinate >= 2:
        result["error"] = "Ship/earth PD0 coordinates need a verified transform"
        return result
    velocity = [value * 0.001 for value in raw]
    if coordinate == 0:
        if fixed[8] != 4:
            result["error"] = "Beam conversion requires four Janus beams"
            return result
        angle = fixed[58] if len(fixed) >= 59 else (15, 20, 30, 0)[fixed[5] & 3]
        if not 0 < angle < 90:
            raise ValueError("Invalid PD0 beam angle")
        a = 1 / (2 * math.sin(math.radians(angle)))
        b = 1 / (4 * math.cos(math.radians(angle)))
        sign = 1 if fixed[4] & 8 else -1
        one, two, three, four = velocity
        velocity = [
            sign * a * (one - two),
            sign * a * (four - three),
            b * sum(velocity),
        ]
    else:
        velocity = velocity[:3]
    result["velocity"] = velocity
    return result


def rotate_velocity(velocity, rpy_deg, sign):
    """Instrument coordinates to ROS body coordinates using Rz * Ry * Rx."""
    roll, pitch, yaw = (math.radians(v) for v in rpy_deg)
    x, y, z = (sign * v for v in velocity)
    y, z = (
        math.cos(roll) * y - math.sin(roll) * z,
        math.sin(roll) * y + math.cos(roll) * z,
    )
    x, z = (
        math.cos(pitch) * x + math.sin(pitch) * z,
        -math.sin(pitch) * x + math.cos(pitch) * z,
    )
    return (
        math.cos(yaw) * x - math.sin(yaw) * y,
        math.sin(yaw) * x + math.cos(yaw) * y,
        z,
    )

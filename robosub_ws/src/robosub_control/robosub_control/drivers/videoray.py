"""Bounded decoding of captured VideoRay packets; never opens a bus."""

import binascii
from collections import deque
import math
import struct
import time

REQUEST_SYNC = b"\xf5\x5f"
RESPONSE_SYNC = b"\xf0\x0f"


class BusDiagnostics:
    """Separate raw activity, echoed requests and CRC-valid motor feedback."""

    def __init__(self):
        self.buffer = bytearray()
        self.first_bytes = bytearray()
        self.last_bytes = bytearray()
        self.byte_count = 0
        self.dropped_bytes = 0
        self.valid_responses = 0
        self.echoed_requests = 0
        self.bad_header_crc = 0
        self.bad_payload_crc = 0
        self.discarded_bytes = 0
        self.invalid_status_payloads = 0
        self.statuses = deque(maxlen=8)
        self.status_time = None

    def capture(self, fields):
        if len(fields) != 4 or fields[0] != "R1":
            raise ValueError("Invalid UART capture response")
        if len(fields[3]) > 96 or len(fields[3]) % 2:
            raise ValueError("UART capture exceeded its bounds")
        dropped = int(fields[2])
        if dropped < 0:
            raise ValueError("Invalid capture-drop count")
        # A dropped chunk can join unrelated packet fragments. Do not decode
        # across that gap, even if a coincidental sync byte is present.
        if dropped != self.dropped_bytes:
            self.buffer.clear()
        self.dropped_bytes = dropped
        self.feed(bytes.fromhex(fields[3]))

    def feed(self, data):
        self.byte_count += len(data)
        self.first_bytes.extend(data[: max(0, 128 - len(self.first_bytes))])
        self.last_bytes = (self.last_bytes + data)[-128:]
        for offset in range(0, len(data), 48):
            self.buffer.extend(data[offset : offset + 48])
            self._drain()

    def _discard(self, count):
        del self.buffer[:count]
        self.discarded_bytes += count

    def _drain(self):
        while len(self.buffer) >= 2:
            candidates = [
                index
                for sync in (REQUEST_SYNC, RESPONSE_SYNC)
                if (index := self.buffer.find(sync)) >= 0
            ]
            if not candidates:
                # Preserve a possible first sync byte for the next chunk.
                keep = int(self.buffer[-1] in (0xF0, 0xF5))
                self._discard(len(self.buffer) - keep)
                return
            self._discard(min(candidates))
            if len(self.buffer) < 10:
                return
            header_crc = struct.unpack_from("<I", self.buffer, 6)[0]
            if binascii.crc32(self.buffer[:6]) != header_crc:
                self.bad_header_crc += 1
                self._discard(1)
                continue
            size = 14 + self.buffer[5]
            if len(self.buffer) < size:
                return
            payload = self.buffer[10 : size - 4]
            payload_crc = struct.unpack_from("<I", self.buffer, size - 4)[0]
            if binascii.crc32(payload) != payload_crc:
                self.bad_payload_crc += 1
                self._discard(1)
                continue
            packet = bytes(self.buffer[:size])
            del self.buffer[:size]
            if packet[:2] == REQUEST_SYNC:
                self.echoed_requests += 1
                continue
            self.valid_responses += 1
            if packet[4] != 0xF0 or len(payload) != 18:
                continue
            device, rpm, voltage, current, temperature, fault = struct.unpack(
                "<BffffB", payload
            )
            if not all(
                math.isfinite(value)
                for value in (rpm, voltage, current, temperature)
            ):
                self.invalid_status_payloads += 1
                continue
            self.statuses.append(
                {
                    "node_id": packet[2],
                    "flag": packet[3],
                    "device_type": device,
                    "rpm": rpm,
                    "bus_voltage_v": voltage,
                    "bus_current_a": current,
                    "temperature_c": temperature,
                    "fault": fault,
                }
            )
            self.status_time = time.monotonic()

    def require_feedback(self, maximum_age=2.0):
        if self.status_time is None:
            raise RuntimeError(
                "No CRC-valid M5 status reply; nonzero output is blocked"
            )
        if time.monotonic() - self.status_time > maximum_age:
            raise RuntimeError("M5 status reply is stale; stopping the pulse")
        if self.statuses[-1]["node_id"] != 1:
            raise RuntimeError("Reply does not match configured reply node 1")
        if self.statuses[-1]["fault"] != 0:
            raise RuntimeError(
                f"M5 reports fault 0x{self.statuses[-1]['fault']:02x}"
            )

    def snapshot(self):
        return {
            "captured_bytes": self.byte_count,
            "dropped_capture_bytes": self.dropped_bytes,
            "crc_valid_responses": self.valid_responses,
            "echoed_request_packets": self.echoed_requests,
            "bad_header_crc": self.bad_header_crc,
            "bad_payload_crc": self.bad_payload_crc,
            "discarded_bytes": self.discarded_bytes,
            "pending_bytes": len(self.buffer),
            "first_rx_hex": self.first_bytes.hex(),
            "last_rx_hex": self.last_bytes.hex(),
            "invalid_status_payloads": self.invalid_status_payloads,
            "last_status": self.statuses[-1] if self.statuses else None,
            "last_status_age_s": (
                time.monotonic() - self.status_time
                if self.status_time is not None
                else None
            ),
        }

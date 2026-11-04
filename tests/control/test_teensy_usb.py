"""Firmware parser, VideoRay CRC and USB transport diagnostics."""

import binascii
import io
import os
import pty
import select
import shutil
import struct
import subprocess
import sys
import tempfile
import threading
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "robosub_ws/src/robosub_control"))
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "tools/diagnostics"))

from robosub_control.drivers.teensy_usb import (  # noqa: E402
    TeensyUsb,
    encode_frame,
    m5_power_slots,
)
from robosub_control.drivers.videoray import BusDiagnostics  # noqa: E402


def command(session, sequence, enabled=False, delta=0, slot=0):
    values = [1500] * 8
    values[slot] += delta
    return encode_frame("C1", session, sequence, int(enabled), *values)


class FirmwareTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("clang++") or shutil.which("g++")
        if compiler is None:
            raise unittest.SkipTest("A C++ compiler is required")
        cls.temporary = tempfile.TemporaryDirectory()
        cls.executable = Path(cls.temporary.name) / "protocol_probe"
        subprocess.run(
            [
                compiler,
                "-std=c++11",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-Wconversion",
                "-pedantic",
                "-I",
                str(ROOT / "firmware/teensy_usb_m5"),
                str(ROOT / "tests/control/teensy_protocol_probe.cpp"),
                "-o",
                str(cls.executable),
            ],
            check=True,
            capture_output=True,
            text=True,
        )

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def run_frames(self, *frames):
        data = "".join(
            f"{timestamp} {frame.decode().strip()}\n"
            for timestamp, frame in frames
        )
        result = subprocess.run(
            [str(self.executable)],
            input=data,
            check=True,
            capture_output=True,
            text=True,
        )
        return [line.split() for line in result.stdout.splitlines()]

    def test_named_motor_powers_reach_exact_wire_slots(self):
        powers = (0.12, -0.24, 0.36, -0.48, 0.60, -0.72)
        values = m5_power_slots(powers)
        rows = self.run_frames(
            (0, encode_frame("H1", 42)),
            (1, command(42, 1)),
            (2, encode_frame("C1", 42, 2, 1, *values)),
        )
        packet = bytes.fromhex(rows[-1][-1])
        actual = struct.unpack("<8f", packet[12:44])
        expected_slots = (*powers[:5], 0, powers[5], 0)
        for observed, expected in zip(actual, expected_slots, strict=True):
            self.assertAlmostEqual(observed, expected, places=6)
        self.assertEqual(
            m5_power_slots(powers, max_power=0.36),
            [1530, 1440, 1590, 1380, 1650, 1500, 1320, 1500],
        )
        limited = m5_power_slots([1.0] * 6, max_power=0.503)
        self.assertTrue(all(abs(value - 1500) <= 251 for value in limited))
        for invalid in ([0] * 5, [float("nan")] * 6, [1.01] * 6):
            with self.assertRaises(ValueError):
                m5_power_slots(invalid)
        for invalid_id in (-1, 8, True, 0):
            with (
                patch.dict(
                    "robosub_control.config.current_m5.M5_MOTOR_IDS",
                    LD=invalid_id,
                ),
                self.assertRaises(ValueError),
            ):
                m5_power_slots(powers)
        with patch.dict("robosub_control.config.current_m5.M5_MOTOR_IDS", LD=7):
            self.assertEqual(m5_power_slots([0, 0, 0, 0, 0, 0.5])[7], 1750)

    def test_neutral_handshake_and_watchdog_require_explicit_restart(self):
        rows = self.run_frames(
            (0, encode_frame("H1", 42)),
            (1, command(42, 1, True, 25)),  # No neutral command yet.
            (2, command(42, 2)),
            (3, command(42, 3, True, 25)),
            (252, b"TICK"),
            (253, b"TICK"),
            (254, command(42, 4, True, 25)),
            (255, command(42, 5)),
            (256, command(42, 6, True, -25)),
        )
        self.assertEqual(
            [row[0] for row in rows],
            ["1", "0", "2", "2", "0", "1", "0", "2", "2"],
        )
        self.assertEqual(rows[5][3:12], ["0"] + ["1500"] * 8)
        self.assertEqual(rows[-1][3:5], ["1", "1475"])

    def test_invalid_crc_overflow_sequence_and_session_stop_every_slot(self):
        bad = [
            command(42, 3, True, 501),
            command(42, 3, False, 1),
            command(43, 3, True, 25),
            command(42, 2, True, 25),
            command(42, 3, True, 25).replace(b"1525", b"1524"),
            encode_frame("C1", 42, 4294967296, 1, *([1500] * 8)),
            encode_frame("H1", 0),
            encode_frame("C1", 42, 3, 1, *([1500] * 9)),
        ]
        for frame in bad:
            with self.subTest(frame=frame):
                rows = self.run_frames(
                    (0, encode_frame("H1", 42)),
                    (1, command(42, 1)),
                    (2, command(42, 2, True, 25)),
                    (3, frame),
                )
                self.assertEqual(rows[-1][0], "0")
                self.assertEqual(rows[-1][3:12], ["0"] + ["1500"] * 8)

    def test_identity_read_preserves_power_and_does_not_renew_watchdog(self):
        rows = self.run_frames(
            (0, encode_frame("H1", 42)),
            (1, command(42, 1)),
            (2, command(42, 2, True, 25)),
            (200, encode_frame("Q1", 42, 127)),
            (252, b"TICK"),
        )
        self.assertEqual(rows[3][0], "3")
        self.assertEqual(rows[3][1:12], rows[2][1:12])
        self.assertEqual(rows[4][3:12], ["0"] + ["1500"] * 8)
        for node in (-1, 128):
            rows = self.run_frames(
                (0, encode_frame("H1", 42)),
                (1, command(42, 1)),
                (2, command(42, 2, True, 25)),
                (3, encode_frame("Q1", 42, node)),
            )
            self.assertEqual(rows[-1][0], "0")
            self.assertEqual(rows[-1][3:12], ["0"] + ["1500"] * 8)


class UsbTransportTests(unittest.TestCase):
    def transport(self, mode):
        master, slave = pty.openpty()
        device = os.ttyname(slave)
        received = []
        errors = []
        done = threading.Event()

        def emulate():
            buffer = b""
            while not done.is_set():
                if not select.select([master], [], [], 0.02)[0]:
                    continue
                try:
                    data = os.read(master, 256)
                    if not data:
                        return
                    buffer += data
                    while b"\n" in buffer:
                        line, _, buffer = buffer.partition(b"\n")
                        body = line.split(b"*")[0].decode().split(",")
                        received.append(body)
                        if body[0] == "H1":
                            response = encode_frame(
                                "I1",
                                body[1],
                                3 if mode == "v3" else 1,
                                8,
                                250,
                                25,
                                1,
                            )
                        elif body[0] == "Q1":
                            response = encode_frame("K1", body[1], body[2])
                        elif mode == "timeout" and body[2] == "2":
                            continue
                        else:
                            response = encode_frame(
                                "A1",
                                body[1],
                                body[2],
                                body[3],
                                len(received),
                                0,
                            )
                        # Fragmented reads can include interleaved telemetry.
                        os.write(master, encode_frame("D1", 403, 0, 0, 0))
                        os.write(master, response[:7])
                        os.write(master, response[7:])
                except BaseException as exc:
                    errors.append(exc)
                    return

        worker = threading.Thread(target=emulate, daemon=True)
        worker.start()
        bridge = None
        try:
            bridge = TeensyUsb(device)
            self.assertEqual(bridge.connect()["watchdog_ms"], 250)
            with self.assertRaises(OSError):
                TeensyUsb(device)
            if mode in ("ok", "v3"):
                bridge.command([1525] + [1500] * 7, enabled=True)
                if mode == "v3":
                    sequence = bridge.sequence
                    bridge.query_motor_id(29)
                    self.assertEqual(bridge.sequence, sequence)
            else:
                with self.assertRaises((TimeoutError, RuntimeError)):
                    bridge.command([1525] + [1500] * 7, enabled=True)
            bridge.close()
            self.assertEqual(received[-1][3:], ["0"] + ["1500"] * 8)
        finally:
            if bridge is not None and bridge.fd is not None:
                os.close(bridge.fd)
            done.set()
            worker.join(timeout=2)
            os.close(master)
            os.close(slave)
        self.assertFalse(errors)

    def test_exclusive_usb_acknowledgment_and_neutral_cleanup(self):
        self.transport("ok")

    def test_missing_ack_does_not_count_as_success(self):
        self.transport("timeout")

    def test_v3_identity_query_has_its_own_acknowledgment(self):
        self.transport("v3")

    def test_missing_motor_reply_blocks_every_nonneutral_attempt(self):
        from diagnostics.teensy_test import main

        commands = []
        info = {"protocol_version": 2, "rs485_serial": 1, "max_delta_us": 500}
        bridge = SimpleNamespace(
            info=info,
            bus=BusDiagnostics(),
            connect=lambda: info,
            command=lambda values, enabled=False: commands.append(
                (list(values), enabled)
            ),
            close=lambda: None,
        )
        with (
            patch("diagnostics.teensy_test.TeensyUsb", return_value=bridge),
            redirect_stdout(io.StringIO()),
        ):
            result = main(
                [
                    "pulse",
                    "--all-slots",
                    "--seconds",
                    "1",
                    "--execute",
                    "--require-feedback",
                ]
            )
        self.assertEqual(result, 1)
        self.assertTrue(commands)
        self.assertTrue(
            all(
                values == [1500] * 8 and not enabled
                for values, enabled in commands
            )
        )


def status_packet(node=1, fault=0, rpm=0.0):
    header = struct.pack("<HBBBB", 0x0FF0, node, 2, 0xF0, 18)
    payload = struct.pack("<BffffB", 1, rpm, 24.0, 0.1, 20.0, fault)
    return (
        header
        + struct.pack("<I", binascii.crc32(header))
        + payload
        + struct.pack("<I", binascii.crc32(payload))
    )


class VideoRayTests(unittest.TestCase):
    def test_motor_assignment_is_read_independently_of_node_address(self):
        bus = BusDiagnostics()
        header = struct.pack("<HBBBB", 0x0FF0, 29, 0x80, 0x4C, 3)
        payload = struct.pack("<BH", 0x81, 6)
        identity = (
            header
            + struct.pack("<I", binascii.crc32(header))
            + payload
            + struct.pack("<I", binascii.crc32(payload))
        )
        bus.feed(identity[:-1] + bytes([identity[-1] ^ 1]))
        self.assertEqual(bus.motor_ids, {})
        bus.feed(identity)
        packet = bytearray(status_packet(node=29))
        packet[10] = 0x81
        packet[-4:] = struct.pack("<I", binascii.crc32(packet[10:-4]))
        with patch("time.monotonic", return_value=10.0):
            bus.feed(packet)
        with patch("time.monotonic", return_value=12.0):
            status = bus.motor_statuses()[0]
        self.assertEqual(status["node_id"], 29)
        self.assertEqual(status["motor_id"], 6)
        self.assertEqual(status["bus_voltage_v"], 24.0)
        self.assertEqual(status["age_s"], 2.0)

    def test_split_frames_decode_voltage_and_fault(self):
        packet = status_packet()
        self.assertEqual(len(packet), 32)
        for split in range(33):
            with self.subTest(split=split):
                bus = BusDiagnostics()
                bus.feed(packet[:split])
                bus.feed(packet[split:])
                self.assertEqual(bus.valid_responses, 1)
                self.assertEqual(bus.statuses[-1]["bus_voltage_v"], 24.0)
                bus.require_feedback()

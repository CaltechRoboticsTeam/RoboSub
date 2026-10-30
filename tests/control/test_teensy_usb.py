"""Firmware parser, VideoRay CRC and USB transport diagnostics."""

import argparse
import binascii
from contextlib import redirect_stdout
import io
import json
import os
from pathlib import Path
import pty
import select
import shutil
import struct
import subprocess
import sys
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "robosub_ws/src/robosub_control"))
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "tools/diagnostics"))

from diagnostics.teensy_test import parse_hold, pulse_values  # noqa: E402
from robosub_control.drivers.teensy_usb import (  # noqa: E402
    TeensyUsb,
    decode_frame,
    depth_sample,
    encode_frame,
    m5_power_slots,
    validate_pwm,
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

    def test_crc_packet_matches_documented_wire_format(self):
        rows = self.run_frames(
            (0, encode_frame("H1", 42)),
            (1, command(42, 1)),
            (2, command(42, 2, True, delta=25, slot=3)),
        )
        packet = bytes.fromhex(rows[-1][-1])
        header = bytes.fromhex("f55f8102f022")
        payload = b"\xaa\x01" + struct.pack("<8f", 0, 0, 0, 0.05, 0, 0, 0, 0)
        self.assertEqual(
            packet,
            header
            + struct.pack("<I", binascii.crc32(header))
            + payload
            + struct.pack("<I", binascii.crc32(payload)),
        )

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

    def test_disconnect_and_millis_wrap_stop(self):
        rows = self.run_frames(
            (4294967200, encode_frame("H1", 42)),
            (4294967201, command(42, 1)),
            (4294967202, command(42, 2, True, 25)),
            (155, b"TICK"),
            (156, b"TICK"),
            (157, b"DISCONNECT"),
            (158, command(42, 3)),
        )
        self.assertEqual(rows[3][0], "0")
        self.assertEqual(rows[4][0], "1")
        self.assertEqual(rows[5][1:4], ["0", "0", "0"])
        self.assertEqual(rows[6][0], "0")

    def test_full_forward_reverse_and_neutral_are_bounded(self):
        rows = self.run_frames(
            (0, encode_frame("H1", 42)),
            (1, command(42, 1)),
            (2, command(42, 2, True, 500)),
            (3, command(42, 3, True, -500)),
            (4, command(42, 4)),
        )
        for row, expected in ((rows[2], 1.0), (rows[3], -1.0)):
            packet = bytes.fromhex(row[-1])
            self.assertEqual(struct.unpack_from("<f", packet, 12)[0], expected)
        self.assertEqual(rows[-1][3:12], ["0"] + ["1500"] * 8)


class UsbTransportTests(unittest.TestCase):
    def test_unlimited_hold_parsing_and_target_values(self):
        self.assertIsNone(parse_hold("forever"))
        self.assertEqual(parse_hold("86400"), 86400)
        self.assertEqual(parse_hold("0"), 0)
        for value in ("nan", "inf", "-1", "invalid"):
            with (
                self.subTest(value=value),
                self.assertRaises(argparse.ArgumentTypeError),
            ):
                parse_hold(value)
        self.assertEqual(pulse_values(None, 500, -1, None, 2), [1500] * 8)
        self.assertEqual(pulse_values(None, 500, 1, None, 2), [1750] * 8)
        self.assertEqual(pulse_values(None, 500, 86400, None, 2), [2000] * 8)
        self.assertEqual(pulse_values(None, 500, 30, 60, 2), [2000] * 8)

    def test_forever_hold_ctrl_c_commands_neutral_and_keeps_sample_count(self):
        from diagnostics.teensy_test import main

        clock = [0.0]
        commands = []
        full_commands = [0]
        bridge = SimpleNamespace(
            info={
                "protocol_version": 1,
                "rs485_serial": 1,
                "max_delta_us": 500,
            },
            bus=BusDiagnostics(),
        )

        def command(values, enabled=False):
            commands.append((list(values), enabled))
            bridge.on_depth(["D1", "403", "0", "0", "0"])
            bridge.last_ack = {"enabled": enabled, "sequence": len(commands)}
            return bridge.last_ack

        def open_bridge(_device, on_depth):
            bridge.on_depth = on_depth
            return bridge

        def sleep(seconds):
            if commands and commands[-1] == ([2000] * 8, True):
                full_commands[0] += 1
                if full_commands[0] == 3:
                    raise KeyboardInterrupt
            clock[0] += seconds

        bridge.command = command
        bridge.connect = lambda: bridge.info
        bridge.close = lambda: command([1500] * 8)
        output = io.StringIO()
        with (
            patch("diagnostics.teensy_test.TeensyUsb", side_effect=open_bridge),
            patch(
                "diagnostics.teensy_test.time.monotonic",
                side_effect=lambda: clock[0],
            ),
            patch("diagnostics.teensy_test.time.sleep", side_effect=sleep),
            redirect_stdout(output),
        ):
            result = main(
                [
                    "pulse",
                    "--all-slots",
                    "--delta-us",
                    "500",
                    "--ramp",
                    "0.25",
                    "--seconds",
                    "1",
                    "--hold",
                    "forever",
                    "--execute",
                ]
            )
        self.assertEqual(result, 0)
        self.assertEqual(commands[-1], ([1500] * 8, False))
        text = output.getvalue()
        report = json.loads(text[text.rfind("\n{\n") + 1 :])
        self.assertTrue(report["stopped_by_operator"])
        self.assertTrue(report["neutral_acknowledged"])
        self.assertTrue(report["hold_until_interrupted"])
        self.assertIsNone(report["hold_s"])
        self.assertIsNone(report["rs485_diagnostics"])
        self.assertEqual(report["depth_samples"], len(commands))

    def test_frame_limits_crc_and_single_slot_ramp(self):
        self.assertEqual(
            decode_frame(encode_frame("D1", 403, 0, 10, 20)),
            ["D1", "403", "0", "10", "20"],
        )
        with self.assertRaises(ValueError):
            decode_frame(b"D1,403,0,10,20*00000000\n")
        for values in (
            [1500] * 7,
            [2001] * 8,
            [999] * 8,
            [1500.0] * 8,
            [True] * 8,
        ):
            with self.assertRaises(ValueError):
                validate_pwm(values, True)
        self.assertEqual(
            pulse_values(3, 25, 0.5, 0.5),
            [1500, 1500, 1500, 1525, 1500, 1500, 1500, 1500],
        )
        self.assertEqual(pulse_values(3, 25, 1, 0.5), [1500] * 8)
        self.assertEqual(pulse_values(None, 25, 0.5, 0.5), [1525] * 8)
        self.assertEqual(pulse_values(None, 25, 1, 0.5), [1500] * 8)
        self.assertEqual(pulse_values(None, 500, 10, 1, 20), [1750] * 8)
        self.assertEqual(pulse_values(None, 500, 20, 1, 20), [2000] * 8)
        self.assertEqual(pulse_values(None, 500, 31, 1, 20), [1750] * 8)
        self.assertEqual(pulse_values(None, 500, 41, 1, 20), [1500] * 8)

    def test_zero_adc_does_not_become_a_physical_depth(self):
        reading = depth_sample(["D1", "0", "-10.2362", "2", "137360"])
        self.assertEqual(reading["adc"], 0)
        self.assertIsNone(reading["depth_m"])
        self.assertIsNone(
            depth_sample(["D1", "1", "-10.2108", "0", "0"])["depth_m"]
        )
        self.assertEqual(reading["serial2_rx_bytes"], 137360)
        self.assertEqual(
            depth_sample(["D1", "403", "0", "0", "0"])["depth_m"], 0
        )
        with self.assertRaises(ValueError):
            depth_sample(["D1", "403", "nan", "0", "0"])

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
                                2 if mode == "capture" else 1,
                                8,
                                250,
                                25,
                                1,
                            )
                        elif mode == "timeout" and body[2] == "2":
                            continue
                        elif mode == "fault" and body[2] == "2":
                            response = encode_frame("F1", body[1], "TIMEOUT")
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
                        if mode == "capture":
                            os.write(
                                master,
                                encode_frame(
                                    "R1",
                                    len(received),
                                    0,
                                    status_packet().hex(),
                                ),
                            )
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
            if mode in ("ok", "capture"):
                bridge.command([1525] + [1500] * 7, enabled=True)
                if mode == "capture":
                    self.assertGreater(bridge.bus.valid_responses, 0)
                    bridge.bus.require_feedback()
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

    def test_firmware_fault_is_propagated_then_neutral_is_sent(self):
        self.transport("fault")

    def test_uart_capture_is_processed_between_usb_acknowledgments(self):
        self.transport("capture")

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
    payload = struct.pack("<BffffB", 1, rpm, 48.0, 0.1, 20.0, fault)
    return (
        header
        + struct.pack("<I", binascii.crc32(header))
        + payload
        + struct.pack("<I", binascii.crc32(payload))
    )


class VideoRayTests(unittest.TestCase):
    def test_split_frames_decode_voltage_and_fault(self):
        packet = status_packet()
        self.assertEqual(len(packet), 32)
        for split in range(33):
            with self.subTest(split=split):
                bus = BusDiagnostics()
                bus.feed(packet[:split])
                bus.feed(packet[split:])
                self.assertEqual(bus.valid_responses, 1)
                self.assertEqual(bus.statuses[-1]["bus_voltage_v"], 48.0)
                bus.require_feedback()

    def test_echo_and_single_noise_bytes_are_not_motor_feedback(self):
        bus = BusDiagnostics()
        header = bytes.fromhex("f55f8102f022")
        payload = b"\xaa\x01" + struct.pack("<8f", *([0.0] * 8))
        request = (
            header
            + struct.pack("<I", binascii.crc32(header))
            + payload
            + struct.pack("<I", binascii.crc32(payload))
        )
        bus.feed(b"\x00" * 2000 + request)
        self.assertEqual(bus.echoed_requests, 1)
        self.assertEqual(bus.valid_responses, 0)
        self.assertLessEqual(len(bus.buffer), 1)
        with self.assertRaisesRegex(RuntimeError, "No CRC-valid"):
            bus.require_feedback()

    def test_corrupt_packet_recovers_at_next_valid_frame(self):
        for position in (6, 12):
            bad = bytearray(status_packet())
            bad[position] ^= 1
            bus = BusDiagnostics()
            bus.feed(bad + status_packet())
            self.assertEqual(bus.valid_responses, 1)
            self.assertGreater(bus.bad_header_crc + bus.bad_payload_crc, 0)

    def test_capture_drop_resets_partial_packet_and_bounds_are_enforced(self):
        bus = BusDiagnostics()
        packet = status_packet()
        bus.capture(["R1", "1", "0", packet[:15].hex()])
        bus.capture(["R1", "2", "48", packet[15:].hex()])
        self.assertEqual(bus.valid_responses, 0)
        bus.capture(["R1", "3", "48", packet.hex()])
        self.assertEqual(bus.valid_responses, 1)
        self.assertEqual(bus.dropped_bytes, 48)
        with self.assertRaises(ValueError):
            bus.capture(["R1", "4", "48", "00" * 49])

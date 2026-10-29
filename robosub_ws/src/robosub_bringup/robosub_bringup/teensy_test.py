"""Neutral USB check and explicit, bounded field pulses."""

import argparse
import json
import math
from pathlib import Path
import signal
import time

from robosub_bringup.teensy_usb import (
    DEVICE,
    MAX_DELTA_US,
    NEUTRAL_US,
    SLOTS,
    TeensyUsb,
    depth_sample,
)


def pulse_values(slot, delta, elapsed, hold, ramp=0.25):
    values = [NEUTRAL_US] * SLOTS
    if elapsed < 0:
        return values
    if hold is None:
        fraction = min(1.0, elapsed / ramp)
    else:
        if elapsed >= 2 * ramp + hold:
            return values
        fraction = min(1.0, elapsed / ramp, (2 * ramp + hold - elapsed) / ramp)
    selected = range(SLOTS) if slot is None else (slot,)
    for index in selected:
        values[index] += round(delta * fraction)
    return values


def interrupted(_signum, _frame):
    raise KeyboardInterrupt


def parse_hold(value):
    if value.lower() == "forever":
        return None
    try:
        seconds = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            "--hold must be nonnegative seconds or 'forever'"
        ) from exc
    if not math.isfinite(seconds) or seconds < 0:
        raise argparse.ArgumentTypeError(
            "--hold must be finite nonnegative seconds or 'forever'"
        )
    return seconds


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("neutral", "pulse"))
    parser.add_argument("--device", default=DEVICE)
    parser.add_argument("--seconds", type=float, default=3.0)
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument("--slot", type=int, choices=range(SLOTS))
    selection.add_argument("--all-slots", action="store_true")
    parser.add_argument("--delta-us", type=int, default=25)
    parser.add_argument(
        "--hold",
        type=parse_hold,
        default=0.5,
        help="seconds at target, or 'forever' until Ctrl-C",
    )
    parser.add_argument("--ramp", type=float, default=3.0)
    parser.add_argument("--require-feedback", action="store_true")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--output", type=Path)
    options = parser.parse_args(args)
    if not math.isfinite(options.seconds) or not 1 <= options.seconds <= 60:
        parser.error("--seconds must be between 1 and 60")
    if not math.isfinite(options.ramp) or not 0.25 <= options.ramp <= 30:
        parser.error("--ramp must be between 0.25 and 30 seconds per ramp")
    if not 1 <= abs(options.delta_us) <= MAX_DELTA_US:
        parser.error("--delta-us must be a nonzero integer within +/-500")
    if (
        options.action == "pulse"
        and options.slot is None
        and not options.all_slots
    ):
        parser.error("pulse requires --slot 0..7 or --all-slots")
    if options.action == "neutral" and (
        options.slot is not None or options.all_slots
    ):
        parser.error("Output selection applies only to pulse")
    report = {
        "action": options.action,
        "device": options.device,
        "slot": options.slot,
        "all_slots": options.all_slots,
        "delta_us": options.delta_us,
        "requested_power": options.delta_us / 500.0,
        "ramp_s": options.ramp,
        "hold_s": options.hold,
        "hold_until_interrupted": options.hold is None,
        "neutral_s": options.seconds,
        "require_feedback": options.require_feedback,
        "executed": options.execute,
        "movement_confirmed": False,
        "neutral_acknowledged": False,
        "nonneutral_commands_attempted": 0,
        "nonneutral_commands_acknowledged": 0,
        "peak_delta_us_attempted": 0,
    }
    if not options.execute:
        print(json.dumps(report, indent=2))
        print("Dry run; USB was not opened. Add --execute to send commands.")
        return 0
    print("USB test owns propulsion commands; PX4 arming is bypassed.")
    print("A USB acknowledgment does not confirm an M5 reply or movement.")
    previous = {
        signum: signal.signal(signum, interrupted)
        for signum in (signal.SIGINT, signal.SIGTERM)
    }
    depth_count = 0
    last_depth = None

    def depth(fields):
        nonlocal depth_count, last_depth
        last_depth = depth_sample(fields)
        depth_count += 1

    bridge = None
    success = False
    try:
        bridge = TeensyUsb(options.device, on_depth=depth)
        report["firmware"] = bridge.connect()
        print("Firmware:", report["firmware"])
        if options.action == "pulse" and bridge.info["rs485_serial"] == 0:
            raise ValueError("Motor bus disabled; verify UART wiring first")
        if (
            options.action == "pulse"
            and abs(options.delta_us) > (bridge.info["max_delta_us"])
        ):
            raise ValueError(
                "Requested target exceeds the installed firmware limit; "
                "install the matching higher-limit image first"
            )
        if options.require_feedback and bridge.info["protocol_version"] < 2:
            raise ValueError("Pulse requires firmware with M5 reply capture")
        start = time.monotonic()
        while time.monotonic() - start < options.seconds:
            bridge.command([NEUTRAL_US] * SLOTS)
            time.sleep(0.05)
        if options.action == "pulse":
            if options.require_feedback:
                bridge.bus.require_feedback()
            selected = (
                "all legacy wire slots"
                if options.all_slots
                else (f"wire slot {options.slot}; all others neutral")
            )
            hold_text = (
                "hold until Ctrl-C; Ctrl-C sends neutral"
                if options.hold is None
                else f"{options.hold}s hold, {options.ramp}s down"
            )
            print(
                f"Pulsing {selected}: {options.delta_us / 5:+.0f}% command, "
                f"{options.ramp}s up, {hold_text}."
            )
            start = time.monotonic()
            duration = (
                None
                if options.hold is None
                else options.hold + 2 * options.ramp
            )
            previous_phase = None
            while True:
                elapsed = time.monotonic() - start
                if duration is not None and elapsed >= duration:
                    break
                phase = (
                    "ramp up"
                    if elapsed < options.ramp
                    else "hold"
                    if options.hold is None
                    or elapsed < options.ramp + options.hold
                    else "ramp down"
                )
                if phase != previous_phase:
                    print("Phase:", phase, flush=True)
                    previous_phase = phase
                if options.require_feedback:
                    bridge.bus.require_feedback()
                values = pulse_values(
                    options.slot,
                    options.delta_us,
                    elapsed,
                    options.hold,
                    options.ramp,
                )
                peak = max(abs(value - NEUTRAL_US) for value in values)
                if peak:
                    report["nonneutral_commands_attempted"] += 1
                report["peak_delta_us_attempted"] = max(
                    report["peak_delta_us_attempted"], peak
                )
                ack = bridge.command(values, enabled=True)
                if peak:
                    report["nonneutral_commands_acknowledged"] += 1
                    report["last_pulse_ack"] = ack
                time.sleep(0.05)
        report["last_ack"] = bridge.command([NEUTRAL_US] * SLOTS)
        report["neutral_acknowledged"] = True
        success = True
    except KeyboardInterrupt:
        report["stopped_by_operator"] = True
        print("Operator stop: sending neutral.")
        success = True
    except (OSError, ValueError, RuntimeError) as exc:
        report["error"] = str(exc) or type(exc).__name__
        print("USB test stopped:", report["error"])
    finally:
        if bridge is not None:
            try:
                bridge.close()
                report["neutral_acknowledged"] = bridge.info is not None
            except (OSError, ValueError, RuntimeError) as exc:
                report["cleanup_error"] = str(exc)
                report["neutral_acknowledged"] = False
                success = False
                print("Neutral ACK missing; remove propulsion power.")
            report["capture_supported"] = (
                bridge.info is not None and bridge.info["protocol_version"] >= 2
            )
            report["rs485_diagnostics"] = (
                bridge.bus.snapshot() if report["capture_supported"] else None
            )
        for signum, handler in previous.items():
            signal.signal(signum, handler)
        report["depth_samples"] = depth_count
        report["last_depth"] = last_depth
        if options.output:
            options.output.parent.mkdir(parents=True, exist_ok=True)
            options.output.write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report, indent=2))
    return 0 if success else 1

"""Current-submarine diagnostics and an explicitly executed bounded pulse.

Default operation only subscribes and reads cached ROS parameters. Physical
output publishers and an arming client are created only after --execute and
successful preflight. No serial ports, firmware or calibration are modified.
"""

import argparse
import json
from pathlib import Path
import signal
import sys
import time

from mavros_msgs.msg import ManualControl
from mavros_msgs.srv import CommandBool
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args

from field_checks import (
    MAX_DELTA_US,
    check_outputs,
    json_finite,
    manual_values,
    preflight,
    pulse_effort,
    pulse_plan,
)
from field_monitor import FieldMonitor


class PulseTest:
    def __init__(self, monitor, plan):
        self.node = monitor
        self.plan = plan
        self.publisher = None
        self.arm_client = None
        self.arm_attempted = False
        self.stopping = False
        self.events = []
        self.commands_sent = 0

    def request_stop(self, _signum, _frame):
        self.stopping = True

    def publish(self, effort):
        x, y, z, yaw = manual_values(self.plan["axis"], effort)
        message = ManualControl()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.x, message.y, message.z, message.r = x, y, z, yaw
        self.publisher.publish(message)
        self.commands_sent += 1

    def guard(self, armed):
        if self.stopping:
            raise RuntimeError("Operator interrupted the pulse")
        state = self.node.messages.get("state")
        age = self.node.age("state")
        if (
            state is None
            or age is None
            or age > 1.5
            or not state.connected
            or state.mode != "MANUAL"
        ):
            raise RuntimeError("Lost MANUAL state/connection telemetry")
        if armed is not None and state.armed != armed:
            raise RuntimeError("Unexpected arming state")
        age = self.node.age("rc_out")
        if age is None or age > 0.75:
            raise RuntimeError("Servo telemetry stopped")
        check_outputs(list(self.node.messages["rc_out"].channels), MAX_DELTA_US)
        if self.node.count_publishers("/mavros/manual_control/send") != 1:
            raise RuntimeError("Another manual-control publisher appeared")
        if self.node.count_publishers("/mavros/rc/override"):
            raise RuntimeError("An RC override publisher appeared")

    def tick(self, effort):
        self.publish(effort)
        self.node.collect(0.05)

    def arm(self):
        self.arm_attempted = True
        self.events.append("arming_requested")
        future = self.arm_client.call_async(CommandBool.Request(value=True))
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            self.guard(armed=None)
            self.tick(0.0)
            state = self.node.messages.get("state")
            if future.done():
                response = future.result()
                if response is None or not response.success:
                    raise RuntimeError("Pixhawk rejected arming")
                if state is not None and state.armed:
                    self.events.append("arming_confirmed")
                    return
        raise RuntimeError("Arming acknowledgment/state timed out")

    def stop_and_disarm(self):
        if self.publisher is None:
            return True
        self.publish(0.0)
        if not self.arm_attempted:
            return True
        self.events.append("neutral_and_disarm_requested")
        future = self.arm_client.call_async(CommandBool.Request(value=False))
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            self.tick(0.0)
            state = self.node.messages.get("state")
            age = self.node.age("state")
            if (
                future.done()
                and future.result() is not None
                and future.result().success
                and state is not None
                and age is not None
                and age < 1.5
                and not state.armed
            ):
                self.events.append("disarming_confirmed")
                return True
        self.events.append("DISARM_NOT_CONFIRMED")
        return False

    def run(self):
        # Re-check immediately before creating any output path.
        preflight(self.node.snapshot())
        self.publisher = self.node.create_publisher(
            ManualControl, "/mavros/manual_control/send", 1
        )
        self.arm_client = self.node.create_client(
            CommandBool, "/mavros/cmd/arming"
        )
        if not self.arm_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("Arming service is unavailable")
        old_handlers = {
            kind: signal.getsignal(kind)
            for kind in (signal.SIGINT, signal.SIGTERM)
        }
        for kind in old_handlers:
            signal.signal(kind, self.request_stop)
        error = None
        try:
            warmup = time.monotonic() + 1.0
            while time.monotonic() < warmup:
                self.guard(armed=False)
                self.tick(0.0)
            self.arm()
            start = time.monotonic()
            total = 2 * self.plan["ramp_s"] + self.plan["hold_s"]
            while time.monotonic() - start < total:
                self.guard(armed=True)
                self.tick(pulse_effort(time.monotonic() - start, self.plan))
            self.events.append("pulse_finished")
        except (Exception, KeyboardInterrupt) as caught:
            error = str(caught)
        finally:
            try:
                disarmed = self.stop_and_disarm()
            except Exception as cleanup_error:
                self.events.append(f"cleanup_failed: {cleanup_error}")
                disarmed = False
            finally:
                for kind, handler in old_handlers.items():
                    signal.signal(kind, handler)
        if not disarmed:
            raise RuntimeError(
                "DISARM NOT CONFIRMED: remove propulsion power locally"
            )
        if error is not None:
            raise RuntimeError(error)


def print_status(report):
    state = report.get("last_messages", {}).get("state", {})
    print(
        f"Pixhawk: connected={state.get('connected')} "
        f"armed={state.get('armed')} mode={state.get('mode')}"
    )
    for key in ("imu", "depth", "dvl_odom", "rc_out"):
        item = report["topics"].get(key)
        print(
            f"{key}: "
            + (
                "no samples"
                if item is None
                else f"{item['rate_hz']:.1f} Hz, age {item['age_s']:.2f}s"
            )
        )
    for port, packet in report["servo_ports_on_jetson_link"].items():
        print(f"SERVO_OUTPUT_RAW port {port}: {packet['channels']}")
    print("Teensy/M5 delivery and motor movement are not confirmed by MAVROS.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("status", "pulse"))
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument(
        "--axis", choices=("throttle", "yaw"), default="throttle"
    )
    parser.add_argument("--effort", type=float, default=0.05)
    parser.add_argument("--hold", type=float, default=0.5)
    parser.add_argument("--ramp", type=float, default=0.25)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="send neutral, arm, pulse, neutral, then disarm",
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(remove_ros_args()[1:])
    if not 1 <= args.seconds <= 60:
        parser.error("--seconds must be between 1 and 60")
    if args.execute and args.action != "pulse":
        parser.error("--execute applies only to pulse")
    try:
        plan = pulse_plan(args.axis, args.effort, args.hold, args.ramp)
    except ValueError as error:
        parser.error(str(error))
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    monitor = FieldMonitor()
    pulse = None
    report = {}
    error = None
    try:
        monitor.read_cached_configuration()
        monitor.collect(args.seconds)
        report = monitor.snapshot()
        print_status(report)
        if args.action == "pulse":
            report["pulse_plan"] = plan
            report["executed"] = args.execute
            preflight(report)
            if args.execute:
                pulse = PulseTest(monitor, plan)
                pulse.run()
            else:
                print(
                    "DRY RUN: no commands sent. --execute will ARM and pulse."
                )
    except (Exception, KeyboardInterrupt) as caught:
        error = str(caught) or "Interrupted"
        print(f"Field check stopped: {error}", file=sys.stderr)
    finally:
        result = monitor.snapshot()
        result.update(
            {
                key: report[key]
                for key in ("pulse_plan", "executed")
                if key in report
            }
        )
        result["commands_sent"] = 0 if pulse is None else pulse.commands_sent
        result["passive"] = result["commands_sent"] == 0
        result["executed"] = not result["passive"]
        result["error"] = error
        if pulse is not None:
            result["pulse_events"] = pulse.events
        if args.output is not None:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(
                json.dumps(json_finite(result), indent=2, allow_nan=False)
                + "\n"
            )
            print(f"Report: {args.output}")
        monitor.destroy_node()
        rclpy.shutdown()
    if error is not None:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

"""Log USB passthrough traffic and request the M5 diagnostic/config menus."""

import argparse
import fcntl
import json
import os
from pathlib import Path
import select
import struct
import sys
import termios
import time


class Console:
    def __init__(self, descriptor, log):
        self.descriptor = descriptor
        self.log = log

    def record(self, direction, data):
        self.log.write(
            json.dumps(
                {
                    "time_ns": time.time_ns(),
                    "direction": direction,
                    "hex": data.hex(),
                }
            )
            + "\n"
        )
        self.log.flush()
        print(f"{direction}: {data!r}", flush=True)

    def send(self, data):
        deadline = time.monotonic() + 1.0
        while data:
            remaining = deadline - time.monotonic()
            if (
                remaining <= 0
                or not select.select(
                    [], [self.descriptor], [], max(0.0, remaining)
                )[1]
            ):
                raise TimeoutError("USB write timed out")
            try:
                count = os.write(self.descriptor, data)
            except BlockingIOError:
                continue
            if count == 0:
                raise ConnectionError("USB write made no progress")
            self.record("USB_TX", data[:count])
            data = data[count:]

    def receive(self, timeout):
        if not select.select([self.descriptor], [], [], timeout)[0]:
            return b""
        try:
            data = os.read(self.descriptor, 4096)
        except BlockingIOError:
            return b""
        if not data:
            raise ConnectionError("Teensy USB disconnected")
        self.record("USB_RX", data)
        return data

    def enable(self, binary=False):
        # Let USB/DTR settle before sending the firmware's local command.
        deadline = time.monotonic() + 0.25
        startup = bytearray()
        while time.monotonic() < deadline:
            startup.extend(self.receive(max(0.0, deadline - time.monotonic())))
            if len(startup) > 8192:
                del startup[:-1024]
        if binary and b"M5-BINARY" not in startup:
            raise RuntimeError("Binary mode needs the updated firmware banner")
        # A fresh USB session is unarmed; clear any partial text command.
        command = b"M5-BINARY" if binary else b"M5-PASSTHROUGH"
        self.send(b"\x1d\r" + command + b"\r")
        confirmation = (
            b"Binary forwarding at 115200 8N1"
            if binary
            else b"Forwarding at 115200 8N1"
        )
        received = bytearray()
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            received.extend(self.receive(0.1))
            if confirmation in received:
                return
            if len(received) > 8192:
                del received[:-1024]
        raise TimeoutError("No passthrough enable confirmation; stop here")

    def enter_diagnostics(self):
        for _ in range(5):
            self.send(b"+")
            deadline = time.monotonic() + 0.25
            while time.monotonic() < deadline:
                self.receive(max(0.0, deadline - time.monotonic()))
        print("Five '+' bytes sent over USB. Waiting for motor output.")


def open_device(device):
    descriptor = os.open(
        device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK | os.O_CLOEXEC
    )
    try:
        fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.ioctl(descriptor, termios.TIOCEXCL)
        attributes = termios.tcgetattr(descriptor)
        attributes[0] = attributes[1] = attributes[3] = 0
        attributes[2] = (
            termios.CS8 | termios.CREAD | termios.CLOCAL | termios.HUPCL
        )
        attributes[4] = attributes[5] = termios.B115200
        attributes[6][termios.VMIN] = attributes[6][termios.VTIME] = 0
        termios.tcsetattr(descriptor, termios.TCSANOW, attributes)
        termios.tcflush(descriptor, termios.TCIOFLUSH)
        if Path(device).name.startswith(("cu.usbmodem", "ttyACM")):
            # Reopening a binary session must reset its mode before arming.
            fcntl.ioctl(
                descriptor,
                termios.TIOCMBIC,
                struct.pack("I", termios.TIOCM_DTR),
            )
            time.sleep(0.05)
            fcntl.ioctl(
                descriptor,
                termios.TIOCMBIS,
                struct.pack("I", termios.TIOCM_DTR),
            )
        return descriptor
    except BaseException:
        os.close(descriptor)
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", required=True)
    parser.add_argument("--log", type=Path, required=True)
    options = parser.parse_args()
    descriptor = open_device(options.device)
    try:
        with options.log.open("a", encoding="utf-8") as log:
            console = Console(descriptor, log)
            try:
                console.enable()
                print(
                    "READY. Power up the isolated motor, then type enter + "
                    "Return within five seconds.\n"
                    "Commands: enter = five paced '+' bytes; config = 'c'; "
                    "quit = close. Other text is never forwarded.\n"
                    "Closing does not stop a running motor.\n"
                    f"Logging to {options.log}",
                    flush=True,
                )
                while True:
                    readable = select.select(
                        [descriptor, sys.stdin], [], [], 0.5
                    )[0]
                    if descriptor in readable:
                        console.receive(0.0)
                    if sys.stdin not in readable:
                        continue
                    line = sys.stdin.readline()
                    if not line or line.strip() == "quit":
                        break
                    command = line.strip()
                    if command == "enter":
                        console.enter_diagnostics()
                    elif command == "config":
                        console.send(b"c")
                    else:
                        print("Use enter, config or quit; nothing sent.")
            finally:
                console.send(b"\x1d")
    except KeyboardInterrupt:
        pass
    finally:
        os.close(descriptor)


if __name__ == "__main__":
    main()

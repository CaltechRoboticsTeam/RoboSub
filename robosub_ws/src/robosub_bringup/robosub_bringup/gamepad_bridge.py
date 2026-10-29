"""Serialize browser input and reject old packets or a second operator."""

import threading
import time

from robosub_control.config import constants

from robosub_control.teleop.gamepad import validate_axes


class BrowserGamepad:
    """One short-lived input lease; publishing occurs under the same lock.

    Sequence numbers prevent a delayed movement request from overtaking a
    newer stop. A different tab can claim input after the old lease expires,
    starting with neutral sticks and the enable bumper released.
    """

    def __init__(self, publish):
        self.publish = publish
        self.lock = threading.Lock()
        self.session = None
        self.sequence = -1
        self.last_rx = -float("inf")

    def update(self, data):
        session = data.get("session")
        sequence = data.get("sequence")
        held = data.get("held")
        stop = data.get("stop", False)
        if type(stop) is not bool:
            raise ValueError("Invalid stop flag")
        if not isinstance(session, str) or not 1 <= len(session) <= 64:
            raise ValueError("Invalid controller session")
        if type(sequence) is not int or not 0 <= sequence < 2**53:
            raise ValueError("Invalid controller sequence")
        if type(held) is not bool:
            raise ValueError("Provide the RB button state")
        axes = validate_axes(data.get("axes", []))
        with self.lock:
            now = time.monotonic()
            expired = now - self.last_rx > constants.GAMEPAD_INPUT_TIMEOUT_S
            if session == self.session and sequence <= self.sequence:
                raise ValueError("Old controller input was discarded")
            if session != self.session and not expired:
                raise ValueError("Another browser has controller input")
            if session != self.session or expired:
                if held or any(
                    abs(value) > constants.GAMEPAD_DEADZONE for value in axes
                ):
                    raise ValueError("Center sticks and release RB to connect")
            self.session = session
            self.sequence = sequence
            self.last_rx = now
            if stop:
                self.publish([], False)
            else:
                self.publish(axes, held)
        return {"successful": True}

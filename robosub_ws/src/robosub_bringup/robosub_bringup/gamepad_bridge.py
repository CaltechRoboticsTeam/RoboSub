"""Handle browser controller input."""

import threading
import time

from robosub_control.config import constants
from robosub_control.teleop.gamepad import (
    NO_BUTTONS,
    validate_axes,
    validate_buttons,
)


class BrowserGamepad:
    """Reject delayed packets and allow one controller session at a time."""

    def __init__(self, publish):
        self.publish = publish
        self.lock = threading.Lock()
        self.session = None
        self.sequence = -1
        self.last_rx = -float("inf")

    def update(self, data):
        session = data.get("session")
        sequence = data.get("sequence")
        enabled = data.get("enabled")
        if type(enabled) is not bool:
            raise ValueError("Provide the operator enabled state")
        if not isinstance(session, str) or not 1 <= len(session) <= 64:
            raise ValueError("Invalid controller session")
        if type(sequence) is not int or not 0 <= sequence < 2**53:
            raise ValueError("Invalid controller sequence")
        axes = validate_axes(data.get("axes", []))
        buttons = validate_buttons(data.get("buttons", NO_BUTTONS))
        with self.lock:
            now = time.monotonic()
            expired = now - self.last_rx > constants.GAMEPAD_INPUT_TIMEOUT_S
            if session == self.session and sequence <= self.sequence:
                raise ValueError("Old controller input was discarded")
            if session != self.session and not expired:
                raise ValueError("Another browser has controller input")
            self.session = session
            self.sequence = sequence
            self.last_rx = now
            self.publish(axes, enabled, buttons)
        return {"successful": True}

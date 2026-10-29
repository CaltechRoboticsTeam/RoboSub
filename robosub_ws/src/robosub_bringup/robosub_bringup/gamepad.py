"""Xbox stick mapping and enable/watchdog state, independent of ROS."""

import math
from numbers import Real

from robosub_sim import constants

ZERO = (0.0, 0.0, 0.0, 0.0)


def validate_axes(axes):
    """Read the two sticks; ignore extra SDL trigger axes."""
    if len(axes) < 4:
        raise ValueError("Expected left X/Y and right X/Y axes")
    values = tuple(axes[:4])
    if any(
        isinstance(value, bool)
        or not isinstance(value, Real)
        or not math.isfinite(value)
        or abs(value) > 1.0
        for value in values
    ):
        raise ValueError("Stick axes must be finite numbers between -1 and 1")
    return values


class GamepadControl:
    """Map an explicitly selected layout into body surge, sway, heave, yaw.

    Each reconnect/timeout requires neutral sticks with RB released before
    movement can be enabled. Freshness uses monotonic wall time, including
    when Gazebo is paused. Efforts are normalized, not speed setpoints.
    """

    def __init__(self, source, max_effort=constants.GAMEPAD_MAX_EFFORT):
        if source not in constants.GAMEPAD_ENABLE_BUTTON:
            raise ValueError("Gamepad source must be browser or local")
        if (
            type(max_effort) not in (int, float)
            or not math.isfinite(max_effort)
            or not 0 < max_effort <= 1
        ):
            raise ValueError("Gamepad maximum effort must be between 0 and 1")
        self.source = source
        self.max_effort = float(max_effort)
        self.deadzone = constants.GAMEPAD_DEADZONE
        self.timeout = constants.GAMEPAD_INPUT_TIMEOUT_S
        if not 0 <= self.deadzone < 1 or not 0 < self.timeout < math.inf:
            raise ValueError("Invalid gamepad deadzone or input timeout")
        self.stop()

    def stop(self):
        self.ready = False
        self.enabled = False
        self.last_rx = None
        self.command = ZERO

    def accept(self, axes, buttons, now):
        try:
            values = validate_axes(axes)
            index = constants.GAMEPAD_ENABLE_BUTTON[self.source]
            if len(buttons) <= index or type(buttons[index]) is not int:
                raise ValueError("Missing/invalid RB enable button")
            if buttons[index] not in (0, 1):
                raise ValueError("RB button must be zero or one")
        except (ValueError, TypeError, OverflowError):
            self.stop()
            raise ValueError("Invalid gamepad input") from None
        # Check here too: a new input can arrive before the watchdog timer.
        self.sample(now)
        self.last_rx = now
        if not buttons[index]:
            self.ready = all(abs(value) <= self.deadzone for value in values)
            self.enabled = False
            self.command = ZERO
            return
        self.enabled = self.ready
        if not self.enabled:
            self.command = ZERO
            return

        def axis(value):
            magnitude = max(0.0, abs(value) - self.deadzone)
            return math.copysign(magnitude / (1.0 - self.deadzone), value)

        # Browser axes are right/down positive. ROS SDL axes are left/up
        # positive; game_controller_node has already inverted SDL's axes.
        polarity = -1.0 if self.source == "browser" else 1.0
        left_x, left_y, right_x, right_y = (
            polarity * axis(value) for value in values
        )
        translation = max(1.0, math.hypot(left_x, left_y))
        self.command = tuple(
            self.max_effort * value
            for value in (
                left_y / translation,
                left_x / translation,
                right_y,
                right_x,
            )
        )

    def sample(self, now):
        if self.last_rx is None or not 0 <= now - self.last_rx <= self.timeout:
            self.stop()
        return self.command

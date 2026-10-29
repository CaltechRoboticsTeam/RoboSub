"""Xbox input mapping and movement toggle, independent of ROS."""

import math
from numbers import Real

from robosub_control.config import constants

ZERO = (0.0, 0.0, 0.0, 0.0, 0.0)


def validate_axes(axes):
    """Validate the two sticks and two normalized browser triggers."""
    if len(axes) != 6:
        raise ValueError("Expected four stick axes followed by LT and RT")
    values = tuple(axes)
    if any(
        isinstance(value, bool)
        or not isinstance(value, Real)
        or not math.isfinite(value)
        or abs(value) > 1.0
        for value in values
    ) or any(value < 0 for value in values[4:]):
        raise ValueError("Invalid stick or trigger values")
    return values


class GamepadControl:
    """Map input to surge/sway/heave/roll/yaw; RB edges toggle movement.

    Startup, invalid input and timeouts require neutral controls with RB
    released, followed by a new press. Releasing RB leaves movement enabled.
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
        self.pressed = False
        self.last_rx = None
        self.command = ZERO

    def accept(self, axes, buttons, now):
        try:
            # ROS SDL inverts trigger axes: released=0, pressed=-1.
            values = list(axes)
            if self.source == "local" and len(values) == 6:
                values[4:] = [-value for value in values[4:]]
            values = validate_axes(values)
            index = constants.GAMEPAD_ENABLE_BUTTON[self.source]
            if len(buttons) <= index or type(buttons[index]) is not int:
                raise ValueError("Missing/invalid RB button")
            if buttons[index] not in (0, 1):
                raise ValueError("RB button must be zero or one")
        except (ValueError, TypeError, OverflowError):
            self.stop()
            raise ValueError("Invalid gamepad input") from None
        self.sample(now)
        self.last_rx = now
        pressed = bool(buttons[index])
        neutral = all(abs(value) <= self.deadzone for value in values)
        rising = pressed and not self.pressed
        self.pressed = pressed
        if not self.ready:
            self.ready = neutral and not pressed
            self.command = ZERO
            return
        if rising:
            if self.enabled:
                self.enabled = False
            elif neutral:
                self.enabled = True
        if not self.enabled:
            self.command = ZERO
            return

        def axis(value):
            magnitude = max(0.0, abs(value) - self.deadzone)
            return math.copysign(magnitude / (1.0 - self.deadzone), value)

        polarity = -1.0 if self.source == "browser" else 1.0
        left_x, left_y, right_x, right_y = (
            polarity * axis(value) for value in values[:4]
        )
        left_trigger, right_trigger = (axis(value) for value in values[4:])
        translation = max(1.0, math.hypot(left_x, left_y))
        self.command = tuple(
            self.max_effort * value
            for value in (
                left_y / translation,
                left_x / translation,
                right_y,
                right_trigger - left_trigger,
                right_x,
            )
        )

    def sample(self, now):
        if self.last_rx is None or not 0 <= now - self.last_rx <= self.timeout:
            self.stop()
        return self.command

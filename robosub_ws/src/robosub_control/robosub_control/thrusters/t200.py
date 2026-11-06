"""Interpolate measured T200 thrust and current curves."""

import json
import math
from bisect import bisect_left
from functools import lru_cache
from importlib.resources import files
from numbers import Real

from robosub_control.config import future_t200 as constants


def finite_number(value, name):
    """Check that a command value is a finite number."""
    try:
        valid = (
            isinstance(value, Real)
            and not isinstance(value, bool)
            and math.isfinite(value)
        )
    except OverflowError:
        valid = False
    if not valid:
        raise ValueError(f"{name} must be a finite number")
    return float(value)


def interpolate(value, coordinates, samples):
    """Piecewise linear interpolation, including repeated sample values."""
    if value <= coordinates[0]:
        return samples[0]
    if value >= coordinates[-1]:
        return samples[-1]
    upper = bisect_left(coordinates, value)
    lower = upper - 1
    weight = (value - coordinates[lower]) / (
        coordinates[upper] - coordinates[lower]
    )
    return samples[lower] + weight * (samples[upper] - samples[lower])


@lru_cache(maxsize=1)
def load_curves():
    source = files("robosub_control.thrusters").joinpath("data/t200.json")
    data = json.loads(source.read_text())
    return {float(voltage): rows for voltage, rows in data["curves"].items()}


class _Branch:
    """One direction, indexed by pulse distance from neutral."""

    def __init__(self, rows, sign, current_limit_a):
        self.offsets = []
        self.forces = []
        self.currents = []
        neutral = constants.ESC_PWM_NEUTRAL_US
        ordered = rows if sign > 0 else list(reversed(rows))
        for pwm, force, current, _rpm in ordered:
            offset = sign * (pwm - neutral)
            if offset < 0:
                continue
            if offset <= constants.ESC_PWM_DEADBAND_US:
                force, current = 0.0, 0.0
            self.offsets.append(float(offset))
            self.forces.append(
                max(sign * force, self.forces[-1] if self.forces else 0.0)
            )
            self.currents.append(
                max(current, self.currents[-1] if self.currents else 0.0)
            )
        limit = interpolate(current_limit_a, self.currents, self.offsets)
        self.max_offset = math.floor(limit)
        self.minimum_force = next((v for v in self.forces if v > 0), 0.0)
        self.maximum_force = interpolate(
            self.max_offset, self.offsets, self.forces
        )
        if self.maximum_force < self.minimum_force:
            self.maximum_force = 0.0

    def force_to_offset(self, force):
        force = min(force, self.maximum_force)
        if force <= 0 or force < self.minimum_force:
            return 0
        offset = interpolate(force, self.forces, self.offsets)
        return min(self.max_offset, int(round(offset)))


class T200Model:
    """Convert thrust in newtons to T200 PWM in microseconds."""

    def __init__(self, voltage_v, current_limit_a=None):
        self.voltage_v = finite_number(voltage_v, "voltage_v")
        limit = (
            constants.ESC_CONTINUOUS_CURRENT_A
            if current_limit_a is None
            else finite_number(current_limit_a, "current_limit_a")
        )
        if not 0 < limit <= constants.ESC_CONTINUOUS_CURRENT_A:
            raise ValueError("Current model limit must be in (0, 30] A")
        curves = load_curves()
        voltages = sorted(curves)
        if not voltages[0] <= self.voltage_v <= voltages[-1]:
            raise ValueError(
                "T200 measurements support 10-20 V; no extrapolation"
            )
        upper = min(bisect_left(voltages, self.voltage_v), len(voltages) - 1)
        lower = max(0, upper - 1)
        if voltages[upper] == self.voltage_v:
            lower = upper
        span = voltages[upper] - voltages[lower]
        weight = (self.voltage_v - voltages[lower]) / span if span else 0.0
        rows = []
        for low, high in zip(
            curves[voltages[lower]], curves[voltages[upper]], strict=True
        ):
            rows.append(
                [
                    low[0],
                    *(
                        a + weight * (b - a)
                        for a, b in zip(low[1:], high[1:], strict=True)
                    ),
                ]
            )
        self.forward = _Branch(rows, 1, limit)
        self.reverse = _Branch(rows, -1, limit)
        self.reference_forward_force_n = rows[-1][1]
        self.reference_forward_rpm = rows[-1][3]
        self.current_limit_a = limit

    def force_limits(self, reversed_motor=False):
        """Return reverse and forward thrust limits."""
        return -self.reverse.maximum_force, self.forward.maximum_force

    def force_to_pwm(self, force_n, reversed_motor=False):
        force = finite_number(force_n, "force_n")
        branch = self.forward if force >= 0 else self.reverse
        offset = branch.force_to_offset(abs(force))
        sign = 1 if force >= 0 else -1
        if reversed_motor:
            sign = -sign
        return constants.ESC_PWM_NEUTRAL_US + sign * offset

    def force_from_pwm(self, pwm_us, reversed_motor=False):
        pwm = self._validate_pwm(pwm_us)
        if reversed_motor:
            pwm = 2 * constants.ESC_PWM_NEUTRAL_US - pwm
        offset = pwm - constants.ESC_PWM_NEUTRAL_US
        branch = self.forward if offset >= 0 else self.reverse
        force = interpolate(abs(offset), branch.offsets, branch.forces)
        sign = 1 if offset >= 0 else -1
        return force * sign

    def current_from_pwm(self, pwm_us, reversed_motor=False):
        pwm = self._validate_pwm(pwm_us)
        if reversed_motor:
            pwm = 2 * constants.ESC_PWM_NEUTRAL_US - pwm
        offset = pwm - constants.ESC_PWM_NEUTRAL_US
        branch = self.forward if offset >= 0 else self.reverse
        return interpolate(abs(offset), branch.offsets, branch.currents)

    @staticmethod
    def _validate_pwm(value):
        pwm = finite_number(value, "pwm_us")
        if not constants.ESC_PWM_MIN_US <= pwm <= constants.ESC_PWM_MAX_US:
            raise ValueError("ESC PWM must be between 1100 and 1900 us")
        return pwm

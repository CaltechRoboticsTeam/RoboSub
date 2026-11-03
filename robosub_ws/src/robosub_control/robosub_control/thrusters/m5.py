"""Team-measured M5 force/command calibration at nominal 24 V."""

from functools import lru_cache
from importlib.resources import files
import json
import math

from robosub_control.thrusters.t200 import finite_number


@lru_cache(maxsize=1)
def load_curve():
    path = files("robosub_control.thrusters").joinpath("data/m5.json")
    return json.loads(path.read_text())


class M5Model:
    def __init__(self, voltage_v=24.0):
        self.voltage_v = finite_number(voltage_v, "M5 supply voltage")
        if not 20.0 <= self.voltage_v <= 25.5:
            raise ValueError("Current M5 calibration uses 20-25.5 V")
        self.curve = load_curve()
        self.newtons_per_lbf = self.curve["newtons_per_lbf"]
        self.denominator = (
            self.voltage_v
            + self.curve["voltage_denominator_quadratic"] * self.voltage_v**2
        )
        self.command_slew_rate = self.curve["command_slew_rate_per_s"]
        self._limits = (
            self.force_from_command(-1.0),
            self.force_from_command(1.0),
        )

    def _signal(self, pounds, coefficients):
        return (
            coefficients["linear"] * pounds
            + coefficients["atan_scale"]
            * math.atan(coefficients["atan_argument"] * pounds)
            + coefficients["tanh_scale"]
            * math.tanh(coefficients["tanh_argument"] * pounds)
        ) / self.denominator

    def force_to_command(self, force_n, reversed_motor=False):
        force = finite_number(force_n, "M5 force")
        if reversed_motor:
            force = -force
        branch = self.curve["positive" if force >= 0 else "negative"]
        command = self._signal(force / self.newtons_per_lbf, branch)
        return max(-1.0, min(1.0, command))

    def force_from_command(self, command, reversed_motor=False):
        command = finite_number(command, "M5 command")
        if abs(command) > 1.0:
            raise ValueError("M5 command must be in [-1, 1]")
        if command == 0:
            return 0.0
        branch = self.curve["positive" if command > 0 else "negative"]
        target = abs(command)
        low = 0.0
        # Positive atan/tanh terms make the linear-only solution an upper bound.
        high = target * self.denominator / branch["linear"]
        for _ in range(48):
            middle = (low + high) / 2
            if self._signal(middle, branch) < target:
                low = middle
            else:
                high = middle
        force = math.copysign((low + high) / 2, command)
        force *= self.newtons_per_lbf
        return -force if reversed_motor else force

    def force_limits(self, reversed_motor=False):
        low, high = self._limits
        return (-high, -low) if reversed_motor else (low, high)

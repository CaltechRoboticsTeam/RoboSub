"""Provisional M5 static thrust model; never used by physical transports."""

from functools import lru_cache
from importlib.resources import files
import json

from robosub_control.thrusters.t200 import finite_number, interpolate


@lru_cache(maxsize=1)
def load_curve():
    """Load approximate manufacturer chart points, not command calibration."""
    path = files("robosub_control.thrusters").joinpath("data/m5.json")
    data = json.loads(path.read_text())
    rows = data["power_w_thrust_kgf"]
    previous = (0.0, 0.0)
    for row in rows:
        if len(row) != 2:
            raise ValueError("Invalid M5 thrust sample")
        power, thrust = (finite_number(v, "M5 chart sample") for v in row)
        if power <= previous[0] or thrust <= previous[1]:
            raise ValueError("M5 chart samples must increase above zero")
        previous = (power, thrust)
    if not rows:
        raise ValueError("M5 thrust chart is empty")
    return data


class M5SimulationModel:
    """Map normalized electrical commands to a static force approximation.

    Fixed 48 V is an assumption, not a measured vehicle supply or a voltage
    label on the published chart. Command magnitude approximates shaft
    speed, and electrical power follows its cube. Below the first chart
    point, thrust follows P**(2/3). Reverse uses an explicit unmeasured ratio.
    """

    def __init__(
        self,
        voltage_v=48.0,
        power_limit_w=570.0,
        command_power_exponent=3.0,
        reverse_thrust_ratio=1.0,
    ):
        self.voltage_v = finite_number(voltage_v, "simulation voltage")
        self.power_limit_w = finite_number(power_limit_w, "power limit")
        self.exponent = finite_number(command_power_exponent, "power exponent")
        self.reverse_ratio = finite_number(
            reverse_thrust_ratio, "reverse ratio"
        )
        data = load_curve()
        self.powers, self.thrusts = zip(
            *data["power_w_thrust_kgf"], strict=True
        )
        self.thrusts = tuple(value * 9.80665 for value in self.thrusts)
        self.rated_power_w = self.powers[-1]
        if self.voltage_v != 48.0:
            raise ValueError("M5 simulation supports only the assumed 48 V")
        if not 0 < self.power_limit_w <= self.rated_power_w:
            raise ValueError("M5 power limit must be in (0, 570] W")
        if not 0 < self.exponent <= 6:
            raise ValueError("M5 command power exponent must be in (0, 6]")
        if not 0 < self.reverse_ratio <= 1:
            raise ValueError("M5 reverse thrust ratio must be in (0, 1]")

    def power_from_command(self, command):
        """Estimate electrical watts; the normalized command is not watts."""
        command = finite_number(command, "M5 command")
        if abs(command) > 1:
            raise ValueError("M5 command must be in [-1, 1]")
        return min(
            self.rated_power_w * abs(command) ** self.exponent,
            self.power_limit_w,
        )

    def thrust_from_power(self, power_w):
        """Interpolate static thrust; extrapolate only the low-power branch."""
        power = finite_number(power_w, "electrical power")
        if not 0 <= power <= self.rated_power_w:
            raise ValueError("M5 chart covers 0-570 W only")
        if power < self.powers[0]:
            return self.thrusts[0] * (power / self.powers[0]) ** (2.0 / 3.0)
        return interpolate(power, self.powers, self.thrusts)

    def force_from_command(self, command):
        """Return signed axial newtons before mounting polarity is applied."""
        force = self.thrust_from_power(self.power_from_command(command))
        return -force * self.reverse_ratio if command < 0 else force

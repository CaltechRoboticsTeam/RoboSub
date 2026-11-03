"""Five-axis geometry allocation in a selected vehicle command domain."""

from dataclasses import dataclass
import math

import numpy as np

from robosub_control.thrusters.m5 import M5Model
from robosub_control.thrusters.t200 import T200Model, finite_number
from robosub_control.config.vehicle_profiles import get_profile

AXES = ("surge", "sway", "heave", "roll", "yaw")
CONTROLLED_ROWS = (0, 1, 2, 3, 5)


@dataclass(frozen=True)
class Allocation:
    targets: tuple
    requested: tuple
    achieved: tuple
    scale: float


class ThrustAllocator:
    """Allocate body forces/torques, then convert forces to motor commands."""

    def __init__(self, voltage_v=None, vehicle="current_m5"):
        self.profile = profile = get_profile(vehicle)
        self.names = profile.THRUSTER_NAMES
        self.is_m5 = profile.ACTUATOR_KIND == "m5"
        if self.is_m5:
            voltage = (
                profile.M5_SUPPLY_VOLTAGE_V if voltage_v is None else voltage_v
            )
            self.model = M5Model(voltage)
        else:
            channels = list(profile.THRUSTER_CHANNELS.values())
            if set(profile.THRUSTER_CHANNELS) != set(self.names):
                raise ValueError("Every T200 needs a board channel")
            if any(type(channel) is not int for channel in channels):
                raise ValueError("Thruster channels must be integers")
            if sorted(channels) != list(range(len(self.names))):
                raise ValueError("Thruster channels must be unique values 0-5")
            voltage = (
                profile.THRUSTER_SUPPLY_VOLTAGE_V
                if voltage_v is None
                else voltage_v
            )
            self.model = T200Model(voltage, profile.ESC_CURRENT_MODEL_LIMIT_A)
        if set(profile.THRUSTER_REVERSED) != set(self.names) or any(
            type(value) is not bool
            for value in profile.THRUSTER_REVERSED.values()
        ):
            raise ValueError("Every thruster reversal setting must be boolean")
        columns = []
        for name in self.names:
            position = np.asarray(
                profile.THRUSTER_POSITIONS_BODY_M[name], dtype=float
            )
            yaw = math.radians(profile.THRUSTER_YAW_CW_DEG[name])
            pitch = math.radians(profile.THRUSTER_PITCH_DOWN_DEG[name])
            direction = np.array(
                (
                    math.cos(yaw) * math.cos(pitch),
                    -math.sin(yaw) * math.cos(pitch),
                    -math.sin(pitch),
                )
            )
            if (
                position.shape != (3,)
                or not np.all(np.isfinite(position))
                or not np.all(np.isfinite(direction))
            ):
                raise ValueError(f"Invalid mount geometry: {name}")
            direction[np.abs(direction) < 1e-12] = 0.0
            columns.append(
                np.concatenate(
                    (
                        direction,
                        np.cross(position, direction),
                    )
                )
            )
        self.wrench_matrix = np.column_stack(columns)
        self.control_matrix = self.wrench_matrix[list(CONTROLLED_ROWS), :]
        if np.linalg.matrix_rank(self.control_matrix) != len(AXES):
            raise ValueError("Thruster layout cannot control all five axes")
        self.inverse = np.linalg.pinv(self.control_matrix, rcond=1e-12)
        self.limits = np.array(
            [
                self.model.force_limits(profile.THRUSTER_REVERSED[name])
                for name in self.names
            ]
        )
        scales = []
        for index, axis in enumerate(AXES):
            positive = self._maximum_scale(self.inverse[:, index])
            negative = self._maximum_scale(-self.inverse[:, index])
            fraction = profile.MIXER_AXIS_FRACTIONS[axis]
            if not math.isfinite(fraction) or not 0 < fraction <= 1:
                raise ValueError(f"Invalid mixer fraction: {axis}")
            scales.append(min(positive, negative) * fraction)
        self.axis_scales = np.array(scales)

    def _maximum_scale(self, forces, bounds=None):
        limits = []
        bounds = self.limits if bounds is None else bounds
        for force, (low, high) in zip(forces, bounds, strict=True):
            if force > 1e-12:
                limits.append(high / force)
            elif force < -1e-12:
                limits.append(low / force)
        if not limits:
            return math.inf
        if min(limits) <= 0:
            raise ValueError("No usable thrust authority for this command")
        return min(limits)

    def allocate(self, efforts):
        efforts = np.asarray(efforts, dtype=float)
        if efforts.shape != (5,) or not np.all(np.isfinite(efforts)):
            raise ValueError("Expected five finite normalized efforts")
        requested = np.clip(efforts, -1.0, 1.0) * self.axis_scales
        forces = self.inverse @ requested
        scale = min(1.0, self._maximum_scale(forces)) if np.any(forces) else 1.0
        forces *= scale
        requested_six = np.zeros(6)
        requested_six[list(CONTROLLED_ROWS)] = requested
        achieved = self.wrench_matrix @ forces
        return Allocation(
            tuple(float(force) for force in forces),
            tuple(float(value) for value in requested_six),
            tuple(float(value) for value in achieved),
            float(scale),
        )

    def commands(self, targets, max_power=1.0):
        """Convert newtons to named M5 commands or channel-ordered T200 PWM."""
        values = np.asarray(targets, dtype=float)
        if values.shape != (len(self.names),) or not np.all(
            np.isfinite(values)
        ):
            raise ValueError("Expected six finite targets in named motor order")
        profile = self.profile
        if self.is_m5:
            cap = finite_number(max_power, "M5 command limit")
            if not 0 < cap <= 1:
                raise ValueError("M5 command limit must be in (0, 1]")
            low = self.model.force_from_command(-cap)
            high = self.model.force_from_command(cap)
            bounds = [
                (-high, -low)
                if profile.THRUSTER_REVERSED[name]
                else (low, high)
                for name in self.names
            ]
            if np.any(values):
                values = values * min(1.0, self._maximum_scale(values, bounds))
            return tuple(
                self.model.force_to_command(
                    float(value), profile.THRUSTER_REVERSED[name]
                )
                for name, value in zip(self.names, values, strict=True)
            )
        pulses = [profile.ESC_PWM_NEUTRAL_US] * len(self.names)
        for name, force in zip(self.names, values, strict=True):
            channel = profile.THRUSTER_CHANNELS[name]
            pulses[channel] = self.model.force_to_pwm(
                float(force), profile.THRUSTER_REVERSED[name]
            )
        return tuple(pulses)

    def pwm_by_channel(self, forces_n):
        if self.is_m5:
            raise ValueError("M5 does not use the Basic ESC PWM interface")
        return self.commands(forces_n)

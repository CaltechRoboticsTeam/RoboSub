"""Plan moves with speed and acceleration limits."""

import math
from collections import deque
from dataclasses import dataclass


def finite_pair(values):
    """Check an XY pair for finite values."""
    pair = tuple(values)
    if len(pair) != 2 or not all(math.isfinite(value) for value in pair):
        raise ValueError("Expected two finite coordinates")
    return pair


@dataclass(frozen=True)
class MotionState:
    position: tuple[float, float]
    velocity: tuple[float, float]


class MotionProfile:
    """Accelerate, cruise and brake; brake before changing direction."""

    def __init__(self, position, velocity=(0.0, 0.0)):
        self.state = MotionState(finite_pair(position), finite_pair(velocity))
        self.goal = self.state.position
        self.phases = deque()

    @property
    def finished(self):
        return not self.phases

    def retarget(self, goal, max_speed, max_acceleration):
        goal = finite_pair(goal)
        phases = deque()
        start = self.state.position
        speed = math.hypot(*self.state.velocity)
        if speed > 0.0:
            duration = speed / max_acceleration
            acceleration = tuple(
                -max_acceleration * value / speed
                for value in self.state.velocity
            )
            phases.append((duration, acceleration))
            start = tuple(
                position + 0.5 * velocity * duration
                for position, velocity in zip(
                    start, self.state.velocity, strict=True
                )
            )
        delta = tuple(
            end - begin for begin, end in zip(start, goal, strict=True)
        )
        distance = math.hypot(*delta)
        if distance > 0.0:
            peak = min(max_speed, math.sqrt(distance * max_acceleration))
            ramp = peak / max_acceleration
            cruise = max(0.0, distance / peak - ramp)
            acceleration = tuple(
                max_acceleration * value / distance for value in delta
            )
            phases.append((ramp, acceleration))
            if cruise > 0.0:
                phases.append((cruise, (0.0, 0.0)))
            phases.append((ramp, tuple(-value for value in acceleration)))
        self.goal = goal
        self.phases = phases
        if self.finished:
            self.state = MotionState(goal, (0.0, 0.0))

    def advance(self, dt):
        """Integrate phases exactly, including ticks crossing phase changes."""
        if not math.isfinite(dt) or dt < 0:
            raise ValueError("Profile time must be finite and nonnegative")
        while dt > 0.0 and self.phases:
            remaining, acceleration = self.phases.popleft()
            step = min(dt, remaining)
            position = tuple(
                position + velocity * step + 0.5 * accel * step**2
                for position, velocity, accel in zip(
                    self.state.position,
                    self.state.velocity,
                    acceleration,
                    strict=True,
                )
            )
            velocity = tuple(
                velocity + accel * step
                for velocity, accel in zip(
                    self.state.velocity, acceleration, strict=True
                )
            )
            self.state = MotionState(position, velocity)
            dt -= step
            remaining -= step
            if remaining > 0.0:
                self.phases.appendleft((remaining, acceleration))
        if self.finished:
            self.state = MotionState(self.goal, (0.0, 0.0))
        return self.state

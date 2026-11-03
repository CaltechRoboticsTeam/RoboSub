"""Validate sensor sample age and progress without depending on ROS."""

import math


def stamp_to_sec(stamp):
    """Convert a normalized, nonnegative ROS stamp; reject malformed stamps."""
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        return math.nan
    return (stamp.sec * 1_000_000_000 + stamp.nanosec) * 1e-9


class SensorFreshness:
    """Keep rejected or repeated samples from renewing sensor validity."""

    def __init__(self, max_age):
        if not math.isfinite(max_age) or max_age <= 0:
            raise ValueError("Sensor timeout must be finite and positive")
        self.max_age = max_age
        self.reset()

    def reset(self):
        self.stamp = None
        self.received = None
        self.clock = None

    def _observe_clock(self, now):
        if not math.isfinite(now):
            return False
        if self.clock is not None and now < self.clock:
            self.reset()
        self.clock = now
        return True

    def accept(self, stamp, now, values=()):
        """Accept a new finite sample only within the current clock epoch."""
        if not self._observe_clock(now):
            return False
        if not math.isfinite(stamp) or not all(map(math.isfinite, values)):
            return False
        if stamp < 0 or not 0 <= now - stamp < self.max_age:
            return False
        if self.stamp is not None and stamp <= self.stamp:
            return False
        self.stamp = stamp
        self.received = now
        return True

    def fresh(self, now):
        """Expire on source age, silence, or a rewind of the node clock."""
        return (
            self._observe_clock(now)
            and self.stamp is not None
            and 0 <= now - self.stamp < self.max_age
            and 0 <= now - self.received < self.max_age
        )

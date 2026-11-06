"""Check sensor timestamps."""

import math


def stamp_to_sec(stamp):
    """Convert a ROS timestamp to seconds, or NaN if invalid."""
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        return math.nan
    return (stamp.sec * 1_000_000_000 + stamp.nanosec) * 1e-9


class SensorFreshness:
    """Track new samples and expire stale data."""

    def __init__(self, max_age):
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
        """Accept a new, finite sample with a recent timestamp."""
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
        """Check sample age and clock continuity."""
        return (
            self._observe_clock(now)
            and self.stamp is not None
            and 0 <= now - self.stamp < self.max_age
            and 0 <= now - self.received < self.max_age
        )

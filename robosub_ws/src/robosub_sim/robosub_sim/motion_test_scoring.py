"""Ground-truth checks for a straight move followed by a stable hold."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class TruthSample:
    stamp: float
    x: float
    y: float
    depth: float
    yaw: float
    speed: float
    roll: float = 0.0
    pitch: float = 0.0


class MoveHoldScorer:
    """Score actual motion; these measurements never feed robot control."""

    def __init__(
        self,
        distance=5.0,
        depth=1.0,
        heading=0.0,
        hold_duration=10.0,
        position_tolerance=0.2,
        depth_tolerance=0.1,
        heading_tolerance=math.pi / 36,
        speed_tolerance=0.05,
        sample_timeout=0.25,
        tilt_tolerance=math.pi / 18,
    ):
        values = (
            distance,
            depth,
            heading,
            hold_duration,
            position_tolerance,
            depth_tolerance,
            heading_tolerance,
            speed_tolerance,
            sample_timeout,
            tilt_tolerance,
        )
        if not all(map(math.isfinite, values)) or any(
            value <= 0 for value in values[3:]
        ):
            raise ValueError("Scoring limits must be finite and positive")
        self.distance, self.depth, self.heading = distance, depth, heading
        self.hold_duration = hold_duration
        self.position_tolerance = position_tolerance
        self.depth_tolerance = depth_tolerance
        self.heading_tolerance = heading_tolerance
        self.speed_tolerance = speed_tolerance
        self.sample_timeout = sample_timeout
        self.tilt_tolerance = tilt_tolerance
        self.max_abs_roll = self.max_abs_pitch = 0.0
        self.origin = self.goal = None
        self.latest = None
        self.holding = False
        self.valid_since = None
        self.held_for = 0.0
        self.samples = 0

    def start_move(self, sample):
        self.origin = (sample.x, sample.y)
        self.goal = (
            sample.x + self.distance * math.cos(self.heading),
            sample.y + self.distance * math.sin(self.heading),
        )

    def start_hold(self):
        if self.goal is None:
            raise ValueError("Hold needs a recorded movement origin")
        self.holding = True
        self.reset_hold()

    def reset_hold(self):
        self.valid_since = None
        self.held_for = 0.0

    def observe(self, sample):
        values = (
            sample.stamp,
            sample.x,
            sample.y,
            sample.depth,
            sample.yaw,
            sample.speed,
            sample.roll,
            sample.pitch,
        )
        if not all(map(math.isfinite, values)):
            self.reset_hold()
            return
        if self.latest is not None:
            gap = sample.stamp - self.latest.stamp
            if gap <= 0:
                self.reset_hold()
                return
            if gap >= self.sample_timeout:
                self.reset_hold()
        self.latest = sample
        self.max_abs_roll = max(self.max_abs_roll, abs(sample.roll))
        self.max_abs_pitch = max(self.max_abs_pitch, abs(sample.pitch))
        self.samples += 1
        if not self.holding:
            return
        heading_error = math.atan2(
            math.sin(sample.yaw - self.heading),
            math.cos(sample.yaw - self.heading),
        )
        valid = (
            math.hypot(sample.x - self.goal[0], sample.y - self.goal[1])
            <= self.position_tolerance
            and abs(sample.depth - self.depth) <= self.depth_tolerance
            and abs(heading_error) <= self.heading_tolerance
            and sample.speed <= self.speed_tolerance
            and abs(sample.roll) <= self.tilt_tolerance
            and abs(sample.pitch) <= self.tilt_tolerance
        )
        if not valid:
            self.reset_hold()
        elif self.valid_since is None:
            self.valid_since = sample.stamp
        else:
            self.held_for = sample.stamp - self.valid_since

    def expire(self, now):
        if self.latest is None or not (
            0 <= now - self.latest.stamp < self.sample_timeout
        ):
            self.reset_hold()

    def result(self, within_tolerance, reason):
        result = {
            "within_tolerance": within_tolerance,
            "reason": reason,
            "requested_distance_m": self.distance,
            "requested_depth_m": self.depth,
            "required_hold_s": self.hold_duration,
            "continuous_truth_hold_s": self.held_for,
            "truth_samples": self.samples,
            "max_abs_roll_deg": math.degrees(self.max_abs_roll),
            "max_abs_pitch_deg": math.degrees(self.max_abs_pitch),
            "scope": "Advisory sim score; physical dynamics need measurement",
        }
        if self.latest is not None:
            result["last_truth"] = {
                "stamp_s": self.latest.stamp,
                "x_m": self.latest.x,
                "y_m": self.latest.y,
                "depth_m": self.latest.depth,
                "yaw_rad": self.latest.yaw,
                "speed_m_s": self.latest.speed,
                "roll_deg": math.degrees(self.latest.roll),
                "pitch_deg": math.degrees(self.latest.pitch),
                "yaw_deg": math.degrees(self.latest.yaw),
            }
        if self.latest is not None and self.origin is not None:
            dx = self.latest.x - self.origin[0]
            dy = self.latest.y - self.origin[1]
            result.update(
                forward_distance_m=(
                    dx * math.cos(self.heading) + dy * math.sin(self.heading)
                ),
                lateral_distance_m=(
                    -dx * math.sin(self.heading) + dy * math.cos(self.heading)
                ),
                final_depth_m=self.latest.depth,
                final_speed_m_s=self.latest.speed,
                final_position_error_m=math.hypot(
                    self.latest.x - self.goal[0], self.latest.y - self.goal[1]
                ),
            )
        return result

"""Passively score the configured Gazebo move/hold scenario and save JSON."""

from collections import deque
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robosub_control.autonomy.sensor_freshness import (
    SensorFreshness,
    stamp_to_sec,
)
from robosub_control.autonomy.state_estimator import q_to_rpy
from robosub_control.utils.tuning import fixed_parameter
from std_msgs.msg import String

from robosub_sim.motion_test_scoring import MoveHoldScorer, TruthSample


class MotionTest(Node):
    def __init__(self, **kwargs):
        super().__init__("motion_test", **kwargs)
        defaults = {
            "distance": 5.0,
            "depth": 1.0,
            "heading_deg": 0.0,
            "hold_duration": 10.0,
            "position_tolerance": 0.2,
            "depth_tolerance": 0.1,
            "heading_tolerance_deg": 5.0,
            "speed_tolerance": 0.05,
            "sample_timeout": 0.25,
            "test_timeout": 180.0,
            "completion_grace": 1.0,
            "clock_timeout": 30.0,
            "output_directory": "~/.ros/robosub_tests",
            "max_tilt_deg": 30.0,
            "hold_tilt_deg": 10.0,
        }
        for name, value in defaults.items():
            fixed_parameter(self, name, value)

        def g(name):
            return self.get_parameter(name).value

        self.scorer = MoveHoldScorer(
            distance=g("distance"),
            depth=g("depth"),
            heading=math.radians(g("heading_deg")),
            hold_duration=g("hold_duration"),
            position_tolerance=g("position_tolerance"),
            depth_tolerance=g("depth_tolerance"),
            heading_tolerance=math.radians(g("heading_tolerance_deg")),
            speed_tolerance=g("speed_tolerance"),
            sample_timeout=g("sample_timeout"),
            tilt_tolerance=math.radians(g("hold_tilt_deg")),
        )
        self.max_tilt_deg = float(g("max_tilt_deg"))
        if not 0 < g("hold_tilt_deg") < self.max_tilt_deg < 60:
            raise ValueError("Require 0 < hold_tilt_deg < max_tilt_deg < 60")
        self.timeout = float(g("test_timeout"))
        self.grace = float(g("completion_grace"))
        self.clock_timeout = float(g("clock_timeout"))
        if not all(
            math.isfinite(v) and v > 0
            for v in (
                self.timeout,
                self.grace,
                self.clock_timeout,
            )
        ):
            raise ValueError("Test deadlines must be finite and positive")
        self.directory = Path(g("output_directory")).expanduser()
        self.run_stem = datetime.now(timezone.utc).strftime(
            "move5_hold-%Y%m%dT%H%M%S.%fZ"
        )
        self.phase = "startup"
        self.last_progress_log = -math.inf
        self.samples = deque(maxlen=100)
        self.freshness = SensorFreshness(self.scorer.sample_timeout)
        self.started = None
        self.last_tick = None
        self.move_stamp = None
        self.succeeded_at = None
        self.finished = False
        self.last_clock_value = self.now()
        self.last_progress_wall = time.monotonic()
        self.create_subscription(Odometry, "/sub/odom", self.on_truth, 20)
        status_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String, "/sub/mission/status", self.on_status, status_qos
        )
        self.create_subscription(
            String, "/sub/control/fault", self.on_fault, 10
        )
        self.create_timer(0.1, self.periodic)

    def on_fault(self, msg):
        try:
            event = json.loads(msg.data)
            reason = event.get("reason", "Controller fault")
        except (ValueError, AttributeError):
            reason = msg.data
        self.finish(False, str(reason))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def check_wall_clock(self):
        """Keep a missing or frozen /clock from suspending test deadlines."""
        if self.finished:
            return
        value = self.now()
        wall = time.monotonic()
        if value < self.last_clock_value:
            self.finish(False, "Simulation clock rewound during test")
        elif value != self.last_clock_value:
            self.last_progress_wall = wall
        elif wall - self.last_progress_wall >= self.clock_timeout:
            self.finish(False, "Simulation clock did not start or stopped")
        self.last_clock_value = value

    def on_truth(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        v = msg.twist.twist.linear
        stamp = stamp_to_sec(msg.header.stamp)
        values = (p.x, p.y, p.z, q.w, q.x, q.y, q.z, v.x, v.y, v.z)
        if abs(math.hypot(q.w, q.x, q.y, q.z) - 1.0) > 0.01:
            return
        if not self.freshness.accept(stamp, self.now(), values):
            return
        roll, pitch, yaw = q_to_rpy((q.w, q.x, q.y, q.z))
        sample = TruthSample(
            stamp, p.x, p.y, -p.z, yaw, math.hypot(v.x, v.y, v.z), roll, pitch
        )
        self.samples.append(sample)
        self.capture_origin()
        self.scorer.observe(sample)
        if (
            max(abs(math.degrees(roll)), abs(math.degrees(pitch)))
            > self.max_tilt_deg
        ):
            self.finish(
                False,
                f"Ground-truth attitude limit at t={stamp:.3f}s: "
                f"roll={math.degrees(roll):+.1f} deg, "
                f"pitch={math.degrees(pitch):+.1f} deg "
                f"(limit {self.max_tilt_deg:.1f} deg)",
            )

    def capture_origin(self):
        if self.move_stamp is None or self.scorer.origin is not None:
            return
        candidates = [
            sample
            for sample in self.samples
            if abs(sample.stamp - self.move_stamp) < self.scorer.sample_timeout
        ]
        if candidates:
            self.scorer.start_move(
                min(
                    candidates,
                    key=lambda sample: abs(sample.stamp - self.move_stamp),
                )
            )

    def on_status(self, msg):
        if self.finished:
            return
        try:
            event = json.loads(msg.data)
            status = event["status"]
            self.phase = f"{status}:{event.get('step', '')}"
            if status in ("failed", "cancelled"):
                self.finish(False, event.get("reason", status))
            elif status == "step_started" and event["step"] == "move":
                if self.move_stamp is not None:
                    raise ValueError("This scorer expects exactly one move")
                self.move_stamp = float(event["stamp"])
                self.capture_origin()
            elif status == "step_started" and event["step"] == "hold":
                self.capture_origin()
                self.scorer.start_hold()
            elif status == "succeeded":
                self.succeeded_at = self.now()
        except (KeyError, TypeError, ValueError) as error:
            self.finish(False, f"Invalid mission event: {error}")

    def periodic(self):
        if self.finished:
            return
        now = self.now()
        if self.started is None:
            self.started = now
        if self.last_tick is not None and now < self.last_tick:
            self.finish(False, "Simulation clock rewound during test")
            return
        self.last_tick = now
        self.scorer.expire(now)
        if now - self.last_progress_log >= 1 and self.scorer.latest is not None:
            self.last_progress_log = now
            sample = self.scorer.latest
            publishers = {
                topic: self.count_publishers(topic)
                for topic in (
                    "/clock",
                    "/sub/state",
                    "/sub/cmd_vel",
                    "/sim/thrust/t0",
                )
            }
            if any(count > 1 for count in publishers.values()):
                self.get_logger().warn(
                    f"Multiple publishers detected: {publishers}",
                    throttle_duration_sec=5.0,
                )
            self.get_logger().info(
                f"sim={now:.2f}s phase={self.phase} "
                f"TRUTH xyz=({sample.x:+.2f},{sample.y:+.2f},"
                f"{-sample.depth:+.2f})m "
                f"rpy=({math.degrees(sample.roll):+.1f},"
                f"{math.degrees(sample.pitch):+.1f},"
                f"{math.degrees(sample.yaw):+.1f})deg "
                f"speed={sample.speed:.2f}m/s"
            )
        if now - self.started >= self.timeout:
            self.finish(False, "Scenario exceeded test deadline")
        elif self.succeeded_at is not None:
            if self.scorer.held_for >= self.scorer.hold_duration:
                self.finish(
                    True, "Requested move and ground-truth hold verified"
                )
            elif now - self.succeeded_at >= self.grace:
                self.finish(
                    False, "Mission ended without the required true hold"
                )

    def finish(self, target_met, reason):
        if self.finished:
            return
        self.finished = True
        result = self.scorer.result(target_met, reason)
        result["publisher_counts"] = {
            topic: self.count_publishers(topic)
            for topic in (
                "/clock",
                "/imu/data_raw",
                "/depth",
                "/dvl/velocity",
                "/sub/state",
                "/sub/cmd_vel",
                "/sim/thrust/t0",
            )
        }
        self.directory.mkdir(parents=True, exist_ok=True)
        filename = self.run_stem + ".json"
        path = self.directory / filename
        path.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
        outcome = "TARGET MET" if target_met else "TARGET NOT MET"
        self.get_logger().info(f"{outcome}: {reason}; score: {path}")


def main():
    rclpy.init()
    node = MotionTest()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
            node.check_wall_clock()
    except KeyboardInterrupt:
        node.finish(False, "Test interrupted")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0

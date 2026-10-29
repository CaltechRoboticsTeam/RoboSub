#!/usr/bin/env python3
"""Measure the real controller callback with synthetic state and no hardware."""

import argparse
import json
import math
import os
from pathlib import Path
import statistics
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rate", type=float, default=1000.0)
    parser.add_argument("--output-rate", type=float, default=100.0)
    parser.add_argument("--seconds", type=float, default=5.0)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 0 < args.seconds <= 30:
        parser.error("seconds must be positive and at most 30")

    # Isolate the probe and remap its output away from actuator command topics.
    os.environ["ROS_DOMAIN_ID"] = "198"
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(root / "robosub_ws/src/robosub_sim"))
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from robosub_sim.depth_heading_controller import DepthHeadingController

    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            f"rate:={args.rate}",
            "-p",
            f"output_rate:={args.output_rate}",
            "-r",
            "/sub/cmd_vel:=/benchmark/control_output",
        ]
    )
    controller = DepthHeadingController()
    controller.state = Odometry()
    controller.state.pose.pose.orientation.w = 1.0
    controller.state.pose.pose.position.z = -1.0
    controller.depth_sp = 1.2
    controller.heading_sp = 0.1
    controller.roll_sp = 0.1
    controller.pos_sp = (0.1, 0.1)
    starts = []
    durations = []
    outputs = []
    original = controller.control_timer.callback

    def measured_callback():
        controller.state_rx = controller.now()
        started = time.perf_counter()
        original()
        durations.append(time.perf_counter() - started)
        starts.append(started)

    controller.control_timer.callback = measured_callback
    observer = Node("control_benchmark")
    observer.create_subscription(
        Twist, "/benchmark/control_output", lambda _msg: outputs.append(1), 10
    )
    executor = SingleThreadedExecutor()
    executor.add_node(controller)
    executor.add_node(observer)
    started = time.perf_counter()
    deadline = started + args.seconds
    try:
        while time.perf_counter() < deadline:
            executor.spin_once(timeout_sec=0.05)
        elapsed = time.perf_counter() - started
        intervals = [b - a for a, b in zip(starts, starts[1:], strict=False)]
        if not intervals:
            raise RuntimeError("Not enough controller callbacks were observed")
        intervals.sort()
        print(
            json.dumps(
                {
                    "scope": "synthetic state; no camera or physical drivers",
                    "requested_loop_hz": args.rate,
                    "output_cap_hz": controller.output_rate,
                    "observed_loop_hz": 1.0 / statistics.mean(intervals),
                    "observed_output_hz": len(outputs) / elapsed,
                    "mean_callback_us": statistics.mean(durations) * 1e6,
                    "p99_interval_ms": intervals[
                        int(0.99 * (len(intervals) - 1))
                    ]
                    * 1000,
                    "max_interval_ms": max(intervals) * 1000,
                },
                indent=2,
            )
        )
    finally:
        executor.shutdown()
        observer.destroy_node()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

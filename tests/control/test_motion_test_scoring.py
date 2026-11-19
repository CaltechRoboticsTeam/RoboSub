"""Save an advisory score when the simulated target is not met."""

import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "robosub_ws/src/robosub_sim"))
sys.path.insert(0, str(ROOT / "robosub_ws/src/robosub_control"))

try:
    import rclpy
except ImportError:
    rclpy = None


@unittest.skipIf(rclpy is None, "ROS Humble is not installed")
class MotionTestResultTests(unittest.TestCase):
    def test_unmet_target_saves_score_and_exits_normally(self):
        from nav_msgs.msg import Odometry
        from rclpy.parameter import Parameter
        from robosub_sim.motion_test import MotionTest
        from robosub_sim import motion_test

        os.environ["ROS_DOMAIN_ID"] = "198"
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
        rclpy.init()
        self.addCleanup(rclpy.shutdown)
        with tempfile.TemporaryDirectory() as directory:
            node = MotionTest(
                parameter_overrides=[
                    Parameter("output_directory", value=directory)
                ]
            )
            try:
                node.now = lambda: 10.0
                truth = Odometry()
                truth.header.stamp.sec = 10
                truth.pose.pose.position.x = 1.5
                truth.pose.pose.position.z = -1.0
                truth.pose.pose.orientation.w = 1.0
                node.on_truth(truth)
                with patch.object(node, "count_publishers", return_value=2):
                    node.finish(False, "dive timed out")
                result = json.loads(
                    next(Path(directory).glob("*.json")).read_text()
                )
                self.assertFalse(result["within_tolerance"])
                self.assertEqual(result["last_truth"]["x_m"], 1.5)
                self.assertEqual(result["publisher_counts"]["/sub/cmd_vel"], 2)
                with (
                    patch.object(motion_test, "rclpy"),
                    patch.object(motion_test, "MotionTest", return_value=node),
                    patch.object(node, "destroy_node"),
                ):
                    self.assertEqual(motion_test.main(), 0)
            finally:
                node.destroy_node()


if __name__ == "__main__":
    unittest.main()

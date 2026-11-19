"""Check live PID changes and rejected parameter batches."""

import os
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "robosub_ws/src/robosub_control"))

try:
    import rclpy
except ImportError:
    rclpy = None


@unittest.skipIf(rclpy is None, "ROS Humble is not installed")
class TuningTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ["ROS_DOMAIN_ID"] = "198"
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        from robosub_control.autonomy.depth_heading_controller import (
            DepthHeadingController,
        )

        self.node = DepthHeadingController()
        self.addCleanup(self.node.destroy_node)

    def test_parameter_service_updates_running_pid(self):
        from rcl_interfaces.srv import SetParameters
        from rclpy.parameter import Parameter

        client = self.node.create_client(
            SetParameters, "/depth_heading_controller/set_parameters"
        )
        self.assertTrue(client.wait_for_service(timeout_sec=3))
        future = client.call_async(
            SetParameters.Request(
                parameters=[
                    Parameter(name, value=value).to_parameter_msg()
                    for name, value in (
                        ("depth_kp", 2.0),
                        ("depth_ki", 0.0),
                        ("depth_kd", 0.0),
                    )
                ]
            )
        )
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=3)
        self.assertTrue(future.done())
        self.assertTrue(all(r.successful for r in future.result().results))
        self.assertEqual(self.node.depth_pid.kp, 2.0)
        self.assertAlmostEqual(self.node.depth_pid.step(0.1, 0.0, 0.01), 0.2)

    def test_invalid_atomic_batch_changes_nothing(self):
        from rclpy.parameter import Parameter

        previous = self.node.depth_pid.kp
        result = self.node.set_parameters_atomically(
            [
                Parameter("depth_kp", value=2.0),
                Parameter("depth_ki", value=-1.0),
            ]
        )
        self.assertFalse(result.successful)
        self.assertEqual(self.node.get_parameter("depth_kp").value, previous)
        self.assertEqual(self.node.depth_pid.kp, previous)


if __name__ == "__main__":
    unittest.main()

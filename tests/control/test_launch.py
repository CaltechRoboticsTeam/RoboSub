"""Check master-launch routing without starting processes or hardware."""

import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "robosub_ws/src/robosub_control"))
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "tools/diagnostics"))
sys.path.insert(0, str(ROOT / "robosub_ws/src/robosub_sim"))
sys.path.insert(0, str(ROOT / "robosub_ws/src/robosub_bringup"))

try:
    from launch import LaunchContext
except ImportError:
    LaunchContext = None


@unittest.skipIf(LaunchContext is None, "ROS launch is not installed")
class LaunchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = ROOT / "robosub_ws/src/robosub_bringup/launch/robot.launch.py"
        spec = importlib.util.spec_from_file_location("robot_launch", source)
        cls.launch = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.launch)

    def setUp(self):
        self.check_driver_station_port = self.launch.check_driver_station_port
        guard = patch.object(self.launch, "check_driver_station_port")
        self.station_guard = guard.start()
        self.addCleanup(guard.stop)

    def context(self, **changes):
        context = LaunchContext()
        context.launch_configurations.update(
            {
                "mode": "sim",
                "world": "pool",
                "vehicle": "current_m5",
                "gui": "false",
                "vision": "auto",
                "vision_config": "",
                "mission": "false",
                "mission_config": "",
                "driver_station": "false",
                "plots": "false",
                "teleop": "off",
                "teleop_scale": "1.0",
                "joy_device": "0",
                "actuators": "preview",
                "teensy_device": "/test/teensy",
                "driver_station_port": "8765",
                "record": "false",
                "log_directory": "/unused/robosub_runs",
                "foxglove": "false",
                "foxglove_address": "127.0.0.1",
                "foxglove_port": "8766",
                "control_rate": "20.0",
                "output_rate": "100.0",
                "max_power": "1.0",
                **changes,
            }
        )
        return context

    def test_recording_captures_all_topics_and_uses_the_matching_clock(self):
        from robosub_bringup import session

        for simulated in (True, False):
            with (
                self.subTest(simulated=simulated),
                tempfile.TemporaryDirectory() as directory,
                patch.object(session, "require_package") as require,
                patch.object(
                    session, "ExecuteProcess", wraps=session.ExecuteProcess
                ) as process,
                patch.object(
                    session, "OnProcessStart", wraps=session.OnProcessStart
                ) as started,
            ):
                sentinel = self.launch.LogInfo(msg="robot starts here")
                context = self.context(
                    mode="sim" if simulated else "real",
                    record="true",
                    log_directory=directory,
                )
                session.session_actions(
                    context, simulated=simulated, robot_actions=[sentinel]
                )
                command = process.call_args.kwargs["cmd"]
                self.assertIn("--all", command)
                self.assertIn("--include-hidden-topics", command)
                self.assertIn("mcap", command)
                self.assertIn("zstd_fast", command)
                self.assertEqual("--use-sim-time" in command, simulated)
                self.assertEqual(
                    started.call_args.kwargs["on_start"], [sentinel]
                )
                bag = Path(command[command.index("--output") + 1])
                self.assertFalse(bag.exists())
                metadata = json.loads((bag.parent / "run.json").read_text())
                self.assertEqual(metadata["bag_directory"], str(bag))
                self.assertEqual(
                    metadata["clock"], "simulation" if simulated else "system"
                )
                self.assertEqual(require.call_count, 2)

    def test_physical_driver_requires_real_m5_teleop_or_mission(self):
        for changes in (
            {},
            {"mode": "real"},
            {"mode": "real", "teleop": "browser", "vehicle": "future_t200"},
        ):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.launch.start(
                    self.context(actuators="teensy_usb", **changes)
                )
        with patch.object(self.launch, "include"):
            actions = self.launch.start(
                self.context(
                    mode="real",
                    vision="false",
                    teleop="browser",
                    actuators="teensy_usb",
                )
            )
        self.assertEqual(len(actions), 6)

    def test_manual_launch_uses_estimator_and_one_command_owner(self):
        source = (
            ROOT / "robosub_ws/src/robosub_control/launch/vehicle.launch.py"
        )
        spec = importlib.util.spec_from_file_location("vehicle_launch", source)
        vehicle = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(vehicle)
        context = self.context()
        context.launch_configurations.update(
            {
                "manual_control": "true",
                "preview_commands": "false",
                "simulate_actuators": "false",
                "use_sim_time": "false",
            }
        )
        with patch.object(vehicle, "Node") as node:
            vehicle.start(context)
        self.assertEqual(
            [call.kwargs["executable"] for call in node.call_args_list],
            ["thruster_mixer", "hardware_sensors", "state_estimator"],
        )
        from robosub_control.config import current_m5

        self.assertEqual(
            node.call_args_list[-1].kwargs["parameters"][0]["dvl_offset"],
            list(current_m5.CONTROL_BOARD_POSITION_BODY_M),
        )
        context.launch_configurations["manual_control"] = "false"
        context.launch_configurations["preview_commands"] = "true"
        with patch.object(vehicle, "Node") as node:
            vehicle.start(context)
        self.assertEqual(
            [call.kwargs["executable"] for call in node.call_args_list],
            [
                "thruster_mixer",
                "actuator_commands",
                "hardware_sensors",
                "state_estimator",
                "depth_heading_controller",
            ],
        )
        context.launch_configurations["simulate_actuators"] = "true"
        with patch.object(vehicle, "Node") as node:
            vehicle.start(context)
        self.assertNotIn(
            "hardware_sensors",
            [call.kwargs["executable"] for call in node.call_args_list],
        )
        estimator = next(
            call
            for call in node.call_args_list
            if call.kwargs["executable"] == "state_estimator"
        )
        self.assertNotIn("dvl_offset", estimator.kwargs["parameters"][0])


if __name__ == "__main__":
    unittest.main()

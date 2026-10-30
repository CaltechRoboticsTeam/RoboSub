"""Check master-launch routing without starting processes or hardware."""

import importlib.util
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "robosub_ws/src/robosub_control"))
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "tools/diagnostics"))
sys.path.insert(0, str(ROOT / "robosub_ws/src/robosub_sim"))

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

    def context(self, **changes):
        context = LaunchContext()
        context.launch_configurations.update(
            {
                "mode": "sim",
                "vehicle": "current_m5",
                "gui": "false",
                "vision": "auto",
                "vision_config": "",
                "mission": "false",
                "tuning": "false",
                "teleop": "off",
                "teleop_scale": "1.0",
                "joy_device": "0",
                "actuators": "preview",
                "teensy_device": "/test/teensy",
                "tuning_file": "",
                "tuning_port": "8765",
                "control_rate": "20.0",
                "output_rate": "100.0",
                **changes,
            }
        )
        return context

    def test_sim_selects_pool_without_loading_zed(self):
        with patch.object(self.launch, "include") as include:
            self.launch.start(self.context())
        include.assert_called_once()
        package, filename, arguments = include.call_args.args
        self.assertEqual((package, filename), ("robosub_sim", "pool.launch.py"))
        self.assertEqual(arguments["control_rate"], "20.0")
        self.assertEqual(arguments["vehicle"], "current_m5")

    def test_real_selects_shared_vehicle_without_simulator(self):
        with patch.object(self.launch, "include") as include:
            self.launch.start(self.context(mode="real", vision="false"))
        include.assert_called_once()
        package, filename, arguments = include.call_args.args
        self.assertEqual(
            (package, filename), ("robosub_control", "vehicle.launch.py")
        )
        self.assertEqual(arguments["use_sim_time"], "false")

    def test_auto_real_vision_uses_vision_launcher(self):
        with (
            patch.object(self.launch, "include") as include,
            patch.object(
                self.launch, "get_package_share_directory", return_value="/tmp"
            ),
        ):
            self.launch.start(self.context(mode="real"))
        self.assertEqual(include.call_count, 2)
        self.assertEqual(
            include.call_args.args[:2],
            (
                "robosub_vision",
                "vision.launch.py",
            ),
        )

    def test_rejects_unimplemented_real_mission_and_sim_stereo(self):
        for settings in (
            {"mode": "real", "mission": "true"},
            {"vision": "true"},
            {"control_rate": "nan"},
            {"output_rate": "0"},
            {"output_rate": "101"},
            {"vehicle": "future_t200", "output_rate": "401"},
            {"vehicle": "unknown"},
        ):
            with self.subTest(settings=settings), self.assertRaises(ValueError):
                self.launch.start(self.context(**settings))

    def test_future_t200_selection_and_its_output_limit(self):
        with patch.object(self.launch, "include") as include:
            self.launch.start(
                self.context(vehicle="future_t200", output_rate="400")
            )
        self.assertEqual(include.call_args.args[2]["vehicle"], "future_t200")

    def test_browser_teleop_owns_commands_in_sim_and_real(self):
        for mode in ("sim", "real"):
            with patch.object(self.launch, "include") as include:
                actions = self.launch.start(
                    self.context(mode=mode, vision="false", teleop="browser")
                )
            self.assertEqual(
                include.call_args.args[2]["manual_control"], "true"
            )
            self.assertEqual(len(actions), 3 if mode == "sim" else 4)

    def test_local_teleop_loads_sdl_driver(self):
        with (
            patch.object(self.launch, "include"),
            patch.object(
                self.launch, "get_package_share_directory", return_value="/tmp"
            ),
        ):
            actions = self.launch.start(self.context(teleop="local"))
        self.assertEqual(len(actions), 4)

    def test_physical_driver_requires_real_m5_teleop(self):
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

    def test_rejects_conflicting_or_invalid_teleop_settings(self):
        for changes in (
            {"teleop": "browser", "mission": "true"},
            {"teleop": "unknown"},
            {"teleop_scale": "nan"},
            {"teleop_scale": "0"},
            {"teleop_scale": "1.1"},
        ):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.launch.start(self.context(**changes))

    def test_live_manual_launch_omits_unused_control_nodes(self):
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
                "tuning_file": "",
            }
        )
        with patch.object(vehicle, "Node") as node:
            vehicle.start(context)
        self.assertEqual(
            [call.kwargs["executable"] for call in node.call_args_list],
            ["thruster_mixer"],
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
                "state_estimator",
                "depth_heading_controller",
            ],
        )

    def test_hard_lock_overrides_tuning_launch_option(self):
        with (
            patch.object(self.launch.constants, "LOCK_TUNING_DEFAULTS", True),
            self.assertRaisesRegex(ValueError, "locked"),
        ):
            self.launch.start(self.context(tuning="true"))


if __name__ == "__main__":
    unittest.main()

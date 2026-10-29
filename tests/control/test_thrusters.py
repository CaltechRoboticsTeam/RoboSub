"""T200 conversions, allocation, and generated simulation geometry."""

import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "robosub_ws/src/robosub_control"))
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "tools/diagnostics"))
sys.path.insert(0, str(ROOT / "robosub_ws/src/robosub_sim"))

from robosub_control.config import current_m5  # noqa: E402
from robosub_control.config import future_t200 as constants  # noqa: E402
from robosub_sim.simulation_world import configured_world  # noqa: E402
from robosub_control.thrusters.t200 import T200Model  # noqa: E402
from robosub_control.thrusters.thrust_allocation import (  # noqa: E402
    AXES,
    CONTROLLED_ROWS,
    ThrustAllocator,
)
from robosub_control.config.vehicle_profiles import get_profile  # noqa: E402


class T200Tests(unittest.TestCase):
    def test_manufacturer_endpoint_units(self):
        model = T200Model(16.0)
        low, high = model.force_limits()
        self.assertAlmostEqual(low, -4.069476226666667 * 9.80665, places=7)
        self.assertAlmostEqual(high, 5.245035493333333 * 9.80665, places=7)

    def test_neutral_deadband_and_small_requests(self):
        model = T200Model(16.0)
        for pulse in (1475, 1500, 1525):
            self.assertEqual(model.force_from_pwm(pulse), 0.0)
        for force in (0.0, 1e-5, -1e-5):
            self.assertEqual(model.force_to_pwm(force), 1500)

    def test_inverse_and_reversal_use_asymmetric_branches(self):
        model = T200Model(16.0)
        forward = model.force_to_pwm(20.0)
        reverse_thrust = model.force_to_pwm(-20.0)
        reversed_motor = model.force_to_pwm(20.0, True)
        self.assertGreater(forward, 1500)
        self.assertLess(reversed_motor, 1500)
        self.assertNotEqual(forward - 1500, 1500 - reverse_thrust)
        self.assertEqual(reversed_motor, 3000 - forward)
        self.assertEqual(model.force_limits(True), model.force_limits())
        self.assertEqual(
            model.current_from_pwm(reversed_motor, True),
            model.current_from_pwm(forward),
        )
        self.assertAlmostEqual(model.force_from_pwm(forward), 20.0, delta=0.25)
        self.assertAlmostEqual(
            model.force_from_pwm(reversed_motor, True), 20.0, delta=0.25
        )

    def test_voltage_interpolation_and_input_rejection(self):
        low, high, middle = T200Model(14.0), T200Model(16.0), T200Model(15.0)
        expected = (low.force_from_pwm(1800) + high.force_from_pwm(1800)) / 2
        self.assertAlmostEqual(middle.force_from_pwm(1800), expected, places=7)
        for voltage in (None, 9.9, 20.1, float("nan"), True):
            with self.subTest(voltage=voltage), self.assertRaises(ValueError):
                T200Model(voltage)
        for force in (float("nan"), float("inf"), True):
            with self.assertRaises(ValueError):
                middle.force_to_pwm(force)

    def test_current_limit_bounds_and_monotone_commands(self):
        for voltage in (10.0, 12.0, 16.8, 20.0):
            model = T200Model(voltage)
            pulses = [
                model.force_to_pwm(float(value))
                for value in np.linspace(-100, 100, 301)
            ]
            self.assertEqual(pulses, sorted(pulses))
            for pulse in pulses:
                self.assertGreaterEqual(pulse, 1100)
                self.assertLessEqual(pulse, 1900)
                self.assertLessEqual(model.current_from_pwm(pulse), 30.0 + 1e-8)


class GeometryTests(unittest.TestCase):
    def test_shared_zero_based_order_and_simulator_connections(self):
        expected = ("FR", "FL", "BR", "BL", "RD", "LD")
        bridge = (
            ROOT / "robosub_ws/src/robosub_sim/config/bridge.yaml"
        ).read_text()
        source = ROOT / "robosub_ws/src/robosub_sim/worlds/pool.sdf"
        for profile in (current_m5, constants):
            self.assertEqual(profile.THRUSTER_NAMES, expected)
            world = ET.fromstring(
                configured_world(
                    source.read_text(), source.parent, vehicle=profile.NAME
                )
            )
            model = world.find("world/model[@name='sub']")
            for index, name in enumerate(expected):
                self.assertEqual(profile.THRUSTER_TOPIC_IDS[name], f"t{index}")
                self.assertIsNotNone(model.find(f"link[@name='thr_{index}']"))
                self.assertIsNotNone(
                    model.find(f"joint[@name='thr_{index}_joint']")
                )
                self.assertIn(f'ros_topic_name: "/sim/thrust/t{index}"', bridge)
                self.assertIn(f'/joint/thr_{index}_joint/cmd_thrust"', bridge)

    def test_measured_frame_conversion(self):
        np.testing.assert_allclose(
            constants.THRUSTER_POSITIONS_BODY_M["FL"], (0.352, 0.229, 0.033)
        )
        np.testing.assert_allclose(
            constants.DVL_POSITION_BODY_M, (-0.153, 0.007, -0.016)
        )
        np.testing.assert_allclose(
            constants.CONTROL_BOARD_POSITION_BODY_M, (0.0, 0.006, 0.049)
        )
        np.testing.assert_allclose(
            constants.CAMERA_POSITION_BODY_M, (-0.157118304, 0.006, -0.0526)
        )
        self.assertEqual(constants.THRUSTER_YAW_CW_DEG["BR"], 225.0)
        self.assertFalse(constants.CAMERA_ORIENTATION_CONFIRMED)

    def test_each_axis_matches_requested_wrench(self):
        allocator = ThrustAllocator(vehicle="future_t200")
        for index, axis in enumerate(AXES):
            for sign in (-1, 1):
                efforts = np.zeros(5)
                efforts[index] = 0.5 * sign
                result = allocator.allocate(efforts)
                with self.subTest(axis=axis, sign=sign):
                    np.testing.assert_allclose(
                        np.array(result.achieved)[list(CONTROLLED_ROWS)],
                        np.array(result.requested)[list(CONTROLLED_ROWS)],
                        atol=1e-10,
                    )

    def test_saturation_preserves_controlled_wrench_direction(self):
        allocator = ThrustAllocator(vehicle="future_t200")
        result = allocator.allocate([1, 1, 1, 1, 1])
        self.assertLess(result.scale, 1)
        forces = np.array(result.targets)
        self.assertTrue(np.all(forces >= allocator.limits[:, 0] - 1e-9))
        self.assertTrue(np.all(forces <= allocator.limits[:, 1] + 1e-9))
        np.testing.assert_allclose(
            np.array(result.achieved)[list(CONTROLLED_ROWS)],
            np.array(result.requested)[list(CONTROLLED_ROWS)] * result.scale,
            atol=1e-10,
        )

    def test_pitch_coupling_is_reported_in_full_wrench(self):
        result = ThrustAllocator(vehicle="future_t200").allocate(
            [0.3, 0.1, 0.2, 0.1, 0.4]
        )
        fx, _, fz, _, pitch, _ = result.achieved
        self.assertAlmostEqual(pitch, 0.033 * fx - 0.012 * fz, places=10)

    def test_channel_mapping_and_reversal(self):
        channels = dict(constants.THRUSTER_CHANNELS)
        channels["FR"], channels["LD"] = channels["LD"], channels["FR"]
        with (
            patch.dict(constants.THRUSTER_CHANNELS, channels),
            patch.dict(constants.THRUSTER_REVERSED, {"FR": True}),
        ):
            allocator = ThrustAllocator(vehicle="future_t200")
            pulses = allocator.pwm_by_channel([20, 0, 0, 0, 0, 0])
            self.assertLess(pulses[5], 1500)
            self.assertEqual(pulses[0], 1500)
            expected = allocator.model.force_to_pwm(20.0, True)
            self.assertEqual(pulses[5], expected)
        with patch.dict(constants.THRUSTER_CHANNELS, {"FR": 1}):
            with self.assertRaises(ValueError):
                ThrustAllocator(vehicle="future_t200")

    def test_invalid_force_frames_and_tiny_efforts(self):
        allocator = ThrustAllocator(vehicle="future_t200")
        for values in ([1, 2], [float("nan")] * 5):
            with self.assertRaises(ValueError):
                allocator.allocate(values)
        result = allocator.allocate([1e-20] * 5)
        self.assertEqual(result.scale, 1.0)
        with self.assertRaises(ValueError):
            allocator.pwm_by_channel([0.0] * 5)

    def test_generated_world_matches_allocator_axes(self):
        source = ROOT / "robosub_ws/src/robosub_sim/worlds/pool.sdf"
        meshes = ROOT / "robosub_ws/src/robosub_sim/meshes"
        original = source.read_text()
        world = configured_world(original, meshes, vehicle="future_t200")
        root = ET.fromstring(world)
        allocator = ThrustAllocator(vehicle="future_t200")
        model = root.find("world/model[@name='sub']")
        for index, name in enumerate(constants.THRUSTER_NAMES):
            number = constants.THRUSTER_TOPIC_IDS[name][1:]
            pose = model.findtext(f"link[@name='thr_{number}']/pose")
            values = [float(value) for value in pose.split()]
            np.testing.assert_allclose(
                values[:3], constants.THRUSTER_POSITIONS_BODY_M[name]
            )
            roll, pitch, yaw = values[3:]
            direction = (
                math.cos(yaw) * math.sin(pitch) * math.cos(roll)
                + math.sin(yaw) * math.sin(roll),
                math.sin(yaw) * math.sin(pitch) * math.cos(roll)
                - math.cos(yaw) * math.sin(roll),
                math.cos(pitch) * math.cos(roll),
            )
            np.testing.assert_allclose(
                direction, allocator.wrench_matrix[:3, index], atol=1e-10
            )
        camera = model.findtext(
            "link[@name='base_link']/sensor[@name='front_camera']/pose"
        )
        np.testing.assert_allclose(
            [float(value) for value in camera.split()[:3]],
            constants.CAMERA_POSITION_BODY_M,
        )
        for plugin in model.findall("plugin"):
            if plugin.get("name") == "gz::sim::systems::Thruster":
                self.assertEqual(
                    float(plugin.findtext("propeller_diameter")),
                    constants.T200_PROPELLER_DIAMETER_M,
                )
        self.assertEqual(source.read_text(), original)


class M5Tests(unittest.TestCase):
    def test_default_and_unconfigured_hardware(self):
        self.assertIs(get_profile(), current_m5)
        self.assertTrue(current_m5.GEOMETRY_CONFIRMED)
        self.assertFalse(constants.GEOMETRY_CONFIRMED)
        self.assertIsNot(
            current_m5.THRUSTER_POSITIONS_ONSHAPE_M,
            constants.THRUSTER_POSITIONS_ONSHAPE_M,
        )
        self.assertTrue(
            current_m5.M5_MOTOR_IDS
            == {"FR": 0, "FL": 1, "BR": 2, "BL": 3, "RD": 4, "LD": 6}
        )
        self.assertIsNone(current_m5.M5_POWER_TO_FORCE_CALIBRATION)
        self.assertIsNone(current_m5.M5_SUPPLY_VOLTAGE_V)
        self.assertEqual(
            current_m5.CAMERA_POSITION_BODY_M, constants.CAMERA_POSITION_BODY_M
        )
        with self.assertRaises(ValueError):
            get_profile("unconfigured")

    def test_m5_power_limits_and_no_pwm_conversion(self):
        allocator = ThrustAllocator()
        self.assertIsNone(allocator.model)
        for efforts in (
            [0] * 5,
            [1] * 5,
            [-1] * 5,
            [0.5, -0.2, 0.4, 0.7, -0.3],
        ):
            result = allocator.allocate(efforts)
            commands = allocator.commands(result.targets)
            self.assertTrue(all(-1 <= value <= 1 for value in commands))
            np.testing.assert_allclose(commands, result.targets)
        with self.assertRaisesRegex(ValueError, "PWM"):
            allocator.pwm_by_channel([0] * 6)
        for invalid in ([0] * 5, [float("nan")] * 6, [1.1] * 6):
            with self.assertRaises(ValueError):
                allocator.commands(invalid)
        with patch.dict(current_m5.THRUSTER_REVERSED, {"FR": True}):
            self.assertEqual(
                ThrustAllocator().commands([0.3, 0, 0, 0, 0, 0])[0], -0.3
            )

    def test_current_world_uses_m5_diameter_and_measured_mounts(self):
        source = ROOT / "robosub_ws/src/robosub_sim/worlds/pool.sdf"
        world = ET.fromstring(
            configured_world(source.read_text(), source.parent)
        )
        model = world.find("world/model[@name='sub']")
        for plugin in model.findall("plugin"):
            if plugin.get("name") == "gz::sim::systems::Thruster":
                self.assertEqual(
                    float(plugin.findtext("propeller_diameter")), 0.09
                )


if __name__ == "__main__":
    unittest.main()

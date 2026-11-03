"""Check that M5 allocation produces the requested body forces."""

from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "robosub_ws/src/robosub_control"))
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "tools/diagnostics"))
sys.path.insert(0, str(ROOT / "robosub_ws/src/robosub_sim"))

from robosub_control.thrusters.thrust_allocation import (  # noqa: E402
    AXES,
    CONTROLLED_ROWS,
    ThrustAllocator,
)
from robosub_control.thrusters.m5 import M5Model  # noqa: E402


class AllocationTests(unittest.TestCase):
    def test_team_force_fit_has_correct_units_voltage_and_reverse(self):
        for voltage, pounds in (
            (20.0, (-3.5, 6.5)),
            (24.0, (-4.8, 8.8)),
            (25.5, (-5.3, 9.7)),
        ):
            model = M5Model(voltage)
            np.testing.assert_allclose(
                np.array(model.force_limits()) / 4.4482216152605,
                pounds,
                atol=0.06,
            )
            for reversed_motor in (False, True):
                for command in (-1.0, -0.5, 0.0, 0.5, 1.0):
                    force = model.force_from_command(command, reversed_motor)
                    self.assertAlmostEqual(
                        model.force_to_command(force, reversed_motor),
                        command,
                    )

    def test_each_axis_matches_requested_wrench(self):
        allocator = ThrustAllocator(vehicle="current_m5")
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
                    commands = allocator.commands(result.targets)
                    forces = [
                        allocator.model.force_from_command(
                            command, allocator.profile.THRUSTER_REVERSED[name]
                        )
                        for name, command in zip(
                            allocator.names, commands, strict=True
                        )
                    ]
                    np.testing.assert_allclose(
                        allocator.wrench_matrix @ forces,
                        result.achieved,
                        atol=1e-9,
                    )
                    limited = allocator.commands(result.targets, max_power=0.2)
                    self.assertLessEqual(max(map(abs, limited)), 0.2 + 1e-12)
                    limited_forces = [
                        allocator.model.force_from_command(
                            command, allocator.profile.THRUSTER_REVERSED[name]
                        )
                        for name, command in zip(
                            allocator.names, limited, strict=True
                        )
                    ]
                    wrench = allocator.control_matrix @ limited_forces
                    off_axis = np.delete(wrench, index)
                    np.testing.assert_allclose(off_axis, 0.0, atol=1e-9)


if __name__ == "__main__":
    unittest.main()

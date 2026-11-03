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


class AllocationTests(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()

from __future__ import annotations

import importlib.util
import math
import sys
import unittest
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "120_measure_localization.py"
SPEC = importlib.util.spec_from_file_location("localization", SCRIPT)
localization = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = localization
SPEC.loader.exec_module(localization)


class LocalizationTests(unittest.TestCase):
    def test_stable_identity_transform_passes(self) -> None:
        samples = [
            {"wall_sec": 0.1 + i, "sim_sec": float(i), "x": 0.01, "y": -0.01, "yaw": 0.01}
            for i in range(8)
        ]
        self.assertEqual(localization.measure(samples, 8.0)["result"], "PASS")

    def test_large_residual_fails(self) -> None:
        samples = [
            {"wall_sec": 0.1 + i, "sim_sec": float(i), "x": 0.5, "y": 0.0, "yaw": 0.0}
            for i in range(8)
        ]
        result = localization.measure(samples, 8.0)
        self.assertEqual(result["result"], "FAIL")
        self.assertTrue(math.isclose(result["final_median"]["x"], 0.5))

    def test_nonzero_expected_transform_passes(self) -> None:
        samples = [
            {"wall_sec": 0.1 + i, "sim_sec": float(i), "x": 12.0, "y": 8.0, "yaw": -0.2}
            for i in range(8)
        ]
        self.assertEqual(localization.measure(samples, 8.0, (12.0, 8.0, -0.2))["result"], "PASS")


if __name__ == "__main__":
    unittest.main()

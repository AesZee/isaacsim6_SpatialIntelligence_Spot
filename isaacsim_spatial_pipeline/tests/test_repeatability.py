from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "101_summarize_repeatability.py"
SPEC = importlib.util.spec_from_file_location("repeatability", SCRIPT)
repeatability = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = repeatability
SPEC.loader.exec_module(repeatability)


class RepeatabilityTests(unittest.TestCase):
    def test_identical_complete_runs_and_variance(self) -> None:
        row = {
            "profile": "wide_diagnostic",
            "trajectory": {"name": "warehouse_mapping_loop"},
            "result": "WARN",
            "motion_complete": True,
            "map_width_cells": 640,
            "map_height_cells": 659,
            "known_ratio": 0.023,
            "occupied_cells": 279,
            "duration_sec": 228.0,
            "scan_frequency_hz": 1.94,
        }
        rows = [dict(row) for _ in range(3)]
        self.assertEqual(repeatability.summarize(rows)["result"], "PASS")
        rows[2]["occupied_cells"] = 1000
        self.assertEqual(repeatability.summarize(rows)["result"], "WARN")


if __name__ == "__main__":
    unittest.main()

from __future__ import annotations

import importlib.util
import math
import sys
import unittest
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "110_evaluate_trajectory_error.py"
SPEC = importlib.util.spec_from_file_location("trajectory_error", SCRIPT)
evaluator = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = evaluator
SPEC.loader.exec_module(evaluator)


class TrajectoryErrorTests(unittest.TestCase):
    def test_known_rigid_alignment_has_zero_error(self) -> None:
        ground = [evaluator.Pose(float(i), float(i), float(i % 2), 0.0, 0.1 * i) for i in range(5)]
        angle, tx, ty = 0.4, 2.0, -1.0
        cosine, sine = math.cos(-angle), math.sin(-angle)
        estimate = [
            evaluator.Pose(
                pose.timestamp,
                cosine * (pose.x - tx) - sine * (pose.y - ty),
                sine * (pose.x - tx) + cosine * (pose.y - ty),
                pose.z,
                pose.yaw - angle,
            )
            for pose in ground
        ]
        report, _ = evaluator.evaluate(list(zip(ground, estimate)), {})
        self.assertLess(report["ate"]["translation_m"]["rmse"], 1e-9)
        self.assertLess(report["ate"]["rotation_rad"]["rmse"], 1e-9)
        self.assertLess(report["rpe"]["translation_m"]["rmse"], 1e-9)

    def test_timestamp_rejections_are_counted(self) -> None:
        ground = [evaluator.Pose(0.0, 0, 0, 0, 0), evaluator.Pose(1.0, 1, 0, 0, 0)]
        estimate = [evaluator.Pose(0.0, 0, 0, 0, 0)]
        pairs, rejected = evaluator.match_poses(ground, estimate, 0.1)
        self.assertEqual(len(pairs), 1)
        self.assertEqual(rejected["timestamp_tolerance"], 1)

    def test_known_scale_error_has_expected_ate_and_rpe(self) -> None:
        ground = [evaluator.Pose(float(i), float(i), 0.0, 0.0, 0.0) for i in range(3)]
        estimate = [evaluator.Pose(float(i), 1.1 * i, 0.0, 0.0, 0.0) for i in range(3)]
        report, _ = evaluator.evaluate(list(zip(ground, estimate)), {})
        self.assertAlmostEqual(report["ate"]["translation_m"]["rmse"], math.sqrt(0.02 / 3.0))
        self.assertAlmostEqual(report["rpe"]["translation_m"]["rmse"], 0.1)
        self.assertEqual(report["ate"]["rotation_rad"]["rmse"], 0.0)


if __name__ == "__main__":
    unittest.main()

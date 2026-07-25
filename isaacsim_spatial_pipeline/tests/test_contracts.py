import json
import unittest
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


class ContractRegressionTests(unittest.TestCase):
    def test_topic_frame_and_artifact_contracts(self) -> None:
        scan = yaml.safe_load((ROOT / "config/m04_pointcloud_to_laserscan.yaml").read_text())[
            "m04_pointcloud_to_laserscan"
        ]["ros__parameters"]
        slam = yaml.safe_load((ROOT / "config/m07_slam_toolbox_sim_odom.yaml").read_text())[
            "slam_toolbox"
        ]["ros__parameters"]
        odom = yaml.safe_load((ROOT / "config/m06_sim_odometry_bridge.yaml").read_text())[
            "m06_sim_odometry_bridge"
        ]["ros__parameters"]
        self.assertEqual(scan["target_frame"], "os1_frame")
        self.assertEqual((slam["map_frame"], slam["odom_frame"], slam["base_frame"], slam["scan_topic"]), ("map", "odom", "base_link", "/scan"))
        self.assertEqual((odom["odom_topic"], odom["odom_frame"], odom["base_frame"]), ("/odom", "odom", "base_link"))
        saver = (ROOT / "scripts/81_save_map_artifacts.py").read_text()
        self.assertIn("map.yaml", saver)
        self.assertIn("map.pgm", saver)
        self.assertIn("map_metadata.json", saver)
        self.assertIn("map_stats.json", saver)
        validator = (ROOT / "scripts/70_validate_lidar_slam_with_sim_odom.py").read_text()
        self.assertIn("scan_finite_ranges", validator)

    def test_robustness_cases_are_bounded_and_have_expectations(self) -> None:
        scenarios = json.loads((ROOT / "config/m13_robustness.json").read_text())["scenarios"]
        self.assertIn("normal_baseline", scenarios)
        self.assertGreaterEqual(len(scenarios), 5)
        for scenario in scenarios.values():
            self.assertTrue(scenario["expected"])
        dropout = scenarios["controlled_scan_dropout"]["launch_arguments"]
        self.assertGreater(dropout["drop_after_sec"], 0)
        self.assertGreater(dropout["drop_duration_sec"], 0)


if __name__ == "__main__":
    unittest.main()

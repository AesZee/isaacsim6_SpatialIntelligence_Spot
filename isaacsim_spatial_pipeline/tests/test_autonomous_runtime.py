from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import time
import unittest
from pathlib import Path

import yaml


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "100_run_autonomous_experiment.py"
SPEC = importlib.util.spec_from_file_location("autonomous_runtime", SCRIPT)
runtime = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = runtime
SPEC.loader.exec_module(runtime)


class AutonomousRuntimeTests(unittest.TestCase):
    def setUp(self) -> None:
        config_path = Path(__file__).resolve().parents[1] / "config" / "autonomous_runtime.yaml"
        self.config = yaml.safe_load(config_path.read_text(encoding="utf-8"))

    def test_configuration_validation(self) -> None:
        runtime.validate_config(self.config)
        invalid = json.loads(json.dumps(self.config))
        invalid["trajectory"]["name"] = "missing"
        with self.assertRaisesRegex(ValueError, "invalid trajectory"):
            runtime.validate_config(invalid)
        invalid = json.loads(json.dumps(self.config))
        invalid["limits"]["max_experiments"] = 0
        with self.assertRaisesRegex(ValueError, "max_experiments"):
            runtime.validate_config(invalid)
        invalid = json.loads(json.dumps(self.config))
        invalid["experiments"][0]["initial_pose_offset"] = [1.1, 0.0, 0.0]
        with self.assertRaisesRegex(ValueError, "initial pose offset"):
            runtime.validate_config(invalid)

    def test_all_declared_trajectories_are_bounded(self) -> None:
        path = Path(self.config["trajectory"]["data_file"])
        names = json.loads(path.read_text(encoding="utf-8"))["trajectories"]
        self.assertGreaterEqual(len(names), 3)
        for name in names:
            config = json.loads(json.dumps(self.config))
            config["trajectory"]["name"] = name
            config["limits"]["trajectory_timeout_sec"] = 300.0
            runtime.validate_config(config)
        invalid = json.loads(json.dumps(self.config))
        invalid["limits"]["startup_timeout_sec"] = 181.0
        with self.assertRaisesRegex(ValueError, "safety maximum"):
            runtime.validate_config(invalid)
        invalid = json.loads(json.dumps(self.config))
        invalid["safety"]["source_usd_writes"] = True
        with self.assertRaisesRegex(ValueError, "source_usd_writes"):
            runtime.validate_config(invalid)

    def test_run_directories_are_unique_and_never_overwritten(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            first_id, first = runtime.create_run_directory(root)
            second_id, second = runtime.create_run_directory(root)
            self.assertNotEqual(first_id, second_id)
            self.assertTrue(first.is_dir())
            self.assertTrue(second.is_dir())
            with self.assertRaises(FileExistsError):
                runtime.create_run_directory(root, first_id)

    def test_process_cleanup(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            manager = runtime.ProcessManager(Path(temporary), shutdown_timeout=1.0)
            managed = manager.start(
                "sleeper",
                [sys.executable, "-c", "import time; time.sleep(30)"],
            )
            manager.cleanup()
            self.assertIsNotNone(managed.process.poll())
            self.assertTrue(manager.all_terminated())

    def test_process_timeout(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            manager = runtime.ProcessManager(Path(temporary), shutdown_timeout=1.0)
            result = manager.run(
                "timeout",
                [sys.executable, "-c", "import time; time.sleep(30)"],
                timeout=0.1,
            )
            self.assertTrue(result.timed_out)
            self.assertIsNotNone(manager.processes["timeout"].process.poll())

    def test_runtime_readiness_is_recorded(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            config = json.loads(json.dumps(self.config))
            config["artifacts"]["output_root"] = temporary
            harness = runtime.AutonomousHarness(
                config,
                Path(config["trajectory"]["data_file"]),
                dry_run=True,
                smoke=True,
            )
            status_path = Path(temporary) / "ready.json"
            status_path.write_text('{"state": "ready"}', encoding="utf-8")
            process = harness.manager.start(
                "runtime",
                [sys.executable, "-c", "import time; time.sleep(30)"],
            )
            try:
                harness.wait_for_runtime(process, status_path)
                self.assertEqual(
                    harness.manifest["steps"][-1]["runtime_status"]["state"],
                    "ready",
                )
            finally:
                harness.manager.cleanup()

    def test_result_classification_requires_observed_evidence(self) -> None:
        observed = {
            "source_usd_unchanged": True,
            "preflight": "PASS",
            "motion_complete": True,
            "map_artifacts_valid": True,
            "scan_diagnostic": "PASS",
            "output_within_limit": True,
            "quality": "PASS",
            "final_validation": "PASS",
        }
        self.assertEqual(runtime.classify_result(observed), "PASS")
        observed["quality"] = "WARN"
        self.assertEqual(runtime.classify_result(observed), "WARN")
        observed["quality"] = "FAIL"
        self.assertEqual(runtime.classify_result(observed), "FAIL")
        observed["quality"] = "WARN"
        observed["motion_complete"] = False
        self.assertEqual(runtime.classify_result(observed), "FAIL")


if __name__ == "__main__":
    unittest.main()

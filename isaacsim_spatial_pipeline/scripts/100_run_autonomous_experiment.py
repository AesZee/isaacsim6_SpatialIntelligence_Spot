#!/usr/bin/env python3
"""Run a bounded, evidence-driven Isaac Sim SLAM experiment."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import re
import shlex
import shutil
import signal
import subprocess
import sys
import time
import uuid
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
RESULTS = {"PASS": 0, "WARN": 1, "FAIL": 2}
SAFETY_MAXIMA = {
    "startup_timeout_sec": 180.0,
    "topic_timeout_sec": 30.0,
    "trajectory_timeout_sec": 300.0,
    "shutdown_timeout_sec": 30.0,
    "total_duration_sec": 3600.0,
    "output_max_mb": 2048.0,
    "max_experiments": 8,
}


class HarnessFailure(RuntimeError):
    pass


@dataclass
class CommandResult:
    name: str
    command: list[str]
    returncode: int
    timed_out: bool
    duration_sec: float
    log_path: Path


@dataclass
class ManagedProcess:
    name: str
    command: list[str]
    process: subprocess.Popen
    log_path: Path
    log_stream: object


class ProcessManager:
    def __init__(self, log_directory: Path, shutdown_timeout: float) -> None:
        self.log_directory = log_directory
        self.shutdown_timeout = shutdown_timeout
        self.processes: dict[str, ManagedProcess] = {}
        self.log_directory.mkdir(parents=True, exist_ok=True)

    def start(self, name: str, command: list[str]) -> ManagedProcess:
        if name in self.processes:
            raise HarnessFailure(f"process name already used: {name}")
        log_path = self.log_directory / f"{name}.log"
        log_stream = log_path.open("x", encoding="utf-8")
        process = subprocess.Popen(
            command,
            stdout=log_stream,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            text=True,
        )
        managed = ManagedProcess(name, command, process, log_path, log_stream)
        self.processes[name] = managed
        return managed

    def run(self, name: str, command: list[str], timeout: float) -> CommandResult:
        managed = self.start(name, command)
        started = time.monotonic()
        timed_out = False
        try:
            managed.process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            timed_out = True
            self.stop(name)
        finally:
            self._close_log(managed)
        return CommandResult(
            name=name,
            command=command,
            returncode=managed.process.returncode if managed.process.returncode is not None else -1,
            timed_out=timed_out,
            duration_sec=time.monotonic() - started,
            log_path=managed.log_path,
        )

    def stop(self, name: str) -> None:
        managed = self.processes.get(name)
        if managed is None:
            return
        process = managed.process
        if process.poll() is None:
            for stop_signal, wait_time in (
                (signal.SIGINT, self.shutdown_timeout * 0.5),
                (signal.SIGTERM, self.shutdown_timeout * 0.3),
                (signal.SIGKILL, self.shutdown_timeout * 0.2),
            ):
                try:
                    os.killpg(process.pid, stop_signal)
                except ProcessLookupError:
                    break
                try:
                    process.wait(timeout=max(0.1, wait_time))
                    break
                except subprocess.TimeoutExpired:
                    continue
        self._close_log(managed)

    def cleanup(self) -> None:
        for name in reversed(list(self.processes)):
            self.stop(name)

    def all_terminated(self) -> bool:
        return all(managed.process.poll() is not None for managed in self.processes.values())

    @staticmethod
    def _close_log(managed: ManagedProcess) -> None:
        if not managed.log_stream.closed:
            managed.log_stream.flush()
            managed.log_stream.close()


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def create_run_directory(output_root: Path, run_id: str | None = None) -> tuple[str, Path]:
    output_root.mkdir(parents=True, exist_ok=True)
    identifier = run_id or (
        f"run_{datetime.now().strftime('%Y%m%dT%H%M%S')}_{os.getpid()}_{uuid.uuid4().hex[:8]}"
    )
    directory = output_root / identifier
    directory.mkdir(exist_ok=False)
    return identifier, directory


def nested(config: dict, *keys: str):
    value = config
    for key in keys:
        if not isinstance(value, dict) or key not in value:
            raise ValueError(f"missing configuration key: {'.'.join(keys)}")
        value = value[key]
    return value


def resolve_trajectory(config: dict) -> dict:
    path = Path(str(nested(config, "trajectory", "data_file")))
    name = str(nested(config, "trajectory", "name"))
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
        trajectory = dict(payload["trajectories"][name])
    except (OSError, KeyError, TypeError, json.JSONDecodeError) as error:
        raise ValueError(f"invalid trajectory '{name}' in {path}: {error}") from error
    trajectory["name"] = name
    trajectory["data_file"] = str(path.resolve())
    return trajectory


def validate_config(config: dict) -> None:
    limits = nested(config, "limits")
    positive_limits = (
        "startup_timeout_sec",
        "topic_timeout_sec",
        "trajectory_timeout_sec",
        "shutdown_timeout_sec",
        "total_duration_sec",
        "output_max_mb",
        "watchdog_window_sec",
    )
    for name in positive_limits:
        if float(nested(limits, name)) <= 0.0:
            raise ValueError(f"limits.{name} must be positive")
    for name, maximum in SAFETY_MAXIMA.items():
        if float(nested(limits, name)) > maximum:
            raise ValueError(f"limits.{name} exceeds the safety maximum {maximum:g}")

    max_experiments = int(nested(limits, "max_experiments"))
    experiments = nested(config, "experiments")
    if max_experiments <= 0:
        raise ValueError("limits.max_experiments must be positive")
    if not isinstance(experiments, list) or not experiments:
        raise ValueError("experiments must contain at least one experiment")
    if len(experiments) > max_experiments:
        raise ValueError("configured experiments exceed limits.max_experiments")
    names = [nested(experiment, "name") for experiment in experiments]
    if len(set(names)) != len(names):
        raise ValueError("experiment names must be unique")
    for experiment in experiments:
        offset = experiment.get("initial_pose_offset", [0.0, 0.0, 0.0])
        if not isinstance(offset, list) or len(offset) != 3:
            raise ValueError("experiment initial_pose_offset must be [x, y, yaw]")
        x, y, yaw = map(float, offset)
        if abs(x) > 1.0 or abs(y) > 1.0 or abs(yaw) > math.pi:
            raise ValueError("experiment initial pose offset exceeds the bounded 1 m / pi rad limit")
        launch_arguments = experiment.get("launch_arguments", {})
        if not isinstance(launch_arguments, dict) or any(
            not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", str(key))
            for key in launch_arguments
        ):
            raise ValueError("experiment launch_arguments must use ROS launch argument names")

    trajectory = resolve_trajectory(config)
    if float(nested(trajectory, "speed_mps")) <= 0.0:
        raise ValueError("trajectory.speed_mps must be positive")
    if float(nested(trajectory, "max_duration_sec")) <= 0.0:
        raise ValueError("trajectory.max_duration_sec must be positive")
    area = nested(trajectory, "permitted_area_relative_m")
    min_x, max_x = float(nested(area, "min_x")), float(nested(area, "max_x"))
    min_y, max_y = float(nested(area, "min_y")), float(nested(area, "max_y"))
    if min_x >= max_x or min_y >= max_y:
        raise ValueError("permitted-area minimums must be less than maximums")
    waypoints = nested(trajectory, "waypoints_m")
    if not isinstance(waypoints, list) or len(waypoints) < 2:
        raise ValueError("trajectory.waypoints_m requires at least two waypoints")
    for point in waypoints:
        if not isinstance(point, list) or len(point) != 2:
            raise ValueError("each trajectory waypoint must be [x, y]")
        x, y = float(point[0]), float(point[1])
        if not min_x <= x <= max_x or not min_y <= y <= max_y:
            raise ValueError(f"trajectory waypoint [{x}, {y}] exceeds the permitted area")
    if float(trajectory["max_duration_sec"]) > float(limits["trajectory_timeout_sec"]):
        raise ValueError("trajectory max duration exceeds trajectory timeout")

    runtime = nested(config, "runtime")
    for key in ("isaac_python", "entry_point", "world_usd", "ros_setup"):
        if not str(nested(runtime, key)).strip():
            raise ValueError(f"runtime.{key} must not be empty")
    artifacts = nested(config, "artifacts")
    if not str(nested(artifacts, "output_root")).strip():
        raise ValueError("artifacts.output_root must not be empty")
    topics = nested(config, "bag", "topics")
    if not isinstance(topics, list) or not topics:
        raise ValueError("bag.topics must be a non-empty list")
    for name in ("overwrite_existing_outputs", "source_usd_writes", "package_installation", "sudo"):
        if nested(config, "safety", name) is not False:
            raise ValueError(f"safety.{name} must be false")
    if not 0.0 < float(nested(config, "evaluation", "min_known_ratio")) <= 1.0:
        raise ValueError("evaluation.min_known_ratio must be in (0, 1]")
    if int(nested(config, "evaluation", "min_occupied_cells")) <= 0:
        raise ValueError("evaluation.min_occupied_cells must be positive")


def classify_result(evidence: dict) -> str:
    if evidence.get("quality") == "FAIL" or evidence.get("final_validation") == "FAIL":
        return "FAIL"
    hard_requirements = (
        evidence.get("source_usd_unchanged"),
        evidence.get("preflight") in {"PASS", "WARN"},
        evidence.get("motion_complete"),
        evidence.get("map_artifacts_valid"),
        evidence.get("scan_diagnostic") == "PASS",
        evidence.get("output_within_limit"),
    )
    if not all(hard_requirements):
        return "FAIL"
    if evidence.get("quality") == "PASS" and evidence.get("final_validation") == "PASS":
        return "PASS"
    return "WARN"


def directory_size_bytes(directory: Path) -> int:
    return sum(path.stat().st_size for path in directory.rglob("*") if path.is_file())


def ros_command(ros_setup: Path, command: list[str]) -> list[str]:
    script = f"source {shlex.quote(str(ros_setup))} && exec {shlex.join(command)}"
    return ["bash", "-lc", script]


def read_overall_result(log_path: Path) -> str | None:
    matches = re.findall(r"Overall result:\s*(PASS|WARN|FAIL)", log_path.read_text(encoding="utf-8"))
    return matches[-1] if matches else None


def read_clock(log_path: Path) -> tuple[int, int] | None:
    text = log_path.read_text(encoding="utf-8")
    seconds = re.search(r"\bsec:\s*(\d+)", text)
    nanoseconds = re.search(r"\bnanosec:\s*(\d+)", text)
    return (int(seconds.group(1)), int(nanoseconds.group(1))) if seconds and nanoseconds else None


def read_json(path: Path) -> dict | None:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None


def git_revision() -> str:
    try:
        result = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
            timeout=5.0,
            check=False,
        )
        revision = result.stdout.strip()
        return revision if result.returncode == 0 and revision else "unavailable"
    except (OSError, subprocess.TimeoutExpired):
        return "unavailable"


class AutonomousHarness:
    def __init__(self, config: dict, config_path: Path, dry_run: bool, smoke: bool) -> None:
        validate_config(config)
        self.config = config
        self.config_path = config_path
        self.dry_run = dry_run
        self.smoke = smoke
        self.cancelled = False
        self.started = time.monotonic()
        self.global_deadline = self.started + float(config["limits"]["total_duration_sec"])
        output_root = Path(config["artifacts"]["output_root"]).resolve()
        self.run_id, self.run_directory = create_run_directory(output_root)
        self.logs = self.run_directory / "logs"
        self.logs.mkdir()
        ros_logs = self.run_directory / "ros_logs"
        ros_logs.mkdir()
        os.environ["ROS_LOG_DIR"] = str(ros_logs)
        self.manager = ProcessManager(
            self.logs,
            min(float(config["limits"]["shutdown_timeout_sec"]), 3.0)
            if smoke
            else float(config["limits"]["shutdown_timeout_sec"]),
        )
        self.manifest = {
            "run_id": self.run_id,
            "mode": "dry-run" if dry_run else ("smoke" if smoke else "full"),
            "started_at": utc_now(),
            "git_revision": git_revision(),
            "config": str(config_path.resolve()),
            "result": "RUNNING",
            "steps": [],
            "experiments": [],
            "limitations": [
                "Motion is simulation-only kinematic Spot root-prim motion.",
                "No collision callback is exposed by the existing runtime; relative permitted-area bounds are enforced.",
                "Map metrics do not prove ground-truth geometric accuracy.",
            ],
        }
        immutable_inputs = [
            Path(path).resolve() for path in config["evaluation"].get("immutable_inputs", [])
        ]
        if immutable_inputs:
            if not all(path.is_file() for path in immutable_inputs):
                raise ValueError("an immutable evaluation input is missing")
            self.manifest["immutable_inputs"] = {
                str(path): {"sha256_before": sha256(path)} for path in immutable_inputs
            }
        (self.run_directory / "effective_config.yaml").write_text(
            yaml.safe_dump(config, sort_keys=False),
            encoding="utf-8",
        )
        self._write_outputs()

    def remaining(self, local_deadline: float | None = None) -> float:
        deadline = min(self.global_deadline, local_deadline) if local_deadline else self.global_deadline
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise HarnessFailure("total-duration limit reached")
        return remaining

    def bounded_timeout(self, requested: float, local_deadline: float | None = None) -> float:
        return max(0.1, min(requested, self.remaining(local_deadline)))

    def add_step(self, name: str, status: str, **evidence) -> None:
        self.manifest["steps"].append(
            {"name": name, "status": status, "observed_at": utc_now(), **evidence}
        )
        self._write_outputs()

    def run_command(
        self,
        name: str,
        command: list[str],
        timeout: float,
        local_deadline: float | None = None,
    ) -> CommandResult:
        if self.cancelled:
            raise HarnessFailure("run cancelled by signal")
        result = self.manager.run(name, command, self.bounded_timeout(timeout, local_deadline))
        self.add_step(
            name,
            "FAIL" if result.timed_out or result.returncode != 0 else "PASS",
            command=result.command,
            returncode=result.returncode,
            timed_out=result.timed_out,
            duration_sec=round(result.duration_sec, 3),
            log=str(result.log_path.relative_to(self.run_directory)),
        )
        if result.timed_out:
            raise HarnessFailure(f"{name} timed out")
        return result

    def check_dependencies(self) -> None:
        runtime = self.config["runtime"]
        paths = {
            "isaac_python": Path(runtime["isaac_python"]),
            "entry_point": Path(runtime["entry_point"]),
            "world_usd": Path(runtime["world_usd"]),
            "ros_setup": Path(runtime["ros_setup"]),
            "slam_launch": Path(self.config["slam"]["launch_file"]),
        }
        missing = [f"{name}: {path}" for name, path in paths.items() if not path.is_file()]
        if missing:
            raise HarnessFailure("missing runtime dependencies: " + "; ".join(missing))
        if not os.access(paths["isaac_python"], os.X_OK):
            raise HarnessFailure(f"Isaac Python entry point is not executable: {paths['isaac_python']}")
        for index, package in enumerate(("pointcloud_to_laserscan", "slam_toolbox", "tf2_ros")):
            result = self.run_command(
                f"dependency_{index}_{package}",
                ros_command(paths["ros_setup"], ["ros2", "pkg", "prefix", package]),
                15.0,
            )
            if result.returncode != 0:
                raise HarnessFailure(f"required ROS package unavailable: {package}")
        if runtime.get("require_gpu", True):
            nvidia_smi = shutil.which("nvidia-smi")
            if not nvidia_smi:
                reason = "nvidia-smi is unavailable; RTX LiDAR runtime dependency is not satisfied"
                if not self.dry_run:
                    raise HarnessFailure(reason)
                self.manifest["live_gate"] = "BLOCKED"
                self.add_step("live_runtime_readiness", "BLOCKED", reason=reason)
                self.add_step("dependency_check", "PASS", paths={name: str(path) for name, path in paths.items()})
                return
            gpu_result = (
                self.manager.run("dependency_gpu", [nvidia_smi, "-L"], 10.0)
                if self.dry_run
                else self.run_command("dependency_gpu", [nvidia_smi, "-L"], 10.0)
            )
            gpu_output = gpu_result.log_path.read_text(encoding="utf-8").strip()
            if gpu_result.returncode != 0 or "GPU " not in gpu_output:
                reason = "no usable NVIDIA GPU was observed; RTX LiDAR smoke test is blocked"
                if not self.dry_run:
                    raise HarnessFailure(reason)
                self.manifest["live_gate"] = "BLOCKED"
                self.add_step(
                    "live_runtime_readiness",
                    "BLOCKED",
                    reason=reason,
                    command=gpu_result.command,
                    returncode=gpu_result.returncode,
                    log=str(gpu_result.log_path.relative_to(self.run_directory)),
                )
            else:
                self.manifest["live_gate"] = "READY"
                self.add_step(
                    "live_runtime_readiness",
                    "PASS",
                    command=gpu_result.command,
                    returncode=gpu_result.returncode,
                    log=str(gpu_result.log_path.relative_to(self.run_directory)),
                )
        self.add_step("dependency_check", "PASS", paths={name: str(path) for name, path in paths.items()})

    def run(self) -> int:
        runtime = self.config["runtime"]
        world_usd = Path(runtime["world_usd"])
        self.manifest["source_usd"] = {
            "path": str(world_usd),
            "sha256_before": sha256(world_usd),
        }
        try:
            self.check_dependencies()
            if self.dry_run:
                self.manifest["result"] = "PASS"
                self.manifest["scope"] = "dry-run validation only; no runtime evidence"
                self.add_step("dry_run", "PASS", runtime_processes_started=0)
                return 0

            for index, experiment in enumerate(self.config["experiments"]):
                if index >= int(self.config["limits"]["max_experiments"]):
                    raise HarnessFailure("maximum experiment count reached")
                self.run_experiment(index, experiment)

            results = [experiment["result"] for experiment in self.manifest["experiments"]]
            self.manifest["result"] = max(results, key=lambda result: RESULTS[result])
            return 1 if self.manifest["result"] == "FAIL" else 0
        except Exception as error:
            self.manifest["result"] = "FAIL"
            self.manifest["failure"] = str(error)
            for experiment in self.manifest["experiments"]:
                if "finished_at" not in experiment:
                    experiment["finished_at"] = utc_now()
                    experiment["failure"] = str(error)
            return 1
        finally:
            self.manager.cleanup()
            self.manifest["cleanup_confirmed"] = self.manager.all_terminated()
            after_hash = sha256(world_usd)
            self.manifest["source_usd"]["sha256_after"] = after_hash
            self.manifest["source_usd"]["unchanged"] = (
                after_hash == self.manifest["source_usd"]["sha256_before"]
            )
            self.manifest["output_bytes"] = directory_size_bytes(self.run_directory)
            output_limit = int(float(self.config["limits"]["output_max_mb"]) * 1024 * 1024)
            if not self.manifest["source_usd"]["unchanged"]:
                self.manifest["result"] = "FAIL"
                self.manifest["failure"] = "source USD hash changed"
            if self.manifest["output_bytes"] > output_limit:
                self.manifest["result"] = "FAIL"
                self.manifest["failure"] = (
                    f"output limit exceeded: {self.manifest['output_bytes']} > {output_limit} bytes"
                )
            if not self.manifest["cleanup_confirmed"]:
                self.manifest["result"] = "FAIL"
                self.manifest["failure"] = "one or more owned child processes remained after cleanup"
            for path_text, evidence in self.manifest.get("immutable_inputs", {}).items():
                path = Path(path_text)
                evidence["sha256_after"] = sha256(path) if path.is_file() else "missing"
                evidence["unchanged"] = evidence["sha256_after"] == evidence["sha256_before"]
                if not evidence["unchanged"]:
                    self.manifest["result"] = "FAIL"
                    self.manifest["failure"] = f"immutable input changed: {path}"
            self.manifest["finished_at"] = utc_now()
            self._write_outputs()

    def run_experiment(self, index: int, experiment: dict) -> dict:
        name = str(experiment["name"])
        profile = str(experiment["profile"])
        initial_pose_offset = [
            float(value) for value in experiment.get("initial_pose_offset", [0, 0, 0])
        ]
        directory = self.run_directory / f"experiment_{index + 1}_{name}"
        directory.mkdir()
        (directory / "control").mkdir()
        (directory / "maps").mkdir()
        motion_start = directory / "control" / "start_motion"
        motion_status = directory / "motion_status.json"
        limits = self.config["limits"]
        runtime = self.config["runtime"]
        trajectory = resolve_trajectory(self.config)
        ros_setup = Path(runtime["ros_setup"])
        evidence = {
            "name": name,
            "profile": profile,
            "trajectory": trajectory,
            "started_at": utc_now(),
            "result": "FAIL",
            "source_usd_unchanged": True,
            "output_within_limit": True,
            "initial_pose_offset": initial_pose_offset,
            "launch_arguments": experiment.get("launch_arguments", {}),
        }
        self.manifest["experiments"].append(evidence)
        self._write_outputs()
        smoke_deadline = None

        runtime_command = ros_command(
            ros_setup,
            [
                str(runtime["isaac_python"]),
                str(runtime["entry_point"]),
                "--world-usd",
                str(runtime["world_usd"]),
                "--auto-play",
                "--enable-scripted-motion",
                "--single-trajectory",
                "--motion-speed",
                str(trajectory["speed_mps"]),
                "--motion-max-duration",
                str(trajectory["max_duration_sec"]),
                "--trajectory-file",
                trajectory["data_file"],
                "--trajectory-name",
                trajectory["name"],
                "--motion-start-file",
                str(motion_start),
                "--motion-status-file",
                str(motion_status),
                "--permitted-min-x",
                str(trajectory["permitted_area_relative_m"]["min_x"]),
                "--permitted-max-x",
                str(trajectory["permitted_area_relative_m"]["max_x"]),
                "--permitted-min-y",
                str(trajectory["permitted_area_relative_m"]["min_y"]),
                "--permitted-max-y",
                str(trajectory["permitted_area_relative_m"]["max_y"]),
                "--initial-x-offset",
                str(initial_pose_offset[0]),
                "--initial-y-offset",
                str(initial_pose_offset[1]),
                "--initial-yaw-offset",
                str(initial_pose_offset[2]),
                "--max-runtime-seconds",
                str(min(float(limits["total_duration_sec"]), 120.0) if self.smoke else limits["total_duration_sec"]),
                *(["--headless"] if runtime.get("headless", True) else []),
            ],
        )
        runtime_process = self.manager.start(f"{name}_isaac", runtime_command)
        self.add_step(
            f"{name}_isaac_start",
            "PASS",
            command=runtime_command,
            pid=runtime_process.process.pid,
            log=str(runtime_process.log_path.relative_to(self.run_directory)),
        )
        self.wait_for_runtime(runtime_process, motion_status)
        if self.smoke:
            smoke_deadline = time.monotonic() + 120.0

        stack_command = ros_command(
            ros_setup,
            [
                "ros2",
                "launch",
                str(self.config["slam"]["launch_file"]),
                f"profile:={profile}",
                *[
                    f"{key}:={value}"
                    for key, value in experiment.get("launch_arguments", {}).items()
                ],
            ],
        )
        stack_process = self.manager.start(f"{name}_slam", stack_command)
        self.add_step(
            f"{name}_slam_start",
            "PASS",
            command=stack_command,
            pid=stack_process.process.pid,
            log=str(stack_process.log_path.relative_to(self.run_directory)),
        )

        preflight = self.run_validator(name, "preflight", smoke_deadline)
        evidence["preflight"] = preflight
        if preflight == "FAIL":
            raise HarnessFailure("preflight topic or TF validation failed")

        previous_clock = self.observe_clock(name, "clock_before_motion", smoke_deadline)
        localization_duration = float(
            self.config["evaluation"].get("localization_duration_sec", 0.0)
        )
        if localization_duration > 0.0:
            localization_output = directory / "localization_metrics.json"
            expected_transform = self.config["evaluation"].get(
                "localization_expected_transform", [0.0, 0.0, 0.0]
            )
            if not isinstance(expected_transform, list) or len(expected_transform) != 3:
                raise HarnessFailure(
                    "evaluation.localization_expected_transform must be [x, y, yaw]"
                )
            localization_result = self.run_command(
                f"{name}_localization",
                ros_command(
                    ros_setup,
                    [
                        sys.executable,
                        str(REPO_ROOT / "scripts" / "120_measure_localization.py"),
                        "--duration",
                        str(localization_duration),
                        "--output",
                        str(localization_output),
                        "--expected-x",
                        str(expected_transform[0]),
                        "--expected-y",
                        str(expected_transform[1]),
                        "--expected-yaw",
                        str(expected_transform[2]),
                    ],
                ),
                localization_duration + 5.0,
                smoke_deadline,
            )
            evidence["localization"] = read_json(localization_output)
            if localization_result.returncode != 0:
                raise HarnessFailure("localization convergence validation failed")
        if self.config["bag"]["enabled"] and not self.smoke:
            bag_directory = directory / "bag"
            bag_directory.mkdir()
            bag_command = ros_command(
                ros_setup,
                [
                    "ros2",
                    "bag",
                    "record",
                    "-o",
                    str(bag_directory / "recording"),
                    *self.config["bag"]["topics"],
                ],
            )
            bag_process = self.manager.start(f"{name}_bag", bag_command)
            self.add_step(
                f"{name}_bag_start",
                "PASS",
                command=bag_command,
                pid=bag_process.process.pid,
                log=str(bag_process.log_path.relative_to(self.run_directory)),
            )

        motion_start.write_text(f"{self.run_id}\n", encoding="utf-8")
        self.add_step(f"{name}_trajectory_start", "PASS", trigger=str(motion_start))
        motion_wall_started = time.monotonic()
        trajectory_deadline = time.monotonic() + float(limits["trajectory_timeout_sec"])
        if smoke_deadline:
            trajectory_deadline = min(
                time.monotonic() + 60.0,
                trajectory_deadline,
                smoke_deadline,
            )
        watchdog_index = 0
        while True:
            self.remaining(trajectory_deadline)
            if runtime_process.process.poll() is not None:
                raise HarnessFailure("Isaac Sim exited during the trajectory")
            status = read_json(motion_status) or {}
            if status.get("state") == "complete" or (
                status.get("state") == "shutdown" and status.get("motion_complete")
            ):
                evidence["motion_complete"] = True
                evidence["motion_status"] = status
                evidence["motion_duration_wall_sec"] = round(
                    time.monotonic() - motion_wall_started,
                    3,
                )
                break
            watchdog = self.run_validator(name, f"watchdog_{watchdog_index}", trajectory_deadline)
            if watchdog == "FAIL":
                raise HarnessFailure("required topic or TF failed during trajectory")
            clock = self.observe_clock(name, f"clock_{watchdog_index}", trajectory_deadline)
            if clock <= previous_clock:
                raise HarnessFailure(f"simulation clock stalled: previous={previous_clock}, current={clock}")
            previous_clock = clock
            self.enforce_output_limit()
            watchdog_index += 1

        self.add_step(
            f"{name}_trajectory_complete",
            "PASS",
            motion_status=evidence["motion_status"],
            duration_wall_sec=evidence["motion_duration_wall_sec"],
        )
        final_validation = self.run_validator(name, "final", smoke_deadline)
        evidence["final_validation"] = final_validation
        if final_validation == "FAIL":
            raise HarnessFailure("final topic or TF validation failed")

        diagnostic_output = directory / "scan_diagnostics.json"
        diagnostic = self.run_diagnostic(name, profile, diagnostic_output, smoke_deadline)
        evidence["scan_diagnostic"] = diagnostic
        evidence["scan_metrics"] = read_json(diagnostic_output)

        saver_result = self.run_command(
            f"{name}_save_map",
            ros_command(
                ros_setup,
                [
                    sys.executable,
                    str(REPO_ROOT / "scripts" / "81_save_map_artifacts.py"),
                    "--duration",
                    str(self.config["evaluation"]["map_save_duration_sec"]),
                    "--output-root",
                    str(directory / "maps"),
                    "--prefix",
                    "map",
                ],
            ),
            float(self.config["evaluation"]["map_save_duration_sec"]) + 5.0,
            smoke_deadline,
        )
        if saver_result.returncode != 0:
            raise HarnessFailure("map saver failed")
        map_directories = sorted(path for path in (directory / "maps").iterdir() if path.is_dir())
        if len(map_directories) != 1:
            raise HarnessFailure("a genuine map artifact directory was not created")
        map_directory = map_directories[0]
        evidence["map_directory"] = str(map_directory)

        artifact_result = self.run_command(
            f"{name}_validate_map",
            ros_command(
                ros_setup,
                [sys.executable, str(REPO_ROOT / "scripts" / "82_validate_saved_map_artifacts.py"), str(map_directory)],
            ),
            10.0,
            smoke_deadline,
        )
        artifact_verdict = read_overall_result(artifact_result.log_path)
        evidence["map_artifacts_valid"] = artifact_result.returncode == 0 and artifact_verdict in {"PASS", "WARN"}
        if not evidence["map_artifacts_valid"]:
            raise HarnessFailure("saved map artifact validation failed")

        quality_result = self.run_command(
            f"{name}_evaluate_map",
            ros_command(
                ros_setup,
                [sys.executable, str(REPO_ROOT / "scripts" / "83_evaluate_map_quality.py"), str(map_directory)],
            ),
            10.0,
            smoke_deadline,
        )
        evidence["artifact_quality"] = read_overall_result(quality_result.log_path) or "FAIL"
        evidence["map_stats"] = read_json(map_directory / "map_stats.json")
        evidence["map_metadata"] = read_json(map_directory / "map_metadata.json")
        stats = evidence["map_stats"] or {}
        meets_targets = (
            float(stats.get("known_ratio", 0.0)) >= float(self.config["evaluation"]["min_known_ratio"])
            and int(stats.get("occupied_cells", 0)) >= int(self.config["evaluation"]["min_occupied_cells"])
        )
        evidence["quality"] = (
            "FAIL"
            if evidence["artifact_quality"] == "FAIL"
            else ("PASS" if meets_targets else "WARN")
        )
        if self.config["evaluation"].get("serialize_posegraph", False):
            posegraph_directory = directory / "posegraph"
            posegraph_directory.mkdir()
            posegraph_prefix = posegraph_directory / "map"
            serialize_result = self.run_command(
                f"{name}_serialize_posegraph",
                ros_command(
                    ros_setup,
                    [
                        "ros2",
                        "service",
                        "call",
                        "/slam_toolbox/serialize_map",
                        "slam_toolbox/srv/SerializePoseGraph",
                        f"{{filename: '{posegraph_prefix}'}}",
                    ],
                ),
                30.0,
                smoke_deadline,
            )
            posegraph_files = [
                posegraph_prefix.with_suffix(suffix) for suffix in (".posegraph", ".data")
            ]
            if serialize_result.returncode != 0 or not all(
                path.is_file() and path.stat().st_size > 0 for path in posegraph_files
            ):
                raise HarnessFailure("slam_toolbox posegraph serialization failed")
            evidence["posegraph"] = {
                "prefix": str(posegraph_prefix),
                "files": [str(path) for path in posegraph_files],
                "sha256": {path.name: sha256(path) for path in posegraph_files},
            }
        evidence["output_within_limit"] = self.enforce_output_limit()
        self.manager.stop(f"{name}_bag")
        self.manager.stop(f"{name}_slam")
        self.manager.stop(f"{name}_isaac")
        evidence["source_usd_unchanged"] = (
            sha256(Path(runtime["world_usd"])) == self.manifest["source_usd"]["sha256_before"]
        )
        evidence["result"] = classify_result(evidence)
        evidence["finished_at"] = utc_now()
        self._write_outputs()
        return evidence

    def wait_for_runtime(self, process: ManagedProcess, status_path: Path) -> None:
        startup_limit = float(self.config["limits"]["startup_timeout_sec"])
        if self.smoke:
            startup_limit = min(startup_limit, 55.0)
        deadline = min(self.global_deadline, time.monotonic() + startup_limit)
        while time.monotonic() < deadline:
            if self.cancelled:
                raise HarnessFailure("run cancelled by signal")
            if process.process.poll() is not None:
                raise HarnessFailure(f"Isaac Sim exited during startup with code {process.process.returncode}")
            status = read_json(status_path)
            if status and status.get("state") in {"ready", "armed"}:
                self.add_step("isaac_runtime_ready", "PASS", runtime_status=status)
                return
            self.enforce_output_limit()
            time.sleep(0.2)
        raise HarnessFailure("Isaac Sim startup timed out")

    def run_validator(self, experiment: str, phase: str, deadline: float | None) -> str:
        duration = float(self.config["limits"]["topic_timeout_sec"])
        if phase != "preflight":
            duration = min(duration, float(self.config["limits"]["watchdog_window_sec"]))
        name = f"{experiment}_{phase}_validator"
        result = self.run_command(
            name,
            ros_command(
                Path(self.config["runtime"]["ros_setup"]),
                [
                    sys.executable,
                    str(REPO_ROOT / "scripts" / "70_validate_lidar_slam_with_sim_odom.py"),
                    "--duration",
                    str(duration),
                    *(["--exit-when-ready"] if phase == "preflight" else []),
                ],
            ),
            duration + 5.0,
            deadline,
        )
        verdict = read_overall_result(result.log_path) or "FAIL"
        self.add_step(f"{name}_evidence", verdict, overall_result=verdict)
        return verdict

    def observe_clock(self, experiment: str, phase: str, deadline: float | None) -> tuple[int, int]:
        result = self.run_command(
            f"{experiment}_{phase}",
            ros_command(
                Path(self.config["runtime"]["ros_setup"]),
                ["ros2", "topic", "echo", "/clock", "--once", "--field", "clock"],
            ),
            float(self.config["limits"]["topic_timeout_sec"]),
            deadline,
        )
        clock = read_clock(result.log_path)
        if result.returncode != 0 or clock is None:
            raise HarnessFailure("could not directly observe /clock")
        return clock

    def run_diagnostic(
        self,
        experiment: str,
        profile: str,
        output: Path,
        deadline: float | None,
    ) -> str:
        profile_path = Path(self.config["slam"]["profile_config"])
        profiles = yaml.safe_load(profile_path.read_text(encoding="utf-8"))["lidar_slice_profiles"]
        if profile not in profiles:
            raise HarnessFailure(f"missing LiDAR profile: {profile}")
        parameters = profiles[profile]
        command = [
            sys.executable,
            str(REPO_ROOT / "scripts" / "92_diagnose_laserscan_quality.py"),
            "--duration",
            str(self.config["evaluation"]["diagnostic_duration_sec"]),
            "--json-output",
            str(output),
        ]
        for name in ("min_height", "max_height", "range_min", "range_max", "angle_min", "angle_max"):
            command.extend([f"--{name.replace('_', '-')}", str(parameters[name])])
        result = self.run_command(
            f"{experiment}_scan_diagnostic",
            ros_command(Path(self.config["runtime"]["ros_setup"]), command),
            float(self.config["evaluation"]["diagnostic_duration_sec"]) + 5.0,
            deadline,
        )
        return "PASS" if result.returncode == 0 else "FAIL"

    def enforce_output_limit(self) -> bool:
        size = directory_size_bytes(self.run_directory)
        maximum = int(float(self.config["limits"]["output_max_mb"]) * 1024 * 1024)
        if size > maximum:
            raise HarnessFailure(f"output limit exceeded: {size} > {maximum} bytes")
        return True

    def _write_outputs(self) -> None:
        manifest_path = self.run_directory / "manifest.json"
        temporary = manifest_path.with_suffix(".json.tmp")
        temporary.write_text(json.dumps(self.manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        temporary.replace(manifest_path)
        csv_path = self.run_directory / "experiments.csv"
        csv_temporary = csv_path.with_suffix(".csv.tmp")
        with csv_temporary.open("w", encoding="utf-8", newline="") as stream:
            fieldnames = [
                "experiment",
                "profile",
                "trajectory",
                "result",
                "min_height",
                "max_height",
                "range_min",
                "range_max",
                "angle_min",
                "angle_max",
                "map_width_cells",
                "map_height_cells",
                "map_resolution_m",
                "known_ratio",
                "occupied_cells",
                "duration_sec",
                "scan_frequency_hz",
                "map_directory",
            ]
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            for experiment in self.manifest["experiments"]:
                stats = experiment.get("map_stats") or {}
                scan_metrics = experiment.get("scan_metrics") or {}
                scan = scan_metrics.get("laserscan") or {}
                parameters = scan_metrics.get("parameters") or {}
                metadata = experiment.get("map_metadata") or {}
                writer.writerow(
                    {
                        "experiment": experiment["name"],
                        "profile": experiment["profile"],
                        "trajectory": (experiment.get("trajectory") or {}).get("name"),
                        "result": experiment["result"],
                        **{name: parameters.get(name) for name in (
                            "min_height", "max_height", "range_min",
                            "range_max", "angle_min", "angle_max",
                        )},
                        "map_width_cells": metadata.get("width"),
                        "map_height_cells": metadata.get("height"),
                        "map_resolution_m": metadata.get("resolution"),
                        "known_ratio": stats.get("known_ratio"),
                        "occupied_cells": stats.get("occupied_cells"),
                        "duration_sec": experiment.get("motion_duration_wall_sec"),
                        "scan_frequency_hz": scan.get("frequency_hz"),
                        "map_directory": experiment.get("map_directory"),
                    }
                )
        csv_temporary.replace(csv_path)

        lines = [
            "# Autonomous Runtime Report",
            "",
            f"- Run ID: `{self.manifest['run_id']}`",
            f"- Mode: `{self.manifest['mode']}`",
            f"- Result: `{self.manifest['result']}`",
            f"- Live gate: `{self.manifest.get('live_gate', 'n/a')}`",
            f"- Git revision: `{self.manifest['git_revision']}`",
            f"- Cleanup confirmed: `{self.manifest.get('cleanup_confirmed', False)}`",
            f"- Started: `{self.manifest['started_at']}`",
            "",
            "## Observed steps",
            "",
            "| Step | Status | Evidence |",
            "| --- | --- | --- |",
        ]
        for step in self.manifest["steps"]:
            evidence = {key: value for key, value in step.items() if key not in {"name", "status", "observed_at"}}
            lines.append(f"| {step['name']} | {step['status']} | `{json.dumps(evidence, sort_keys=True)}` |")
        lines.extend(["", "## Experiment results", ""])
        if self.manifest["experiments"]:
            lines.extend(
                [
                    "| Experiment | Profile | Result | Quality | Map |",
                    "| --- | --- | --- | --- | --- |",
                ]
            )
            for experiment in self.manifest["experiments"]:
                lines.append(
                    f"| {experiment['name']} | {experiment['profile']} | "
                    f"{experiment['result']} | {experiment.get('quality', 'n/a')} | "
                    f"`{experiment.get('map_directory', 'n/a')}` |"
                )
        else:
            lines.append("No live experiment was executed.")
        lines.extend(["", "## Limitations", ""])
        lines.extend(f"- {item}" for item in self.manifest["limitations"])
        if self.manifest.get("failure"):
            lines.extend(["", "## Failure", "", self.manifest["failure"]])
        (self.run_directory / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        default=str(REPO_ROOT / "config" / "autonomous_runtime.yaml"),
    )
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--smoke", action="store_true")
    parser.add_argument(
        "--experiment",
        action="append",
        help="Run only the named declared experiment; repeat to select more than one.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config_path = Path(args.config)
    try:
        config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        if args.experiment:
            declared = {item["name"]: item for item in config["experiments"]}
            missing = [name for name in args.experiment if name not in declared]
            if missing:
                raise ValueError(f"unknown experiment selection: {', '.join(missing)}")
            config["experiments"] = [declared[name] for name in args.experiment]
        validate_config(config)
    except (OSError, ValueError, yaml.YAMLError) as error:
        print(f"FAIL: invalid autonomous runtime configuration: {error}")
        return 1

    if args.smoke:
        config = json.loads(json.dumps(config))
        config["limits"]["max_experiments"] = 1
        config["limits"]["output_max_mb"] = min(float(config["limits"]["output_max_mb"]), 500.0)
        config["experiments"] = config["experiments"][:1]
        config["evaluation"]["diagnostic_duration_sec"] = min(
            float(config["evaluation"]["diagnostic_duration_sec"]),
            5.0,
        )
        config["evaluation"]["map_save_duration_sec"] = min(
            float(config["evaluation"]["map_save_duration_sec"]),
            5.0,
        )
        config["bag"]["enabled"] = False
        validate_config(config)

    os.environ["ROS_DOMAIN_ID"] = str(config["runtime"]["ros_domain_id"])
    harness = AutonomousHarness(config, config_path, args.dry_run, args.smoke)

    def cancel(_signum, _frame) -> None:
        harness.cancelled = True

    signal.signal(signal.SIGINT, cancel)
    signal.signal(signal.SIGTERM, cancel)
    result = harness.run()
    print(f"Run directory: {harness.run_directory}")
    print(f"Overall result: {harness.manifest['result']}")
    return result


if __name__ == "__main__":
    raise SystemExit(main())

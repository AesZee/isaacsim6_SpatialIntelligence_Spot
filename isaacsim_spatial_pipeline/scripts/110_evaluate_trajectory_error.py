#!/usr/bin/env python3
"""Align timestamped SE(2) poses and calculate ATE/RPE."""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import statistics
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Pose:
    timestamp: float
    x: float
    y: float
    z: float
    yaw: float


def wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_yaw(row: dict[str, str]) -> float:
    x, y, z, w = (float(row[name]) for name in ("qx", "qy", "qz", "qw"))
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def load_poses(path: Path) -> list[Pose]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    poses = [
        Pose(float(row["timestamp"]), float(row["x"]), float(row["y"]), float(row["z"]), quaternion_yaw(row))
        for row in rows
    ]
    if any(b.timestamp <= a.timestamp for a, b in zip(poses, poses[1:])):
        raise ValueError(f"timestamps must be strictly increasing: {path}")
    return poses


def match_poses(ground: list[Pose], estimate: list[Pose], tolerance: float) -> tuple[list[tuple[Pose, Pose]], dict]:
    timestamps = [pose.timestamp for pose in estimate]
    used = set()
    pairs = []
    rejected = {"no_estimate": 0, "timestamp_tolerance": 0, "duplicate_estimate": 0}
    for truth in ground:
        index = bisect.bisect_left(timestamps, truth.timestamp)
        candidates = [item for item in (index - 1, index) if 0 <= item < len(estimate)]
        if not candidates:
            rejected["no_estimate"] += 1
            continue
        nearest = min(candidates, key=lambda item: abs(estimate[item].timestamp - truth.timestamp))
        if abs(estimate[nearest].timestamp - truth.timestamp) > tolerance:
            rejected["timestamp_tolerance"] += 1
        elif nearest in used:
            rejected["duplicate_estimate"] += 1
        else:
            used.add(nearest)
            pairs.append((truth, estimate[nearest]))
    return pairs, rejected


def align(pairs: list[tuple[Pose, Pose]]) -> tuple[float, float, float, float]:
    if len(pairs) < 2:
        raise ValueError("at least two synchronized pose pairs are required")
    gx = statistics.fmean(pair[0].x for pair in pairs)
    gy = statistics.fmean(pair[0].y for pair in pairs)
    ex = statistics.fmean(pair[1].x for pair in pairs)
    ey = statistics.fmean(pair[1].y for pair in pairs)
    dot = cross = 0.0
    for truth, estimate in pairs:
        a, b = estimate.x - ex, estimate.y - ey
        c, d = truth.x - gx, truth.y - gy
        dot += a * c + b * d
        cross += a * d - b * c
    yaw = math.atan2(cross, dot)
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return yaw, gx - (cosine * ex - sine * ey), gy - (sine * ex + cosine * ey), statistics.fmean(
        truth.z - estimate.z for truth, estimate in pairs
    )


def transform(pose: Pose, alignment: tuple[float, float, float, float]) -> Pose:
    yaw, tx, ty, tz = alignment
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return Pose(
        pose.timestamp,
        cosine * pose.x - sine * pose.y + tx,
        sine * pose.x + cosine * pose.y + ty,
        pose.z + tz,
        wrap(pose.yaw + yaw),
    )


def stats(values: list[float]) -> dict:
    ordered = sorted(values)
    p95 = ordered[min(len(ordered) - 1, math.ceil(0.95 * len(ordered)) - 1)]
    return {
        "count": len(values),
        "mean": statistics.fmean(values),
        "median": statistics.median(values),
        "rmse": math.sqrt(statistics.fmean(value * value for value in values)),
        "p95": p95,
        "max": max(values),
    }


def evaluate(pairs: list[tuple[Pose, Pose]], rejected: dict) -> tuple[dict, list[dict]]:
    alignment = align(pairs)
    aligned = [(truth, transform(estimate, alignment)) for truth, estimate in pairs]
    ate_translation = [
        math.dist((truth.x, truth.y, truth.z), (estimate.x, estimate.y, estimate.z))
        for truth, estimate in aligned
    ]
    ate_rotation = [abs(wrap(estimate.yaw - truth.yaw)) for truth, estimate in aligned]
    rpe_translation = []
    rpe_rotation = []
    series = []
    for index, ((truth, estimate), translation, rotation) in enumerate(zip(aligned, ate_translation, ate_rotation)):
        row = {
            "timestamp": truth.timestamp,
            "ate_translation_m": translation,
            "ate_rotation_rad": rotation,
            "rpe_translation_m": None,
            "rpe_rotation_rad": None,
        }
        if index:
            previous_truth, previous_estimate = aligned[index - 1]
            truth_delta = (truth.x - previous_truth.x, truth.y - previous_truth.y, truth.z - previous_truth.z)
            estimate_delta = (
                estimate.x - previous_estimate.x,
                estimate.y - previous_estimate.y,
                estimate.z - previous_estimate.z,
            )
            row["rpe_translation_m"] = math.dist(truth_delta, estimate_delta)
            row["rpe_rotation_rad"] = abs(
                wrap((estimate.yaw - previous_estimate.yaw) - (truth.yaw - previous_truth.yaw))
            )
            rpe_translation.append(row["rpe_translation_m"])
            rpe_rotation.append(row["rpe_rotation_rad"])
        series.append(row)
    report = {
        "result": "PASS",
        "sample_count": len(aligned),
        "rejected_samples": rejected,
        "alignment": {"yaw_rad": alignment[0], "translation_m": list(alignment[1:])},
        "ate": {"translation_m": stats(ate_translation), "rotation_rad": stats(ate_rotation)},
        "rpe": {"translation_m": stats(rpe_translation), "rotation_rad": stats(rpe_rotation)},
        "limitations": [
            "Evaluation is planar-yaw aligned; roll and pitch are not scored.",
            "Simulation-derived odometry re-expresses Isaac ground-truth motion and is not independent physical odometry.",
        ],
    }
    return report, series


def write_outputs(directory: Path, report: dict, series: list[dict]) -> None:
    directory.mkdir(parents=True, exist_ok=False)
    (directory / "trajectory_error.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    with (directory / "trajectory_error_series.csv").open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(series[0]))
        writer.writeheader()
        writer.writerows(series)
    lines = [
        "# ATE/RPE Report",
        "",
        f"- Result: `{report['result']}`",
        f"- Synchronized samples: `{report['sample_count']}`",
        f"- ATE translation RMSE: `{report['ate']['translation_m']['rmse']:.6f} m`",
        f"- ATE rotation RMSE: `{report['ate']['rotation_rad']['rmse']:.6f} rad`",
        f"- RPE translation RMSE: `{report['rpe']['translation_m']['rmse']:.6f} m`",
        f"- RPE rotation RMSE: `{report['rpe']['rotation_rad']['rmse']:.6f} rad`",
        "",
        "Simulation-derived odometry is not an independent physical measurement.",
    ]
    (directory / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ground-truth", required=True)
    parser.add_argument("--estimate", required=True)
    parser.add_argument("--trajectory-id", required=True)
    parser.add_argument("--timestamp-tolerance", type=float, default=0.05)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()
    if args.timestamp_tolerance < 0:
        raise ValueError("timestamp tolerance must be non-negative")
    pairs, rejected = match_poses(
        load_poses(Path(args.ground_truth)),
        load_poses(Path(args.estimate)),
        args.timestamp_tolerance,
    )
    report, series = evaluate(pairs, rejected)
    report["trajectory_id"] = args.trajectory_id
    report["timestamp_tolerance_sec"] = args.timestamp_tolerance
    write_outputs(Path(args.output_dir), report, series)
    print(f"Overall result: {report['result']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

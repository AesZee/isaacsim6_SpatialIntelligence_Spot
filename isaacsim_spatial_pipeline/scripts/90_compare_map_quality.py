#!/usr/bin/env python3
"""Compare saved Milestone #8/#9 map quality artifacts."""

from __future__ import annotations

import argparse
import csv
import glob
import json
from dataclasses import dataclass
from pathlib import Path
from tempfile import TemporaryDirectory

import yaml


PROFILE_CONFIG = Path(__file__).resolve().parents[1] / "config" / "m09_lidar_slice_profiles.yaml"
PARAMETER_NAMES = ("min_height", "max_height", "range_min", "range_max", "angle_min", "angle_max")


@dataclass
class MapMetrics:
    directory: Path
    width: int
    height: int
    resolution: float
    total_cells: int
    unknown_ratio: float
    known_ratio: float
    free_ratio: float
    occupied_ratio: float
    occupied_cells: int
    known_area_m2: float
    occupied_area_m2: float

    @property
    def score(self) -> float:
        score = self.known_ratio * 100.0
        score += min(self.occupied_cells, 1000) / 1000.0
        score += self.occupied_ratio * 20.0
        if self.unknown_ratio > 0.98:
            score -= (self.unknown_ratio - 0.98) * 100.0
        if self.occupied_cells == 0:
            score -= 1.0
        return score

    @property
    def label(self) -> str:
        if self.total_cells <= 0 or self.width <= 0 or self.height <= 0:
            return "FAIL"
        if self.known_ratio >= 0.05 and self.occupied_ratio > 0.001 and self.unknown_ratio < 0.98:
            return "PASS"
        return "WARN"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Compare saved map quality metrics.")
    parser.add_argument("map_directories", nargs="*")
    parser.add_argument(
        "--experiment",
        action="append",
        default=[],
        metavar="[LABEL:]PROFILE=MAP_DIRECTORY",
        help="Associate a saved map with a named profile; repeat for each experiment.",
    )
    parser.add_argument("--profile-config", default=str(PROFILE_CONFIG))
    parser.add_argument("--json-output")
    parser.add_argument("--csv-output")
    parser.add_argument("--self-check", action="store_true")
    return parser.parse_args()


def expand_inputs(inputs: list[str]) -> list[Path]:
    paths: list[Path] = []
    for item in inputs:
        matches = glob.glob(item)
        if matches:
            paths.extend(Path(match) for match in matches)
        else:
            paths.append(Path(item))
    return sorted({path.resolve() for path in paths})


def load_json(path: Path) -> dict | None:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None


def load_yaml_minimal(path: Path) -> dict:
    payload = {}
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError:
        return payload
    for line in lines:
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or ":" not in stripped:
            continue
        key, value = stripped.split(":", 1)
        payload[key.strip()] = value.strip().strip("'\"")
    return payload


def read_pgm(path: Path) -> tuple[int, int, list[int]] | None:
    try:
        with path.open("rb") as stream:
            magic = stream.readline().strip()
            if magic not in {b"P5", b"P2"}:
                return None
            line = stream.readline().strip()
            while line.startswith(b"#"):
                line = stream.readline().strip()
            width_raw, height_raw = line.split()[:2]
            width = int(width_raw)
            height = int(height_raw)
            max_value = int(stream.readline().strip())
            if width <= 0 or height <= 0 or max_value <= 0:
                return None
            if magic == b"P5":
                data = list(stream.read(width * height))
            else:
                data = [int(value) for value in stream.read().split()]
            return width, height, data[: width * height]
    except (OSError, ValueError):
        return None


def metrics_from_json(directory: Path) -> MapMetrics | None:
    metadata = load_json(directory / "map_metadata.json")
    stats = load_json(directory / "map_stats.json")
    if not metadata or not stats:
        return None
    try:
        width = int(metadata["width"])
        height = int(metadata["height"])
        resolution = float(metadata["resolution"])
        total_cells = int(stats["total_cells"])
        unknown_ratio = float(stats["unknown_ratio"])
        free_ratio = float(stats["free_ratio"])
        occupied_ratio = float(stats["occupied_ratio"])
        known_ratio = float(stats.get("known_ratio", 1.0 - unknown_ratio))
        occupied_cells = int(stats["occupied_cells"])
        known_area_m2 = float(stats.get("known_area_m2", known_ratio * total_cells * resolution * resolution))
        occupied_area_m2 = float(stats.get("occupied_area_m2", occupied_cells * resolution * resolution))
    except (KeyError, TypeError, ValueError):
        return None
    return MapMetrics(
        directory=directory,
        width=width,
        height=height,
        resolution=resolution,
        total_cells=total_cells,
        unknown_ratio=unknown_ratio,
        known_ratio=known_ratio,
        free_ratio=free_ratio,
        occupied_ratio=occupied_ratio,
        occupied_cells=occupied_cells,
        known_area_m2=known_area_m2,
        occupied_area_m2=occupied_area_m2,
    )


def metrics_from_pgm(directory: Path) -> MapMetrics | None:
    yaml_payload = load_yaml_minimal(directory / "map.yaml")
    image_name = yaml_payload.get("image", "map.pgm")
    pgm = read_pgm(directory / image_name)
    if not pgm:
        return None
    width, height, data = pgm
    total = len(data)
    if total == 0:
        return None
    resolution = float(yaml_payload.get("resolution", 0.0) or 0.0)
    unknown = sum(1 for value in data if 190 <= value <= 220)
    free = sum(1 for value in data if value >= 250)
    occupied = sum(1 for value in data if value <= 20)
    known = free + occupied
    cell_area = resolution * resolution
    return MapMetrics(
        directory=directory,
        width=width,
        height=height,
        resolution=resolution,
        total_cells=total,
        unknown_ratio=unknown / total,
        known_ratio=known / total,
        free_ratio=free / total,
        occupied_ratio=occupied / total,
        occupied_cells=occupied,
        known_area_m2=known * cell_area,
        occupied_area_m2=occupied * cell_area,
    )


def load_metrics(directory: Path) -> MapMetrics | None:
    if not directory.is_dir():
        return None
    return metrics_from_json(directory) or metrics_from_pgm(directory)


def load_profiles(path: Path) -> dict[str, dict[str, float]]:
    try:
        payload = yaml.safe_load(path.read_text(encoding="utf-8"))
        profiles = payload["lidar_slice_profiles"]
        return {
            name: {parameter: float(values[parameter]) for parameter in PARAMETER_NAMES}
            for name, values in profiles.items()
        }
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as error:
        raise ValueError(f"invalid profile config {path}: {error}") from error


def parse_experiment(specification: str) -> tuple[str, str, Path]:
    try:
        descriptor, directory = specification.split("=", 1)
        label, profile = descriptor.split(":", 1) if ":" in descriptor else (descriptor, descriptor)
    except ValueError as error:
        raise ValueError(
            f"invalid experiment '{specification}'; expected [LABEL:]PROFILE=MAP_DIRECTORY"
        ) from error
    if not label or not profile or not directory:
        raise ValueError(f"invalid experiment '{specification}'; label, profile, and directory are required")
    return label, profile, Path(directory).resolve()


def metrics_dict(metric: MapMetrics) -> dict:
    return {
        "label": metric.label,
        "score": metric.score,
        "width": metric.width,
        "height": metric.height,
        "resolution": metric.resolution,
        "total_cells": metric.total_cells,
        "unknown_ratio": metric.unknown_ratio,
        "known_ratio": metric.known_ratio,
        "free_ratio": metric.free_ratio,
        "occupied_ratio": metric.occupied_ratio,
        "occupied_cells": metric.occupied_cells,
        "known_area_m2": metric.known_area_m2,
        "occupied_area_m2": metric.occupied_area_m2,
    }


def build_experiment_records(
    specifications: list[str],
    profiles: dict[str, dict[str, float]],
) -> tuple[list[dict], list[MapMetrics]]:
    records = []
    metrics = []
    for specification in specifications:
        label, profile, directory = parse_experiment(specification)
        if profile not in profiles:
            raise ValueError(f"unknown profile '{profile}'; available: {', '.join(sorted(profiles))}")
        metric = load_metrics(directory)
        if metric is None:
            raise ValueError(f"could not read map metrics from {directory}")
        metrics.append(metric)
        records.append(
            {
                "experiment": label,
                "profile": profile,
                "map_directory": str(directory),
                "parameters": profiles[profile],
                "metrics": metrics_dict(metric),
            }
        )
    return records, metrics


def write_experiment_outputs(records: list[dict], json_output: Path, csv_output: Path) -> None:
    json_output.parent.mkdir(parents=True, exist_ok=True)
    csv_output.parent.mkdir(parents=True, exist_ok=True)
    winner = max(records, key=lambda record: record["metrics"]["score"])
    payload = {
        "selected_winner": {
            "experiment": winner["experiment"],
            "profile": winner["profile"],
            "score": winner["metrics"]["score"],
            "known_ratio": winner["metrics"]["known_ratio"],
            "occupied_cells": winner["metrics"]["occupied_cells"],
        },
        "targets": {"known_ratio": 0.10, "occupied_cells": 500},
        "targets_met": (
            winner["metrics"]["known_ratio"] >= 0.10
            and winner["metrics"]["occupied_cells"] >= 500
        ),
        "experiments": records,
    }
    with json_output.open("x", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write("\n")

    metric_names = list(records[0]["metrics"])
    fieldnames = ["experiment", "profile", "map_directory", *PARAMETER_NAMES, *metric_names]
    with csv_output.open("x", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for record in records:
            writer.writerow(
                {
                    "experiment": record["experiment"],
                    "profile": record["profile"],
                    "map_directory": record["map_directory"],
                    **record["parameters"],
                    **record["metrics"],
                }
            )


def print_table(metrics: list[MapMetrics]) -> None:
    headers = [
        "rank",
        "label",
        "score",
        "directory",
        "width",
        "height",
        "res",
        "cells",
        "unknown",
        "known",
        "free",
        "occupied",
        "occ_cells",
        "known_m2",
        "occ_m2",
    ]
    rows = []
    for index, metric in enumerate(metrics, start=1):
        rows.append(
            [
                str(index),
                metric.label,
                f"{metric.score:.3f}",
                str(metric.directory),
                str(metric.width),
                str(metric.height),
                f"{metric.resolution:.3f}",
                str(metric.total_cells),
                f"{metric.unknown_ratio:.4f}",
                f"{metric.known_ratio:.4f}",
                f"{metric.free_ratio:.4f}",
                f"{metric.occupied_ratio:.6f}",
                str(metric.occupied_cells),
                f"{metric.known_area_m2:.3f}",
                f"{metric.occupied_area_m2:.3f}",
            ]
        )
    widths = [len(header) for header in headers]
    for row in rows:
        widths = [max(width, len(value)) for width, value in zip(widths, row)]
    print("  ".join(header.ljust(width) for header, width in zip(headers, widths)))
    print("  ".join("-" * width for width in widths))
    for row in rows:
        print("  ".join(value.ljust(width) for value, width in zip(row, widths)))


def self_check() -> None:
    with TemporaryDirectory() as temporary_directory:
        root = Path(temporary_directory)
        map_directory = root / "map"
        map_directory.mkdir()
        (map_directory / "map_metadata.json").write_text(
            json.dumps({"width": 10, "height": 10, "resolution": 0.05}),
            encoding="utf-8",
        )
        (map_directory / "map_stats.json").write_text(
            json.dumps(
                {
                    "total_cells": 100,
                    "unknown_ratio": 0.90,
                    "known_ratio": 0.10,
                    "free_ratio": 0.08,
                    "occupied_ratio": 0.02,
                    "occupied_cells": 2,
                    "known_area_m2": 0.025,
                    "occupied_area_m2": 0.005,
                }
            ),
            encoding="utf-8",
        )
        profiles = {"baseline": {name: float(index) for index, name in enumerate(PARAMETER_NAMES)}}
        records, metrics = build_experiment_records([f"trial:baseline={map_directory}"], profiles)
        json_output = root / "experiments.json"
        csv_output = root / "experiments.csv"
        write_experiment_outputs(records, json_output, csv_output)
        assert len(metrics) == 1
        assert records[0]["experiment"] == "trial"
        assert json.loads(json_output.read_text(encoding="utf-8"))["experiments"][0]["profile"] == "baseline"
        assert len(csv_output.read_text(encoding="utf-8").splitlines()) == 2
    print("PASS: map experiment comparison self-check")


def main() -> int:
    args = parse_args()
    if args.self_check:
        self_check()
        return 0
    if bool(args.json_output) != bool(args.csv_output):
        print("FAIL: --json-output and --csv-output must be provided together")
        return 1
    if not args.map_directories and not args.experiment:
        print("FAIL: provide map directories or at least one --experiment")
        return 1

    experiment_records = []
    experiment_metrics = []
    if args.experiment:
        try:
            profiles = load_profiles(Path(args.profile_config))
            experiment_records, experiment_metrics = build_experiment_records(args.experiment, profiles)
        except ValueError as error:
            print(f"FAIL: {error}")
            return 1

    candidates = expand_inputs(args.map_directories)
    metrics = [metric for path in candidates if (metric := load_metrics(path)) is not None]
    metrics.extend(experiment_metrics)
    metrics = list({metric.directory: metric for metric in metrics}.values())
    metrics.sort(key=lambda item: item.score, reverse=True)

    print("Milestone #9 Map Quality Comparison")
    print("Metrics are occupancy-grid statistics, not ground-truth geometric accuracy.")
    if not metrics:
        print("FAIL: no valid map directories could be read")
        return 1
    print_table(metrics)
    if args.json_output and args.csv_output:
        if not experiment_records:
            print("FAIL: structured outputs require at least one --experiment with a named parameter profile")
            return 1
        write_experiment_outputs(
            experiment_records,
            Path(args.json_output),
            Path(args.csv_output),
        )
        print(f"\nJSON experiment record: {args.json_output}")
        print(f"CSV experiment record: {args.csv_output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

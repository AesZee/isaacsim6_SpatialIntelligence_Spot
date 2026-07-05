#!/usr/bin/env python3
"""Compare saved Milestone #8/#9 map quality artifacts."""

from __future__ import annotations

import argparse
import glob
import json
from dataclasses import dataclass
from pathlib import Path


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
    parser.add_argument("map_directories", nargs="+")
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


def main() -> int:
    args = parse_args()
    candidates = expand_inputs(args.map_directories)
    metrics = [metric for path in candidates if (metric := load_metrics(path)) is not None]
    metrics.sort(key=lambda item: item.score, reverse=True)

    print("Milestone #9 Map Quality Comparison")
    print("Metrics are occupancy-grid statistics, not ground-truth geometric accuracy.")
    if not metrics:
        print("FAIL: no valid map directories could be read")
        return 1
    print_table(metrics)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

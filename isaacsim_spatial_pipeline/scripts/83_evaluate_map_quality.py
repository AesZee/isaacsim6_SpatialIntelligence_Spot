#!/usr/bin/env python3
"""Evaluate saved Milestone #8 map quality metrics without ROS runtime."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Evaluate saved Milestone #8 map quality.")
    parser.add_argument("map_directory")
    return parser.parse_args()


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def fail(message: str) -> int:
    print("Milestone #8 Map Quality Evaluation")
    print(f"FAIL: {message}")
    return 1


def main() -> int:
    args = parse_args()
    directory = Path(args.map_directory)
    metadata_path = directory / "map_metadata.json"
    stats_path = directory / "map_stats.json"

    if not directory.is_dir():
        return fail(f"missing directory: {directory}")
    if not metadata_path.exists():
        return fail(f"missing {metadata_path}")
    if not stats_path.exists():
        return fail(f"missing {stats_path}")

    try:
        metadata = load_json(metadata_path)
        stats = load_json(stats_path)
    except (OSError, json.JSONDecodeError) as error:
        return fail(f"invalid JSON: {error}")

    try:
        width = int(metadata["width"])
        height = int(metadata["height"])
        resolution = float(metadata["resolution"])
        total_cells = int(stats["total_cells"])
        unknown_ratio = float(stats["unknown_ratio"])
        free_ratio = float(stats["free_ratio"])
        occupied_ratio = float(stats["occupied_ratio"])
        map_area_m2 = float(stats["map_area_m2"])
        known_area_m2 = float(stats["known_area_m2"])
    except (KeyError, TypeError, ValueError) as error:
        return fail(f"missing or invalid metric: {error}")

    if width <= 0 or height <= 0 or resolution <= 0.0 or total_cells <= 0:
        return fail(
            f"invalid dimensions: width={width}, height={height}, resolution={resolution}, total_cells={total_cells}"
        )

    known_ratio = 1.0 - unknown_ratio
    width_m = width * resolution
    height_m = height * resolution

    suggestions = []
    if unknown_ratio >= 0.98:
        suggestions.append("Map is mostly unknown: move the robot more, increase exploration coverage, and wait longer before saving.")
    if occupied_ratio <= 0.001:
        suggestions.append("No or very few occupied cells: check LaserScan range, frame transform, slam_toolbox scan matching, and occupancy thresholds.")
    if known_ratio < 0.05:
        suggestions.append("Known coverage is low: move through a longer trajectory before saving.")
    if map_area_m2 < 1.0:
        suggestions.append("Map area is very small: move robot through a longer trajectory.")

    if known_ratio >= 0.05 and occupied_ratio > 0.001 and unknown_ratio < 0.98:
        verdict = "PASS"
    else:
        verdict = "WARN"

    print("Milestone #8 Map Quality Evaluation")
    print(f"Map directory: {directory}")
    print(f"Map dimensions: {width} x {height} cells")
    print(f"Map dimensions: {width_m:.3f} x {height_m:.3f} m")
    print(f"Resolution: {resolution:.6f} m/cell")
    print(f"Total area: {map_area_m2:.3f} m^2")
    print(f"Known area: {known_area_m2:.3f} m^2")
    print(f"Known ratio: {known_ratio:.6f}")
    print(f"Unknown ratio: {unknown_ratio:.6f}")
    print(f"Free ratio: {free_ratio:.6f}")
    print(f"Occupied ratio: {occupied_ratio:.6f}")
    print(f"\nOverall result: {verdict}")

    if suggestions:
        print("\nSuggestions")
        for suggestion in suggestions:
            print(f"  - {suggestion}")
    else:
        print("\nSuggestions")
        print("  - Map quality is sufficient for lightweight portfolio documentation.")

    return 1 if verdict == "FAIL" else 0


if __name__ == "__main__":
    raise SystemExit(main())

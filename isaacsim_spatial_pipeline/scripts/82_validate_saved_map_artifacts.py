#!/usr/bin/env python3
"""Validate Milestone #8 saved map artifact directories."""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from enum import IntEnum
from pathlib import Path


class Level(IntEnum):
    PASS = 0
    WARN = 1
    FAIL = 2


@dataclass
class CheckResult:
    level: Level
    label: str
    detail: str


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Validate saved Milestone #8 map artifacts.")
    parser.add_argument("map_directory")
    return parser.parse_args()


def load_yaml(path: Path) -> tuple[dict, str | None]:
    try:
        import yaml  # type: ignore

        with path.open("r", encoding="utf-8") as stream:
            payload = yaml.safe_load(stream) or {}
        return payload, None
    except ImportError:
        payload = {}
        for line in path.read_text(encoding="utf-8").splitlines():
            stripped = line.strip()
            if not stripped or stripped.startswith("#") or ":" not in stripped:
                continue
            key, value = stripped.split(":", 1)
            payload[key.strip()] = value.strip().strip("'\"")
        return payload, "PyYAML unavailable; used minimal key-value parser"


def load_json(path: Path) -> dict | None:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None


def validate_pgm(path: Path) -> CheckResult:
    try:
        with path.open("rb") as stream:
            magic = stream.readline().strip()
            if magic not in {b"P5", b"P2"}:
                return CheckResult(Level.FAIL, "PGM header", f"unsupported magic {magic!r}")
            dimensions = stream.readline().strip()
            while dimensions.startswith(b"#"):
                dimensions = stream.readline().strip()
            parts = dimensions.split()
            if len(parts) != 2:
                return CheckResult(Level.FAIL, "PGM dimensions", f"invalid dimensions line {dimensions!r}")
            width = int(parts[0])
            height = int(parts[1])
            max_value = int(stream.readline().strip())
            if width <= 0 or height <= 0 or max_value <= 0:
                return CheckResult(Level.FAIL, "PGM metadata", f"width={width}, height={height}, max={max_value}")
            return CheckResult(Level.PASS, "PGM header", f"{magic.decode()} {width}x{height} max={max_value}")
    except (OSError, ValueError) as error:
        return CheckResult(Level.FAIL, "PGM header", str(error))


def check_required_files(directory: Path) -> tuple[list[CheckResult], dict[str, Path]]:
    paths = {
        "map.yaml": directory / "map.yaml",
        "map.pgm": directory / "map.pgm",
        "map_metadata.json": directory / "map_metadata.json",
        "map_stats.json": directory / "map_stats.json",
    }
    results = []
    for label, path in paths.items():
        results.append(CheckResult(Level.PASS if path.exists() else Level.FAIL, label, str(path)))
    return results, paths


def validate_metadata(payload: dict | None) -> list[CheckResult]:
    if payload is None:
        return [CheckResult(Level.FAIL, "metadata JSON", "could not parse JSON")]
    required = ["resolution", "width", "height", "origin", "frame_id"]
    results = []
    for key in required:
        results.append(CheckResult(Level.PASS if key in payload else Level.FAIL, f"metadata.{key}", "present" if key in payload else "missing"))
    return results


def validate_stats(payload: dict | None, metadata: dict | None) -> list[CheckResult]:
    if payload is None:
        return [CheckResult(Level.FAIL, "stats JSON", "could not parse JSON")]
    required = [
        "total_cells",
        "known_cells",
        "unknown_cells",
        "free_cells",
        "occupied_cells",
        "known_ratio",
        "unknown_ratio",
        "free_ratio",
        "occupied_ratio",
    ]
    results = []
    for key in required:
        results.append(CheckResult(Level.PASS if key in payload else Level.FAIL, f"stats.{key}", "present" if key in payload else "missing"))

    if metadata and "width" in metadata and "height" in metadata and "total_cells" in payload:
        expected = int(metadata["width"]) * int(metadata["height"])
        observed = int(payload["total_cells"])
        results.append(
            CheckResult(
                Level.PASS if expected == observed else Level.FAIL,
                "width*height",
                f"{expected} expected, {observed} total_cells",
            )
        )

    for key in ["known_ratio", "unknown_ratio", "free_ratio", "occupied_ratio"]:
        if key in payload:
            value = float(payload[key])
            results.append(
                CheckResult(
                    Level.PASS if 0.0 <= value <= 1.0 else Level.FAIL,
                    key,
                    f"{value:.6f}",
                )
            )
    return results


def print_results(results: list[CheckResult]) -> Level:
    worst = Level.PASS
    for result in results:
        worst = max(worst, result.level)
        print(f"  {result.level.name:<4} {result.label}: {result.detail}")
    return worst


def main() -> int:
    args = parse_args()
    directory = Path(args.map_directory)
    results = [CheckResult(Level.PASS if directory.is_dir() else Level.FAIL, "directory", str(directory))]
    if not directory.is_dir():
        print("Milestone #8 Saved Map Artifact Validation")
        worst = print_results(results)
        print(f"\nOverall result: {worst.name}")
        return 1

    file_results, paths = check_required_files(directory)
    results.extend(file_results)

    yaml_payload = {}
    yaml_warning = None
    if paths["map.yaml"].exists():
        yaml_payload, yaml_warning = load_yaml(paths["map.yaml"])
        results.append(CheckResult(Level.PASS, "YAML parse", "map.yaml parsed"))
        if yaml_warning:
            results.append(CheckResult(Level.WARN, "YAML parser", yaml_warning))
        image = yaml_payload.get("image")
        image_path = directory / str(image) if image else None
        results.append(
            CheckResult(
                Level.PASS if image_path and image_path.exists() else Level.FAIL,
                "YAML image",
                str(image_path) if image_path else "missing image key",
            )
        )

    if paths["map.pgm"].exists():
        results.append(validate_pgm(paths["map.pgm"]))

    metadata = load_json(paths["map_metadata.json"]) if paths["map_metadata.json"].exists() else None
    stats = load_json(paths["map_stats.json"]) if paths["map_stats.json"].exists() else None
    results.extend(validate_metadata(metadata))
    results.extend(validate_stats(stats, metadata))

    print("Milestone #8 Saved Map Artifact Validation")
    worst = print_results(results)
    print(f"\nOverall result: {worst.name}")
    return 1 if worst == Level.FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())

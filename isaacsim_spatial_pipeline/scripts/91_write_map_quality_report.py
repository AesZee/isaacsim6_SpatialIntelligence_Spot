#!/usr/bin/env python3
"""Write a Markdown report comparing saved Milestone #9 map artifacts."""

from __future__ import annotations

import argparse
import importlib.util
import sys
from datetime import datetime
from pathlib import Path


COMPARE_SCRIPT = Path(__file__).with_name("90_compare_map_quality.py")


def load_compare_module():
    spec = importlib.util.spec_from_file_location("m09_compare_map_quality", COMPARE_SCRIPT)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load {COMPARE_SCRIPT}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Write Milestone #9 map quality Markdown report.")
    parser.add_argument("--output", required=True)
    parser.add_argument("map_directories", nargs="+")
    return parser.parse_args()


def markdown_table(metrics) -> str:
    headers = [
        "Rank",
        "Label",
        "Score",
        "Directory",
        "Cells",
        "Resolution",
        "Known Ratio",
        "Unknown Ratio",
        "Occupied Ratio",
        "Occupied Cells",
        "Known m2",
        "Occupied m2",
    ]
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    for index, metric in enumerate(metrics, start=1):
        values = [
            str(index),
            metric.label,
            f"{metric.score:.3f}",
            str(metric.directory),
            str(metric.total_cells),
            f"{metric.resolution:.3f}",
            f"{metric.known_ratio:.4f}",
            f"{metric.unknown_ratio:.4f}",
            f"{metric.occupied_ratio:.6f}",
            str(metric.occupied_cells),
            f"{metric.known_area_m2:.3f}",
            f"{metric.occupied_area_m2:.3f}",
        ]
        lines.append("| " + " | ".join(values) + " |")
    return "\n".join(lines)


def build_report(metrics, inputs: list[str]) -> str:
    timestamp = datetime.now().isoformat(timespec="seconds")
    best = metrics[0] if metrics else None
    lines = [
        "# Milestone #9 Map Quality Report",
        "",
        f"Generated: {timestamp}",
        "",
        "## Scope",
        "",
        "This report compares saved occupancy-grid map artifacts from Milestone #8/#9 runs.",
        "Metrics are occupancy-grid statistics only, not ground-truth geometric accuracy.",
        "",
        "## Inputs",
        "",
    ]
    for item in inputs:
        lines.append(f"- `{item}`")
    lines.extend(["", "## Comparison", ""])
    if metrics:
        lines.append(markdown_table(metrics))
        lines.extend(
            [
                "",
                "## Best Map By Heuristic",
                "",
                f"- Directory: `{best.directory}`",
                f"- Score: `{best.score:.3f}`",
                f"- Label: `{best.label}`",
                f"- Known ratio: `{best.known_ratio:.4f}`",
                f"- Occupied ratio: `{best.occupied_ratio:.6f}`",
                f"- Occupied cells: `{best.occupied_cells}`",
            ]
        )
    else:
        lines.append("No valid map directories were readable.")

    lines.extend(
        [
            "",
            "## Notes",
            "",
            "- The ranking prefers higher known ratio and non-trivial occupied cells.",
            "- The ranking penalizes very high unknown ratio and zero occupied cells.",
            "- A high score does not prove metric accuracy or loop-closure correctness.",
            "",
            "## Experiment Placeholders",
            "",
            "Fill these in manually for each saved map used in portfolio documentation.",
            "",
            "```text",
            "Selected LiDAR slice profile:",
            "Trajectory notes:",
            "RViz screenshot path:",
            "Isaac Sim scene notes:",
            "Reason this map was selected:",
            "Known issues:",
            "```",
            "",
        ]
    )
    return "\n".join(lines)


def main() -> int:
    args = parse_args()
    compare = load_compare_module()
    candidates = compare.expand_inputs(args.map_directories)
    metrics = [metric for path in candidates if (metric := compare.load_metrics(path)) is not None]
    metrics.sort(key=lambda item: item.score, reverse=True)

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(build_report(metrics, args.map_directories), encoding="utf-8")

    print(f"Wrote Milestone #9 map quality report: {output_path}")
    if not metrics:
        print("WARN: no valid map directories were readable")
        return 1
    print(f"Best map: {metrics[0].directory}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

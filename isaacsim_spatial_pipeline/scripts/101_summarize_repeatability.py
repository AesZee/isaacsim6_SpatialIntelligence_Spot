#!/usr/bin/env python3
"""Summarize repeated autonomous-run manifests without inventing missing data."""

import argparse
import csv
import json
import statistics
from pathlib import Path


METRICS = (
    "map_width_cells",
    "map_height_cells",
    "known_ratio",
    "occupied_cells",
    "duration_sec",
    "scan_frequency_hz",
)


def rows_from_manifest(path: Path) -> list[dict]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    rows = []
    for experiment in payload.get("experiments", []):
        stats = experiment.get("map_stats") or {}
        metadata = experiment.get("map_metadata") or {}
        scan = ((experiment.get("scan_metrics") or {}).get("laserscan") or {})
        rows.append(
            {
                "run_id": payload.get("run_id"),
                "experiment": experiment.get("name"),
                "profile": experiment.get("profile"),
                "trajectory": experiment.get("trajectory"),
                "result": experiment.get("result"),
                "motion_complete": bool(experiment.get("motion_complete")),
                "map_width_cells": metadata.get("width"),
                "map_height_cells": metadata.get("height"),
                "known_ratio": stats.get("known_ratio"),
                "occupied_cells": stats.get("occupied_cells"),
                "duration_sec": experiment.get("motion_duration_wall_sec"),
                "scan_frequency_hz": scan.get("frequency_hz"),
            }
        )
    return rows


def summarize(rows: list[dict]) -> dict:
    declared_inputs = {
        json.dumps({"profile": row["profile"], "trajectory": row["trajectory"]}, sort_keys=True)
        for row in rows
    }
    summary = {
        "result": (
            "PASS"
            if (
                len(rows) >= 3
                and len(declared_inputs) == 1
                and all(row["motion_complete"] for row in rows)
                and all(row["result"] in {"PASS", "WARN"} for row in rows)
                and all(row[name] is not None for row in rows for name in METRICS)
            )
            else "FAIL"
        ),
        "run_count": len(rows),
        "identical_declared_inputs": len(declared_inputs) == 1,
        "metrics": {},
    }
    for name in METRICS:
        values = [float(row[name]) for row in rows if row[name] is not None]
        summary["metrics"][name] = (
            {
                "count": len(values),
                "min": min(values),
                "mean": statistics.fmean(values),
                "max": max(values),
                "stdev": statistics.stdev(values) if len(values) > 1 else 0.0,
                "coefficient_of_variation": (
                    statistics.stdev(values) / abs(statistics.fmean(values))
                    if len(values) > 1 and statistics.fmean(values)
                    else 0.0
                ),
            }
            if values
            else None
        )
    summary["major_variance"] = any(
        summary["metrics"][name]
        and summary["metrics"][name]["coefficient_of_variation"] > 0.20
        for name in ("known_ratio", "occupied_cells", "duration_sec", "scan_frequency_hz")
    )
    if summary["result"] == "PASS" and summary["major_variance"]:
        summary["result"] = "WARN"
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directories", nargs="+")
    parser.add_argument("--json-output", required=True)
    parser.add_argument("--csv-output", required=True)
    args = parser.parse_args()
    rows = []
    for directory in args.run_directories:
        rows.extend(rows_from_manifest(Path(directory) / "manifest.json"))
    summary = summarize(rows)
    with Path(args.json_output).open("x", encoding="utf-8") as stream:
        json.dump({"summary": summary, "runs": rows}, stream, indent=2, sort_keys=True)
        stream.write("\n")
    with Path(args.csv_output).open("x", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=["run_id", "experiment", "profile", "result", "motion_complete", *METRICS],
        )
        writer.writeheader()
        writer.writerows({key: value for key, value in row.items() if key != "trajectory"} for row in rows)
    print(f"Overall result: {summary['result']}")
    return 0 if summary["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())

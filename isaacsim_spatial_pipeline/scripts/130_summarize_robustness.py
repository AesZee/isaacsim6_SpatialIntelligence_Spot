#!/usr/bin/env python3
"""Compare bounded robustness manifests with declared expectations."""

import argparse
import csv
import json
from datetime import datetime
from pathlib import Path


def expectation_met(row: dict) -> bool:
    expected = row["expected"]
    operational_pass = (
        row["actual"] in {"PASS", "WARN"}
        and row["motion_complete"]
        and row["final_validation"] in {"PASS", "WARN"}
        and row["cleanup_confirmed"]
    )
    if expected in {"PASS", "WARN_OR_PASS", "DELAYED_THEN_PASS"}:
        return operational_pass
    if expected == "DETECTED":
        return (
            row["actual"] == "FAIL"
            and row["cleanup_confirmed"]
            and "required topic or TF failed" in (row["failure"] or "")
        )
    return False


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario-config", required=True)
    parser.add_argument("--run", action="append", required=True)
    parser.add_argument("--json-output", required=True)
    parser.add_argument("--csv-output", required=True)
    args = parser.parse_args()
    scenarios = json.loads(Path(args.scenario_config).read_text(encoding="utf-8"))["scenarios"]
    rows = []
    for directory in args.run:
        manifest = json.loads((Path(directory) / "manifest.json").read_text(encoding="utf-8"))
        for experiment in manifest.get("experiments", []):
            name = experiment["name"]
            if name not in scenarios:
                continue
            preflight = next(
                (
                    step
                    for step in manifest.get("steps", [])
                    if step["name"] == f"{name}_preflight_validator"
                ),
                {},
            )
            row = {
                "scenario": name,
                "expected": scenarios[name]["expected"],
                "actual": experiment.get("result"),
                "motion_complete": bool(experiment.get("motion_complete")),
                "final_validation": experiment.get("final_validation"),
                "failure": experiment.get("failure") or manifest.get("failure"),
                "cleanup_confirmed": bool(manifest.get("cleanup_confirmed")),
                "source_usd_unchanged": bool(manifest.get("source_usd", {}).get("unchanged")),
                "scan_frequency_hz": (
                    ((experiment.get("scan_metrics") or {}).get("laserscan") or {}).get("frequency_hz")
                ),
                "preflight_duration_sec": preflight.get("duration_sec"),
                "run_duration_sec": (
                    datetime.fromisoformat(manifest["finished_at"])
                    - datetime.fromisoformat(manifest["started_at"])
                ).total_seconds(),
            }
            row["expected_met"] = expectation_met(row) and row["source_usd_unchanged"]
            rows.append(row)
    observed = {row["scenario"] for row in rows}
    result = "PASS" if observed == set(scenarios) and all(row["expected_met"] for row in rows) else "FAIL"
    payload = {"result": result, "declared_scenarios": list(scenarios), "observed_scenarios": sorted(observed), "rows": rows}
    with Path(args.json_output).open("x", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write("\n")
    with Path(args.csv_output).open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(f"Overall result: {result}")
    return 0 if result == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())

# Milestone 10: Repeatable Trajectory Experiments

## Status

`COMPLETE — PASS (map quality remains a separate Milestone 9 WARN).`

Trajectories are data in `config/trajectories.json`: `short_smoke`,
`warehouse_mapping_loop`, and `repeatability_near_structure`. The runtime
loads the selected waypoint list and validates every waypoint against its
declared relative permitted area before motion.

`config/m10_repeatability.yaml` declares three experiments with the same
trajectory and LiDAR profile. Each gets a distinct immutable experiment
directory. Summarize a completed batch into a separate no-overwrite directory:

```bash
python3 scripts/101_summarize_repeatability.py \
  artifacts/m10_repeatability/<run_id> \
  --json-output artifacts/m10_repeatability/<summary_id>/repeatability.json \
  --csv-output artifacts/m10_repeatability/<summary_id>/repeatability.csv
```

The summary reports identical-input status plus min/mean/max/sample standard
deviation and coefficient of variation for dimensions, known ratio, occupied
cells, duration, and scan frequency. It returns `FAIL` when fewer than three
completed runs are present.

## Observed Result

Batch `run_20260725T155013_102304_7bb8f0b1` completed three declared
`wide_diagnostic` plus `warehouse_mapping_loop` experiments. All trajectories
returned to `[0, 0]` after 43.35 simulated seconds, all final contract
validators passed, all genuine maps passed artifact validation, child cleanup
was confirmed, and the source USD hash was unchanged.

| Metric | Minimum | Mean | Maximum | Coefficient of variation |
| --- | ---: | ---: | ---: | ---: |
| map width (cells) | 641 | 641 | 641 | 0 |
| map height (cells) | 659 | 659 | 659 | 0 |
| known ratio | 0.022935 | 0.023150 | 0.023339 | 0.00880 |
| occupied cells | 276 | 279.33 | 281 | 0.01033 |
| wall duration (s) | 227.755 | 228.855 | 229.593 | 0.00424 |
| scan frequency (Hz) | 1.92294 | 1.93429 | 1.94446 | 0.00559 |

The statistical summary reports identical inputs, three complete runs, no
major variance, and `PASS`. This configuration is the reproducible simulation
baseline. Its absolute map quality remains `WARN` because it does not meet the
unchanged 0.10 known-ratio and 500 occupied-cell targets.

Evidence:

- `artifacts/m10_repeatability/run_20260725T155013_102304_7bb8f0b1`
- `artifacts/m10_repeatability/summary_20260725T1605/repeatability.json`
- `artifacts/m10_repeatability/summary_20260725T1605/repeatability.csv`

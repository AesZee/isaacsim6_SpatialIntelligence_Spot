# Milestone 10: Repeatable Trajectory Experiments

## Status

`BLOCKED — static implementation complete; three live runs require a usable NVIDIA GPU.`

Trajectories are data in `config/trajectories.json`: `short_smoke`,
`warehouse_mapping_loop`, and `repeatability_near_structure`. The runtime
loads the selected waypoint list and validates every waypoint against its
declared relative permitted area before motion.

`config/m10_repeatability.yaml` declares three experiments with the same
trajectory and LiDAR profile. Each gets a distinct immutable output directory.
After a live batch, summarize it with:

```bash
python3 scripts/101_summarize_repeatability.py \
  artifacts/m10_repeatability/<run_id> \
  --json-output artifacts/m10_repeatability/<run_id>/repeatability.json \
  --csv-output artifacts/m10_repeatability/<run_id>/repeatability.csv
```

The summary reports identical-input status plus min/mean/max/sample standard
deviation for known ratio, occupied cells, duration, and scan frequency. It
returns `FAIL` when fewer than three completed runs are present.

Current runtime blocker: `nvidia-smi -L` returns code 9, so no repeatability
measurements or baseline claim exists.

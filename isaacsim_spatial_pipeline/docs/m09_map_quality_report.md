# Milestone #9 Map Quality Report

Generated: 2026-07-05T12:07:55

## Scope

This report compares saved occupancy-grid map artifacts from Milestone #8/#9 runs.
Metrics are occupancy-grid statistics only, not ground-truth geometric accuracy.

## Inputs

- `m09_baseline_m08_20260705_120240`
- `m09_medium_structure_20260705_120610`

## Comparison

| Rank | Label | Score | Directory | Cells | Resolution | Known Ratio | Unknown Ratio | Occupied Ratio | Occupied Cells | Known m2 | Occupied m2 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | WARN | -1.600 | m09_medium_structure_20260705_120610 | 283910 | 0.050 | 0.0070 | 0.9930 | 0.000000 | 0 | 4.970 | 0.000 |
| 2 | WARN | -2.150 | m09_baseline_m08_20260705_120240 | 235690 | 0.050 | 0.0043 | 0.9957 | 0.000000 | 0 | 2.505 | 0.000 |

## Best Map By Heuristic

- Directory: `m09_medium_structure_20260705_120610`
- Score: `-1.600`
- Label: `WARN`
- Known ratio: `0.0070`
- Occupied ratio: `0.000000`
- Occupied cells: `0`

## Notes

- The ranking prefers higher known ratio and non-trivial occupied cells.
- The ranking penalizes very high unknown ratio and zero occupied cells.
- A high score does not prove metric accuracy or loop-closure correctness.

## Experiment Placeholders

Fill these in manually for each saved map used in portfolio documentation.

```text
Selected LiDAR slice profile:
Trajectory notes:
RViz screenshot path:
Isaac Sim scene notes:
Reason this map was selected:
Known issues:
```

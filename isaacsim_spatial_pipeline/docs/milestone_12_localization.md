# Milestone 12: Saved-Map Localization and Relocalization

## Status

`COMPLETE — WARN (localization works at selected starts but is not repeatably robust).`

## Backend and Immutable Input

The installed `slam_toolbox` localization executable is used separately from
mapping mode; unavailable Nav2/AMCL packages were neither installed nor
emulated. Mapping run `run_20260725T162421_157320_5c77b261` serialized a
genuine posegraph after the map and topic/TF gates passed:

```text
map.posegraph: 3bfc52338d4f091894b0c34db28b6d9dcf9e2e7bd3bbcd54a4d0c8257b2bbc03
map.data:      f3cb14ad1df5936175ce78b37b6014f030cc865e33065b7d6b8acb7c251bb540
```

Every localization run hashed both inputs before and after execution. All
hashes matched, proving localization mode did not modify the saved map input.
The source USD hash also remained unchanged.

`config/m12_localization.yaml` launches
`localization_slam_toolbox_node` with the saved posegraph, the validated
`wide_diagnostic` LaserScan profile, simulation odometry, and the existing
topic/TF aliases. Runtime-only start offsets are bounded to 1 m and pi radians.

## Quantitative Criteria

An eight-second stationary observation must contain at least five
`map -> odom` samples, produce its first sample within five seconds, advance
simulation time, drift by at most 0.10 m and 0.10 rad, and finish within
0.25 m and 0.25 rad of the original-pose map-frame reference.

The posegraph's map frame is not identity-aligned with odom. Original-pose
observation established the fixed coordinate reference:

```text
map -> odom: x=11.901681 m, y=7.698127 m, yaw=-0.223336 rad
```

This reference changes the expected coordinate origin, not the thresholds.

## Observed Results

| Case | Runtime start / initial guess | Translation residual | Yaw residual | Result |
| --- | --- | ---: | ---: | --- |
| Original pose | `[0,0,0]` / `[0,0,0]` | 0.000 m | 0.000 rad | PASS |
| East start | `[0.5,0,0]` / `[0.5,0,0]` | 1.074 m | 0.105 rad | FAIL |
| Recovery attempt 1 | `[0,0.5,0.35]` / `[0.25,0.25,0.15]` | 0.164 m | 0.00691 rad | PASS |
| Recovery attempt 2 | identical inputs | 1.013 m | 0.0802 rad | FAIL |

All observations produced the first transform in 0.09 seconds or less and had
zero measured stationary drift. Successful cases completed the bounded smoke
trajectory and retained valid maps and TF. Failed cases stopped before motion,
as required. Cleanup, source USD integrity, and posegraph immutability passed
in every case.

The repeated recovery variance is material. The most likely observed cause is
ambiguous scan matching against the sparse map (about 2.3% known) rather than
process or TF instability: each selected transform was stable, but some stable
solutions were more than 1 m from the original-pose reference. No threshold
was relaxed and no failed result was hidden.

Evidence:

- `artifacts/m12_mapping/run_20260725T162421_157320_5c77b261`
- `artifacts/m12_localization/run_20260725T163303_171068_a9edd193`
- `artifacts/m12_localization/run_20260725T163556_176333_0d4c2bc4`
- `artifacts/m12_localization/run_20260725T163752_179930_50f1fa71`

## Limitations

- The occupancy/posegraph source remains structurally sparse.
- The east start did not converge within the declared error bound.
- Identical recovery inputs were not repeatable.
- Simulation-derived odometry is not independent physical odometry.
- These results do not establish physical-robot localization readiness.

# Milestone 11: Ground-Truth ATE/RPE Evaluation

## Status

`COMPLETE — PASS WITH LIMITATIONS.`

`scripts/110_evaluate_trajectory_error.py` consumes synchronized CSV pose
series with columns:

```text
timestamp,x,y,z,qx,qy,qz,qw
```

During a live bounded trajectory, record both sources using their simulation
timestamps:

```bash
python3 scripts/111_record_pose_pairs.py \
  --duration 60 \
  --trajectory-id repeatability_near_structure \
  --output-dir artifacts/m11/<run_id>/poses
```

The recorder writes `world -> body` Isaac TF as ground truth and `/odom` as the
estimated series. It fails rather than passing when either source has fewer
than two samples.

It nearest-matches timestamps within a declared tolerance, rejects duplicate
or out-of-window samples, aligns estimated poses to Isaac ground truth with a
best-fit planar rigid transform, and writes:

```text
trajectory_error.json
trajectory_error_series.csv
report.md
```

ATE/RPE translational and yaw-rotational min, mean, median, RMSE, p95, and max
statistics are recorded with sample counts and rejected-sample reasons.

RPE compares consecutive SE(2) relative transforms in each prior-pose frame,
not world-frame displacement vectors.

## Frame and Timestamp Alignment

The recorder takes `world -> body` from Isaac `/tf` as ground truth and the
pose field of `/odom` (`odom`, child `base_link`) as the estimate. The validated
static `odom -> world` and `body -> base_link` aliases are identity transforms,
so a best-fit planar rigid transform aligns the two numeric pose series.
Constant body height is removed by the fitted Z translation because the
simulation-odometry bridge is configured `planar_only`.

Both message sources use simulation timestamps. Ground-truth samples are
nearest-matched one-to-one to `/odom` within 0.05 seconds. The ground-truth
publisher is faster, so 1,911 otherwise valid ground samples were rejected to
avoid reusing an estimate; four exceeded the tolerance.

## Observed Result

Live run `run_20260725T161523_143730_b191d0b1` completed the declared
`warehouse_mapping_loop`, passed final topic/TF validation and cleanup, and
left the source USD unchanged. Capture `pose_capture_20260725T1616` persisted
2,871 Isaac TF samples and 956 simulation-odometry samples.

| Metric | RMSE | Mean | P95 | Maximum |
| --- | ---: | ---: | ---: | ---: |
| ATE translation (m) | 0.010305 | 0.001462 | 0.001020 | 0.192897 |
| ATE rotation (rad) | 0.000159 | 0.000137 | 0.000131 | 0.001813 |
| RPE translation (m) | 0.004472 | 0.000494 | 0.000437 | 0.075597 |
| RPE rotation (rad) | 0.000122 | 0.000013 | ~0 | 0.001944 |

The evaluator produced identical JSON and CSV SHA-256 hashes on a second run
over the same immutable inputs:

```text
trajectory_error.json:
93df0417d0e19ca68111bb130fec847ab6a2a0dfdb83ff188f10c17f87a3d8ad
trajectory_error_series.csv:
d1aeb9bdd94521f15d1c0b179b564a5b036be35fc7e000c9fb7cc68a804559e8
```

Evidence:

- `artifacts/m11/pose_capture_20260725T1616`
- `artifacts/m11/evaluation_20260725T1620`
- `artifacts/m11/evaluation_replay_20260725T1620`
- `artifacts/m11/run_20260725T161523_143730_b191d0b1`

The simulation odometry limitation is fundamental: `/odom` re-expresses the
same Isaac `world -> body` motion and is not independent wheel, leg, visual, or
inertial odometry. These small errors therefore measure bridge timing and
sampling behavior, not physical localization accuracy.

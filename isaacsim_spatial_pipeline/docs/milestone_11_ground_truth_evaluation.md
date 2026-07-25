# Milestone 11: Ground-Truth ATE/RPE Evaluation

## Status

`BLOCKED — synthetic tests are available; no real simulation pose series can be recorded without the GPU runtime.`

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

The simulation odometry limitation is explicit: `/odom` is derived from Isaac
`world -> body` ground-truth motion, so it is not an independent physical
odometry measurement. A real experiment and reproducibility gate remain
blocked by the unavailable NVIDIA driver.

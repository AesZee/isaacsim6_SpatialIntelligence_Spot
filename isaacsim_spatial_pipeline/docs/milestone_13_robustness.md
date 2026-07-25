# Milestone 13: Robustness and Regression Evaluation

## Status

`COMPLETE — PASS (absolute map quality remains an inherited WARN).`

## Method

`config/m13_robustness.yaml` uses the same short bounded route, source USD,
simulation odometry, topic/frame contracts, and map validators for every case.
Faults are applied only through existing launch parameters:

- relay every second scan;
- suppress `/scan` for a bounded 10-second window;
- use the measured 10 m range-limited projection profile;
- delay `slam_toolbox` startup by 10 seconds.

All controls default off. No camera/IMU fusion, source PointCloud mutation,
host change, package installation, or USD write is involved. Hard-failure
cases run independently so the supervisor can stop immediately; recovery is a
fresh declared normal run.

## Results

| Scenario | Expected | Observed | Key measurement |
| --- | --- | --- | --- |
| Normal baseline | operational PASS | PASS | 1.964 Hz scan; final contracts/map/cleanup pass |
| Reduced scan frequency | WARN or PASS | PASS | 0.976 Hz; motion and final contracts pass |
| Controlled scan dropout | detected | detected | `/scan` absent while cloud/odom/map/TF/SLAM remained healthy |
| Recovery after dropout | operational PASS | PASS | full pipeline restored in 14.948 s preflight |
| 10 m range limit | detected | detected | finite scan failed during watchdog 1 |
| Delayed SLAM startup | delayed then pass | PASS | preflight ready in 15.346 s, below 30 s limit |

The harness-level result for successful cases is WARN only because their small
maps do not meet the unchanged Milestone 9 portfolio target. Robustness
classification treats those cases as operational passes only when motion,
final topic/TF validation, genuine map validation, cleanup, and source USD
integrity all pass.

The dropout was detected on watchdog 2 and terminated after 49 wall seconds.
The 10 m range case was detected on watchdog 1 and terminated after 42 wall
seconds. Both owned-process cleanup checks passed. The post-dropout recovery
run measured 1.956 Hz and completed the trajectory.

The first aggregate file in `summary_20260725T1650` is intentionally retained:
it exposed a stale `WARN_OR_PASS` declaration for the 10 m case. Milestone 9
had already measured and documented that case as a finite-scan failure, and
the expected detection was stated before this run. The corrected declaration
and duration-enhanced replacement summary are in `summary_20260725T1651`.

Evidence:

- `artifacts/m13_robustness/run_20260725T164205_185346_dba3c800`
- `artifacts/m13_robustness/run_20260725T164345_188551_f2cc1296`
- `artifacts/m13_robustness/run_20260725T164520_191613_9a020baa`
- `artifacts/m13_robustness/run_20260725T164628_193838_4e576ef4`
- `artifacts/m13_robustness/run_20260725T164804_196908_d62e9b05`
- `artifacts/m13_robustness/run_20260725T164853_198630_f37a7534`
- `artifacts/m13_robustness/summary_20260725T1651/robustness.json`
- `artifacts/m13_robustness/summary_20260725T1651/robustness.csv`

## Regression Contracts

The automated suite preserves:

- PointCloud2 `/spot/lidar/points`;
- LaserScan `/scan`, frame `os1_frame`;
- `/odom` as `odom -> base_link`;
- `map/odom/base_link` SLAM frames and `slam_toolbox` ownership;
- finite LaserScan safety checks;
- four genuine saved-map artifact names;
- declared robustness expectations and cleanup-required fault detection.

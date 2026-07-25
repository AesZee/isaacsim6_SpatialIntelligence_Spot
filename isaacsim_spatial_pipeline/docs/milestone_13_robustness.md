# Milestone 13: Robustness and Regression Evaluation

## Status

`BLOCKED — reversible scenarios and regression tests exist; runtime evidence requires the unavailable GPU.`

`config/m13_robustness.json` declares a normal baseline, every-second-scan
throttling, a five-second controlled scan dropout, a 10 m range limit, and a
10-second delayed SLAM start. The QoS relay implements throttling/dropout
without altering the source PointCloud2 or USD. All controls default off.

`tests/test_contracts.py` preserves `/scan` in `os1_frame`, `/odom` in
`odom -> base_link`, SLAM `map/odom/base_link`, and the four saved-map artifact
names. No camera or IMU fusion is introduced.

No scenario result or recovery time is claimed. `nvidia-smi -L` returns code
9, so the normal baseline and robustness runtime gates are blocked.

# Milestone Index

## Milestone #7

Live LiDAR SLAM with simulation-only odometry. Produces real `slam_toolbox`
outputs `/map` and `map -> odom` when Isaac Sim is running and playing.

## Milestone #8

Map saving, replay, and SLAM evaluation. Inspects real `/map` output, saves map
artifacts only after receiving a real `nav_msgs/msg/OccupancyGrid`, validates
saved artifacts, and computes lightweight occupancy-grid quality metrics.

## Milestone #9

Map quality improvement through trajectory and LiDAR slice tuning. Improves and
compares map quality from the real Milestone #8 `/map` output. It does not
create fake maps or fake transforms, and it remains simulation-only.

The bounded autonomous execution option is documented in
`docs/autonomous_runtime_harness.md`.

## Milestones #10–#14

- [Milestone 10 repeatability](milestone_10_repeatable_trajectories.md)
- [Milestone 11 ATE/RPE](milestone_11_ground_truth_evaluation.md)
- [Milestone 12 localization](milestone_12_localization.md)
- [Milestone 13 robustness](milestone_13_robustness.md)
- [Milestone 14 final portfolio](final_portfolio.md)

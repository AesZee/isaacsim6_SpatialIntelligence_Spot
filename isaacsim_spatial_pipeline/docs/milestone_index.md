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

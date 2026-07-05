# Milestone #8: Map Saving, Replay, and SLAM Evaluation

## Purpose

Milestone #8 adds a controlled evaluation layer around the Milestone #7 live
LiDAR SLAM result. It verifies that a map produced by `slam_toolbox` can be
inspected, saved, validated as artifacts, and evaluated with lightweight quality
metrics.

This milestone is read-only-first. It must report missing or weak SLAM output
honestly instead of hiding failures with fake topics, fake maps, or fake
transforms.

## Scope

Milestone #8 builds on the existing live Milestone #7 stack:

```text
/clock
/tf
/tf_static
/spot/lidar/points
/scan
/odom
/map
map -> odom
```

This remains simulation-only SLAM. It is not physical robot SLAM, real wheel
odometry, leg odometry, visual odometry, LiDAR odometry, camera/depth/IMU
fusion, or real-world localization.

## Preserved Contract

Milestone #8 must not:

```text
Modify or save Isaac Sim USD stages.
Rename Isaac prims.
Change existing Isaac ROS2 bridge topic names.
Install packages.
Record rosbag files from Codex.
Create fake /map publishers.
Create fake map -> odom transforms.
Introduce camera, depth, or IMU fusion.
Claim physical robot SLAM.
```

Codex must not run Isaac Sim, press Play, record bags, fabricate a map, or
fabricate `map -> odom`.

The Isaac-derived frame contract remains:

```text
world
body
lidar_link
OS1
sensor
rsd455_link
RSD455
Camera_OmniVision_OV9782_Color
Camera_Pseudo_Depth
Imu_Sensor
```

The Milestone #4 aliases remain:

```text
body -> base_link
sensor -> os1_frame
```

The Milestone #7 TF composition remains:

```text
odom -> world
world -> body
body -> base_link
world -> sensor
sensor -> os1_frame
```

SLAM outputs must come from `slam_toolbox`:

```text
/map [nav_msgs/msg/OccupancyGrid]
map -> odom on /tf
```

Map saving is only valid if `/map` is actually published by `slam_toolbox`. If
`/map` is missing, report `WARN`, not `PASS`. If `/map` exists but is empty,
degenerate, mostly unknown, or has no occupied cells, report `WARN` or `FAIL`
depending on severity.

## Inherited Milestone #7 Stack

`launch/m08_map_evaluation.launch.py` can include:

```text
launch/m07_lidar_slam_with_sim_odom.launch.py
```

The Milestone #8 launch file does not start Isaac Sim, save maps automatically,
record bags, or add extra publishers by default beyond the inherited Milestone
#7 stack.

## Manual Run Sequence

Terminal 1: live Isaac Sim runtime:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
/home/aes/isaacsim/python.sh /home/aes/isaac_ws/isaacsim_spatial_pipeline/scripts/10_run_sim.py
```

After Isaac Sim opens, manually press Play. Codex must not press Play.

Terminal 2: launch the Milestone #8 evaluation stack:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch /home/aes/isaac_ws/isaacsim_spatial_pipeline/launch/m08_map_evaluation.launch.py
```

Terminal 3: inspect live map output:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
python3 /home/aes/isaac_ws/isaacsim_spatial_pipeline/scripts/80_inspect_slam_map.py --duration 15.0
```

Terminal 4: open RViz:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
rviz2 -d /home/aes/isaac_ws/isaacsim_spatial_pipeline/rviz/m08_map_evaluation.rviz
```

RViz uses `Fixed Frame: map` so missing map or transform problems remain
visible.

## Manual Map Saving Sequence

Save a map only after `/map` is genuinely publishing from `slam_toolbox`:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
python3 /home/aes/isaac_ws/isaacsim_spatial_pipeline/scripts/81_save_map_artifacts.py --duration 15.0
```

The saver writes local ignored artifacts under:

```text
/home/aes/isaac_ws/maps/m08_<timestamp>/
```

Expected files:

```text
map.yaml
map.pgm
map_metadata.json
map_stats.json
```

The script must not create files if no real `/map` message is received.

## Validation Sequence

Validate saved artifacts:

```bash
python3 /home/aes/isaac_ws/isaacsim_spatial_pipeline/scripts/82_validate_saved_map_artifacts.py /home/aes/isaac_ws/maps/<map_dir>
```

Evaluate map quality:

```bash
python3 /home/aes/isaac_ws/isaacsim_spatial_pipeline/scripts/83_evaluate_map_quality.py /home/aes/isaac_ws/maps/<map_dir>
```

## Map Quality Metrics

Milestone #8 records or reports:

```text
Map dimensions in cells.
Map dimensions in meters.
Resolution.
Total area.
Known area.
Unknown cell count and ratio.
Free cell count and ratio.
Occupied cell count and ratio.
Output file paths.
```

If the map is useful enough for portfolio documentation, record the observed map
dimensions, resolution, occupied/free/unknown cell ratios, and output file paths
in the Observed Result section.

## Pass Criteria

Milestone #8 passes when:

```text
Live Isaac Sim is running and playing.
Milestone #7 stack launches without breaking inherited topics.
/clock publishes.
/tf publishes.
/tf_static publishes.
/scan publishes as sensor_msgs/msg/LaserScan.
/scan frame_id is os1_frame.
/odom publishes as nav_msgs/msg/Odometry.
/map publishes as nav_msgs/msg/OccupancyGrid from slam_toolbox.
map -> odom is observed.
80_inspect_slam_map.py reports PASS or a well-explained WARN.
81_save_map_artifacts.py saves map.yaml, map.pgm, map_metadata.json, and map_stats.json only from a real /map.
82_validate_saved_map_artifacts.py reports PASS.
83_evaluate_map_quality.py reports PASS or a useful WARN with metrics.
RViz shows /map and /scan in Fixed Frame map when SLAM is working.
No USD files are modified.
No bags are recorded by Codex.
No fake /map or fake map -> odom is introduced.
```

## Warn Criteria

Milestone #8 reports `WARN` when:

```text
SLAM stack works but /map is not received within the validation window.
/map exists but is mostly unknown.
/map exists but has very few occupied cells.
map -> odom is delayed but appears later.
Saved artifacts exist but map quality is weak.
```

## Fail Criteria

Milestone #8 fails when:

```text
Required inherited topics are broken.
/scan is missing or wrong type.
/odom is missing while sim odom is enabled.
/map is fabricated by custom code.
map -> odom is fabricated by custom code.
Saved map artifacts are created without receiving /map.
Map metadata is invalid.
The milestone changes USD files, records bags, installs packages, or introduces camera/depth/IMU fusion.
```

## Known Limitations

This evaluation is lightweight and simulation-only. It does not prove physical
robot readiness, global localization accuracy, loop-closure correctness, or
metric map fidelity against ground truth.

The quality scripts use simple occupancy-grid statistics. A map can pass these
checks while still being geometrically poor. Use RViz screenshots and manual
inspection before using a saved result in a portfolio.

## Observed Result

Milestone #8 infrastructure is complete and the live map inspection path is
working. The current observed result is `WARN`, not `FAIL`: the inherited
Milestone #7 stack publishes a real `/map` from `slam_toolbox`, `map -> odom`
is available, and the TF chain is valid, but the produced occupancy grid is
still sparse and mostly unknown.

Current PointCloud2-to-LaserScan tuning on disk:

```text
min_height: -0.20
max_height: 0.20
range_max: 20.0
```

### Live Inspection Result

Command:

```bash
python3 /home/aes/isaac_ws/isaacsim_spatial_pipeline/scripts/80_inspect_slam_map.py --duration 30.0
```

Result:

```text
Overall result: WARN
```

Observed health checks:

```text
/clock: PASS
/scan: PASS, sensor_msgs/msg/LaserScan, frame_id os1_frame
/odom: PASS, nav_msgs/msg/Odometry, header.frame_id odom, child_frame_id base_link
/map: PASS, nav_msgs/msg/OccupancyGrid, frame_id map
map -> odom: PASS
odom -> base_link: PASS
```

Observed map metrics:

```text
Map width x height cells: 245 x 543
Resolution: 0.05000000074505806 m/cell
Total cells: 133035
Unknown cells: 131006
Free cells: 1999
Occupied cells: 30
Unknown ratio: 0.9847
Free ratio: 0.0150
Occupied ratio: 0.000226
Known ratio: 0.0153
```

Interpretation:

```text
The map is real and structurally valid, but it is not yet portfolio-quality.
The occupancy grid is still mostly unknown and has very few occupied cells.
This is a map-quality WARN caused by sparse useful LaserScan returns, not a
pipeline wiring failure.
```

### Tuning Notes

The earlier widened height slice test:

```text
min_height: -0.50
max_height: 0.50
```

increased the map bounds but did not materially improve occupied structure. It
produced a larger, mostly unknown map and a lower occupied ratio. The current
better baseline is to keep the original height slice and reduce long-range ray
dominance with:

```text
min_height: -0.20
max_height: 0.20
range_max: 20.0
```

Recommended next mapping actions:

```text
Move the robot closer to walls and objects.
Use longer trajectories with turns near structure.
Wait for /map updates before saving final artifacts.
Save artifacts only after occupied_ratio and known_ratio improve enough for the intended portfolio use.
```

### Saved Artifact Result

Saved map artifacts were not recorded in this observed result section yet.
After running `81_save_map_artifacts.py`, fill in:

```text
Date:
Isaac Sim stage:
ROS_DOMAIN_ID:
Saved map directory:
map.yaml:
map.pgm:
map_metadata.json:
map_stats.json:
Artifact validation result:
Quality evaluation result:
RViz screenshot path:
Notes:
```

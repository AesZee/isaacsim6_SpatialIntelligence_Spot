# Milestone #9: Map Quality Improvement Through Trajectory and LiDAR Slice Tuning

## Purpose

Milestone #9 improves, compares, and documents the quality of real
`slam_toolbox` maps produced by the Milestone #7/#8 live simulation SLAM stack.
This milestone is about improving the quality of a real `/map`, not making the
ROS graph look correct.

## Scope

Milestone #9 provides:

```text
Named LiDAR slice and range profiles for controlled manual experiments.
A launch file that applies one selected profile without overwriting existing configs.
Offline comparison tooling for saved map artifacts.
Markdown report generation for portfolio documentation.
RViz configuration for manual map inspection.
```

It remains simulation-only and depends on a user-operated live Isaac Sim run.

## Preserved Contract

Milestone #9 must not:

```text
Modify or save Isaac Sim USD stages.
Rename Isaac prims.
Change existing Isaac ROS2 bridge topic names.
Install packages.
Record rosbag files from Codex.
Start Isaac Sim from Codex as an automated action.
Press Play in Isaac Sim.
Create fake /map publishers.
Create fake map -> odom transforms.
Introduce camera, depth, or IMU fusion.
Claim physical robot SLAM.
```

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

Milestone #4 aliases remain:

```text
body -> base_link
sensor -> os1_frame
```

Milestone #7 TF composition remains:

```text
odom -> world
world -> body
body -> base_link
world -> sensor
sensor -> os1_frame
```

SLAM outputs must still come from `slam_toolbox`:

```text
/map [nav_msgs/msg/OccupancyGrid]
map -> odom on /tf
```

## Why Milestone #8 Was WARN

Milestone #8 proved the live SLAM pipeline was wired correctly:

```text
/clock: PASS
/scan: PASS, sensor_msgs/msg/LaserScan, frame_id os1_frame
/odom: PASS, nav_msgs/msg/Odometry, header.frame_id odom, child_frame_id base_link
/map: PASS, nav_msgs/msg/OccupancyGrid, frame_id map
map -> odom: PASS
odom -> base_link: PASS
```

The result was still `WARN` because the map was sparse and mostly unknown. This
is a map-quality problem, not a graph-integrity failure.

## Current Baseline Map Metrics From Milestone #8

```text
width x height: 245 x 543 cells
resolution: 0.05 m/cell
total cells: 133035
unknown cells: 131006
free cells: 1999
occupied cells: 30
unknown ratio: 0.9847
known ratio: 0.0153
occupied ratio: 0.000226
```

## Current Baseline LiDAR Slice Parameters

```text
min_height: -0.20
max_height: 0.20
range_min: 0.20
range_max: 20.0
```

The earlier widened height slice:

```text
min_height: -0.50
max_height: 0.50
```

did not materially improve occupied structure. It expanded map bounds and
increased unknown space.

## What Milestone #9 Changes

Milestone #9 adds a profile library and experiment launch for controlled scan
conversion parameters:

```text
config/m09_lidar_slice_profiles.yaml
launch/m09_map_quality_experiment.launch.py
```

The selected profile is applied only to the launched experiment node. The
existing Milestone #4 and Milestone #7 config files are not overwritten.

Milestone #9 can also use the runtime-only scripted motion option in
`scripts/10_run_sim.py` so Spot changes LiDAR viewpoints during playback. This
is kinematic simulation motion of the Spot root prim. It is not physical Spot
leg locomotion and it is not saved to the USD stage.

## What Milestone #9 Does Not Change

Milestone #9 does not change Isaac Sim, USD assets, bridge topic names, map
publishers, SLAM source, odometry source, TF contract, or saved map artifact
format. It does not save maps automatically.

## Manual Run Sequence

Set the local installation paths once:

```bash
export ISAAC_SIM_DIR=/path/to/isaacsim
export WORKSPACE_DIR=/path/to/isaac_ws
export OUTPUT_DIR="$WORKSPACE_DIR/maps"
```

Terminal 1: start live Isaac Sim manually:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
"$ISAAC_SIM_DIR/python.sh" "$WORKSPACE_DIR/isaacsim_spatial_pipeline/scripts/10_run_sim.py"
```

Then manually press Play.

For the moving Spot experiment, use this Terminal 1 command instead:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
"$ISAAC_SIM_DIR/python.sh" "$WORKSPACE_DIR/isaacsim_spatial_pipeline/scripts/10_run_sim.py" --enable-scripted-motion
```

Optional scripted-motion tuning:

```bash
"$ISAAC_SIM_DIR/python.sh" "$WORKSPACE_DIR/isaacsim_spatial_pipeline/scripts/10_run_sim.py" \
  --enable-scripted-motion \
  --motion-speed 0.4 \
  --motion-radius-x 5.0 \
  --motion-radius-y 3.5
```

Terminal 2: launch one Milestone #9 experiment:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch "$WORKSPACE_DIR/isaacsim_spatial_pipeline/launch/m09_map_quality_experiment.launch.py" profile:=baseline_m08
```

Alternative profiles:

```bash
ros2 launch "$WORKSPACE_DIR/isaacsim_spatial_pipeline/launch/m09_map_quality_experiment.launch.py" profile:=narrow_low_noise
ros2 launch "$WORKSPACE_DIR/isaacsim_spatial_pipeline/launch/m09_map_quality_experiment.launch.py" profile:=medium_structure
ros2 launch "$WORKSPACE_DIR/isaacsim_spatial_pipeline/launch/m09_map_quality_experiment.launch.py" profile:=near_structure
ros2 launch "$WORKSPACE_DIR/isaacsim_spatial_pipeline/launch/m09_map_quality_experiment.launch.py" profile:=wide_diagnostic
```

Explicit override example:

```bash
ros2 launch "$WORKSPACE_DIR/isaacsim_spatial_pipeline/launch/m09_map_quality_experiment.launch.py" profile:=baseline_m08 range_max:=12.0
```

Terminal 3: inspect live SLAM map:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
python3 "$WORKSPACE_DIR/isaacsim_spatial_pipeline/scripts/80_inspect_slam_map.py" --duration 30.0
```

Terminal 4: save map artifacts manually after the map improves:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
python3 "$WORKSPACE_DIR/isaacsim_spatial_pipeline/scripts/81_save_map_artifacts.py" --duration 15.0 --prefix m09
```

Terminal 5: validate and evaluate saved artifact:

```bash
python3 "$WORKSPACE_DIR/isaacsim_spatial_pipeline/scripts/82_validate_saved_map_artifacts.py" "$OUTPUT_DIR/<map_dir>"
python3 "$WORKSPACE_DIR/isaacsim_spatial_pipeline/scripts/83_evaluate_map_quality.py" "$OUTPUT_DIR/<map_dir>"
```

Terminal 6: compare multiple saved maps:

```bash
python3 "$WORKSPACE_DIR/isaacsim_spatial_pipeline/scripts/90_compare_map_quality.py" "$OUTPUT_DIR"/m09_*
```

Terminal 7: write report:

```bash
python3 "$WORKSPACE_DIR/isaacsim_spatial_pipeline/scripts/91_write_map_quality_report.py" \
  --output "$WORKSPACE_DIR/isaacsim_spatial_pipeline/docs/m09_map_quality_report.md" \
  "$OUTPUT_DIR"/m09_*
```

## Suggested Isaac Sim Driving Strategy

```text
Move Spot closer to walls, racks, boxes, and warehouse structures.
Avoid only driving through open empty space.
Use slow turns near geometry.
Make several overlapping passes.
Wait for /map updates before saving.
Prefer runs where known_ratio and occupied_ratio improve over the Milestone #8 baseline.
Keep RViz open to visually inspect whether the map is becoming structurally meaningful.
```

If using `--enable-scripted-motion`, the script moves the Spot root prim around
a rectangular warehouse loop while Play is active. The motion is intended to
create changing real Isaac LiDAR returns for `slam_toolbox`; it does not create
fake `/map`, fake `/odom`, or fake `map -> odom`.

## Tuning Experiment Matrix

| Profile | min_height | max_height | range_min | range_max | Intent |
| --- | ---: | ---: | ---: | ---: | --- |
| baseline_m08 | -0.20 | 0.20 | 0.20 | 20.0 | Current best Milestone #8 baseline |
| narrow_low_noise | -0.10 | 0.15 | 0.30 | 15.0 | Reduce clutter and far sparse returns |
| medium_structure | -0.30 | 0.30 | 0.20 | 18.0 | Include more structure without full wide slice |
| near_structure | -0.25 | 0.25 | 0.20 | 10.0 | Emphasize nearby geometry |
| wide_diagnostic | -0.50 | 0.50 | 0.20 | 20.0 | Diagnostic only; previously weak for occupied ratio |

Run one profile at a time. Stop the previous launch before starting the next.

## Map Artifact Naming Convention

Use the save script with `--prefix m09`:

```text
$OUTPUT_DIR/m09_<timestamp>/
```

Record the selected LiDAR profile and trajectory notes in the generated report
or in the Observed Result section.

## Metric Comparison Workflow

For each profile:

```text
Launch profile.
Drive manually.
Inspect live map.
Save artifacts only after /map is real and improving.
Validate artifacts.
Evaluate quality.
Compare all saved map directories.
Write report.
```

The comparison heuristic prefers higher known ratio and non-trivial occupied
cell count. It penalizes very high unknown ratio and zero occupied cells. It
does not prove geometric correctness.

## RViz Inspection Guidance

Open:

```bash
rviz2 -d "$WORKSPACE_DIR/isaacsim_spatial_pipeline/rviz/m09_map_quality_improvement.rviz"
```

Use `Fixed Frame: map`. Do not change the fixed frame to hide missing map or TF
warnings. Inspect whether `/map` shows recognizable occupied structure and
whether `/scan` aligns with nearby geometry.

## Pass Criteria

Milestone #9 passes when:

```text
Existing Milestone #8 stack still works.
/clock, /tf, /tf_static, /scan, /odom, /map are available when SLAM is working.
/scan frame_id remains os1_frame.
/odom frame IDs remain odom and base_link.
/map comes from slam_toolbox.
map -> odom is observed.
Map artifacts are saved only from real /map data.
At least one saved map improves over the Milestone #8 baseline in known_ratio and/or occupied cell count.
Comparison script reads multiple map directories and ranks them.
Report script creates a Markdown report.
No fake maps, fake transforms, USD edits, rosbag recording, or sensor fusion are introduced.
```

## Warn Criteria

Milestone #9 reports `WARN` when:

```text
The pipeline runs but map quality remains weak.
/map exists but remains mostly unknown.
Occupied cells remain very low.
Comparison scripts work but no experiment beats the Milestone #8 baseline.
```

## Fail Criteria

Milestone #9 fails when:

```text
Required inherited topics are broken.
/scan is missing or no longer uses os1_frame.
/odom is missing while sim odom is enabled.
/map is fabricated.
map -> odom is fabricated.
Scripts save artifacts without a real /map.
Launch changes existing contracts destructively.
USD files are modified.
Codex records bags or installs packages.
Camera/depth/IMU fusion is introduced.
```

## Observed Result

Milestone #9 was run against live Isaac Sim with the user manually running and
playing the simulation. Codex did not start Isaac Sim, press Play, record bags,
modify USD files, fabricate `/map`, or fabricate `map -> odom`.

During the first `baseline_m08` attempt, `/scan` incorrectly appeared with
`frame_id sensor` because the Milestone #9 launch used a new
`pointcloud_to_laserscan` node name while relying on a node-scoped Milestone #4
parameter file. The Milestone #9 launch was corrected to pass the full scan
conversion parameters explicitly, including:

```text
target_frame: os1_frame
```

After that correction, `/scan` remained in `os1_frame` for valid experiments.

### Experiment Outcomes

| Profile | Inspection result | Saved | Artifact validation | Quality result | Notes |
| --- | --- | --- | --- | --- | --- |
| baseline_m08 | WARN | yes | PASS | WARN | Valid map, mostly unknown, zero occupied cells |
| near_structure | FAIL | no | not run | not run | Degenerate zero-size map |
| medium_structure | WARN | yes | PASS | WARN | Best of saved maps by known ratio |
| narrow_low_noise | FAIL | no | not run | not run | Degenerate zero-size map |

Saved map directories:

```text
$OUTPUT_DIR/m09_baseline_m08_20260705_120240
$OUTPUT_DIR/m09_medium_structure_20260705_120610
```

Generated report:

```text
$WORKSPACE_DIR/isaacsim_spatial_pipeline/docs/m09_map_quality_report.md
```

### Comparison Result

`90_compare_map_quality.py` ranked the saved maps as:

```text
Rank 1: m09_medium_structure_20260705_120610
  known_ratio: 0.0070
  unknown_ratio: 0.9930
  occupied_ratio: 0.000000
  occupied_cells: 0
  known_area_m2: 4.970

Rank 2: m09_baseline_m08_20260705_120240
  known_ratio: 0.0043
  unknown_ratio: 0.9957
  occupied_ratio: 0.000000
  occupied_cells: 0
  known_area_m2: 2.505
```

Interpretation:

```text
Milestone #9 tooling works: profile launch, real map inspection, artifact
saving, artifact validation, comparison, and report generation all ran.
Map quality remains WARN. medium_structure improved known area over the saved
baseline, but none of the saved maps have occupied cells. The next improvement
requires better manual trajectory near scene geometry and/or further scan
tuning; it should not be solved with fake maps or fake transforms.
```

Manual fields still to fill for portfolio documentation:

```text
Trajectory notes:
RViz screenshot path:
Isaac Sim scene notes:
Reason selected map is acceptable or not acceptable:
```

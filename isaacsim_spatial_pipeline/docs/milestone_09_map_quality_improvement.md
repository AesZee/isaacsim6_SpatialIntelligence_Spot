# Milestone #9: LaserScan and Map Quality Optimization

## Status

`COMPLETE — WARN (measured improvement is reproducible; portfolio targets are not met).`

The last validated Milestone #8 map is a real `slam_toolbox` map, but its
quality is `WARN`:

```text
known_ratio: 0.0153
occupied_cells: 30
unknown_ratio: 0.9847
```

Milestone #9 adds read-only scan diagnostics, six configurable projection
parameters, and reproducible JSON/CSV experiment records. The observed results
below come from bounded autonomous runs on 2026-07-25.

## Purpose

Determine where useful OS1 returns are lost between
`/spot/lidar/points [sensor_msgs/msg/PointCloud2]` and
`/scan [sensor_msgs/msg/LaserScan]`, then compare controlled tuning runs against
the Milestone #8 baseline without changing validated topic, frame, odometry, or
SLAM ownership contracts.

## Static Inspection

The current conversion path is:

```text
Isaac ROS2RtxLidarHelper
  /spot/lidar/points, frame_id sensor
    -> pointcloud_to_laserscan, target_frame os1_frame
      -> /scan_raw, best effort
        -> Milestone #7 QoS relay
          -> /scan, reliable, frame_id os1_frame
            -> slam_toolbox
```

The converter baseline is:

```text
min_height: -0.20
max_height: 0.20
range_min: 0.20
range_max: 20.0
angle_min: -3.14159
angle_max: 3.14159
angle_increment: 0.0087
scan_time: 0.3333
```

`slam_toolbox` consumes `/scan` with `map_frame: map`, `odom_frame: odom`, and
`base_frame: base_link`. Its `max_laser_range: 30.0` does not restore points
discarded by the converter's `range_max: 20.0`.

The Isaac Sim publisher scripts use the Isaac Sim 6.0 `isaacsim.*` node
families, an explicit `isaacsim.ros2.bridge.ROS2Context`, playback execution,
and simulation timestamps. The LiDAR graph uses
`isaacsim.ros2.bridge.ROS2RtxLidarHelper` in `point_cloud` mode. Milestone #9
does not change these OmniGraphs.

## TF Composition and Conflict Review

The combined stack intentionally composes:

```text
map -> odom                 from slam_toolbox
odom -> world               static Milestone #7 alias
world -> body               from Isaac Sim
body -> base_link           static Milestone #4 alias
world -> sensor             from Isaac Sim
sensor -> os1_frame         static Milestone #4 alias
```

The Milestone #6 bridge publishes `/odom` messages but is launched with
`publish_tf:=false`. Therefore it does not also publish a direct
`odom -> base_link` edge and does not give `base_link` a second parent.
`sensor -> os1_frame` is identity, so the diagnostic's cloud-frame projection
matches the converter's target-frame geometry while the validated alias is
unchanged.

## Hypotheses

The static configuration cannot establish the runtime root cause. The
diagnostic separates these testable hypotheses:

1. **Height-slice starvation:** few finite cloud points fall between
   `min_height` and `max_height`.
2. **Range rejection or long-ray dominance:** nearby structural returns are
   scarce relative to distant returns and infinite scan bins.
3. **Angular sparsity:** valid returns occupy only a small part of the
   configured 360-degree beam set.
4. **Input cadence:** PointCloud2 or LaserScan frequency is too low or
   irregular for useful overlap.
5. **Trajectory/viewpoint:** scan density is adequate while the saved map
   remains sparse, indicating insufficient user-driven coverage near
   structure rather than conversion loss.

## Read-Only Diagnostic

Run this while the user has Isaac Sim playing and one Milestone #9 profile is
launched:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
python3 /home/aes/isaac_ws/isaacsim_spatial_pipeline/scripts/92_diagnose_laserscan_quality.py \
  --duration 30.0 \
  --min-height -0.20 \
  --max-height 0.20 \
  --range-min 0.20 \
  --range-max 20.0 \
  --angle-min -3.14159 \
  --angle-max 3.14159
```

It reports:

```text
PointCloud2 input frequency
total points per cloud
finite XYZ points per cloud
points in the selected height slice per cloud
unique valid projected scan beams per cloud
LaserScan frequency
finite, infinite, and other invalid scan ranges
minimum, median, and maximum finite scan range
observed cloud and scan frame IDs
```

The projected-beam count is a read-only estimate using the same height, range,
angle, and angle-increment tests as the converter. It publishes nothing.

Interpretation:

```text
low finite XYZ / total points       -> inspect the live Isaac LiDAR publisher
low height-slice / finite XYZ       -> test wide_diagnostic
low projected beams / slice points  -> points cluster in angle/range or duplicate bins
many infinite / few finite ranges   -> too few structural hits reach /scan
adequate scan density, weak map     -> prioritize a better manual trajectory
wrong /scan frame_id                -> FAIL; stop tuning and restore os1_frame
```

## Configurable Parameters

Every named profile in `config/m09_lidar_slice_profiles.yaml` records:

```text
min_height
max_height
range_min
range_max
angle_min
angle_max
```

Select a profile:

```bash
ros2 launch /home/aes/isaac_ws/isaacsim_spatial_pipeline/launch/m09_map_quality_experiment.launch.py \
  profile:=baseline_m08
```

Override any parameter without editing source code:

```bash
ros2 launch /home/aes/isaac_ws/isaacsim_spatial_pipeline/launch/m09_map_quality_experiment.launch.py \
  profile:=baseline_m08 \
  min_height:=-0.30 \
  max_height:=0.30 \
  range_min:=0.20 \
  range_max:=12.0 \
  angle_min:=-2.35619 \
  angle_max:=2.35619
```

For a recorded experiment, prefer adding a named parameter set to the YAML
profile library instead of using anonymous overrides. This makes the launch
configuration and JSON/CSV record agree.

## Controlled Experiment Matrix

Change one factor at a time from `baseline_m08`.

| Profile | min_h | max_h | range_min | range_max | angle_min | angle_max | Tests |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `baseline_m08` | -0.20 | 0.20 | 0.20 | 20.0 | -3.14159 | 3.14159 | Milestone #8 control |
| `wide_diagnostic` | -0.50 | 0.50 | 0.20 | 20.0 | -3.14159 | 3.14159 | Height-slice starvation |
| `near_range_only` | -0.20 | 0.20 | 0.20 | 10.0 | -3.14159 | 3.14159 | Long-range ray dominance |
| `front_180_only` | -0.20 | 0.20 | 0.20 | 20.0 | -1.57080 | 1.57080 | Angular sparsity diagnostic |

Use the same scene reset, starting pose, manual route, speed, duration, and map
save timing for every run. Stop the previous launch before starting the next.

## Exact Experiment Commands

This remains the manual experiment sequence. When autonomous execution is
explicitly authorized, use `docs/autonomous_runtime_harness.md`; that harness
keeps the source USD read-only and applies bounded runtime, motion, output, and
cleanup limits.

Set paths in each terminal:

```bash
export PIPELINE_DIR=/home/aes/isaac_ws/isaacsim_spatial_pipeline
export OUTPUT_DIR=/home/aes/isaac_ws/maps
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
```

### Experiment A: baseline_m08

```bash
ros2 launch "$PIPELINE_DIR/launch/m09_map_quality_experiment.launch.py" profile:=baseline_m08
```

In another terminal:

```bash
python3 "$PIPELINE_DIR/scripts/92_diagnose_laserscan_quality.py" \
  --duration 30.0 --min-height -0.20 --max-height 0.20 \
  --range-min 0.20 --range-max 20.0 --angle-min -3.14159 --angle-max 3.14159
python3 "$PIPELINE_DIR/scripts/80_inspect_slam_map.py" --duration 30.0
python3 "$PIPELINE_DIR/scripts/81_save_map_artifacts.py" \
  --duration 15.0 --output-root "$OUTPUT_DIR" --prefix m09_baseline_m08
```

### Experiment B: wide_diagnostic

```bash
ros2 launch "$PIPELINE_DIR/launch/m09_map_quality_experiment.launch.py" profile:=wide_diagnostic
```

In another terminal:

```bash
python3 "$PIPELINE_DIR/scripts/92_diagnose_laserscan_quality.py" \
  --duration 30.0 --min-height -0.50 --max-height 0.50 \
  --range-min 0.20 --range-max 20.0 --angle-min -3.14159 --angle-max 3.14159
python3 "$PIPELINE_DIR/scripts/80_inspect_slam_map.py" --duration 30.0
python3 "$PIPELINE_DIR/scripts/81_save_map_artifacts.py" \
  --duration 15.0 --output-root "$OUTPUT_DIR" --prefix m09_wide_diagnostic
```

### Experiment C: near_range_only

```bash
ros2 launch "$PIPELINE_DIR/launch/m09_map_quality_experiment.launch.py" profile:=near_range_only
```

In another terminal:

```bash
python3 "$PIPELINE_DIR/scripts/92_diagnose_laserscan_quality.py" \
  --duration 30.0 --min-height -0.20 --max-height 0.20 \
  --range-min 0.20 --range-max 10.0 --angle-min -3.14159 --angle-max 3.14159
python3 "$PIPELINE_DIR/scripts/80_inspect_slam_map.py" --duration 30.0
python3 "$PIPELINE_DIR/scripts/81_save_map_artifacts.py" \
  --duration 15.0 --output-root "$OUTPUT_DIR" --prefix m09_near_range_only
```

### Experiment D: front_180_only

```bash
ros2 launch "$PIPELINE_DIR/launch/m09_map_quality_experiment.launch.py" profile:=front_180_only
```

In another terminal:

```bash
python3 "$PIPELINE_DIR/scripts/92_diagnose_laserscan_quality.py" \
  --duration 30.0 --min-height -0.20 --max-height 0.20 \
  --range-min 0.20 --range-max 20.0 --angle-min -1.57080 --angle-max 1.57080
python3 "$PIPELINE_DIR/scripts/80_inspect_slam_map.py" --duration 30.0
python3 "$PIPELINE_DIR/scripts/81_save_map_artifacts.py" \
  --duration 15.0 --output-root "$OUTPUT_DIR" --prefix m09_front_180_only
```

For every saved directory:

```bash
python3 "$PIPELINE_DIR/scripts/82_validate_saved_map_artifacts.py" "$OUTPUT_DIR/<saved_map_directory>"
python3 "$PIPELINE_DIR/scripts/83_evaluate_map_quality.py" "$OUTPUT_DIR/<saved_map_directory>"
```

Record all four named parameter sets and resulting map metrics:

```bash
python3 "$PIPELINE_DIR/scripts/90_compare_map_quality.py" \
  --experiment baseline_m08="$OUTPUT_DIR/<baseline_saved_directory>" \
  --experiment wide_diagnostic="$OUTPUT_DIR/<wide_saved_directory>" \
  --experiment near_range_only="$OUTPUT_DIR/<near_saved_directory>" \
  --experiment front_180_only="$OUTPUT_DIR/<front_saved_directory>" \
  --json-output "$OUTPUT_DIR/m09_experiments.json" \
  --csv-output "$OUTPUT_DIR/m09_experiments.csv"
```

The comparison helper reads real `map_metadata.json` and `map_stats.json`
artifacts. It does not subscribe, publish, start simulation, move the robot, or
create map data.

## Pass, Warn, and Fail Criteria

### PASS

```text
/spot/lidar/points remains sensor_msgs/msg/PointCloud2.
/scan remains sensor_msgs/msg/LaserScan with frame_id os1_frame.
/odom retains header.frame_id odom and child_frame_id base_link.
map -> odom is published by slam_toolbox.
The TF validator finds no multiple-parent conflict.
At least one experiment materially beats known_ratio 0.0153 and occupied_cells 30.
The selected map meets the initial portfolio targets:
  known_ratio >= 0.10
  occupied_cells >= 500
Any changed target is justified by measured warehouse geometry, never lowered to force PASS.
Saved artifacts validate, and JSON/CSV records contain all six parameters.
RViz shows recognizable occupied structure aligned with /scan.
```

### WARN

```text
Topic and TF contracts pass, but no experiment beats both baseline metrics.
/map is real but remains mostly unknown or structurally sparse.
The diagnostic shows adequate scan density but the controlled trajectory was incomplete.
An experiment improves known coverage while occupied structure remains weak.
```

### FAIL

```text
PointCloud2 or LaserScan is absent during an intended live test.
/scan frame_id is not os1_frame.
/odom or map -> odom ownership/frames change.
A SLAM-critical child has multiple TF parents.
A saved map is missing, degenerate, invalid, or not sourced from real /map.
Any map, odometry, or transform is fabricated.
USD is modified, a bag is recorded, packages are installed, or sensor fusion is introduced.
```

## Observed Result

Isaac Sim 6.0 and ROS 2 Jazzy used the read-only `Warehouse.usd` stage and the
same `warehouse_mapping_loop` for every completed comparison. Validators
confirmed `/spot/lidar/points`, finite `/scan` in `os1_frame`, `/odom`,
`map -> odom`, and no severe TF parent conflict before, during, and after
motion. Every completed map passed artifact validation.

| Profile | Result | known ratio | occupied cells | scan Hz |
| --- | --- | ---: | ---: | ---: |
| `baseline_m08` | WARN | 0.020394 | 98 | 1.947 |
| `wide_diagnostic` | WARN | 0.023300 | 279 | 1.942 |
| `near_range_only` | FAIL | — | — | — |
| `front_180_only` | WARN | 0.020004 | 49 | 1.940 |

`near_range_only` passed preflight with 105 finite ranges, then the first
motion watchdog observed no finite `/scan` return for its full four-second
window. The harness stopped before map saving, preserved the failure record,
terminated its children, and left the source USD unchanged.

The comparison selected `wide_diagnostic`. Three subsequent identical runs
produced known ratios 0.022935–0.023339, occupied-cell counts 276–281,
identical 641×659 dimensions, and no major variance. This materially improves
the Milestone 8 baseline of 0.0153/30 and establishes reproducibility, but it
does not meet the unchanged 0.10/500 portfolio targets. No RViz screenshot was
captured; machine-readable scan, TF, map, and artifact evidence is retained.

Evidence:

- `artifacts/m09_matrix/run_20260725T153155_71653_a1d15a1d`
- `artifacts/m09_matrix/run_20260725T154313_91410_af69dd1a`
- `artifacts/m09_matrix/summary_20260725T1549/m09_experiments.json`
- `artifacts/m09_matrix/summary_20260725T1549/m09_experiments.csv`
- `artifacts/m10_repeatability/run_20260725T155013_102304_7bb8f0b1`
- `artifacts/m10_repeatability/summary_20260725T1605/repeatability.json`

Overall result: `WARN`.

## Evidence Required Before Completion

1. Output from `92_diagnose_laserscan_quality.py` for the baseline and each
   selected candidate profile.
2. Output from `70_validate_lidar_slam_with_sim_odom.py` showing the topic and
   TF contracts, including no severe parent conflict.
3. `80_inspect_slam_map.py` output from each controlled run.
4. A validated saved-map directory per experiment, with the matching manual
   route notes.
5. `m09_experiments.json` and `m09_experiments.csv` containing all six
   projection parameters and map metrics.
6. RViz screenshots showing `/scan` alignment and recognizable occupied map
   structure.
7. A comparison against `known_ratio: 0.0153` and `occupied_cells: 30`.

The runtime evidence exists. The unresolved quality target and missing visual
capture remain explicit WARN items.

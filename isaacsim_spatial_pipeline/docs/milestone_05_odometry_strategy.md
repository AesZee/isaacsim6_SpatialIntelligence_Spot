# Milestone #5: Odometry Source and odom -> base_link Strategy

## Purpose

Milestone #5 adds a documented, testable odometry strategy on top of the
Milestone #4 LiDAR SLAM compatibility setup. The default result is
inspection-only: it reports whether a defensible odometry source already exists
in the running ROS2 graph, but it does not publish synthetic odometry, `odom`,
`map`, or localization transforms.

This milestone does not modify Isaac Sim USD files, save stages, rename prims,
change bridge topics, record bags, install packages, or introduce camera,
depth, or IMU fusion.

## Why Milestone #4 Produced WARN

Milestone #4 validated that the Milestone #3 bag replays with `/clock`, that
`/spot/lidar/points` can be converted to `/scan`, and that compatibility
aliases exist:

```text
body -> base_link
sensor -> os1_frame
```

The expected validator result was `WARN`, not full `PASS`, because the replay
contract does not contain wheel odometry or an `odom -> base_link` transform.
Without that transform chain, `slam_toolbox` can receive `/scan` but may not
publish `/map`, `map`, or `odom`.

## Why SLAM Needs odom -> base_link

LiDAR SLAM needs a coherent relationship between the robot base and the local
odometry frame. For this pipeline the SLAM configuration expects:

```text
map_frame: map
odom_frame: odom
base_frame: base_link
scan_topic: /scan
```

The `odom -> base_link` chain gives SLAM a local motion estimate to connect
successive scans. Publishing arbitrary odometry would make the TF graph look
complete while hiding an unvalidated motion source, so this milestone keeps the
default strategy read-only.

## Inherited Frames and Topics

Milestone #5 preserves the Isaac-derived frame contract:

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

It also preserves the Milestone #4 compatibility aliases:

```text
body -> base_link
sensor -> os1_frame
```

Inherited topics used for inspection:

```text
/clock
/tf
/tf_static
/spot/lidar/points
/scan
/odom
```

## What This Milestone Adds

Milestone #5 adds:

```text
isaacsim_spatial_pipeline/docs/milestone_05_odometry_strategy.md
isaacsim_spatial_pipeline/scripts/50_inspect_odometry_inputs.py
isaacsim_spatial_pipeline/scripts/51_validate_odometry_strategy.py
isaacsim_spatial_pipeline/launch/m05_odometry_strategy.launch.py
isaacsim_spatial_pipeline/config/m05_odometry_strategy.yaml
isaacsim_spatial_pipeline/rviz/m05_odometry_strategy.rviz
```

Default strategy:

```text
strategy: inspect_only
publish_odom_tf: false
publish_odom_topic: false
```

## What This Milestone Does Not Add

This milestone does not add:

```text
fake map frames
fake odom frames
unvalidated odom -> base_link transforms
wheel odometry estimation
visual odometry
LiDAR odometry
camera/depth/IMU fusion
USD changes
new rosbag recording
```

## Terminal 1: Replay Milestone #3 Bag

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
ros2 bag play /home/aes/isaac_ws/bags/m03_sensor_tf_baseline_20260628_123742 --clock --loop
```

## Terminal 2: Launch Milestone #4 Aliases

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch /home/aes/isaac_ws/isaacsim_spatial_pipeline/launch/m04_static_aliases.launch.py
```

## Terminal 3: Launch PointCloud2 To LaserScan

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch /home/aes/isaac_ws/isaacsim_spatial_pipeline/launch/m04_pointcloud_to_laserscan.launch.py
```

## Terminal 4: Inspect Odometry Inputs

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
python3 /home/aes/isaac_ws/isaacsim_spatial_pipeline/scripts/50_inspect_odometry_inputs.py
```

Expected with the current Milestone #3 bag:

```text
/clock: PASS
/tf: PASS
/tf_static: PASS
/scan: PASS when Milestone #4 conversion is active
base_link: PASS
os1_frame: PASS
/odom: WARN if absent
odom -> base_link: WARN if absent
map -> odom: WARN if absent
Overall result: WARN
```

## Terminal 5: Launch Milestone #5 Strategy

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch /home/aes/isaac_ws/isaacsim_spatial_pipeline/launch/m05_odometry_strategy.launch.py
```

This launch file loads `config/m05_odometry_strategy.yaml` and logs the selected
strategy. In the default `inspect_only` mode it starts no publisher.

## Terminal 6: Validate

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
python3 /home/aes/isaac_ws/isaacsim_spatial_pipeline/scripts/51_validate_odometry_strategy.py
```

The validator returns:

```text
PASS only when odom -> base_link is genuinely observed
WARN when inherited pieces work but odometry is absent
FAIL when required inherited pieces are broken
```

## Terminal 7: RViz

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
rviz2 -d /home/aes/isaac_ws/isaacsim_spatial_pipeline/rviz/m05_odometry_strategy.rviz
```

The fixed frame is `world` for odometry inspection. TF, `/scan`,
`/spot/lidar/points`, `/odom`, and `/map` displays are included so missing
odometry or map data remains visible instead of being hidden by frame changes.

Expected RViz interpretation with the current Milestone #3 bag:

```text
TF display: should show world, body, sensor, base_link, and os1_frame.
LaserScan /scan: should show scan data when Milestone #4 conversion is active.
PointCloud2 /spot/lidar/points: should show raw LiDAR points during replay.
Odometry /odom: may show no drawable data because /odom is absent.
Map /map: expected WARN with "No map received" unless SLAM genuinely publishes a map.
```

The RViz `Odometry` display can show `Status: Ok` even when no `/odom` topic is
currently observed. That means the display is configured correctly, not that a
validated odometry source exists. Trust the validator result for odometry
readiness.

## Observed Result

Observed on 2026-07-02 with the Milestone #3 bag replaying, Milestone #4
aliases active, and Milestone #4 PointCloud2-to-LaserScan conversion active:

```text
50_inspect_odometry_inputs.py overall result: WARN
51_validate_odometry_strategy.py overall result: WARN
```

Passing checks:

```text
/clock [rosgraph_msgs/msg/Clock]
/tf [tf2_msgs/msg/TFMessage]
/tf_static [tf2_msgs/msg/TFMessage]
/spot/lidar/points [sensor_msgs/msg/PointCloud2]
/scan [sensor_msgs/msg/LaserScan]
/scan frame_id: os1_frame
world observed in TF
body observed in TF
sensor observed in TF
base_link observed in TF
os1_frame observed in TF
world -> body chain observed
body -> base_link chain observed
sensor -> os1_frame chain observed
```

Expected warnings:

```text
/odom topic not observed
No nav_msgs/msg/Odometry messages received
odom frame not observed in TF
map frame not observed in TF
odom -> base_link chain not observed
map -> odom chain not observed
RViz Map display: WARN, No map received
```

Observed TF edges included the inherited Isaac motion relationship and the
Milestone #4 aliases:

```text
world -> body
world -> sensor
body -> base_link
sensor -> os1_frame
```

Interpretation: Milestone #5 inspection mode is behaving correctly. The replay
and Milestone #4 compatibility setup are intact, `/scan` is available, and the
tools clearly report that no validated odometry source is present. This is a
valid inspection-mode result, but it is not full odometry readiness and must not
be reported as SLAM success.

## Pass Criteria

Milestone #5 passes in inspection mode when:

```text
The Milestone #3 bag replays with /clock.
Milestone #4 aliases still publish base_link and os1_frame.
/scan is available if pointcloud conversion is active.
The inspection script clearly reports whether odometry is available.
The validation script does not falsely claim full SLAM success.
The documentation states that the current replay lacks validated odometry if /odom or odom -> base_link is absent.
No USD files are modified.
No fake map, odom, or localization transforms are added silently.
```

Full odometry `PASS` requires:

```text
A genuine or explicitly justified odometry source exists.
/odom exists as nav_msgs/msg/Odometry, or an equivalent validated TF source is documented.
odom -> base_link is observed.
The validation script reports PASS.
The strategy is documented as real, derived, or simulation-only with limitations.
```

## Fail Criteria

Milestone #5 fails if:

```text
Existing Milestone #4 frames are broken.
/clock is missing during replay.
/tf or /tf_static is missing.
base_link or os1_frame aliases disappear.
A fake odom or map transform is added without clear documentation.
The validator reports PASS without a genuine odom -> base_link chain.
USD files are changed.
Camera/depth/IMU fusion is introduced.
```

## Next Milestone Recommendation

Milestone #6 should choose one odometry source explicitly:

```text
Replay a validated /odom topic from Isaac or robot telemetry, or
derive a documented simulation-only odometry bridge from an observed Isaac TF motion relationship, or
defer SLAM mapping until the replay contract includes wheel odometry or another validated base motion source.
```

If a simulation-only bridge is implemented, it should be disabled by default,
publish both topic and TF only when explicitly requested, document its
assumptions, and validate against observed `world -> body` motion rather than
inventing arbitrary motion.

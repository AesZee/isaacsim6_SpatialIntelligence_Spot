# Milestone #4: First LiDAR SLAM Bring-up

## Purpose

Milestone #4 starts LiDAR-only SLAM from the validated Milestone #3 replay
contract. It does not modify Isaac Sim, save USD files, rename Isaac prims, or
change the existing ROS2 bridge topics.

The input is the Milestone #3 bag:

```text
/home/aes/isaac_ws/bags/m03_sensor_tf_baseline_20260628_123742
```

Validated replay topics:

```text
/clock
/tf
/spot/lidar/points
/spot/d455/color/image
/spot/d455/color/camera_info
/spot/d455/depth/image
/spot/d455/imu
```

Milestone #4 uses only:

```text
/clock
/tf
/spot/lidar/points
```

Camera, depth, and IMU fusion are intentionally out of scope.

## Observed Result

Observed on 2026-06-28 with the Milestone #3 bag replaying via `--clock`:

```text
Validator overall result: WARN
```

Passing checks:

```text
/scan [sensor_msgs/msg/LaserScan]
/scan frame_id: os1_frame
/tf [tf2_msgs/msg/TFMessage]
/tf_static [tf2_msgs/msg/TFMessage]
/clock [rosgraph_msgs/msg/Clock]
base_link observed on /tf_static
os1_frame observed on /tf_static
```

Expected warnings:

```text
/map not received yet
map frame not observed yet
odom frame not observed yet
```

Interpretation: Milestone #4 LiDAR compatibility bring-up is working. The
PointCloud2-to-LaserScan path is valid, and the downstream compatibility alias
frames are present. Full `map`/`odom` SLAM output is blocked by the current
replay contract not containing odometry or an `odom -> base_link` transform.
Do not hide this by adding a fake odometry transform in Milestone #4.

## Frame Policy

Keep the Isaac-derived frame contract intact:

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

Milestone #4 adds compatibility aliases only:

```text
body -> base_link
sensor -> os1_frame
```

These aliases do not replace or remove Isaac frames. They exist only so
downstream SLAM packages can use conventional frame names.

SLAM frames are introduced by SLAM tooling:

```text
map
odom
```

Do not publish additional static aliases for `map` or `odom` in this milestone.

## Package Prerequisites

Check installed packages:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
ros2 pkg list | grep -E 'pointcloud_to_laserscan|slam_toolbox|tf2_ros'
```

If packages are missing, install them manually:

```bash
sudo apt update
sudo apt install ros-jazzy-pointcloud-to-laserscan ros-jazzy-slam-toolbox
```

Do not install packages from Codex.

## Files

Milestone #4 adds:

```text
isaacsim_spatial_pipeline/launch/m04_static_aliases.launch.py
isaacsim_spatial_pipeline/launch/m04_pointcloud_to_laserscan.launch.py
isaacsim_spatial_pipeline/launch/m04_slam_toolbox.launch.py
isaacsim_spatial_pipeline/config/m04_pointcloud_to_laserscan.yaml
isaacsim_spatial_pipeline/config/m04_slam_toolbox.yaml
isaacsim_spatial_pipeline/rviz/m04_lidar_slam_bringup.rviz
isaacsim_spatial_pipeline/scripts/40_validate_slam_topics.py
```

## Terminal 1: Replay Milestone #3 Bag With Clock

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
ros2 bag play /home/aes/isaac_ws/bags/m03_sensor_tf_baseline_20260628_123742 --clock --loop
```

Use `--loop` while tuning the first bring-up. Stop replay with `Ctrl-C`.

Verify the replay contract:

```bash
ros2 topic list -t
ros2 topic echo /clock --once
ros2 topic echo /spot/lidar/points --field header --once
ros2 topic echo /tf --once
```

Expected LiDAR frame:

```text
frame_id: sensor
```

## Terminal 2: Launch Static Alias Frames

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch /home/aes/isaac_ws/isaacsim_spatial_pipeline/launch/m04_static_aliases.launch.py
```

This publishes:

```text
body -> base_link
sensor -> os1_frame
```

Check aliases:

```bash
ros2 topic echo /tf_static --once
```

## Terminal 3: Convert PointCloud2 To LaserScan

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch /home/aes/isaac_ws/isaacsim_spatial_pipeline/launch/m04_pointcloud_to_laserscan.launch.py
```

Configuration:

```text
input: /spot/lidar/points
output: /scan
target_frame: os1_frame
use_sim_time: true
```

`os1_frame` is used because it is a direct compatibility alias of the validated
LiDAR frame `sensor`. `base_link` remains the SLAM base frame through the
`body -> base_link` alias.

Validate `/scan`:

```bash
ros2 topic list -t | grep /scan
ros2 topic echo /scan --field header --once
ros2 topic hz /scan
```

Expected:

```text
/scan [sensor_msgs/msg/LaserScan]
frame_id: os1_frame
```

## Terminal 4: Launch slam_toolbox

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch /home/aes/isaac_ws/isaacsim_spatial_pipeline/launch/m04_slam_toolbox.launch.py
```

Configuration:

```text
map_frame: map
odom_frame: odom
base_frame: base_link
scan_topic: /scan
use_sim_time: true
```

Important first-bring-up limitation: the Milestone #3 replay contract does not
include wheel odometry or an `odom -> base_link` transform. `slam_toolbox` may
start, consume `/scan`, and still warn or delay map publication while waiting
for an odometry/base transform chain. Do not hide this by publishing a fake
`odom` transform in Milestone #4 unless that becomes an explicit follow-up
decision.

Check SLAM topics:

```bash
ros2 topic list -t | grep -E '/scan|/map|/tf'
ros2 topic echo /map --once
ros2 topic echo /tf --once
```

## Terminal 5: Open RViz2

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
rviz2 -d /home/aes/isaac_ws/isaacsim_spatial_pipeline/rviz/m04_lidar_slam_bringup.rviz
```

RViz2 configuration:

```text
Fixed Frame: map
TF display: enabled
LaserScan: /scan
Map: /map
PointCloud2: /spot/lidar/points
```

If `map` is not available yet, RViz2 may show transform errors. Confirm `/scan`
first, then inspect slam_toolbox logs for missing `odom` or `base_link`
transforms.

## Validation Script

Run while replay, aliases, pointcloud conversion, and slam_toolbox are active:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
python3 /home/aes/isaac_ws/isaacsim_spatial_pipeline/scripts/40_validate_slam_topics.py
```

The validator checks:

- `/scan` exists.
- `/scan` type is `sensor_msgs/msg/LaserScan`.
- `/map` exists after SLAM starts, reported as a warning if absent.
- `/tf_static` includes alias frames `base_link` and `os1_frame`.
- `/tf` includes `map` and `odom` when SLAM publishes them.
- `/clock` is present for simulation-time-compatible replay.

## Pass Criteria

Milestone #4 first bring-up passes when:

- The Milestone #3 bag replays with `--clock`.
- Static aliases publish `base_link` and `os1_frame` without removing Isaac
  frames.
- `/spot/lidar/points` is converted into `/scan`.
- `/scan` has type `sensor_msgs/msg/LaserScan`.
- `/scan` uses `frame_id: os1_frame`.
- `slam_toolbox` starts with `use_sim_time: true`.
- `slam_toolbox` is configured with `map`, `odom`, `base_link`, and `/scan`.
- RViz2 opens with `Fixed Frame: map`.
- Any missing `map` or `odom` behavior is documented as an odometry/SLAM
  follow-up, not hidden with unvalidated transforms.

## Fail Criteria

Milestone #4 fails if:

- The replay does not publish `/clock`.
- `/spot/lidar/points` is missing or no longer uses `frame_id: sensor`.
- Alias frames replace Isaac frames instead of extending them.
- `/scan` is missing or has the wrong message type.
- `/scan` uses an unexpected frame ID.
- `slam_toolbox` is configured against a topic other than `/scan`.
- USD files are modified.
- Camera, depth, or IMU fusion is introduced.
- Unvalidated `odom`, `map`, or fake localization transforms are added to make
  RViz2 look correct.

## Next Decisions

After first bring-up, decide one of these before deeper SLAM work:

1. Add an odometry source or controlled `odom -> base_link` strategy.
2. Use a SLAM configuration that can operate acceptably without wheel odometry,
   if available and defensible.
3. Keep Milestone #4 as a documented compatibility test and move odometry
   integration into the next milestone.

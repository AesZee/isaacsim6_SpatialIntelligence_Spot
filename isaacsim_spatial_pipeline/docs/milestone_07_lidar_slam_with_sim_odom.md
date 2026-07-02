# Milestone #7: LiDAR SLAM With Simulation-Only Odometry

## Purpose

Milestone #7 runs LiDAR SLAM in live Isaac Sim using the Milestone #6
simulation-only odometry bridge. The goal is to validate whether
`slam_toolbox` can publish:

```text
/map [nav_msgs/msg/OccupancyGrid]
map -> odom on /tf
```

when the live simulation graph provides:

```text
/clock
/tf
/tf_static
/spot/lidar/points
/scan
/odom
odom -> base_link
```

This is simulation-only SLAM bring-up. It is not physical Spot odometry,
physical robot SLAM, real wheel odometry, leg odometry, visual odometry, LiDAR
odometry, camera/depth/IMU fusion, or real-world localization.

## Scope

This milestone uses live Isaac Sim. Milestone #3 rosbag replay alone is not
enough because `/odom` comes from the Milestone #6 bridge observing live
`world -> body` TF motion.

Inputs:

```text
/spot/lidar/points [sensor_msgs/msg/PointCloud2] from live Isaac Sim
/scan [sensor_msgs/msg/LaserScan] from Milestone #4 PointCloud2-to-LaserScan
/odom [nav_msgs/msg/Odometry] from Milestone #6 simulation-only odometry
odom -> base_link from Milestone #6 simulation-only odometry
```

SLAM outputs must come from `slam_toolbox`:

```text
/map
map -> odom
```

Do not add custom fake publishers for `/map` or `map -> odom`.

## Preserved Contract

Milestone #7 does not modify USD files, save Isaac Sim stages, rename prims,
change existing ROS2 bridge topic names, install packages, record bags from
Codex, or introduce camera, depth, or IMU fusion.

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

The Milestone #4 compatibility aliases remain:

```text
body -> base_link
sensor -> os1_frame
```

The Milestone #6 odometry bridge remains:

```text
source: world -> body
output: /odom
output TF: odom -> base_link
```

## Files

Milestone #7 adds:

```text
isaacsim_spatial_pipeline/docs/milestone_07_lidar_slam_with_sim_odom.md
isaacsim_spatial_pipeline/config/m07_slam_toolbox_sim_odom.yaml
isaacsim_spatial_pipeline/launch/m07_lidar_slam_with_sim_odom.launch.py
isaacsim_spatial_pipeline/scripts/70_validate_lidar_slam_with_sim_odom.py
isaacsim_spatial_pipeline/rviz/m07_lidar_slam_with_sim_odom.rviz
```

## Configuration

`config/m07_slam_toolbox_sim_odom.yaml` configures `slam_toolbox` for ROS2
Jazzy with simulation time:

```text
map_frame: map
odom_frame: odom
base_frame: base_link
scan_topic: /scan
use_sim_time: true
mode: mapping
```

SLAM consumes `/scan`, not `/spot/lidar/points` directly. The raw PointCloud2
topic is converted by the Milestone #4 PointCloud2-to-LaserScan node.

The configuration keeps Jazzy-compatible parameter names already used by the
Milestone #4 `slam_toolbox` configuration and narrows `max_laser_range` to
`30.0` for this simulation odometry experiment.

## Launch Sequence

Terminal 1: live Isaac Sim runtime:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
/home/aes/isaacsim/python.sh /home/aes/isaac_ws/isaacsim_spatial_pipeline/scripts/10_run_sim.py
```

After Isaac Sim opens, manually press Play. Codex must not press Play, save the
stage, or record bags.

Terminal 2: launch Milestone #7 stack:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch /home/aes/isaac_ws/isaacsim_spatial_pipeline/launch/m07_lidar_slam_with_sim_odom.launch.py
```

The launch file starts:

```text
m04_static_aliases.launch.py
m04_pointcloud_to_laserscan.launch.py
m06_sim_odometry_bridge.launch.py enable_sim_odom:=true publish_tf:=true
slam_toolbox with config/m07_slam_toolbox_sim_odom.yaml
```

It does not start Isaac Sim. Start Isaac Sim separately as shown above.

Launch arguments:

```text
enable_sim_odom default true
publish_odom_tf default true
use_sim_time default true
slam_params_file default /home/aes/isaac_ws/isaacsim_spatial_pipeline/config/m07_slam_toolbox_sim_odom.yaml
```

## Validation Sequence

Terminal 3: validate:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
python3 /home/aes/isaac_ws/isaacsim_spatial_pipeline/scripts/70_validate_lidar_slam_with_sim_odom.py
```

Use a longer bounded collection window if SLAM startup is slow:

```bash
python3 /home/aes/isaac_ws/isaacsim_spatial_pipeline/scripts/70_validate_lidar_slam_with_sim_odom.py --duration 15.0
```

The validator is read-only. It checks:

```text
/clock exists and publishes
/tf exists and publishes
/tf_static exists and publishes
/spot/lidar/points exists as sensor_msgs/msg/PointCloud2
/scan exists as sensor_msgs/msg/LaserScan
/scan header.frame_id == os1_frame
/odom exists as nav_msgs/msg/Odometry
/odom header.frame_id == odom
/odom child_frame_id == base_link
odom -> base_link observed in TF
body -> base_link alias observed or conflict reported
sensor -> os1_frame alias observed
/map exists as nav_msgs/msg/OccupancyGrid
map frame observed in TF
odom frame observed in TF
map -> odom observed in TF
```

It also checks whether a node named `slam_toolbox` is active and whether `/map`
is published by that node.

Terminal 4: RViz:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
rviz2 -d /home/aes/isaac_ws/isaacsim_spatial_pipeline/rviz/m07_lidar_slam_with_sim_odom.rviz
```

RViz uses:

```text
Fixed Frame: map
TF display
LaserScan display for /scan
Map display for /map
Odometry display for /odom
PointCloud2 display for /spot/lidar/points
```

If `/map` is absent, RViz should warn visibly. Do not hide missing-map warnings
with fake map frames or arbitrary transforms.

## Pass Criteria

Milestone #7 passes when:

```text
Live Isaac Sim is running and playing.
/clock publishes.
/tf publishes.
/tf_static publishes.
/spot/lidar/points publishes.
/scan publishes as sensor_msgs/msg/LaserScan.
/scan uses frame_id os1_frame.
/odom publishes as nav_msgs/msg/Odometry from Milestone #6 sim odom.
/odom uses header.frame_id odom.
/odom uses child_frame_id base_link.
odom -> base_link is observed.
/map publishes as nav_msgs/msg/OccupancyGrid from slam_toolbox.
map -> odom is observed from slam_toolbox.
RViz opens with Fixed Frame map.
RViz shows /scan and /map when SLAM is working.
No severe TF parent conflict is detected.
No fake map or fake map -> odom is published.
No USD files are modified or saved.
No bags are recorded by Codex.
```

## Warn Criteria

Milestone #7 reports `WARN` when:

```text
Live Isaac Sim works.
Aliases work.
/scan works.
Simulation odometry works.
/odom works.
odom -> base_link works.
slam_toolbox starts but /map or map -> odom is not observed within the validation window.
```

Interpret this as a SLAM configuration, startup timing, motion, exploration, or
scan-matching issue. Do not hide it by publishing fake `/map` or fake
`map -> odom`.

## Fail Criteria

Milestone #7 fails if:

```text
/clock is missing.
/tf is missing.
/tf_static is missing.
/spot/lidar/points is missing.
/scan is missing or has the wrong type.
/scan frame_id is not os1_frame.
/odom is missing while enable_sim_odom is true.
/odom has wrong frame IDs.
odom -> base_link is missing.
slam_toolbox consumes the wrong scan topic.
A fake map or fake map -> odom is published by custom code.
A severe TF conflict is detected and not reported.
USD files are changed.
Camera/depth/IMU fusion is introduced.
Bags are recorded by Codex.
```

## TF Conflict Handling

Milestone #4 publishes:

```text
body -> base_link
```

Milestone #6 can publish:

```text
odom -> base_link
```

Those two edges give `base_link` two parents when both are active. A TF tree
should not silently contain competing parents for a child frame. Milestone #7
keeps the inherited behavior visible and reports this as a severe TF conflict
instead of hiding it with arbitrary transforms.

Possible future fixes require an explicit documented decision, such as:

```text
odom -> body, while body -> base_link remains static
```

or:

```text
odom -> base_link, while removing or disabling body -> base_link only in a
clearly documented future milestone
```

This milestone does not destructively change Milestone #4 or Milestone #6.

## Known Limitations

This milestone validates a simulation pipeline, not a physical robot pipeline.

Known limitations:

```text
The odometry source is live Isaac TF motion, not measured robot odometry.
The Milestone #6 bridge is planar by default.
SLAM may need robot motion and useful scan overlap before /map appears.
The current combined Milestone #4 alias and Milestone #6 odom TF can conflict on base_link.
The validator cannot prove physical correctness; it only verifies ROS topic and TF behavior.
Milestone #3 rosbag replay alone does not satisfy this milestone.
```

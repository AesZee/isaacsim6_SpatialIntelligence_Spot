# Milestone #6: Simulation-Only Odometry Bridge From Isaac TF Motion

## Purpose

Milestone #6 adds an opt-in odometry bridge for live Isaac Sim runs. It observes
the Isaac-published `world -> body` transform, derives a planar simulation-only
odometry estimate, and can publish:

```text
/odom [nav_msgs/msg/Odometry]
odom -> base_link on /tf
```

This is intended to unblock controlled LiDAR SLAM experiments in simulation. It
is not real Spot wheel, leg, visual, LiDAR, camera, depth, or IMU odometry.

Publishing is disabled unless explicitly requested:

```text
enable_sim_odom: false by default
```

## Why This Is Simulation-Only

The odometry pose comes from Isaac TF motion:

```text
source: world -> body
output: odom -> base_link
```

The bridge does not estimate contact, wheel slip, leg kinematics, inertial
motion, visual motion, or LiDAR scan matching. It simply re-expresses the live
simulated base motion as conventional ROS odometry for downstream simulation
experiments. Treat it as a controlled simulation bridge, not a physical robot
odometry source.

## Preserved Contract

Milestone #6 does not modify USD files, save stages, rename prims, change ROS2
bridge topic names, install packages, record bags, or introduce camera/depth/IMU
fusion.

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

Milestone #4 compatibility aliases remain separate:

```text
body -> base_link
sensor -> os1_frame
```

Milestone #6 does not publish:

```text
map
map -> odom
fake map frames
arbitrary odometry unrelated to observed Isaac TF motion
```

`map -> odom` remains the responsibility of SLAM or localization.

## Files

Milestone #6 adds:

```text
isaacsim_spatial_pipeline/docs/milestone_06_sim_odometry_bridge.md
isaacsim_spatial_pipeline/config/m06_sim_odometry_bridge.yaml
isaacsim_spatial_pipeline/launch/m06_sim_odometry_bridge.launch.py
isaacsim_spatial_pipeline/scripts/60_sim_odometry_bridge.py
isaacsim_spatial_pipeline/scripts/61_validate_sim_odometry_bridge.py
isaacsim_spatial_pipeline/rviz/m06_sim_odometry_bridge.rviz
```

## Terminal 1: Run Live Isaac Sim

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
/home/aes/isaacsim/python.sh /home/aes/isaac_ws/isaacsim_spatial_pipeline/scripts/10_run_sim.py
```

After Isaac Sim opens, press Play manually. Codex must not press Play, save the
stage, or record bags.

## Terminal 2: Launch Milestone #4 Aliases

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch /home/aes/isaac_ws/isaacsim_spatial_pipeline/launch/m04_static_aliases.launch.py
```

This preserves:

```text
body -> base_link
sensor -> os1_frame
```

## Terminal 3: Launch PointCloud2 To LaserScan

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch /home/aes/isaac_ws/isaacsim_spatial_pipeline/launch/m04_pointcloud_to_laserscan.launch.py
```

Expected:

```text
/spot/lidar/points -> /scan
/scan frame_id: os1_frame
```

## Terminal 4A: Launch Bridge Disabled

Use this first to confirm the bridge is safe by default:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch /home/aes/isaac_ws/isaacsim_spatial_pipeline/launch/m06_sim_odometry_bridge.launch.py
```

Expected:

```text
The node observes /tf and /clock.
The node logs that enable_sim_odom is false.
No /odom is published by this bridge.
No odom -> base_link TF is published by this bridge.
```

## Terminal 4B: Launch Bridge Enabled

Use this only for controlled simulation odometry:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch /home/aes/isaac_ws/isaacsim_spatial_pipeline/launch/m06_sim_odometry_bridge.launch.py enable_sim_odom:=true
```

Expected after Isaac Sim is playing and `world -> body` is observed:

```text
/odom [nav_msgs/msg/Odometry]
/odom header.frame_id: odom
/odom child_frame_id: base_link
odom -> base_link on /tf
```

The bridge computes linear and yaw velocity by finite difference between
consecutive observed `world -> body` poses using simulation time. If the robot
is stationary, zero or near-zero velocity is acceptable.

## Terminal 5: Validate Disabled Bridge

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
python3 /home/aes/isaac_ws/isaacsim_spatial_pipeline/scripts/61_validate_sim_odometry_bridge.py
```

Expected result when inherited live-sim pieces work and the bridge is disabled:

```text
Overall result: WARN
```

This `WARN` is correct because `/odom` and `odom -> base_link` are intentionally
absent.

## Terminal 5: Validate Enabled Bridge

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
python3 /home/aes/isaac_ws/isaacsim_spatial_pipeline/scripts/61_validate_sim_odometry_bridge.py --bridge-enabled --publish-tf-enabled
```

Expected result when Isaac Sim is playing, aliases are active, conversion is
active, and the bridge is enabled:

```text
Overall result: PASS
```

`PASS` requires:

```text
/clock publishes
/tf publishes
/tf_static publishes
world -> body is observed
body -> base_link is preserved
sensor -> os1_frame is preserved
/scan exists with frame_id os1_frame when conversion is active
/odom exists as nav_msgs/msg/Odometry
/odom header.frame_id is odom
/odom child_frame_id is base_link
odom -> base_link is observed on /tf
```

## Terminal 6: RViz

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
rviz2 -d /home/aes/isaac_ws/isaacsim_spatial_pipeline/rviz/m06_sim_odometry_bridge.rviz
```

The fixed frame is `odom`. RViz includes TF, `/scan`,
`/spot/lidar/points`, `/odom`, and optional `/map`.

Expected with the bridge enabled:

```text
TF should show odom -> base_link.
Odometry /odom should display poses.
LaserScan /scan should remain in os1_frame.
PointCloud2 /spot/lidar/points should remain visible when transformable.
Map /map may warn with "No map received" unless SLAM genuinely publishes it.
```

Do not hide missing map warnings with fake frames. `map` and `map -> odom` are
not Milestone #6 outputs.

## Rosbag Replay Note

Milestone #3 rosbag replay alone will not produce this odometry. The bridge
targets live Isaac Sim TF motion. A future user-controlled workflow may manually
record generated `/odom`, but Codex must not record bags.

## Pass Criteria

Milestone #6 passes when:

```text
Live Isaac Sim is running and playing.
/clock publishes.
/tf publishes.
/tf_static publishes.
world -> body is observed from Isaac TF.
body -> base_link and sensor -> os1_frame aliases are preserved.
/odom is published only when enable_sim_odom is true.
/odom has header.frame_id odom and child_frame_id base_link.
odom -> base_link is published only when enable_sim_odom is true and publish_tf is true.
No map or map -> odom is published by this bridge.
No USD files are modified or saved.
No bags are recorded by Codex.
```

## Warn Criteria

Milestone #6 should report `WARN` when:

```text
Inherited live-sim pieces work but enable_sim_odom is false.
/odom is intentionally absent.
odom -> base_link is intentionally absent.
/scan is absent only because pointcloud conversion is not running.
/map is absent because SLAM/localization has not published it.
```

## Fail Criteria

Milestone #6 fails if:

```text
/clock is missing during live simulation.
/tf is missing.
/tf_static is missing.
world -> body is not observed.
body -> base_link alias is missing.
sensor -> os1_frame alias is missing.
/odom exists with the wrong type.
/odom uses the wrong header.frame_id or child_frame_id.
odom -> base_link is missing while enable_sim_odom and publish_tf are enabled.
The bridge publishes map or map -> odom.
The bridge publishes motion not derived from observed world -> body TF.
USD files are changed or saved.
Camera/depth/IMU fusion is introduced.
```

## Known Limitations

This bridge is useful for controlled simulation experiments, but it has clear
limits:

```text
It depends on live Isaac TF.
It is planar by default.
It does not validate physical locomotion or wheel/leg odometry.
It does not solve localization.
It does not publish map -> odom.
It does not make rosbag-only Milestone #3 replay full SLAM-ready by itself.
```

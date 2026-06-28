# Milestone #3: RViz2 And Rosbag Replay Validation

## Purpose

Milestone #3 validates that the Milestone #2 ROS2 topic and TF contract is
usable in RViz2 and survives a manual rosbag2 record/replay cycle. This
milestone is still sensor and transform validation only.

Do not redesign frame names in this milestone. Current frame naming is
Isaac-derived and remains the contract:

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

`base_link`, `odom`, `map`, and static transform aliases are not introduced in
Milestone #3. SLAM/localization is Milestone #4.

## Validated Topics

Record and replay only the validated baseline topics:

```text
/clock
/tf
/spot/lidar/points
/spot/d455/color/image
/spot/d455/color/camera_info
/spot/d455/depth/image
/spot/d455/imu
```

Expected topic types:

```text
/clock [rosgraph_msgs/msg/Clock]
/tf [tf2_msgs/msg/TFMessage]
/spot/lidar/points [sensor_msgs/msg/PointCloud2]
/spot/d455/color/image [sensor_msgs/msg/Image]
/spot/d455/color/camera_info [sensor_msgs/msg/CameraInfo]
/spot/d455/depth/image [sensor_msgs/msg/Image]
/spot/d455/imu [sensor_msgs/msg/Imu]
```

Expected sensor `header.frame_id` values:

```text
/spot/lidar/points -> sensor
/spot/d455/color/image -> Camera_OmniVision_OV9782_Color
/spot/d455/color/camera_info -> Camera_OmniVision_OV9782_Color
/spot/d455/depth/image -> Camera_Pseudo_Depth
/spot/d455/imu -> Imu_Sensor
```

## Terminal 1: Run Isaac Sim Runtime

Run the existing Isaac Sim runtime. Do not save the USD stage.

```bash
/home/aes/isaacsim/python.sh /home/aes/isaac_ws/isaacsim_spatial_pipeline/scripts/10_run_sim.py
```

After Isaac Sim opens, press Play so `/clock`, `/tf`, and sensor topics publish.

## Terminal 2: Source ROS2 Jazzy

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
```

Confirm the live topic contract:

```bash
ros2 topic list -t
python3 /home/aes/isaac_ws/isaacsim_spatial_pipeline/scripts/20_validate_tf_contract.py
```

## Terminal 3: Open RViz2

Source ROS2 before launching RViz2:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
rviz2 -d /home/aes/isaac_ws/isaacsim_spatial_pipeline/rviz/m03_sensor_tf_baseline.rviz
```

The baseline config sets:

```text
Fixed Frame: world
TF display: enabled
PointCloud2 topic: /spot/lidar/points
Color image topic: /spot/d455/color/image
Depth image topic: /spot/d455/depth/image
IMU topic: /spot/d455/imu
```

RViz2 should show a coherent TF tree and LiDAR points in the `world` fixed
frame. The image displays should update while Isaac Sim playback is active.

If RViz2 reports missing transforms, re-run:

```bash
ros2 topic echo /tf --once
ros2 topic hz /tf
ros2 topic echo /spot/lidar/points --field header --once
```

## Manual Rosbag Recording

Record bags manually only. Do not record bags from Codex.

Create a local ignored bag directory if needed:

```bash
mkdir -p /home/aes/isaac_ws/bags
```

Start recording after Isaac Sim playback and Milestone #2 validation are
passing:

```bash
ros2 bag record \
  -o /home/aes/isaac_ws/bags/m03_sensor_tf_baseline_$(date +%Y%m%d_%H%M%S) \
  /clock \
  /tf \
  /spot/lidar/points \
  /spot/d455/color/image \
  /spot/d455/color/camera_info \
  /spot/d455/depth/image \
  /spot/d455/imu
```

Stop recording with `Ctrl-C` after a short controlled run. Keep bags local; the
`bags/` directory is ignored and should not be committed.

Inspect the recorded bag metadata:

```bash
ros2 bag info /home/aes/isaac_ws/bags/<bag_directory>
```

Run the read-only bag contract helper:

```bash
python3 /home/aes/isaac_ws/isaacsim_spatial_pipeline/scripts/30_check_bag_contract.py /home/aes/isaac_ws/bags/<bag_directory>
```

## Rosbag Replay With Clock

Use a fresh ROS2 terminal:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
```

Replay with recorded timing and `/clock`:

```bash
ros2 bag play /home/aes/isaac_ws/bags/<bag_directory> --clock
```

In another sourced terminal, verify replayed topics:

```bash
ros2 topic list -t
ros2 topic echo /clock --once
ros2 topic echo /tf --once
ros2 topic hz /tf
ros2 topic hz /spot/lidar/points
```

Check replayed sensor headers:

```bash
ros2 topic echo /spot/lidar/points --field header --once
ros2 topic echo /spot/d455/color/image --field header --once
ros2 topic echo /spot/d455/color/camera_info --field header --once
ros2 topic echo /spot/d455/depth/image --field header --once
ros2 topic echo /spot/d455/imu --field header --once
```

Expected frame IDs after replay:

```text
/spot/lidar/points: sensor
/spot/d455/color/image: Camera_OmniVision_OV9782_Color
/spot/d455/color/camera_info: Camera_OmniVision_OV9782_Color
/spot/d455/depth/image: Camera_Pseudo_Depth
/spot/d455/imu: Imu_Sensor
```

Open RViz2 against replay:

```bash
rviz2 -d /home/aes/isaac_ws/isaacsim_spatial_pipeline/rviz/m03_sensor_tf_baseline.rviz
```

Use `world` as the fixed frame. Do not change frame names to make replay look
better; any mismatch should be documented as a contract issue.

## Pass Criteria

Milestone #3 passes when:

- Isaac Sim runtime publishes the validated topics without saving USD changes.
- RViz2 opens with `Fixed Frame` set to `world`.
- RViz2 TF display shows the Isaac-derived frame tree.
- RViz2 displays `/spot/lidar/points` without fixed-frame transform errors.
- RViz2 image displays update for color and depth topics.
- Manual bag recording contains all validated topics with expected types.
- The read-only bag contract helper reports `Overall result: PASS`.
- `ros2 bag play <bag> --clock` republishes `/clock`, `/tf`, and all sensor
  topics.
- Replayed sensor headers keep the expected `frame_id` values.
- No USD files are saved or modified.
- No `base_link`, `odom`, `map`, or static alias frames are introduced by this
  milestone.

## Fail Criteria

Milestone #3 fails if:

- Any validated baseline topic is missing during live validation or replay.
- Any baseline topic has a different message type than expected.
- Sensor `header.frame_id` values differ from the Milestone #2 contract.
- RViz2 cannot transform `/spot/lidar/points` into `world`.
- The recorded bag omits `/clock` or `/tf`.
- Replay does not publish `/clock` when started with `--clock`.
- Fixing the issue requires changing frame names, adding aliases, or starting
  SLAM/localization. Those decisions belong to Milestone #4 planning.

## Out Of Scope

Milestone #3 does not include:

- LiDAR SLAM.
- Localization.
- Map generation.
- `map` or `odom` publishing.
- `base_link` alias creation.
- Camera optical-frame alias creation.
- Static transform publisher setup.
- USD edits or saved ROS2 bridge graphs.

## Next Milestone

Milestone #4 should choose the SLAM/localization path based on the validated
live and replay contracts. Before configuring SLAM, decide whether to consume
the Isaac-derived frames directly or add explicit alias/static frames in a
controlled, documented step.

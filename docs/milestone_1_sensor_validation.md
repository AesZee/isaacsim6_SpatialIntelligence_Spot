# Milestone 1 Sensor Validation

## Purpose

Milestone 1 validates the live ROS2 outputs from Isaac Sim before any SLAM implementation. The goal is to replace placeholder topic and frame names with verified names, message types, frame IDs, timestamps, and TF connectivity.

Do not implement SLAM until this milestone is complete.

## Preconditions

- Start Isaac Sim manually.
- Open the warehouse Spot scene manually.
- Enable and run the Isaac Sim ROS2 bridge manually.
- Do not run Isaac Sim GUI from Codex.
- Source ROS2 in the terminal that will run validation commands.

If `$ROS_DISTRO` is not set, choose the ROS2 distro installed on your machine and source it explicitly:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
```

Check the environment:

```bash
echo "$ROS_DISTRO"
ros2 --version
```

Expected result:

- `echo "$ROS_DISTRO"` prints a distro name such as `humble`, `jazzy`, or another installed distro.
- `ros2 --version` prints the installed ROS2 CLI version.

If `$ROS_DISTRO` is empty, set it for the current shell or source the correct setup file directly. Do not edit `.bashrc` as part of this project setup.

## Expected Topic Name Placeholders

Replace these placeholders after Isaac Sim publishes actual topics:

```text
<lidar_points_topic>
<camera_color_topic>
<camera_depth_topic>
<camera_info_topic>
<imu_topic>
<clock_topic>
```

Likely message families to look for:

- LiDAR point cloud: `sensor_msgs/msg/PointCloud2`
- RGB image: `sensor_msgs/msg/Image`
- Depth image: `sensor_msgs/msg/Image`
- Camera info: `sensor_msgs/msg/CameraInfo`
- IMU: `sensor_msgs/msg/Imu`
- Clock: `rosgraph_msgs/msg/Clock`
- TF: `tf2_msgs/msg/TFMessage`

Do not assume the exact topic names.

## List Topics

```bash
ros2 topic list
```

More detail:

```bash
ros2 topic list -t
```

Expected result:

- Sensor topics appear for LiDAR, RGB image, depth image, camera info, and IMU.
- `/tf` and/or `/tf_static` appear.
- `/clock` may appear if Isaac Sim publishes simulation time.

## Check Topic Types

Run this for each relevant topic:

```bash
ros2 topic type <topic>
```

Examples after replacing placeholders:

```bash
ros2 topic type <lidar_points_topic>
ros2 topic type <camera_color_topic>
ros2 topic type <camera_depth_topic>
ros2 topic type <camera_info_topic>
ros2 topic type <imu_topic>
```

Expected result:

- LiDAR topic should usually be `sensor_msgs/msg/PointCloud2`.
- Camera image topics should usually be `sensor_msgs/msg/Image`.
- Camera info should usually be `sensor_msgs/msg/CameraInfo`.
- IMU should usually be `sensor_msgs/msg/Imu`.

## Check Topic Frequency

Run frequency checks one topic at a time:

```bash
ros2 topic hz <topic>
```

Examples:

```bash
ros2 topic hz <lidar_points_topic>
ros2 topic hz <camera_color_topic>
ros2 topic hz <camera_depth_topic>
ros2 topic hz <imu_topic>
```

Expected result:

- Output reports average rate, min, max, and standard deviation.
- The rate should be stable enough for downstream SLAM or visualization.
- Record the observed rates in project notes.

## Echo Message Headers

Inspect message headers to verify frame IDs and timestamps.

For small header samples:

```bash
ros2 topic echo <topic> --once
```

For high-volume image or point cloud topics, use field selection when available:

```bash
ros2 topic echo <lidar_points_topic> --field header --once
ros2 topic echo <camera_color_topic> --field header --once
ros2 topic echo <camera_depth_topic> --field header --once
ros2 topic echo <camera_info_topic> --field header --once
ros2 topic echo <imu_topic> --field header --once
```

Expected result:

- Each sensor message has a non-empty `header.frame_id`.
- Header timestamps are changing while simulation runs.
- Frame IDs match or can be connected through TF.

If `--field` is not supported by the installed ROS2 CLI, use `--once` and stop after the header is visible.

## Check Clock

```bash
ros2 topic list -t | grep /clock
ros2 topic echo /clock --once
```

Expected result:

- If `/clock` exists, it should publish simulation time.
- RViz2 and ROS2 nodes may need `use_sim_time:=true` later.

If `/clock` does not exist, document that Isaac Sim is not currently publishing simulation time through ROS2.

## Check TF Topics

```bash
ros2 topic type /tf
ros2 topic type /tf_static
ros2 topic echo /tf --once
ros2 topic echo /tf_static --once
```

Expected result:

- `/tf` and `/tf_static` should be `tf2_msgs/msg/TFMessage` when present.
- Dynamic transforms should appear on `/tf`.
- Static sensor mount transforms should ideally appear on `/tf_static`.

If `/tf_static` does not print with `--once`, try:

```bash
ros2 topic echo /tf_static
```

Then stop it manually after a sample is printed.

## Generate TF Tree

Install nothing from this repository. Use the ROS2 installation already present on the machine.

```bash
ros2 run tf2_tools view_frames
```

Expected result:

- A frames report is generated in the current directory, commonly `frames.pdf` or `frames.gv`.
- The TF tree should connect robot base, LiDAR, camera, and IMU frames.
- `map` may be absent at this stage because SLAM/localization has not been started.

If `tf2_tools` is not available, document that the package is missing and continue with topic-level TF inspection. Do not install packages from Codex.

## RViz2 Visualization

Start RViz2 manually:

```bash
rviz2
```

Recommended displays:

- TF
- RobotModel, if a robot description is available later
- PointCloud2 for the LiDAR topic
- Image for RGB
- Image for depth
- Imu, if available in the installed RViz2 plugins

RViz2 checks:

- Set the fixed frame to a verified frame such as `base_link`, `odom`, or the actual frame reported by TF.
- If `/clock` is published, enable simulation time in RViz2 if needed.
- Confirm the point cloud appears in the expected direction relative to Spot.
- Confirm RGB/depth images update.
- Confirm TF axes are stable and connected.

## Manual Rosbag2 Recording

Do not record bags from Codex. Run this manually only after topics and frame IDs are verified.

Create a local ignored bag directory:

```bash
mkdir -p bags
```

Record a short validation bag by replacing placeholders:

```bash
ros2 bag record -o bags/m1_sensor_validation \
  /tf \
  /tf_static \
  <lidar_points_topic> \
  <camera_color_topic> \
  <camera_depth_topic> \
  <camera_info_topic> \
  <imu_topic>
```

If `/clock` exists, include it:

```bash
ros2 bag record -o bags/m1_sensor_validation \
  /clock \
  /tf \
  /tf_static \
  <lidar_points_topic> \
  <camera_color_topic> \
  <camera_depth_topic> \
  <camera_info_topic> \
  <imu_topic>
```

Stop recording manually with `Ctrl-C` after a short run.

Expected result:

- A new ignored directory appears under `bags/`.
- The bag is not committed.
- The bag contains sensor topics plus TF and optionally `/clock`.

## Manual Rosbag2 Replay

Replay manually:

```bash
ros2 bag info bags/m1_sensor_validation
ros2 bag play bags/m1_sensor_validation
```

If the bag includes `/clock`, later consumers may need simulation time enabled.

Expected result:

- `ros2 bag info` shows recorded topics, message counts, duration, and storage ID.
- `ros2 bag play` republishes recorded topics.
- RViz2 can visualize replayed TF and sensor data when fixed frame and time settings are correct.

## Documentation Template

After running the checks, record:

```text
ROS_DISTRO:
Isaac Sim version:
ROS2 bridge status:

LiDAR topic:
LiDAR type:
LiDAR frame_id:
LiDAR rate:

RGB topic:
RGB type:
RGB frame_id:
RGB rate:

Depth topic:
Depth type:
Depth frame_id:
Depth rate:

Camera info topic:
Camera info type:
Camera info frame_id:

IMU topic:
IMU type:
IMU frame_id:
IMU rate:

/clock present:
/tf present:
/tf_static present:
Connected TF tree:
RViz2 fixed frame used:

Notes:
```

## Pass Criteria

Milestone 1 is complete when:

- Actual topic names are known.
- Actual message types are known.
- Sensor rates are measured.
- Sensor message headers have valid frame IDs and timestamps.
- TF connectivity is understood.
- RViz2 displays the relevant sensor outputs.
- A short manual rosbag2 recording profile is documented.

Only then should the project move to SLAM configuration.

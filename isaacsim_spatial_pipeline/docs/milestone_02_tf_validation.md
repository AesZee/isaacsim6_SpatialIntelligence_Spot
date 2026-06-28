# Milestone #2: TF / Frame Tree Validation

## Purpose

Milestone #2 validates the current Isaac Sim ROS2 topic and TF contract before
SLAM or localization configuration. The goal is to confirm that every published
sensor message `frame_id` exists in `/tf`, that important robot frames are
observable, and that the current Isaac-derived frame naming is documented.

This milestone does not redesign the TF tree, save USD files, publish aliases,
or record rosbag files.

## Terminal 1: Run Isaac Sim Runtime

Start Isaac Sim with the existing runtime script:

```bash
/home/aes/isaacsim/python.sh /home/aes/isaac_ws/isaacsim_spatial_pipeline/scripts/10_run_sim.py
```

After the GUI opens, press Play so the in-memory ROS2 bridge graphs publish
`/clock`, `/tf`, and sensor topics.

The runtime script opens:

```text
/home/aes/isaac_ws/scenes/Warehouse.usd
```

It creates ROS2 bridge OmniGraphs in memory only and should not save USD files.

## Terminal 2: ROS2 Environment

In a separate terminal:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
```

## Automated Contract Validation

Run the standalone validator while Isaac Sim is running and playback is active:

```bash
python3 /home/aes/isaac_ws/isaacsim_spatial_pipeline/scripts/20_validate_tf_contract.py
```

Optional bounded collection duration:

```bash
python3 /home/aes/isaac_ws/isaacsim_spatial_pipeline/scripts/20_validate_tf_contract.py --duration 5.0
```

The validator checks:

- `/clock` exists and publishes messages.
- `/tf` exists and publishes `tf2_msgs/msg/TFMessage`.
- Expected sensor topics exist.
- Expected topic types match.
- One message from each sensor topic has the expected `header.frame_id`.
- Required sensor frames are present in observed TF.
- Important robot frames are present when observed during the bounded window.
- `base_link`, `odom`, and `map` are not expected yet.

The script is read-only. It does not save USD files and does not record bags.

## Observed Result

Observed on 2026-06-28 with Isaac Sim running and ROS2 Jazzy sourced:

```text
Automated validator overall result: PASS
```

Observed topic list and types:

```text
/clock [rosgraph_msgs/msg/Clock]
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]
/spot/d455/color/camera_info [sensor_msgs/msg/CameraInfo]
/spot/d455/color/image [sensor_msgs/msg/Image]
/spot/d455/depth/image [sensor_msgs/msg/Image]
/spot/d455/imu [sensor_msgs/msg/Imu]
/spot/lidar/points [sensor_msgs/msg/PointCloud2]
/tf [tf2_msgs/msg/TFMessage]
```

## Manual Topic Checks

Use these checks if the automated validator reports a warning or failure:

```bash
ros2 topic list -t
ros2 topic echo /tf --once
ros2 topic hz /tf
ros2 topic hz /spot/lidar/points
```

Additional one-message checks:

```bash
ros2 topic echo /clock --once
ros2 topic echo /spot/lidar/points --once
ros2 topic echo /spot/d455/color/image --once
ros2 topic echo /spot/d455/color/camera_info --once
ros2 topic echo /spot/d455/depth/image --once
ros2 topic echo /spot/d455/imu --once
```

## Expected Topics

```text
/clock
  type: rosgraph_msgs/msg/Clock

/tf
  type: tf2_msgs/msg/TFMessage

/spot/lidar/points
  type: sensor_msgs/msg/PointCloud2

/spot/d455/color/image
  type: sensor_msgs/msg/Image

/spot/d455/color/camera_info
  type: sensor_msgs/msg/CameraInfo

/spot/d455/depth/image
  type: sensor_msgs/msg/Image

/spot/d455/imu
  type: sensor_msgs/msg/Imu
```

## Expected Sensor Frame IDs

```text
/spot/lidar/points
  frame_id: sensor

/spot/d455/color/image
  frame_id: Camera_OmniVision_OV9782_Color

/spot/d455/color/camera_info
  frame_id: Camera_OmniVision_OV9782_Color

/spot/d455/depth/image
  frame_id: Camera_Pseudo_Depth

/spot/d455/imu
  frame_id: Imu_Sensor
```

## Required Sensor Frames In TF

These frames must be observed in `/tf` because sensor messages reference them:

```text
sensor
Camera_OmniVision_OV9782_Color
Camera_Pseudo_Depth
Imu_Sensor
```

## Important Robot Frames

These frames are expected from the current Isaac-derived TF tree. Missing values
should be treated as warnings first because the validator observes `/tf` for a
bounded duration.

```text
world
body
lidar_link
OS1
rsd455_link
RSD455
```

Observed TF may also include:

```text
Camera_OmniVision_OV9782_Left
Camera_OmniVision_OV9782_Right
Spot leg and foot frames
```

## Known Limitations

- Current TF is Isaac-derived and uses Isaac prim names.
- `body` is currently the base-like frame.
- `base_link` is not currently expected.
- `odom` is not currently expected.
- `map` is not currently expected.
- SLAM/localization may later introduce `map` and `odom`.
- Frame aliases such as `base_link`, `os1_frame`,
  `rsd455_color_optical_frame`, `rsd455_depth_optical_frame`, and
  `rsd455_imu_frame` should be added only after this contract is stable.
- The validator samples `/tf` for a short bounded window, so important robot
  frames that are not observed are warnings unless they are sensor frame
  dependencies.

## Pass Criteria

Milestone #2 passes when:

- `/clock` exists and publishes messages.
- `/tf` exists and publishes messages.
- All expected topics exist with expected message types.
- Each expected sensor topic publishes at least one message.
- Each sensor message has the expected `header.frame_id`.
- Required sensor frames are present in `/tf`.
- Missing `base_link`, `odom`, and `map` are reported as expected for this
  stage.
- No USD, rosbag, MCAP, or DB3 files are created by validation.

Warnings for important robot frames should be reviewed manually before SLAM
configuration, but they do not automatically require a TF redesign.

## Next Milestone Recommendation

After this contract is stable:

1. Validate the same topics and frames in RViz2.
2. Define a conservative manual rosbag recording profile.
3. Confirm replay behavior with `/clock`, `/tf`, LiDAR, camera, and IMU topics.
4. Decide whether downstream SLAM will consume Isaac prim-name frames directly
   or whether explicit alias/static frames should be introduced.

# ROS2 Topic Contract

This file records observed ROS2 bridge outputs from live Isaac Sim validation.
Do not replace these with desired names unless the USD and bridge graphs have
also been changed and revalidated.

## Environment

Observed on 2026-06-25:

```text
ROS_DISTRO: jazzy
ROS_DOMAIN_ID: 0
/clock available: yes
/tf available: yes
/tf_static available: not observed in the current topic list
```

Run terminals with:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
```

## Validated Topics

```text
/clock
  type: rosgraph_msgs/msg/Clock
  notes: Isaac simulation time is published.

/tf
  type: tf2_msgs/msg/TFMessage
  notes: Dynamic transforms are published from Isaac prims.

/spot/lidar/points
  type: sensor_msgs/msg/PointCloud2
  observed frequency: about 2.12 Hz
  configured header.frame_id: sensor
  notes: OmniLidar prim is /World/spot_lidar_realsense/body/lidar_link/OS1/sensor.

/spot/d455/color/image
  type: sensor_msgs/msg/Image
  configured header.frame_id: Camera_OmniVision_OV9782_Color

/spot/d455/color/camera_info
  type: sensor_msgs/msg/CameraInfo
  configured header.frame_id: Camera_OmniVision_OV9782_Color

/spot/d455/depth/image
  type: sensor_msgs/msg/Image
  configured header.frame_id: Camera_Pseudo_Depth

/spot/d455/imu
  type: sensor_msgs/msg/Imu
  configured header.frame_id: Imu_Sensor
```

The current scripts intentionally set sensor message frame IDs to names that
exist in `/tf`. If USD prim names change, update both the graph scripts and this
contract.

## TF

Observed TF frames include:

```text
world
body
lidar_link
OS1
sensor
rsd455_link
RSD455
Camera_Pseudo_Depth
Camera_OmniVision_OV9782_Color
Camera_OmniVision_OV9782_Left
Camera_OmniVision_OV9782_Right
Imu_Sensor
Spot leg and foot frames
```

Current TF behavior:

```text
map: not published by Isaac; expected later from SLAM/localization
odom: not published by this script
base_link: not currently present; Isaac publishes body
LiDAR data frame: sensor
camera color frame: Camera_OmniVision_OV9782_Color
camera depth frame: Camera_Pseudo_Depth
IMU frame: Imu_Sensor
```

The current TF tree is Isaac-derived and uses prim names. Before SLAM, decide
whether to keep these names or add explicit aliases such as `base_link`,
`os1_frame`, and camera optical frames.

## Manual Validation Commands

```bash
ros2 topic list -t
ros2 topic echo /clock --once
ros2 topic echo /tf --once
ros2 topic echo /spot/lidar/points --once
ros2 topic echo /spot/d455/color/image --once
ros2 topic echo /spot/d455/depth/image --once
ros2 topic echo /spot/d455/imu --once
```

Frequency checks:

```bash
ros2 topic hz /clock
ros2 topic hz /tf
ros2 topic hz /spot/lidar/points
ros2 topic hz /spot/d455/color/image
ros2 topic hz /spot/d455/depth/image
ros2 topic hz /spot/d455/imu
```

Stop each `ros2 topic hz` command with `Ctrl-C` after a few samples.

## Bag Profile

Do not record rosbag files from Codex. After RViz2 validation passes, use a
manual profile such as:

```bash
ros2 bag record \
  /clock \
  /tf \
  /spot/lidar/points \
  /spot/d455/color/image \
  /spot/d455/color/camera_info \
  /spot/d455/depth/image \
  /spot/d455/imu
```

# OmniGraph Notes

The ROS2 graph scripts currently author graphs in memory and do not save USD
changes. The primary runtime entry point is:

```bash
/home/aes/isaacsim/python.sh /home/aes/isaac_ws/isaacsim_spatial_pipeline/scripts/10_run_sim.py
```

## Current Graphs

```text
/World/ROS2/Clock
  OnPlaybackTick
  IsaacReadSimulationTime
  ROS2Context
  ROS2PublishClock

/World/ROS2/TF
  OnPlaybackTick
  IsaacReadSimulationTime
  ROS2Context
  IsaacComputeTransformTree
  ROS2PublishTransformTree

/World/ROS2/Sensors
  OnPlaybackTick
  OgnIsaacRunOneSimulationFrame
  ROS2Context
  IsaacCreateRenderProduct for OmniLidar
  ROS2RtxLidarHelper
  IsaacCreateRenderProduct for color camera
  ROS2CameraHelper
  ROS2CameraInfoHelper
  IsaacCreateRenderProduct for depth camera
  ROS2CameraHelper
  IsaacReadIMU
  ROS2PublishImu
```

## Validated Sensor Prims

```text
Robot root:
  /World/spot_lidar_realsense

LiDAR model container:
  /World/spot_lidar_realsense/body/lidar_link/OS1

Actual OmniLidar prim:
  /World/spot_lidar_realsense/body/lidar_link/OS1/sensor

Color camera:
  /World/spot_lidar_realsense/body/rsd455_link/RSD455/Camera_OmniVision_OV9782_Color

Depth camera:
  /World/spot_lidar_realsense/body/rsd455_link/RSD455/Camera_Pseudo_Depth

IMU:
  /World/spot_lidar_realsense/body/rsd455_link/RSD455/Imu_Sensor
```

Important LiDAR detail: `/OS1` is an `Xform` container. The render product must
target `/OS1/sensor`, which is the actual `OmniLidar` prim with
`OmniSensorGenericLidarCoreAPI`.

## Authoring Order

1. Open the warehouse stage.
2. Verify Spot, LiDAR, camera, and IMU prim paths.
3. Enable `isaacsim.ros2.bridge`.
4. Warm up the app for several frames so the ROS2 extension finishes startup.
5. Create `/clock` publishing.
6. Create TF publishing from verified prim paths.
7. Create sensor publishing graphs.
8. Press Play in Isaac Sim.
9. Validate ROS2 topics, message types, frame IDs, timestamps, and frequencies.

## Current Limitations

The current TF graph publishes Isaac prim names. It does not yet create the
portfolio-preferred frame aliases:

```text
base_link
os1_frame
rsd455_color_optical_frame
rsd455_depth_optical_frame
rsd455_imu_frame
```

For Milestone 1, the sensor message `frame_id` values are aligned to the TF
frames Isaac actually publishes. Add aliases later only after deciding whether
to modify USD prim names or publish additional static transforms.

Do not save a ROS2-ready USD until the topic and TF contract is stable.

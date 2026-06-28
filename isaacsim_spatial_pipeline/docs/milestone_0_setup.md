# Milestone 0 Setup

## Summary

Milestone 0 establishes the simulated robotics scene used by the spatial intelligence pipeline. The current repository contains Isaac Sim assets and a top-level warehouse scene for a Spot-based autonomous mobile robot setup.

## Simulator

- Isaac Sim 6
- Scene file: `scenes/Warehouse.usd`
- Environment: indoor warehouse/logistics scene

## Robot

- Robot: Boston Dynamics Spot
- Spot asset collection: `assets/Collected_spot/`
- Current Spot scene variants include:
  - `assets/Collected_spot/spot.usd`
  - `assets/Collected_spot/spot_lidar_realsense.usd`

## Sensors

### Ouster OS1 LiDAR

- Asset collection: `assets/Collected_OS1_REV6_32_10hz___1024_resolution/`
- Manual Isaac Sim mount point: `lidar_link`
- Intended use: primary LiDAR input for SLAM and mapping evaluation.

### Intel RealSense D455 RGB-D Camera

- Asset collection: `assets/Collected_D455/`
- Manual Isaac Sim mount point: `rsd455_link`
- Intended use: RGB-D perception context, RViz2 validation, and future sensor fusion or qualitative scene documentation.

### D455 IMU

- Mounted under the RealSense sensor tree in Isaac Sim.
- Intended use: timestamp, frame, and inertial topic validation before any downstream integration.

## Conceptual TF Structure

The desired conceptual structure is:

```text
map
└── odom
    └── base_link
        ├── lidar_link
        │   └── os1_frame
        └── rsd455_link
            ├── rsd455_color_optical_frame
            ├── rsd455_depth_optical_frame
            └── rsd455_imu_frame
```

Important notes:

- `map` should be created by SLAM or localization, not manually by Isaac Sim.
- `odom` should represent robot odometry.
- `base_link` should represent the Spot body/base frame.
- `lidar_link` and `rsd455_link` are conceptual mount frames from the current Isaac Sim setup.
- `os1_frame`, camera optical frames, and IMU frame names are placeholders until verified from live ROS2 messages and TF.

## Current Assumptions

- Isaac Sim can publish sensor data through the ROS2 bridge.
- The OS1 LiDAR, D455 RGB-D camera, and D455 IMU are mounted in the stage.
- The repository should treat Isaac Sim USD/assets as source assets, not generated files to rewrite during ROS2 validation.
- ROS2 will be sourced manually by the user before validation commands are run.

## Current Unknowns

- Actual ROS2 topic names published by Isaac Sim.
- Actual message types for LiDAR, RGB, depth, camera info, and IMU topics.
- Actual `header.frame_id` values in each sensor message.
- Actual TF frame names and transform connectivity.
- Whether `/clock` is published and whether RViz2 should use simulation time.
- Sensor publish rates and timestamp consistency.
- Whether static sensor transforms are published on `/tf_static` or only represented in USD.

These unknowns are the focus of Milestone 1.

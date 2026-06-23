# Codex Instructions

## Project Identity

This repository is the portfolio project for:

Spatial Intelligence Pipeline for AMR: LiDAR SLAM, Localization, and Mapping Evaluation

The project demonstrates a clean ROS2-based spatial intelligence workflow for an autonomous mobile robot in simulation. The intended pipeline covers Isaac Sim sensor publishing, ROS2 topic and TF validation, rosbag2 logging, LiDAR SLAM, localization, and mapping evaluation.

## Current Simulation Setup

- Simulator: Isaac Sim 6
- Robot: Boston Dynamics Spot
- Environment: indoor warehouse/logistics scene
- Primary scene: `scenes/Warehouse.usd`
- Existing asset collections:
  - `assets/Collected_spot/`
  - `assets/Collected_D455/`
  - `assets/Collected_OS1_REV6_32_10hz___1024_resolution/`
  - `assets/Collected_full_warehouse/`
  - `assets/Collected_Go2/`
- Sensor setup in Isaac Sim:
  - Ouster OS1 LiDAR mounted under `lidar_link`
  - Intel RealSense D455 RGB-D camera mounted under `rsd455_link`
  - D455 IMU mounted under the RealSense sensor tree

Do not assume the current USD stage publishes the expected ROS2 topics or frame IDs. Verify the actual Isaac Sim ROS2 bridge outputs before implementing downstream robotics code.

## Expected Repository Structure

- `assets/`: Isaac Sim USD assets and collected dependencies. Do not rewrite or regenerate these files casually.
- `scenes/`: top-level Isaac Sim stages.
- `scripts/`: lightweight local validation helpers and small scripts.
- `docs/`: milestone documentation and validation guides.
- `ros2_ws/`: future ROS2 workspace. Generated `build/`, `install/`, and `log/` directories are ignored.
- `bags/`: future local rosbag2 recordings. This directory is ignored and should not be committed.
- `logs/`: local logs or run notes. Avoid committing large generated output.

## Robotics Constraints

- Always inspect ROS2 topics before assuming names.
- Always inspect message types before writing launch files, subscribers, or SLAM configuration.
- Confirm frame IDs from message headers and TF, not from desired naming alone.
- Treat timestamps as a first-class validation target:
  - Check `header.stamp` on sensor messages.
  - Check whether Isaac Sim publishes simulation time.
  - Check `/clock` availability when `use_sim_time` is required.
- Validate TF before SLAM:
  - Confirm `/tf` and `/tf_static` exist.
  - Confirm transform connectivity between robot base, LiDAR, camera, IMU, odometry, and map frames.
  - Confirm no duplicate frame IDs or unexpected disconnected TF trees.
- Use RViz2 for visual checks after terminal checks pass.
- Use rosbag2 only after topics, frame IDs, and timestamps are understood.
- Do not record rosbag files automatically from Codex. Provide manual commands for the user to run.

Preferred conceptual TF tree to document and compare against:

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

`map` is created by SLAM/localization, not manually by Isaac Sim. Actual Isaac Sim topic names and frame IDs must be verified first.

## Milestone Plan

- Milestone 0: Isaac Sim robot, warehouse, and sensor setup.
  - Spot loaded in the warehouse scene.
  - OS1, D455 RGB-D, and D455 IMU mounted in Isaac Sim.
- Milestone 1: Sensor validation.
  - Validate ROS2 environment.
  - List topics and message types.
  - Inspect topic frequencies.
  - Inspect message headers, frame IDs, and timestamps.
  - Inspect `/tf` and `/tf_static`.
  - Validate RViz2 visualization.
  - Define safe rosbag2 recording profiles.
- Milestone 2: Rosbag2 capture workflow.
  - Create repeatable manual bagging profiles.
  - Define naming conventions and metadata notes.
  - Verify replay behavior.
- Milestone 3: LiDAR SLAM integration.
  - Select SLAM package after sensor and TF validation.
  - Configure SLAM using verified topics and frames.
- Milestone 4: Localization.
  - Add map-based localization once mapping output exists.
  - Validate localization stability and transform behavior.
- Milestone 5: Mapping evaluation.
  - Define metrics for map quality, drift, repeatability, and runtime behavior.
  - Evaluate maps from controlled simulated runs.
- Milestone 6: Portfolio packaging.
  - Add final diagrams, run instructions, results, and demo material.

## Development Rules

- Keep changes small and reviewable.
- Prefer documentation and validation helpers before feature implementation.
- Do not implement SLAM until Milestone 1 validation is complete.
- Do not silently assume a ROS2 distro. If `$ROS_DISTRO` is not set, document that the user must set or source it.
- Do not add heavy dependencies.
- Do not install system packages.
- Do not modify `.bashrc`.
- Do not run Isaac Sim GUI from Codex.
- Do not record rosbag files from Codex.
- Do not create large binary files.
- Do not commit unless the user explicitly asks.

## Safety Rules

- Do not delete existing files.
- Do not overwrite Isaac Sim USD files or collected assets.
- Do not move asset folders.
- Do not rewrite scene files as part of documentation or validation setup.
- Treat generated ROS2 workspace directories and bags as local artifacts.
- Ask before any destructive Git operation.

## Verification Expectations

For repository setup tasks:

- Run `git status --short` before and after changes.
- Confirm that only intended text/script files changed.
- Confirm no USD, PNG, MDL, DB3, MCAP, or bag files were created or modified.

For ROS2 validation tasks:

- Verify `$ROS_DISTRO` or explicitly document that it is unset.
- Run topic, type, frequency, header, TF, and RViz2 checks manually with Isaac Sim and the ROS2 bridge running.
- Save observations in docs or a small text note, not in large generated logs.

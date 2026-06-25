# Isaac Sim Agent Guidelines

## Purpose

This file gives future coding agents a concise operating guide for this repository's Isaac Sim and ROS 2 workflow. It is written for the portfolio project:

Spatial Intelligence Pipeline for AMR: LiDAR SLAM, Localization, and Mapping Evaluation

Follow the repository `AGENTS.md` first. Use this document as Isaac Sim-specific guidance when adding validation notes, helper scripts, ROS 2 launch/config files, or milestone documentation.

## Documentation Baseline

- Primary documentation requested for this project: Isaac Sim 6.0.1, https://docs.isaacsim.omniverse.nvidia.com/6.0.1/index.html
- When checking NVIDIA docs, prefer versioned Isaac Sim 6.0.1 pages when available. If NVIDIA redirects or serves the current docs path, confirm that the page applies to Isaac Sim 6.x before using it.
- Treat documentation examples as patterns, not as proof that this repository's stage publishes the same topics, frames, rates, or QoS settings.

## Agent Operating Rules

- Do not run Isaac Sim GUI from Codex.
- Do not modify `.bashrc`.
- Do not install system packages.
- Do not record rosbag files from Codex.
- Do not edit USD scene files or collected assets unless the user explicitly asks for that exact change.
- Do not assume ROS 2 topic names, message types, frame IDs, timestamps, or TF connectivity.
- Prefer small text docs and lightweight validation scripts before downstream robotics implementation.
- Keep generated data out of git: bags, large logs, generated ROS workspace directories, screenshots, videos, and binary simulation outputs.

## ROS 2 Environment Rules

Before writing ROS 2 code or launch files, verify the active environment:

```bash
echo "$ROS_DISTRO"
command -v ros2
ros2 --version
```

If `$ROS_DISTRO` is empty, document that ROS 2 has not been sourced. Do not silently choose a distro. The Isaac Sim docs recommend Humble and Jazzy for Isaac Sim ROS 2 workflows, but this repository must use the user's installed and sourced environment.

Use terminal-local setup commands in docs and examples:

```bash
source /opt/ros/<distro>/setup.bash
```

Do not add source lines to `.bashrc`.

## ROS 2 Bridge Rules

- Isaac Sim communicates with ROS through the ROS 2 bridge extension.
- Source the intended ROS 2 environment before launching Isaac Sim when using native ROS libraries.
- For same-machine ROS 2 Bridge use, default Fast DDS is usually the first path to verify.
- For multiple machines or containers, document middleware and discovery assumptions explicitly.
- If changing DDS middleware, document `RMW_IMPLEMENTATION` and retest topic visibility.
- If ROS topics are missing, first verify that Isaac Sim is playing, the bridge is enabled, and the relevant OmniGraph nodes or helper publishers exist.

## Sensor Publication Rules

Validate live topics before implementing subscribers or SLAM configuration:

```bash
ros2 topic list -t
ros2 topic type <topic>
ros2 topic hz <topic>
ros2 topic echo <topic> --field header --once
```

Expected message families to look for:

- LiDAR point clouds: `sensor_msgs/msg/PointCloud2`
- RGB and depth images: `sensor_msgs/msg/Image`
- Camera calibration: `sensor_msgs/msg/CameraInfo`
- IMU: `sensor_msgs/msg/Imu`
- Clock: `rosgraph_msgs/msg/Clock`
- TF: `tf2_msgs/msg/TFMessage`

Do not hard-code these as facts until validated on the live stage.

## RTX LiDAR Rules

- Isaac Sim RTX LiDAR can publish ROS 2 `LaserScan` and `PointCloud2` outputs through RTX LiDAR ROS 2 helper workflows.
- Each RTX sensor needs a render product path in the publishing pipeline.
- In Isaac Sim 6.x, RTX LiDAR publish rates are governed by the `omni:sensor:tickRate` attribute on the `OmniLidar` prim, not by older helper-node frame-skip assumptions.
- For accumulated scans, check that `omni:sensor:tickRate` and `omni:sensor:Core:scanRateBaseHz` are consistent.
- Record the verified point cloud topic, frame ID, rate, point density behavior, and timestamp behavior before choosing a SLAM package.

## Camera And Depth Rules

- ROS 2 camera publishing should be verified from the actual Camera Helper or equivalent OmniGraph setup.
- Confirm the RGB image, depth image, and camera info topics separately.
- Confirm that image and camera info headers use compatible timestamps and frame IDs.
- Do not assume RealSense-style topic names just because the simulated sensor is a D455.
- If RViz2 image visualization fails, reduce the problem to topic type, frequency, header, and QoS checks before changing simulation assets.

## IMU Rules

- Confirm the IMU sensor is attached to a rigid body prim in Isaac Sim.
- Confirm the ROS 2 IMU topic type, header frame ID, publish rate, angular velocity, linear acceleration, and orientation fields.
- Document whether gravity is included in the linear acceleration values if that setting is known from the stage or observed behavior.
- Do not use IMU data for localization or sensor fusion until frame orientation and timestamps are understood.

## Clock And Timestamp Rules

Treat time as a first-class validation target:

```bash
ros2 topic list -t | grep /clock
ros2 topic echo /clock --once
ros2 topic echo <sensor_topic> --field header --once
```

- If Isaac Sim publishes `/clock`, downstream ROS 2 nodes and RViz2 may need `use_sim_time:=true`.
- Confirm whether the stage publishes simulation time or system time.
- Check that sensor header stamps advance while simulation plays.
- If simulation is stopped and restarted, note whether time continues monotonically or resets.

## TF And Odometry Rules

Validate TF before SLAM:

```bash
ros2 topic list -t | grep -E '^/tf|^/tf_static'
ros2 topic type /tf
ros2 topic type /tf_static
ros2 topic echo /tf --once
ros2 topic echo /tf_static --once
```

Use `tf2_tools` only if it is already installed:

```bash
ros2 run tf2_tools view_frames
```

Compare the live tree to the conceptual project tree:

```text
map
`-- odom
    `-- base_link
        |-- lidar_link
        |   `-- os1_frame
        `-- rsd455_link
            |-- rsd455_color_optical_frame
            |-- rsd455_depth_optical_frame
            `-- rsd455_imu_frame
```

Important constraints:

- `map` is created by SLAM or localization, not manually by Isaac Sim during sensor validation.
- Isaac Sim TF publishing may require Compute Transform Tree plus ROS 2 Publish Transform Tree OmniGraph nodes.
- `odom -> base_link` must be verified from odometry or transform publisher output, not invented in launch files.
- Do not patch over disconnected TF trees with static transforms until the actual published frames are documented.

## QoS Rules

- Inspect QoS when topics are visible but subscribers, RViz2, or rosbag2 do not receive data:

```bash
ros2 topic info <topic> -v
```

- Sensor streams may use sensor-data QoS profiles.
- If depth appears as unknown in `ros2 topic info -v`, document the middleware and avoid assuming the setting was not configured.
- Match downstream subscriber QoS to the verified publisher behavior.

## RViz2 Rules

Start RViz2 manually after terminal checks pass:

```bash
rviz2
```

Recommended initial displays:

- TF
- PointCloud2 for the verified LiDAR topic
- Image for verified RGB and depth topics
- CameraInfo if needed for camera model debugging
- Odometry if a verified odometry topic exists

Set `use_sim_time` only after `/clock` is confirmed:

```bash
ros2 param set /rviz use_sim_time true
```

## Rosbag2 Rules

Do not record bags from Codex. Provide manual commands only after topics, frames, timestamps, rates, and QoS are understood.

Suggested manual recording pattern after validation:

```bash
ros2 bag record \
  /clock \
  /tf \
  /tf_static \
  <lidar_points_topic> \
  <camera_color_topic> \
  <camera_depth_topic> \
  <camera_info_topic> \
  <imu_topic>
```

Document bag metadata in a small text or markdown note:

- Isaac Sim version
- Scene file
- Robot and sensor setup
- ROS distro
- Topic list and types
- Frame IDs
- Whether `/clock` was used
- Short motion profile or route description

## SLAM And Localization Rules

- Do not implement SLAM until Milestone 1 validation is complete.
- Choose the SLAM package only after the LiDAR topic, frame ID, TF tree, clock behavior, and odometry availability are known.
- Do not create `map` or `odom` frames manually to satisfy a package unless the source of truth for those transforms is documented.
- Localization comes after a mapping output exists and replay behavior has been checked.

## Verification Checklist For Agent Changes

Before changes:

```bash
git status --short
```

After changes:

```bash
git status --short
git diff --stat
```

Confirm:

- Only intended text or script files changed.
- No USD, PNG, MDL, DB3, MCAP, bag, or large generated files were created or modified.
- Any existing unrelated modified files were not reverted.

## Source Links

- Isaac Sim 6.0.1 documentation index: https://docs.isaacsim.omniverse.nvidia.com/6.0.1/index.html
- Isaac Sim ROS 2 overview: https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/ros2_landing_page.html
- ROS 2 installation and bridge setup: https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html
- ROS 2 clock workflow: https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_ros2_clock.html
- ROS 2 camera workflow: https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_ros2_camera.html
- RTX LiDAR ROS 2 workflow: https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_ros2_rtx_lidar.html
- ROS 2 TF and odometry workflow: https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_ros2_tf.html
- ROS 2 QoS workflow: https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_ros2_qos.html
- Isaac Sim IMU sensor documentation: https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/isaacsim_sensors_physics_imu.html

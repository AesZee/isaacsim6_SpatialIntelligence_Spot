# Task Queue

## Completed

- Milestone 0 - Robot, environment, and sensor setup
  - Spot robot placed in the Isaac Sim warehouse/logistics scene.
  - Ouster OS1 LiDAR mounted under `lidar_link`.
  - Intel RealSense D455 RGB-D camera mounted under `rsd455_link`.
  - D455 IMU mounted under the RealSense sensor tree.
  - Asset collections committed locally for Spot, D455, OS1, and warehouse content.

## Active

- Milestone 1 - Sensor validation
  - Start Isaac Sim manually with the ROS2 bridge enabled.
  - Source the correct ROS2 environment.
  - List live topics.
  - Record actual topic names and message types.
  - Check topic frequencies.
  - Echo representative message headers.
  - Inspect TF and static TF.
  - Visualize robot, point cloud, camera, and TF in RViz2.

## Next

- TF and timestamp inspection
  - Verify `header.frame_id` for LiDAR, RGB, depth, camera info, and IMU messages.
  - Verify `header.stamp` behavior.
  - Check whether `/clock` is published.
  - Check whether RViz2 needs `use_sim_time`.
  - Generate a TF tree with `ros2 run tf2_tools view_frames`.
  - Compare actual frames against the conceptual TF tree documented in `docs/milestone_0_setup.md`.

## Later

- Rosbag2 profiles
  - Define small manual recording profiles for validation and later mapping.
  - Document bag naming conventions and metadata.
  - Verify replay timing and TF behavior.
- LiDAR SLAM
  - Select and configure a SLAM stack only after Milestone 1 is complete.
  - Use verified topics, frames, and timestamp behavior.
- Localization
  - Add map-based localization using the generated map.
  - Validate localization TF output and stability.
- Evaluation
  - Define map quality, drift, repeatability, and runtime metrics.
  - Compare mapping runs under controlled conditions.
- Portfolio packaging
  - Add diagrams, final runbook, screenshots, and evaluation results.

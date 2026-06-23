# Project Goals

## North Star

Build a clean, reproducible spatial intelligence pipeline for an autonomous mobile robot in an indoor warehouse simulation, using Isaac Sim 6, Spot, ROS2, LiDAR SLAM, localization, rosbag2 logging, and mapping evaluation.

## Final Portfolio Capabilities

- Isaac Sim warehouse scene with Spot and mounted OS1 LiDAR, D455 RGB-D camera, and D455 IMU.
- Verified ROS2 topic, frame, timestamp, and TF setup from the Isaac Sim ROS2 bridge.
- Repeatable rosbag2 capture and replay workflow for simulation runs.
- LiDAR SLAM pipeline configured against verified sensor topics and frames.
- Localization pipeline using a generated map.
- Mapping evaluation workflow with documented metrics and results.
- Clear portfolio documentation showing architecture, validation, results, and limitations.

## Current Milestone

Milestone 1 - Sensor Validation

## Current Task Focus

Validate the live Isaac Sim ROS2 bridge outputs before implementing SLAM:

- ROS2 environment and distro setup
- Sensor topic names and message types
- Topic frequencies
- Message headers, frame IDs, and timestamps
- `/tf` and `/tf_static`
- RViz2 visualization
- Manual rosbag2 recording and replay commands

Do not move to SLAM implementation until these checks are complete and documented.

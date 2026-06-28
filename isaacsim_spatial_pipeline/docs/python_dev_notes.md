# Python Dev Notes

Run Isaac Python scripts with the Isaac Sim launcher:

```bash
/home/aes/isaacsim/python.sh isaacsim_spatial_pipeline/scripts/00_open_stage.py
```

The scripts in this skeleton load existing assets only. By default they do not
save USD files, record bags, install packages, or assume downstream SLAM
configuration.

Current expected asset paths:

```text
/home/aes/isaac_ws/scenes/Warehouse.usd
/home/aes/isaac_ws/assets/Collected_spot/spot_lidar_realsense.usd
```

## Main Runtime

Use the GUI runtime script for live ROS2 validation:

```bash
/home/aes/isaacsim/python.sh /home/aes/isaac_ws/isaacsim_spatial_pipeline/scripts/10_run_sim.py
```

The script:

1. Opens `/home/aes/isaac_ws/scenes/Warehouse.usd`.
2. Enables the Isaac Sim ROS2 bridge.
3. Creates in-memory `/clock`, `/tf`, and sensor publisher graphs.
4. Keeps Isaac Sim open until the GUI is closed manually.

It does not save the stage.

## Current Sensor Paths

```text
Spot root:
  /World/spot_lidar_realsense

OmniLidar:
  /World/spot_lidar_realsense/body/lidar_link/OS1/sensor

D455 color:
  /World/spot_lidar_realsense/body/rsd455_link/RSD455/Camera_OmniVision_OV9782_Color

D455 depth:
  /World/spot_lidar_realsense/body/rsd455_link/RSD455/Camera_Pseudo_Depth

D455 IMU:
  /World/spot_lidar_realsense/body/rsd455_link/RSD455/Imu_Sensor
```

## ROS2 Terminal Setup

In a separate terminal:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
```

Then validate:

```bash
ros2 topic list -t
ros2 topic echo /spot/lidar/points --once
ros2 topic echo /tf --once
```

Use terminal ROS2 checks and RViz2 before downstream SLAM or localization work.

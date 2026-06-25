# Python Dev Notes

Run Isaac Python scripts with the Isaac Sim launcher:

```bash
/home/aes/isaacsim/python.sh isaacsim_spatial_pipeline/scripts/00_open_stage.py
```

The scripts in this skeleton load existing assets only. They do not save USD
files, record bags, install packages, or assume ROS2 topic names.

Current expected asset paths:

```text
/home/aes/isaac_ws/scenes/Warehouse.usd
/home/aes/isaac_ws/assets/Collected_spot/spot_lidar_realsense.usd
```

Use terminal ROS2 checks before downstream SLAM or localization work.

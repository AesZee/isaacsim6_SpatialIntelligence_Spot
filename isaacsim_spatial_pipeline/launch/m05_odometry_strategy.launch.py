"""Milestone #5 odometry strategy launcher.

The default strategy is inspection-only. This launch file intentionally does
not publish odometry, odom TF, map TF, or localization output.
"""

from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import LogInfo


REPO_ROOT = Path("/home/aes/isaac_ws")
CONFIG_PATH = (
    REPO_ROOT
    / "isaacsim_spatial_pipeline"
    / "config"
    / "m05_odometry_strategy.yaml"
)


def _load_strategy() -> dict:
    with CONFIG_PATH.open("r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)
    return config["m05_odometry_strategy"]["ros__parameters"]


def generate_launch_description():
    params = _load_strategy()
    strategy = params.get("strategy", "inspect_only")
    odom_frame = params.get("odom_frame", "odom")
    base_frame = params.get("base_frame", "base_link")
    publish_odom_tf = params.get("publish_odom_tf", False)
    publish_odom_topic = params.get("publish_odom_topic", False)

    return LaunchDescription(
        [
            LogInfo(msg=f"Milestone #5 odometry strategy: {strategy}"),
            LogInfo(msg=f"Configured odometry chain target: {odom_frame} -> {base_frame}"),
            LogInfo(msg=f"publish_odom_tf: {publish_odom_tf}"),
            LogInfo(msg=f"publish_odom_topic: {publish_odom_topic}"),
            LogInfo(
                msg=(
                    "Inspection-only mode is safe to run beside bag replay, "
                    "Milestone #4 aliases, pointcloud_to_laserscan, and slam_toolbox."
                )
            ),
        ]
    )

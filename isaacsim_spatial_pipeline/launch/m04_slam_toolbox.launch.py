"""Start slam_toolbox against the Milestone #4 /scan topic."""

from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


REPO_ROOT = Path("/home/aes/isaac_ws")
CONFIG_PATH = (
    REPO_ROOT
    / "isaacsim_spatial_pipeline"
    / "config"
    / "m04_slam_toolbox.yaml"
)


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                parameters=[str(CONFIG_PATH)],
                output="screen",
            ),
        ]
    )

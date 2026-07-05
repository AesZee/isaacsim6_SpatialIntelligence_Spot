"""Convert the validated Isaac LiDAR PointCloud2 topic into /scan."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


REPO_ROOT = Path("/home/aes/isaac_ws")
CONFIG_PATH = (
    REPO_ROOT
    / "isaacsim_spatial_pipeline"
    / "config"
    / "m04_pointcloud_to_laserscan.yaml"
)


def generate_launch_description():
    cloud_topic = LaunchConfiguration("cloud_topic")
    scan_topic = LaunchConfiguration("scan_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument("cloud_topic", default_value="/spot/lidar/points"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="m04_pointcloud_to_laserscan",
                parameters=[str(CONFIG_PATH)],
                remappings=[
                    ("cloud_in", cloud_topic),
                    ("scan", scan_topic),
                ],
                output="screen",
            ),
        ]
    )

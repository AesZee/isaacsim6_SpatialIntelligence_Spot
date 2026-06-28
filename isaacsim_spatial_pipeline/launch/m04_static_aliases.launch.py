"""Publish Milestone #4 compatibility aliases without changing Isaac frames."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="m04_body_to_base_link_alias",
                output="screen",
                arguments=[
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--qx",
                    "0",
                    "--qy",
                    "0",
                    "--qz",
                    "0",
                    "--qw",
                    "1",
                    "--frame-id",
                    "body",
                    "--child-frame-id",
                    "base_link",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="m04_sensor_to_os1_frame_alias",
                output="screen",
                arguments=[
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--qx",
                    "0",
                    "--qy",
                    "0",
                    "--qz",
                    "0",
                    "--qw",
                    "1",
                    "--frame-id",
                    "sensor",
                    "--child-frame-id",
                    "os1_frame",
                ],
            ),
        ]
    )

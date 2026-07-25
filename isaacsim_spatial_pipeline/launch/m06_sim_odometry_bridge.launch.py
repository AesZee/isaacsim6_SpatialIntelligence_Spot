"""Launch the opt-in Milestone #6 simulation-only odometry bridge."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


PIPELINE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = PIPELINE_ROOT / "config" / "m06_sim_odometry_bridge.yaml"
SCRIPT_PATH = PIPELINE_ROOT / "scripts" / "60_sim_odometry_bridge.py"


def generate_launch_description():
    enable_sim_odom = LaunchConfiguration("enable_sim_odom")
    publish_tf = LaunchConfiguration("publish_tf")
    publish_odom_topic = LaunchConfiguration("publish_odom_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")
    source_parent_frame = LaunchConfiguration("source_parent_frame")
    source_child_frame = LaunchConfiguration("source_child_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    odom_topic = LaunchConfiguration("odom_topic")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_sim_odom", default_value="false"),
            DeclareLaunchArgument("publish_tf", default_value="true"),
            DeclareLaunchArgument("publish_odom_topic", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("source_parent_frame", default_value="world"),
            DeclareLaunchArgument("source_child_frame", default_value="body"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument("publish_rate_hz", default_value="20.0"),
            LogInfo(
                msg=(
                    "Milestone #6 simulation-only odometry bridge. "
                    "Use enable_sim_odom:=true to publish /odom or odom TF."
                )
            ),
            Node(
                executable=str(SCRIPT_PATH),
                name="m06_sim_odometry_bridge",
                output="screen",
                parameters=[
                    str(CONFIG_PATH),
                    {
                        "enable_sim_odom": ParameterValue(enable_sim_odom, value_type=bool),
                        "publish_tf": ParameterValue(publish_tf, value_type=bool),
                        "publish_odom_topic": ParameterValue(publish_odom_topic, value_type=bool),
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "source_parent_frame": source_parent_frame,
                        "source_child_frame": source_child_frame,
                        "odom_frame": odom_frame,
                        "base_frame": base_frame,
                        "odom_topic": odom_topic,
                        "publish_rate_hz": ParameterValue(publish_rate_hz, value_type=float),
                    },
                ],
            ),
        ]
    )

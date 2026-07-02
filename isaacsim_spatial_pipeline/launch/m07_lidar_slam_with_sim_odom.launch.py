"""Launch Milestone #7 LiDAR SLAM with simulation-only odometry."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


REPO_ROOT = Path("/home/aes/isaac_ws")
PIPELINE_ROOT = REPO_ROOT / "isaacsim_spatial_pipeline"
CONFIG_PATH = PIPELINE_ROOT / "config" / "m07_slam_toolbox_sim_odom.yaml"
LAUNCH_ROOT = PIPELINE_ROOT / "launch"


def generate_launch_description():
    enable_sim_odom = LaunchConfiguration("enable_sim_odom")
    publish_odom_tf = LaunchConfiguration("publish_odom_tf")
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params_file = LaunchConfiguration("slam_params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_sim_odom", default_value="true"),
            DeclareLaunchArgument("publish_odom_tf", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("slam_params_file", default_value=str(CONFIG_PATH)),
            LogInfo(
                msg=(
                    "Milestone #7 LiDAR SLAM with simulation-only odometry. "
                    "Start Isaac Sim separately and press Play manually."
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(LAUNCH_ROOT / "m04_static_aliases.launch.py")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(LAUNCH_ROOT / "m04_pointcloud_to_laserscan.launch.py")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(LAUNCH_ROOT / "m06_sim_odometry_bridge.launch.py")),
                launch_arguments={
                    "enable_sim_odom": enable_sim_odom,
                    "publish_tf": publish_odom_tf,
                    "publish_odom_topic": "true",
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                parameters=[
                    slam_params_file,
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                ],
                output="screen",
            ),
        ]
    )

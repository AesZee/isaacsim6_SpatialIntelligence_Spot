"""Launch Milestone #8 map evaluation on top of Milestone #7 SLAM."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


REPO_ROOT = Path("/home/aes/isaac_ws")
PIPELINE_ROOT = REPO_ROOT / "isaacsim_spatial_pipeline"
M07_LAUNCH_PATH = PIPELINE_ROOT / "launch" / "m07_lidar_slam_with_sim_odom.launch.py"


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_sim_odom = LaunchConfiguration("enable_sim_odom")
    start_slam_stack = LaunchConfiguration("start_slam_stack")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("enable_sim_odom", default_value="true"),
            DeclareLaunchArgument("start_slam_stack", default_value="true"),
            LogInfo(
                msg=(
                    "Milestone #8 map evaluation. Start Isaac Sim separately, "
                    "press Play manually, and save maps only with the explicit script."
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(M07_LAUNCH_PATH)),
                condition=IfCondition(start_slam_stack),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "enable_sim_odom": enable_sim_odom,
                }.items(),
            ),
        ]
    )

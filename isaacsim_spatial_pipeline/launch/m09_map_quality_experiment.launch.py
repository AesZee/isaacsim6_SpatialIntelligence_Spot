"""Launch Milestone #9 map quality experiments with LiDAR slice profiles."""

from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, LogInfo, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition


PIPELINE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_ROOT = PIPELINE_ROOT / "config"
LAUNCH_ROOT = PIPELINE_ROOT / "launch"
M04_SCAN_CONFIG_PATH = CONFIG_ROOT / "m04_pointcloud_to_laserscan.yaml"
M07_SLAM_CONFIG_PATH = CONFIG_ROOT / "m07_slam_toolbox_sim_odom.yaml"
M09_PROFILE_CONFIG_PATH = CONFIG_ROOT / "m09_lidar_slice_profiles.yaml"
LASERSCAN_QOS_RELAY_PATH = PIPELINE_ROOT / "scripts" / "71_laserscan_qos_relay.py"


def _optional_float(context, name: str, default: float) -> float:
    value = LaunchConfiguration(name).perform(context).strip()
    return float(value) if value else default


def _load_profile(profile_name: str) -> dict[str, float]:
    with M09_PROFILE_CONFIG_PATH.open("r", encoding="utf-8") as config_file:
        profiles = yaml.safe_load(config_file)["lidar_slice_profiles"]
    if profile_name not in profiles:
        available = ", ".join(sorted(profiles))
        raise RuntimeError(f"Unknown Milestone #9 LiDAR profile '{profile_name}'. Available profiles: {available}")
    return profiles[profile_name]


def _load_scan_defaults() -> dict:
    with M04_SCAN_CONFIG_PATH.open("r", encoding="utf-8") as config_file:
        return yaml.safe_load(config_file)["m04_pointcloud_to_laserscan"]["ros__parameters"]


def _launch_setup(context, *_, **__):
    profile_name = LaunchConfiguration("profile").perform(context)
    profile = _load_profile(profile_name)
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_sim_odom = LaunchConfiguration("enable_sim_odom")
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")
    slam_start_delay = _optional_float(context, "slam_start_delay_sec", 0.0)

    scan_overrides = {
        "min_height": _optional_float(context, "min_height", float(profile["min_height"])),
        "max_height": _optional_float(context, "max_height", float(profile["max_height"])),
        "range_min": _optional_float(context, "range_min", float(profile["range_min"])),
        "range_max": _optional_float(context, "range_max", float(profile["range_max"])),
        "angle_min": _optional_float(context, "angle_min", float(profile["angle_min"])),
        "angle_max": _optional_float(context, "angle_max", float(profile["angle_max"])),
    }
    if scan_overrides["min_height"] >= scan_overrides["max_height"]:
        raise RuntimeError("min_height must be less than max_height")
    if scan_overrides["range_min"] < 0.0 or scan_overrides["range_min"] >= scan_overrides["range_max"]:
        raise RuntimeError("range_min must be non-negative and less than range_max")
    if scan_overrides["angle_min"] >= scan_overrides["angle_max"]:
        raise RuntimeError("angle_min must be less than angle_max")
    scan_parameters = {**_load_scan_defaults(), **scan_overrides}
    scan_parameters["target_frame"] = "os1_frame"
    scan_parameters["use_sim_time"] = ParameterValue(use_sim_time, value_type=bool)

    slam_toolbox_node = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        parameters=[
            str(M07_SLAM_CONFIG_PATH),
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "use_lifecycle_manager": ParameterValue(use_lifecycle_manager, value_type=bool),
            },
        ],
        output="screen",
    )

    return [
        LogInfo(
            msg=(
                f"Milestone #9 LiDAR slice profile '{profile_name}': "
                f"min_height={scan_overrides['min_height']}, "
                f"max_height={scan_overrides['max_height']}, "
                f"range_min={scan_overrides['range_min']}, "
                f"range_max={scan_overrides['range_max']}, "
                f"angle_min={scan_overrides['angle_min']}, "
                f"angle_max={scan_overrides['angle_max']}"
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(LAUNCH_ROOT / "m04_static_aliases.launch.py")),
            launch_arguments={
                "publish_base_link_alias": "true",
            }.items(),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="m09_odom_to_world_alias",
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
                "odom",
                "--child-frame-id",
                "world",
            ],
        ),
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="m09_pointcloud_to_laserscan",
            parameters=[scan_parameters],
            remappings=[
                ("cloud_in", "/spot/lidar/points"),
                ("scan", "/scan_raw"),
            ],
            output="screen",
        ),
        Node(
            executable=str(LASERSCAN_QOS_RELAY_PATH),
            name="m09_laserscan_qos_relay",
            output="screen",
            parameters=[
                {
                    "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    "input_topic": "/scan_raw",
                    "output_topic": "/scan",
                    "publish_every_n": ParameterValue(LaunchConfiguration("publish_every_n"), value_type=int),
                    "drop_after_sec": ParameterValue(LaunchConfiguration("drop_after_sec"), value_type=float),
                    "drop_duration_sec": ParameterValue(LaunchConfiguration("drop_duration_sec"), value_type=float),
                },
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(LAUNCH_ROOT / "m06_sim_odometry_bridge.launch.py")),
            launch_arguments={
                "enable_sim_odom": enable_sim_odom,
                "publish_tf": "false",
                "publish_odom_topic": "true",
                "use_sim_time": use_sim_time,
            }.items(),
        ),
        TimerAction(
            period=slam_start_delay,
            actions=[
                slam_toolbox_node,
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_toolbox_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
                RegisterEventHandler(
                    OnStateTransition(
                        target_lifecycle_node=slam_toolbox_node,
                        start_state="configuring",
                        goal_state="inactive",
                        entities=[
                            LogInfo(msg="[LifecycleLaunch] slam_toolbox node is activating."),
                            EmitEvent(
                                event=ChangeState(
                                    lifecycle_node_matcher=matches_action(slam_toolbox_node),
                                    transition_id=Transition.TRANSITION_ACTIVATE,
                                )
                            ),
                        ],
                    )
                ),
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("profile", default_value="baseline_m08"),
            DeclareLaunchArgument("min_height", default_value=""),
            DeclareLaunchArgument("max_height", default_value=""),
            DeclareLaunchArgument("range_min", default_value=""),
            DeclareLaunchArgument("range_max", default_value=""),
            DeclareLaunchArgument("angle_min", default_value=""),
            DeclareLaunchArgument("angle_max", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("enable_sim_odom", default_value="true"),
            DeclareLaunchArgument("use_lifecycle_manager", default_value="false"),
            DeclareLaunchArgument("publish_every_n", default_value="1"),
            DeclareLaunchArgument("drop_after_sec", default_value="-1.0"),
            DeclareLaunchArgument("drop_duration_sec", default_value="0.0"),
            DeclareLaunchArgument("slam_start_delay_sec", default_value="0.0"),
            LogInfo(
                msg=(
                    "Milestone #9 map quality experiment. Start Isaac Sim separately, "
                    "press Play manually, and save maps only with the explicit script."
                )
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )

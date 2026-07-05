"""Launch Milestone #7 LiDAR SLAM with simulation-only odometry."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, LogInfo, RegisterEventHandler
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition


REPO_ROOT = Path("/home/aes/isaac_ws")
PIPELINE_ROOT = REPO_ROOT / "isaacsim_spatial_pipeline"
CONFIG_PATH = PIPELINE_ROOT / "config" / "m07_slam_toolbox_sim_odom.yaml"
LAUNCH_ROOT = PIPELINE_ROOT / "launch"
LASERSCAN_QOS_RELAY_PATH = PIPELINE_ROOT / "scripts" / "71_laserscan_qos_relay.py"


def generate_launch_description():
    enable_sim_odom = LaunchConfiguration("enable_sim_odom")
    publish_odom_tf = LaunchConfiguration("publish_odom_tf")
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params_file = LaunchConfiguration("slam_params_file")
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")

    slam_toolbox_node = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        parameters=[
            slam_params_file,
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "use_lifecycle_manager": ParameterValue(use_lifecycle_manager, value_type=bool),
            },
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_sim_odom", default_value="true"),
            DeclareLaunchArgument("publish_odom_tf", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("use_lifecycle_manager", default_value="false"),
            DeclareLaunchArgument("slam_params_file", default_value=str(CONFIG_PATH)),
            LogInfo(
                msg=(
                    "Milestone #7 LiDAR SLAM with simulation-only odometry. "
                    "Start Isaac Sim separately and press Play manually."
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
                name="m07_odom_to_world_alias",
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(LAUNCH_ROOT / "m04_pointcloud_to_laserscan.launch.py")),
                launch_arguments={
                    "scan_topic": "/scan_raw",
                }.items(),
            ),
            Node(
                executable=str(LASERSCAN_QOS_RELAY_PATH),
                name="m07_laserscan_qos_relay",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "input_topic": "/scan_raw",
                        "output_topic": "/scan",
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
        ]
    )

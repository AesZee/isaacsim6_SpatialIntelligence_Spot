"""Create an in-memory ROS2 /tf OmniGraph for the warehouse Spot stage.

Run with:
    /home/aes/isaacsim/python.sh isaacsim_spatial_pipeline/scripts/04_create_ros2_tf_graph.py --gui

By default this does not save USD changes. Use --save-as only after reviewing
the printed graph and validating the frame tree from ROS2.
"""

import argparse
import os
from pathlib import Path

from isaacsim import SimulationApp


REPO_ROOT = Path("/home/aes/isaac_ws")
DEFAULT_WORLD_USD = REPO_ROOT / "scenes" / "Warehouse.usd"
DEFAULT_GRAPH_PATH = "/World/ROS2/TF"
DEFAULT_TOPIC_NAME = "tf"
DEFAULT_TARGET_PRIMS = (
    "/World/spot_lidar_realsense",
    "/World/spot_lidar_realsense/body/lidar_link",
    "/World/spot_lidar_realsense/body/lidar_link/OS1",
    "/World/spot_lidar_realsense/body/lidar_link/OS1/sensor",
    "/World/spot_lidar_realsense/body/rsd455_link",
    "/World/spot_lidar_realsense/body/rsd455_link/RSD455/Camera_Pseudo_Depth",
    "/World/spot_lidar_realsense/body/rsd455_link/RSD455/Imu_Sensor",
    "/World/spot_lidar_realsense/body/rsd455_link/RSD455/Camera_OmniVision_OV9782_Color",
    "/World/spot_lidar_realsense/body/rsd455_link/RSD455/Camera_OmniVision_OV9782_Left",
    "/World/spot_lidar_realsense/body/rsd455_link/RSD455/Camera_OmniVision_OV9782_Right",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--world-usd", default=str(DEFAULT_WORLD_USD))
    parser.add_argument("--graph-path", default=DEFAULT_GRAPH_PATH)
    parser.add_argument("--topic-name", default=DEFAULT_TOPIC_NAME)
    parser.add_argument("--target-prim", action="append", dest="target_prims")
    parser.add_argument("--parent-prim", default=None)
    parser.add_argument("--node-namespace", default="")
    parser.add_argument("--save-as", default=None)
    parser.add_argument("--headless", action="store_true", default=True)
    parser.add_argument("--gui", action="store_true", help="Run with the Isaac Sim GUI instead of headless mode.")
    return parser.parse_args()


args = parse_args()
simulation_app = SimulationApp({"headless": False if args.gui else args.headless})

import omni.usd
import omni.graph.core as og
from isaacsim.core.experimental.utils import app as app_utils
from pxr import Sdf


def wait_for_stage_load() -> None:
    context = omni.usd.get_context()
    while simulation_app.is_running():
        _, _, loading = context.get_stage_loading_status()
        if loading == 0:
            break
        simulation_app.update()


def print_ros_environment() -> None:
    ros_distro = os.environ.get("ROS_DISTRO")
    ros_domain_id = os.environ.get("ROS_DOMAIN_ID")
    print("ROS_DISTRO:", ros_distro if ros_distro else "<unset>")
    print("ROS_DOMAIN_ID:", ros_domain_id if ros_domain_id else "<unset>")
    if not ros_distro:
        print("Warning: ROS_DISTRO is unset. Source your ROS2 environment before validating /tf.")


def validate_prims(stage, prim_paths: tuple[str, ...] | list[str]) -> list[Sdf.Path]:
    valid_paths = []
    missing_paths = []
    for prim_path in prim_paths:
        prim = stage.GetPrimAtPath(prim_path)
        if prim.IsValid():
            valid_paths.append(Sdf.Path(prim_path))
        else:
            missing_paths.append(prim_path)

    if missing_paths:
        print("Missing target prims:")
        for prim_path in missing_paths:
            print("  ", prim_path)
        raise RuntimeError("One or more TF target prims are missing.")

    return valid_paths


def create_tf_graph(
    graph_path: str,
    topic_name: str,
    target_prims: list[Sdf.Path],
    parent_prim: str | None,
    node_namespace: str,
) -> None:
    set_values = [
        ("ComputeTransformTree.inputs:targetPrims", target_prims),
        ("PublishTransformTree.inputs:topicName", topic_name),
        ("PublishTransformTree.inputs:nodeNamespace", node_namespace),
    ]
    if parent_prim:
        set_values.append(("ComputeTransformTree.inputs:parentPrim", [Sdf.Path(parent_prim)]))

    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("ComputeTransformTree", "isaacsim.core.nodes.IsaacComputeTransformTree"),
                ("PublishTransformTree", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ],
            og.Controller.Keys.SET_VALUES: set_values,
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "ComputeTransformTree.inputs:execIn"),
                ("ComputeTransformTree.outputs:execOut", "PublishTransformTree.inputs:execIn"),
                ("ComputeTransformTree.outputs:parentFrames", "PublishTransformTree.inputs:parentFrames"),
                ("ComputeTransformTree.outputs:childFrames", "PublishTransformTree.inputs:childFrames"),
                ("ComputeTransformTree.outputs:translations", "PublishTransformTree.inputs:translations"),
                ("ComputeTransformTree.outputs:orientations", "PublishTransformTree.inputs:orientations"),
                ("ReadSimTime.outputs:simulationTime", "PublishTransformTree.inputs:timeStamp"),
                ("Context.outputs:context", "PublishTransformTree.inputs:context"),
            ],
        },
    )


def print_graph_summary(graph_path: str, target_prims: list[Sdf.Path]) -> None:
    print("TF graph:", graph_path)
    for node_name in ("OnPlaybackTick", "ReadSimTime", "Context", "ComputeTransformTree", "PublishTransformTree"):
        node = og.Controller.node(f"{graph_path}/{node_name}")
        print(f"  {node_name}: {node.get_type_name() if node.is_valid() else '<missing>'}")
    print("Target prims:")
    for prim_path in target_prims:
        print("  ", prim_path)


world_usd = Path(args.world_usd)
if not world_usd.is_file():
    raise FileNotFoundError(world_usd)

print_ros_environment()

if not app_utils.enable_extension("isaacsim.ros2.bridge"):
    raise RuntimeError("Failed to enable isaacsim.ros2.bridge")

context = omni.usd.get_context()
context.disable_save_to_recent_files()
opened = context.open_stage(str(world_usd))
context.enable_save_to_recent_files()
if not opened:
    raise RuntimeError(f"Failed to open stage: {world_usd}")
wait_for_stage_load()

stage = context.get_stage()
target_prim_strings = args.target_prims if args.target_prims else list(DEFAULT_TARGET_PRIMS)
target_prims = validate_prims(stage, target_prim_strings)
if args.parent_prim and not stage.GetPrimAtPath(args.parent_prim).IsValid():
    raise RuntimeError(f"Parent prim does not exist: {args.parent_prim}")

create_tf_graph(args.graph_path, args.topic_name, target_prims, args.parent_prim, args.node_namespace)
print("Opened stage:", stage.GetRootLayer().identifier)
print_graph_summary(args.graph_path, target_prims)

if args.save_as:
    save_path = Path(args.save_as)
    if save_path.exists():
        raise FileExistsError(f"Refusing to overwrite existing USD: {save_path}")
    if not context.save_as_stage(str(save_path)):
        raise RuntimeError(f"Failed to save stage as: {save_path}")
    print("Saved stage as:", save_path)
else:
    print("No USD saved. Pass --save-as <new_file.usd> to write a reviewed copy.")

print("Validate manually with: ros2 run tf2_tools view_frames")
print("Also inspect live TF with: ros2 topic echo /tf --once")

simulation_app.close(skip_cleanup=True)

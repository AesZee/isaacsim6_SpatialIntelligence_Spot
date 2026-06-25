"""Create an in-memory ROS2 /clock OmniGraph for the warehouse stage.

Run with:
    /home/aes/isaacsim/python.sh isaacsim_spatial_pipeline/scripts/03_create_ros2_clock_graph.py

By default this does not save USD changes. Use --save-as only after reviewing
the printed graph and deciding on a ROS2-ready USD target.
"""

import argparse
import os
from pathlib import Path

from isaacsim import SimulationApp


REPO_ROOT = Path("/home/aes/isaac_ws")
DEFAULT_WORLD_USD = REPO_ROOT / "scenes" / "Warehouse.usd"
DEFAULT_GRAPH_PATH = "/World/ROS2/Clock"
DEFAULT_TOPIC_NAME = "clock"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--world-usd", default=str(DEFAULT_WORLD_USD))
    parser.add_argument("--graph-path", default=DEFAULT_GRAPH_PATH)
    parser.add_argument("--topic-name", default=DEFAULT_TOPIC_NAME)
    parser.add_argument("--steps", type=int, default=0)
    parser.add_argument("--save-as", default=None)
    parser.add_argument("--headless", action="store_true", default=True)
    parser.add_argument("--gui", action="store_true", help="Run with the Isaac Sim GUI instead of headless mode.")
    return parser.parse_args()


args = parse_args()
simulation_app = SimulationApp({"headless": False if args.gui else args.headless})

import omni.usd
import omni.graph.core as og
from isaacsim.core.experimental.utils import app as app_utils


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
        print("Warning: ROS_DISTRO is unset. Source your ROS2 environment before validating /clock.")


def create_clock_graph(graph_path: str, topic_name: str) -> None:
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("PublishClock.inputs:topicName", topic_name),
            ],
        },
    )


def print_graph_summary(graph_path: str) -> None:
    print("Clock graph:", graph_path)
    for node_name in ("OnPlaybackTick", "ReadSimTime", "Context", "PublishClock"):
        node = og.Controller.node(f"{graph_path}/{node_name}")
        print(f"  {node_name}: {node.get_type_name() if node.is_valid() else '<missing>'}")


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

create_clock_graph(args.graph_path, args.topic_name)
print("Opened stage:", context.get_stage().GetRootLayer().identifier)
print_graph_summary(args.graph_path)

for _ in range(args.steps):
    if not simulation_app.is_running():
        break
    simulation_app.update()

if args.save_as:
    save_path = Path(args.save_as)
    if save_path.exists():
        raise FileExistsError(f"Refusing to overwrite existing USD: {save_path}")
    if not context.save_as_stage(str(save_path)):
        raise RuntimeError(f"Failed to save stage as: {save_path}")
    print("Saved stage as:", save_path)
else:
    print("No USD saved. Pass --save-as <new_file.usd> to write a reviewed copy.")

print("Validate manually with: ros2 topic echo /clock --once")

simulation_app.close(skip_cleanup=True)

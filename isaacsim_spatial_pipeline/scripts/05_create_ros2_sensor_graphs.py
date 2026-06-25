"""Create in-memory ROS2 sensor publisher OmniGraphs for the warehouse Spot stage.

Run with:
    /home/aes/isaacsim/python.sh isaacsim_spatial_pipeline/scripts/05_create_ros2_sensor_graphs.py --gui

By default this does not save USD changes. Use --save-as only after reviewing
the printed graph and validating live ROS2 topics, frame IDs, and timestamps.
"""

import argparse
import os
from pathlib import Path

from isaacsim import SimulationApp


REPO_ROOT = Path("/home/aes/isaac_ws")
DEFAULT_WORLD_USD = REPO_ROOT / "scenes" / "Warehouse.usd"
DEFAULT_GRAPH_PATH = "/World/ROS2/Sensors"

LIDAR_PRIM = "/World/spot_lidar_realsense/body/lidar_link/OS1/sensor"
COLOR_CAMERA_PRIM = "/World/spot_lidar_realsense/body/rsd455_link/RSD455/Camera_OmniVision_OV9782_Color"
DEPTH_CAMERA_PRIM = "/World/spot_lidar_realsense/body/rsd455_link/RSD455/Camera_Pseudo_Depth"
IMU_PRIM = "/World/spot_lidar_realsense/body/rsd455_link/RSD455/Imu_Sensor"

LIDAR_FRAME_ID = "sensor"
COLOR_CAMERA_FRAME_ID = "Camera_OmniVision_OV9782_Color"
DEPTH_CAMERA_FRAME_ID = "Camera_Pseudo_Depth"
IMU_FRAME_ID = "Imu_Sensor"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--world-usd", default=str(DEFAULT_WORLD_USD))
    parser.add_argument("--graph-path", default=DEFAULT_GRAPH_PATH)
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
        print("Warning: ROS_DISTRO is unset. Source your ROS2 environment before validating sensor topics.")


def require_prim(stage, prim_path: str) -> Sdf.Path:
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise RuntimeError(f"Required sensor prim does not exist: {prim_path}")
    return Sdf.Path(prim_path)


def validate_lidar_prim(stage, prim_path: str) -> None:
    prim = stage.GetPrimAtPath(prim_path)
    if prim.GetTypeName() == "OmniLidar" and prim.HasAPI("OmniSensorGenericLidarCoreAPI"):
        return

    print("LiDAR prim is not publishable by ROS2RtxLidarHelper:")
    print(f"  path: {prim_path}")
    print(f"  type: {prim.GetTypeName() if prim.IsValid() else '<missing>'}")
    print(f"  schemas: {prim.GetAppliedSchemas() if prim.IsValid() else []}")
    print("  required: type OmniLidar with OmniSensorGenericLidarCoreAPI")
    print("LiDAR-like prims currently loaded:")
    for candidate in stage.Traverse():
        candidate_path = str(candidate.GetPath())
        if (
            candidate.GetTypeName() == "OmniLidar"
            or candidate.HasAPI("OmniSensorGenericLidarCoreAPI")
            or "lidar" in candidate_path.lower()
            or "os1" in candidate_path.lower()
        ):
            print(
                f"  {candidate_path} "
                f"type={candidate.GetTypeName()} "
                f"schemas={candidate.GetAppliedSchemas()}"
            )
    raise RuntimeError("No valid OmniLidar prim is available for /spot/lidar/points.")


def create_sensor_graph(graph_path: str, node_namespace: str) -> None:
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("RunOnce", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("LidarRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("LidarPointCloud", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
                ("ColorRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("ColorImage", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("ColorCameraInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ("DepthRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("DepthImage", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("ReadImu", "isaacsim.sensors.physics.IsaacReadIMU"),
                ("PublishImu", "isaacsim.ros2.bridge.ROS2PublishImu"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("LidarRenderProduct.inputs:cameraPrim", Sdf.Path(LIDAR_PRIM)),
                ("LidarPointCloud.inputs:topicName", "/spot/lidar/points"),
                ("LidarPointCloud.inputs:type", "point_cloud"),
                ("LidarPointCloud.inputs:frameId", LIDAR_FRAME_ID),
                ("LidarPointCloud.inputs:nodeNamespace", node_namespace),
                ("LidarPointCloud.inputs:resetSimulationTimeOnStop", True),
                ("ColorRenderProduct.inputs:cameraPrim", Sdf.Path(COLOR_CAMERA_PRIM)),
                ("ColorImage.inputs:topicName", "/spot/d455/color/image"),
                ("ColorImage.inputs:type", "rgb"),
                ("ColorImage.inputs:frameId", COLOR_CAMERA_FRAME_ID),
                ("ColorImage.inputs:nodeNamespace", node_namespace),
                ("ColorImage.inputs:resetSimulationTimeOnStop", True),
                ("ColorCameraInfo.inputs:topicName", "/spot/d455/color/camera_info"),
                ("ColorCameraInfo.inputs:frameId", COLOR_CAMERA_FRAME_ID),
                ("ColorCameraInfo.inputs:nodeNamespace", node_namespace),
                ("ColorCameraInfo.inputs:resetSimulationTimeOnStop", True),
                ("DepthRenderProduct.inputs:cameraPrim", Sdf.Path(DEPTH_CAMERA_PRIM)),
                ("DepthImage.inputs:topicName", "/spot/d455/depth/image"),
                ("DepthImage.inputs:type", "depth"),
                ("DepthImage.inputs:frameId", DEPTH_CAMERA_FRAME_ID),
                ("DepthImage.inputs:nodeNamespace", node_namespace),
                ("DepthImage.inputs:resetSimulationTimeOnStop", True),
                ("ReadImu.inputs:imuPrim", Sdf.Path(IMU_PRIM)),
                ("PublishImu.inputs:topicName", "/spot/d455/imu"),
                ("PublishImu.inputs:frameId", IMU_FRAME_ID),
                ("PublishImu.inputs:nodeNamespace", node_namespace),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "RunOnce.inputs:execIn"),
                ("RunOnce.outputs:step", "LidarRenderProduct.inputs:execIn"),
                ("RunOnce.outputs:step", "ColorRenderProduct.inputs:execIn"),
                ("RunOnce.outputs:step", "DepthRenderProduct.inputs:execIn"),
                ("LidarRenderProduct.outputs:execOut", "LidarPointCloud.inputs:execIn"),
                ("LidarRenderProduct.outputs:renderProductPath", "LidarPointCloud.inputs:renderProductPath"),
                ("ColorRenderProduct.outputs:execOut", "ColorImage.inputs:execIn"),
                ("ColorRenderProduct.outputs:renderProductPath", "ColorImage.inputs:renderProductPath"),
                ("ColorRenderProduct.outputs:execOut", "ColorCameraInfo.inputs:execIn"),
                ("ColorRenderProduct.outputs:renderProductPath", "ColorCameraInfo.inputs:renderProductPath"),
                ("DepthRenderProduct.outputs:execOut", "DepthImage.inputs:execIn"),
                ("DepthRenderProduct.outputs:renderProductPath", "DepthImage.inputs:renderProductPath"),
                ("OnPlaybackTick.outputs:tick", "ReadImu.inputs:execIn"),
                ("ReadImu.outputs:execOut", "PublishImu.inputs:execIn"),
                ("ReadImu.outputs:angVel", "PublishImu.inputs:angularVelocity"),
                ("ReadImu.outputs:linAcc", "PublishImu.inputs:linearAcceleration"),
                ("ReadImu.outputs:orientation", "PublishImu.inputs:orientation"),
                ("ReadImu.outputs:sensorTime", "PublishImu.inputs:timeStamp"),
                ("Context.outputs:context", "LidarPointCloud.inputs:context"),
                ("Context.outputs:context", "ColorImage.inputs:context"),
                ("Context.outputs:context", "ColorCameraInfo.inputs:context"),
                ("Context.outputs:context", "DepthImage.inputs:context"),
                ("Context.outputs:context", "PublishImu.inputs:context"),
            ],
        },
    )


def print_graph_summary(graph_path: str) -> None:
    print("Sensor graph:", graph_path)
    for node_name in (
        "OnPlaybackTick",
        "RunOnce",
        "Context",
        "LidarRenderProduct",
        "LidarPointCloud",
        "ColorRenderProduct",
        "ColorImage",
        "ColorCameraInfo",
        "DepthRenderProduct",
        "DepthImage",
        "ReadImu",
        "PublishImu",
    ):
        node = og.Controller.node(f"{graph_path}/{node_name}")
        print(f"  {node_name}: {node.get_type_name() if node.is_valid() else '<missing>'}")

    print("Draft topics:")
    print("  /spot/lidar/points -> sensor_msgs/msg/PointCloud2")
    print("  /spot/d455/color/image -> sensor_msgs/msg/Image")
    print("  /spot/d455/color/camera_info -> sensor_msgs/msg/CameraInfo")
    print("  /spot/d455/depth/image -> sensor_msgs/msg/Image")
    print("  /spot/d455/imu -> sensor_msgs/msg/Imu")


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
for required_prim in (LIDAR_PRIM, COLOR_CAMERA_PRIM, DEPTH_CAMERA_PRIM, IMU_PRIM):
    require_prim(stage, required_prim)
validate_lidar_prim(stage, LIDAR_PRIM)

create_sensor_graph(args.graph_path, args.node_namespace)
print("Opened stage:", stage.GetRootLayer().identifier)
print_graph_summary(args.graph_path)

if args.save_as:
    save_path = Path(args.save_as)
    if save_path.exists():
        raise FileExistsError(f"Refusing to overwrite existing USD: {save_path}")
    if not context.save_as_stage(str(save_path)):
        raise RuntimeError(f"Failed to save stage as: {save_path}")
    print("Saved stage as:", save_path)
else:
    print("No USD saved. Pass --save-as <new_file.usd> to write a reviewed copy.")

print("Validate manually with: ros2 topic list -t")
print("Then inspect headers with: ros2 topic echo <topic> --once")

simulation_app.close(skip_cleanup=True)

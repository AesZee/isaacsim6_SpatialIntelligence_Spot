"""Open the warehouse Spot scene with in-memory ROS2 sensor graphs.

Run with:
    /home/aes/isaacsim/python.sh isaacsim_spatial_pipeline/scripts/10_run_sim.py

The script does not save USD changes. It opens the warehouse scene, validates
the existing Spot sensor prims, creates ROS2 clock/TF/sensor publisher
graphs, and keeps Isaac Sim open until the GUI is closed manually.
"""

import argparse
import os
from pathlib import Path

from isaacsim import SimulationApp


REPO_ROOT = Path("/home/aes/isaac_ws")
DEFAULT_WORLD_USD = REPO_ROOT / "scenes" / "Warehouse.usd"

CLOCK_GRAPH_PATH = "/World/ROS2/Clock"
TF_GRAPH_PATH = "/World/ROS2/TF"
SENSOR_GRAPH_PATH = "/World/ROS2/Sensors"

ROBOT_PRIM = "/World/spot_lidar_realsense"
LIDAR_LINK_PRIM = f"{ROBOT_PRIM}/body/lidar_link"
LIDAR_MODEL_PRIM = f"{LIDAR_LINK_PRIM}/OS1"
LIDAR_SENSOR_PRIM = f"{LIDAR_MODEL_PRIM}/sensor"
RSD455_LINK_PRIM = f"{ROBOT_PRIM}/body/rsd455_link"
DEPTH_CAMERA_PRIM = f"{RSD455_LINK_PRIM}/RSD455/Camera_Pseudo_Depth"
IMU_PRIM = f"{RSD455_LINK_PRIM}/RSD455/Imu_Sensor"
COLOR_CAMERA_PRIM = f"{RSD455_LINK_PRIM}/RSD455/Camera_OmniVision_OV9782_Color"
LEFT_CAMERA_PRIM = f"{RSD455_LINK_PRIM}/RSD455/Camera_OmniVision_OV9782_Left"
RIGHT_CAMERA_PRIM = f"{RSD455_LINK_PRIM}/RSD455/Camera_OmniVision_OV9782_Right"

LIDAR_FRAME_ID = "sensor"
COLOR_CAMERA_FRAME_ID = "Camera_OmniVision_OV9782_Color"
DEPTH_CAMERA_FRAME_ID = "Camera_Pseudo_Depth"
IMU_FRAME_ID = "Imu_Sensor"

TF_TARGET_PRIMS = (
    ROBOT_PRIM,
    LIDAR_LINK_PRIM,
    LIDAR_MODEL_PRIM,
    LIDAR_SENSOR_PRIM,
    RSD455_LINK_PRIM,
    DEPTH_CAMERA_PRIM,
    IMU_PRIM,
    COLOR_CAMERA_PRIM,
    LEFT_CAMERA_PRIM,
    RIGHT_CAMERA_PRIM,
)

SENSOR_PRIMS = (
    LIDAR_SENSOR_PRIM,
    COLOR_CAMERA_PRIM,
    DEPTH_CAMERA_PRIM,
    IMU_PRIM,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--world-usd", default=str(DEFAULT_WORLD_USD))
    return parser.parse_args()


args = parse_args()
simulation_app = SimulationApp({"headless": False})

import omni.graph.core as og
import omni.usd
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
        print("Warning: ROS_DISTRO is unset. Source ROS2 before checking topics.")


def enable_ros2_bridge() -> None:
    if not app_utils.enable_extension("isaacsim.ros2.bridge"):
        raise RuntimeError("Failed to enable isaacsim.ros2.bridge")

    # Extension startup is deferred until app updates; let ROS2 nodes initialize
    # before authoring graphs that may execute on the next tick.
    for _ in range(10):
        if not simulation_app.is_running():
            break
        simulation_app.update()


def validate_prims(stage, prim_paths: tuple[str, ...]) -> list[Sdf.Path]:
    valid_paths = []
    missing_paths = []
    for prim_path in prim_paths:
        if stage.GetPrimAtPath(prim_path).IsValid():
            valid_paths.append(Sdf.Path(prim_path))
        else:
            missing_paths.append(prim_path)

    if missing_paths:
        print("Missing required prims:")
        for prim_path in missing_paths:
            print("  ", prim_path)
        raise RuntimeError("Stage is missing one or more required Spot sensor prims.")

    return valid_paths


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


def create_clock_graph() -> None:
    og.Controller.edit(
        {"graph_path": CLOCK_GRAPH_PATH, "evaluator_name": "execution"},
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
                ("PublishClock.inputs:topicName", "clock"),
            ],
        },
    )


def create_tf_graph(target_prims: list[Sdf.Path]) -> None:
    og.Controller.edit(
        {"graph_path": TF_GRAPH_PATH, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("ComputeTransformTree", "isaacsim.core.nodes.IsaacComputeTransformTree"),
                ("PublishTransformTree", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("ComputeTransformTree.inputs:targetPrims", target_prims),
                ("PublishTransformTree.inputs:topicName", "tf"),
            ],
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


def create_sensor_graph() -> None:
    og.Controller.edit(
        {"graph_path": SENSOR_GRAPH_PATH, "evaluator_name": "execution"},
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
                ("LidarRenderProduct.inputs:cameraPrim", Sdf.Path(LIDAR_SENSOR_PRIM)),
                ("LidarPointCloud.inputs:topicName", "/spot/lidar/points"),
                ("LidarPointCloud.inputs:type", "point_cloud"),
                ("LidarPointCloud.inputs:frameId", LIDAR_FRAME_ID),
                ("LidarPointCloud.inputs:resetSimulationTimeOnStop", True),
                ("ColorRenderProduct.inputs:cameraPrim", Sdf.Path(COLOR_CAMERA_PRIM)),
                ("ColorImage.inputs:topicName", "/spot/d455/color/image"),
                ("ColorImage.inputs:type", "rgb"),
                ("ColorImage.inputs:frameId", COLOR_CAMERA_FRAME_ID),
                ("ColorImage.inputs:resetSimulationTimeOnStop", True),
                ("ColorCameraInfo.inputs:topicName", "/spot/d455/color/camera_info"),
                ("ColorCameraInfo.inputs:frameId", COLOR_CAMERA_FRAME_ID),
                ("ColorCameraInfo.inputs:resetSimulationTimeOnStop", True),
                ("DepthRenderProduct.inputs:cameraPrim", Sdf.Path(DEPTH_CAMERA_PRIM)),
                ("DepthImage.inputs:topicName", "/spot/d455/depth/image"),
                ("DepthImage.inputs:type", "depth"),
                ("DepthImage.inputs:frameId", DEPTH_CAMERA_FRAME_ID),
                ("DepthImage.inputs:resetSimulationTimeOnStop", True),
                ("ReadImu.inputs:imuPrim", Sdf.Path(IMU_PRIM)),
                ("PublishImu.inputs:topicName", "/spot/d455/imu"),
                ("PublishImu.inputs:frameId", IMU_FRAME_ID),
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


def print_graph_summary(label: str, graph_path: str, node_names: tuple[str, ...]) -> None:
    print(f"{label} graph:", graph_path)
    for node_name in node_names:
        node = og.Controller.node(f"{graph_path}/{node_name}")
        print(f"  {node_name}: {node.get_type_name() if node.is_valid() else '<missing>'}")


world_usd = Path(args.world_usd)
if not world_usd.is_file():
    raise FileNotFoundError(world_usd)

print_ros_environment()

enable_ros2_bridge()

context = omni.usd.get_context()
context.disable_save_to_recent_files()
opened = context.open_stage(str(world_usd))
context.enable_save_to_recent_files()
if not opened:
    raise RuntimeError(f"Failed to open stage: {world_usd}")
wait_for_stage_load()

stage = context.get_stage()
tf_target_paths = validate_prims(stage, TF_TARGET_PRIMS)
validate_prims(stage, SENSOR_PRIMS)
validate_lidar_prim(stage, LIDAR_SENSOR_PRIM)

create_clock_graph()
create_tf_graph(tf_target_paths)
create_sensor_graph()

print("Opened stage:", stage.GetRootLayer().identifier)
print("Robot prim:", ROBOT_PRIM)
print("No USD saved.")
print_graph_summary("Clock", CLOCK_GRAPH_PATH, ("OnPlaybackTick", "ReadSimTime", "Context", "PublishClock"))
print_graph_summary(
    "TF",
    TF_GRAPH_PATH,
    ("OnPlaybackTick", "ReadSimTime", "Context", "ComputeTransformTree", "PublishTransformTree"),
)
print_graph_summary(
    "Sensor",
    SENSOR_GRAPH_PATH,
    (
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
    ),
)

print("Topics to check after pressing Play in Isaac Sim:")
print("  ros2 topic list -t")
print("  ros2 topic echo /clock --once")
print("  ros2 topic echo /tf --once")
print("  ros2 topic echo /spot/lidar/points --once")
print("  ros2 topic echo /spot/d455/imu --once")
print("Keeping Isaac Sim open. Close the GUI window to exit.")

while simulation_app.is_running():
    simulation_app.update()

simulation_app.close(skip_cleanup=True)

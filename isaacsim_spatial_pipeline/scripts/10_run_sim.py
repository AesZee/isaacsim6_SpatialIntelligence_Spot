"""Open the warehouse Spot scene with in-memory ROS2 sensor graphs.

Run with:
    $ISAAC_SIM_DIR/python.sh $WORKSPACE_DIR/isaacsim_spatial_pipeline/scripts/10_run_sim.py

The script does not save USD changes. It opens the warehouse scene, validates
the existing Spot sensor prims, creates ROS2 clock/TF/sensor publisher
graphs, and keeps Isaac Sim open until the GUI is closed manually.
"""

import argparse
import json
import math
import os
import signal
import time
from pathlib import Path

from isaacsim import SimulationApp


REPO_ROOT = Path(os.environ.get("WORKSPACE_DIR", Path(__file__).resolve().parents[2]))
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
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--auto-play", action="store_true")
    parser.add_argument(
        "--max-runtime-seconds",
        type=float,
        default=0.0,
        help="Stop Isaac Sim after this wall-clock duration; zero keeps the existing unbounded behavior.",
    )
    parser.add_argument(
        "--enable-scripted-motion",
        action="store_true",
        help="Move the Spot root prim with a runtime-only kinematic path while playback is running.",
    )
    parser.add_argument(
        "--motion-speed",
        type=float,
        default=0.5,
        help="Kinematic scripted motion speed in meters per second.",
    )
    parser.add_argument(
        "--motion-radius-x",
        type=float,
        default=6.0,
        help="Half-width of the scripted warehouse loop in meters.",
    )
    parser.add_argument(
        "--motion-radius-y",
        type=float,
        default=4.0,
        help="Half-height of the scripted warehouse loop in meters.",
    )
    parser.add_argument("--trajectory-file")
    parser.add_argument("--trajectory-name")
    parser.add_argument("--motion-max-duration", type=float, default=0.0)
    parser.add_argument("--motion-start-file")
    parser.add_argument("--motion-status-file")
    parser.add_argument("--single-trajectory", action="store_true")
    parser.add_argument("--permitted-min-x", type=float)
    parser.add_argument("--permitted-max-x", type=float)
    parser.add_argument("--permitted-min-y", type=float)
    parser.add_argument("--permitted-max-y", type=float)
    parser.add_argument("--initial-x-offset", type=float, default=0.0)
    parser.add_argument("--initial-y-offset", type=float, default=0.0)
    parser.add_argument("--initial-yaw-offset", type=float, default=0.0)
    return parser.parse_args()


args = parse_args()
simulation_app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720})

import omni.graph.core as og
import omni.timeline
import omni.usd
from isaacsim.core.experimental.utils import app as app_utils
from pxr import Gf, Sdf, Usd, UsdGeom


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


class ScriptedSpotMotion:
    """Runtime-only kinematic motion for SLAM coverage experiments."""

    def __init__(
        self,
        stage,
        prim_path: str,
        speed: float,
        radius_x: float,
        radius_y: float,
        points: tuple[tuple[float, float], ...] | None,
        max_duration: float,
        single_trajectory: bool,
        permitted_area: tuple[float, float, float, float] | None,
        initial_pose_offset: tuple[float, float, float],
    ) -> None:
        if speed <= 0.0:
            raise ValueError("--motion-speed must be positive")
        if radius_x <= 0.0 or radius_y <= 0.0:
            raise ValueError("--motion-radius-x and --motion-radius-y must be positive")

        self.prim = stage.GetPrimAtPath(prim_path)
        if not self.prim.IsValid():
            raise RuntimeError(f"Cannot enable scripted motion; prim is missing: {prim_path}")

        self.xform_api = UsdGeom.XformCommonAPI(self.prim)
        self.timeline = omni.timeline.get_timeline_interface()
        self.speed = speed
        self.max_duration = max_duration
        self.single_trajectory = single_trajectory
        self.permitted_area = permitted_area
        self.points = points or (
            (0.0, 0.0),
            (radius_x, 0.0),
            (radius_x, radius_y),
            (-radius_x, radius_y),
            (-radius_x, -radius_y),
            (radius_x, -radius_y),
            (radius_x, 0.0),
            (0.0, 0.0),
        )
        self.segment_lengths = [
            math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(self.points, self.points[1:])
        ]
        self.loop_length = sum(self.segment_lengths)
        self.start_time = None
        self.complete = False
        self.last_status = {"state": "ready", "elapsed": 0.0, "x": 0.0, "y": 0.0}

        translation, rotation = self._read_initial_pose()
        self.origin = Gf.Vec3d(
            translation[0] + initial_pose_offset[0],
            translation[1] + initial_pose_offset[1],
            translation[2],
        )
        self.initial_rotation = Gf.Vec3f(
            rotation[0],
            rotation[1],
            rotation[2] + math.degrees(initial_pose_offset[2]),
        )
        self.xform_api.SetTranslate(self.origin)
        self.xform_api.SetRotate(self.initial_rotation)
        for x, y in self.points:
            self._validate_permitted_area(x, y)

    def _read_initial_pose(self) -> tuple[Gf.Vec3d, Gf.Vec3f]:
        try:
            translation, rotation, _, _, _ = self.xform_api.GetXformVectors(Usd.TimeCode.Default())
            return Gf.Vec3d(translation), Gf.Vec3f(rotation)
        except Exception:
            transform = UsdGeom.Xformable(self.prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            return Gf.Vec3d(transform.ExtractTranslation()), Gf.Vec3f(0.0, 0.0, 0.0)

    def _validate_permitted_area(self, x: float, y: float) -> None:
        if self.permitted_area is None:
            return
        min_x, max_x, min_y, max_y = self.permitted_area
        if not min_x <= x <= max_x or not min_y <= y <= max_y:
            raise RuntimeError(
                f"Scripted motion left permitted relative area: x={x:.3f}, y={y:.3f}, "
                f"bounds=({min_x:.3f}, {max_x:.3f}, {min_y:.3f}, {max_y:.3f})"
            )

    def update(self) -> dict:
        if self.complete:
            return self.last_status
        if not self.timeline.is_playing():
            self.start_time = None
            return {"state": "waiting_for_timeline", "elapsed": 0.0, "x": 0.0, "y": 0.0}

        now = self.timeline.get_current_time()
        if self.start_time is None:
            self.start_time = now

        elapsed = max(0.0, now - self.start_time)
        distance = elapsed * self.speed
        reason = None
        if self.max_duration > 0.0 and elapsed >= self.max_duration:
            reason = "max_duration"
        if self.single_trajectory and distance >= self.loop_length:
            reason = "trajectory_complete"
        if reason:
            distance = min(distance, self.loop_length)
            self.complete = True
        elif not self.single_trajectory:
            distance %= self.loop_length

        x_offset, y_offset, yaw = self._sample_path(distance)
        self._validate_permitted_area(x_offset, y_offset)
        self.xform_api.SetTranslate(
            Gf.Vec3d(self.origin[0] + x_offset, self.origin[1] + y_offset, self.origin[2])
        )
        self.xform_api.SetRotate(
            Gf.Vec3f(self.initial_rotation[0], self.initial_rotation[1], math.degrees(yaw))
        )
        self.last_status = {
            "state": "complete" if self.complete else "running",
            "reason": reason,
            "elapsed": elapsed,
            "x": x_offset,
            "y": y_offset,
        }
        return self.last_status

    def _sample_path(self, distance: float) -> tuple[float, float, float]:
        remaining = distance
        for index, length in enumerate(self.segment_lengths):
            start = self.points[index]
            end = self.points[index + 1]
            if remaining <= length:
                ratio = remaining / length if length > 0.0 else 0.0
                x = start[0] + (end[0] - start[0]) * ratio
                y = start[1] + (end[1] - start[1]) * ratio
                yaw = math.atan2(end[1] - start[1], end[0] - start[0])
                return x, y, yaw
            remaining -= length

        final = self.points[-1]
        previous = self.points[-2]
        yaw = math.atan2(final[1] - previous[1], final[0] - previous[0])
        return final[0], final[1], yaw


def optional_permitted_area() -> tuple[float, float, float, float] | None:
    values = (
        args.permitted_min_x,
        args.permitted_max_x,
        args.permitted_min_y,
        args.permitted_max_y,
    )
    if all(value is None for value in values):
        return None
    if any(value is None for value in values):
        raise ValueError("All four permitted-area bounds must be provided together")
    min_x, max_x, min_y, max_y = values
    if min_x >= max_x or min_y >= max_y:
        raise ValueError("Permitted-area minimums must be less than maximums")
    return min_x, max_x, min_y, max_y


def write_motion_status(path: Path | None, payload: dict) -> None:
    if path is None:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(path)


def load_trajectory() -> tuple[tuple[float, float], ...] | None:
    if not args.trajectory_file and not args.trajectory_name:
        return None
    if not args.trajectory_file or not args.trajectory_name:
        raise ValueError("--trajectory-file and --trajectory-name must be provided together")
    payload = json.loads(Path(args.trajectory_file).read_text(encoding="utf-8"))
    try:
        raw_points = payload["trajectories"][args.trajectory_name]["waypoints_m"]
        points = tuple(tuple(float(value) for value in point) for point in raw_points)
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"Invalid trajectory '{args.trajectory_name}' in {args.trajectory_file}") from error
    if len(points) < 2 or any(len(point) != 2 for point in points):
        raise ValueError("Trajectory requires at least two [x, y] waypoints")
    return points


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

scripted_motion = None
if args.enable_scripted_motion:
    scripted_motion = ScriptedSpotMotion(
        stage=stage,
        prim_path=ROBOT_PRIM,
        speed=args.motion_speed,
        radius_x=args.motion_radius_x,
        radius_y=args.motion_radius_y,
        points=load_trajectory(),
        max_duration=args.motion_max_duration,
        single_trajectory=args.single_trajectory,
        permitted_area=optional_permitted_area(),
        initial_pose_offset=(
            args.initial_x_offset,
            args.initial_y_offset,
            args.initial_yaw_offset,
        ),
    )

motion_start_file = Path(args.motion_start_file) if args.motion_start_file else None
motion_status_file = Path(args.motion_status_file) if args.motion_status_file else None
stop_requested = False


def request_stop(signum, _frame) -> None:
    global stop_requested
    print(f"Received signal {signum}; stopping Isaac Sim.")
    stop_requested = True


signal.signal(signal.SIGINT, request_stop)
signal.signal(signal.SIGTERM, request_stop)

print("Opened stage:", stage.GetRootLayer().identifier)
print("Robot prim:", ROBOT_PRIM)
print("No USD saved.")
if scripted_motion:
    print("Scripted Spot motion: enabled")
    print("  mode: runtime-only kinematic root-prim motion")
    print(f"  speed: {args.motion_speed:.3f} m/s")
    print(f"  loop half-size: x={args.motion_radius_x:.3f} m, y={args.motion_radius_y:.3f} m")
    print(f"  maximum duration: {args.motion_max_duration:.3f} s")
    print(f"  single trajectory: {args.single_trajectory}")
    print(f"  permitted relative area: {optional_permitted_area()}")
    print(f"  start gate: {motion_start_file if motion_start_file else '<timeline playback>'}")
else:
    print("Scripted Spot motion: disabled")
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

print("Topics to check while Isaac Sim is playing:")
print("  ros2 topic list -t")
print("  ros2 topic echo /clock --once")
print("  ros2 topic echo /tf --once")
print("  ros2 topic echo /spot/lidar/points --once")
print("  ros2 topic echo /spot/d455/imu --once")
if args.auto_play:
    omni.timeline.get_timeline_interface().play()
    print("Simulation timeline started programmatically.")
else:
    print("Start the simulation timeline manually.")

write_motion_status(
    motion_status_file,
    {
        "state": "ready",
        "motion_enabled": scripted_motion is not None,
        "world_usd": str(world_usd),
    },
)

runtime_started = time.monotonic()
motion_started = False
last_status_write = 0.0
last_motion_state = None
try:
    while simulation_app.is_running() and not stop_requested:
        if args.max_runtime_seconds > 0.0 and time.monotonic() - runtime_started >= args.max_runtime_seconds:
            print("Maximum Isaac Sim runtime reached.")
            break

        if scripted_motion:
            start_allowed = motion_start_file is None or motion_start_file.exists()
            if start_allowed:
                motion_started = True
                motion_state = scripted_motion.update()
                now = time.monotonic()
                if now - last_status_write >= 0.5 or motion_state["state"] != last_motion_state:
                    write_motion_status(motion_status_file, motion_state)
                    last_status_write = now
                    last_motion_state = motion_state["state"]
            elif time.monotonic() - last_status_write >= 1.0:
                write_motion_status(motion_status_file, {"state": "armed"})
                last_status_write = time.monotonic()

        simulation_app.update()
finally:
    timeline = omni.timeline.get_timeline_interface()
    if timeline.is_playing():
        timeline.stop()
    write_motion_status(
        motion_status_file,
        {
            "state": "shutdown",
            "motion_started": motion_started,
            "motion_complete": bool(scripted_motion and scripted_motion.complete),
        },
    )
    simulation_app.close(skip_cleanup=True)

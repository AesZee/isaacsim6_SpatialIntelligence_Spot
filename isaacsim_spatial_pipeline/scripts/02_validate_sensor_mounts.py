"""Validate expected sensor mount prim names without assuming ROS2 frame IDs."""

import argparse
from pathlib import Path

from isaacsim import SimulationApp


REPO_ROOT = Path("/home/aes/isaac_ws")
DEFAULT_WORLD_USD = REPO_ROOT / "scenes" / "Warehouse.usd"
DEFAULT_SPOT_USD = REPO_ROOT / "assets" / "Collected_spot" / "spot_lidar_realsense.usd"
DEFAULT_SPOT_PRIM = "/World/Spot"
SPOT_PRIM_CANDIDATES = (DEFAULT_SPOT_PRIM, "/World/spot_lidar_realsense")
SENSOR_NAME_HINTS = ("lidar", "os1", "rsd455", "d455", "camera", "imu")
SENSOR_TYPE_NAMES = ("Camera", "IsaacImuSensor")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--world-usd", default=str(DEFAULT_WORLD_USD))
    parser.add_argument("--spot-usd", default=str(DEFAULT_SPOT_USD))
    parser.add_argument("--spot-prim", default=DEFAULT_SPOT_PRIM)
    parser.add_argument("--headless", action="store_true", default=True)
    return parser.parse_args()


args = parse_args()
simulation_app = SimulationApp({"headless": args.headless})

import omni.usd
from isaacsim.core.experimental.utils.stage import add_reference_to_stage


def wait_for_stage_load() -> None:
    context = omni.usd.get_context()
    while simulation_app.is_running():
        _, _, loading = context.get_stage_loading_status()
        if loading == 0:
            break
        simulation_app.update()


def find_robot_prim(stage):
    requested_prim = stage.GetPrimAtPath(args.spot_prim)
    if requested_prim.IsValid():
        return requested_prim

    for prim_path in SPOT_PRIM_CANDIDATES:
        prim = stage.GetPrimAtPath(prim_path)
        if prim.IsValid():
            return prim

    return None


context = omni.usd.get_context()
if not context.open_stage(args.world_usd):
    raise RuntimeError(f"Failed to open stage: {args.world_usd}")
wait_for_stage_load()

stage = context.get_stage()
robot_prim = find_robot_prim(stage)
if robot_prim is None:
    add_reference_to_stage(args.spot_usd, args.spot_prim)
    wait_for_stage_load()
    robot_prim = stage.GetPrimAtPath(args.spot_prim)

if robot_prim is None or not robot_prim.IsValid():
    raise RuntimeError("Unable to find or add a valid Spot prim.")

matches_by_category = {
    "mounts": [],
    "sensors": [],
    "render_products": [],
}

for prim in stage.Traverse():
    path = prim.GetPath().pathString
    if not path.startswith(robot_prim.GetPath().pathString):
        continue

    name = prim.GetName().lower()
    type_name = prim.GetTypeName()
    item = (path, type_name)

    if type_name in SENSOR_TYPE_NAMES:
        matches_by_category["sensors"].append(item)
    elif type_name == "RenderProduct":
        matches_by_category["render_products"].append(item)
    elif type_name == "Xform" and any(hint in name for hint in SENSOR_NAME_HINTS):
        matches_by_category["mounts"].append(item)

print("Robot prim:", robot_prim.GetPath())
for category, matches in matches_by_category.items():
    print(f"\n{category}:")
    if not matches:
        print("  <none>")
        continue
    for path, type_name in matches:
        print(f"  {path} ({type_name})")

if not any(matches_by_category.values()):
    raise RuntimeError("No sensor-like prim names found. Inspect the Spot USD manually.")

simulation_app.close()

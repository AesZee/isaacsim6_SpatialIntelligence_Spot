"""Inspect key prims in the warehouse and Spot runtime stage."""

import argparse
from pathlib import Path

from isaacsim import SimulationApp


REPO_ROOT = Path("/home/aes/isaac_ws")
DEFAULT_WORLD_USD = REPO_ROOT / "scenes" / "Warehouse.usd"
DEFAULT_SPOT_USD = REPO_ROOT / "assets" / "Collected_spot" / "spot_lidar_realsense.usd"
DEFAULT_SPOT_PRIM = "/World/Spot"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--world-usd", default=str(DEFAULT_WORLD_USD))
    parser.add_argument("--spot-usd", default=str(DEFAULT_SPOT_USD))
    parser.add_argument("--spot-prim", default=DEFAULT_SPOT_PRIM)
    parser.add_argument("--max-prims", type=int, default=80)
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


context = omni.usd.get_context()
if not context.open_stage(args.world_usd):
    raise RuntimeError(f"Failed to open stage: {args.world_usd}")
wait_for_stage_load()

stage = context.get_stage()
spot_prim = stage.GetPrimAtPath(args.spot_prim)
if not spot_prim.IsValid():
    add_reference_to_stage(args.spot_usd, args.spot_prim)
    wait_for_stage_load()

print("Root layer:", stage.GetRootLayer().identifier)
print("Default prim:", stage.GetDefaultPrim().GetPath() if stage.GetDefaultPrim() else "<none>")
print("Key prim:", args.spot_prim, "valid=", stage.GetPrimAtPath(args.spot_prim).IsValid())

for index, prim in enumerate(stage.Traverse()):
    if index >= args.max_prims:
        print(f"... truncated at {args.max_prims} prims")
        break
    print(f"{prim.GetPath()} ({prim.GetTypeName()})")

simulation_app.close()

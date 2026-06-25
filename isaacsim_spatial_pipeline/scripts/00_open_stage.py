"""Open the warehouse stage and reference the collected Spot asset.

Run with:
    /home/aes/isaacsim/python.sh isaacsim_spatial_pipeline/scripts/00_open_stage.py
"""

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
    parser.add_argument("--steps", type=int, default=120)
    parser.add_argument("--headless", action="store_true")
    return parser.parse_args()


args = parse_args()
simulation_app = SimulationApp({"headless": args.headless})

import omni.usd
from isaacsim.core.experimental.utils.stage import add_reference_to_stage


def wait_for_stage_load() -> None:
    usd_context = omni.usd.get_context()
    while simulation_app.is_running():
        _, _, loading = usd_context.get_stage_loading_status()
        if loading == 0:
            break
        simulation_app.update()


def require_file(path: Path) -> None:
    if not path.is_file():
        raise FileNotFoundError(path)


world_usd = Path(args.world_usd)
spot_usd = Path(args.spot_usd)
require_file(world_usd)
require_file(spot_usd)

usd_context = omni.usd.get_context()
usd_context.disable_save_to_recent_files()
opened = usd_context.open_stage(str(world_usd))
usd_context.enable_save_to_recent_files()
if not opened:
    raise RuntimeError(f"Failed to open stage: {world_usd}")

wait_for_stage_load()
stage = usd_context.get_stage()
spot_prim = stage.GetPrimAtPath(args.spot_prim)
if not spot_prim.IsValid():
    spot_prim = add_reference_to_stage(str(spot_usd), args.spot_prim)
    wait_for_stage_load()

print("World stage:", stage.GetRootLayer().identifier)
print("Spot prim:", args.spot_prim)
print("Spot valid:", stage.GetPrimAtPath(args.spot_prim).IsValid())

for _ in range(args.steps):
    if not simulation_app.is_running():
        break
    simulation_app.update()

simulation_app.close()

import os
from pathlib import Path

from isaacsim import SimulationApp


REPO_ROOT = Path("/home/aes/isaac_ws")
WORLD_USD = REPO_ROOT / "scenes" / "Warehouse.usd"
SPOT_USD = REPO_ROOT / "assets" / "Collected_spot" / "spot_lidar_realsense.usd"
SPOT_PRIM_PATH = "/World/Spot"
MAX_STEPS = int(os.environ.get("ISAAC_TEST_STEPS", "300"))


simulation_app = SimulationApp({"headless": False})

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
        raise FileNotFoundError(f"Required USD file does not exist: {path}")


require_file(WORLD_USD)
require_file(SPOT_USD)

print(f"Opening warehouse stage: {WORLD_USD}")
usd_context = omni.usd.get_context()
usd_context.disable_save_to_recent_files()
opened = usd_context.open_stage(str(WORLD_USD))
usd_context.enable_save_to_recent_files()
if not opened:
    raise RuntimeError(f"Failed to open warehouse stage: {WORLD_USD}")

wait_for_stage_load()

stage = omni.usd.get_context().get_stage()
if stage is None:
    raise RuntimeError("No USD stage is available after opening the warehouse.")

spot_prim = stage.GetPrimAtPath(SPOT_PRIM_PATH)
if spot_prim.IsValid():
    print(f"Spot prim already exists at: {SPOT_PRIM_PATH}")
else:
    print(f"Adding Spot reference: {SPOT_USD} -> {SPOT_PRIM_PATH}")
    spot_prim = add_reference_to_stage(str(SPOT_USD), SPOT_PRIM_PATH)
    wait_for_stage_load()

stage = omni.usd.get_context().get_stage()
spot_prim = stage.GetPrimAtPath(SPOT_PRIM_PATH)

print("Warehouse stage:", stage.GetRootLayer().identifier)
print("Spot valid:", spot_prim.IsValid())
print("Spot path:", spot_prim.GetPath() if spot_prim.IsValid() else SPOT_PRIM_PATH)
print("Spot type:", spot_prim.GetTypeName() if spot_prim.IsValid() else "<invalid>")
print("Spot children:", [child.GetName() for child in spot_prim.GetChildren()] if spot_prim.IsValid() else [])

for step in range(MAX_STEPS):
    if not simulation_app.is_running():
        break
    simulation_app.update()

simulation_app.close()

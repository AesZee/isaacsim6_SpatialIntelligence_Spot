from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": False
})

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
import numpy as np

world = World()
world.scene.add_default_ground_plane()

cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/TestCube",
        name="test_cube",
        position=np.array([0.0, 0.0, 1.0]),
        scale=np.array([0.5, 0.5, 0.5]),
    )
)

world.reset()

for i in range(300):
    world.step(render=True)

simulation_app.close()

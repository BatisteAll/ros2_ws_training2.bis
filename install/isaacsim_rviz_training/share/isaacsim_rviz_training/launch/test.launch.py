from omni.isaac.core import World

from omni.isaac.core.utils.stage import open_stage

from omni.isaac.core.utils.nucleus import get_assets_root_path

assets_root_path = get_assets_root_path()

scene_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"

# scene_path = "/home/ksu1rng/franka_alt_fingers.usd "

open_stage(usd_path=scene_path)

world = World()

world.reset()

while True:

world.step()

simulation_app.close()
#! /usr/bin/env python3.10

########################
#     START ISAAC      #
########################
print('00000000000000000000000000000000000000000000000000000000000000000000000000000000')
print('00000000000000000000000000000000000000000000000000000000000000000000000000000000')
# Note: the omniverse app need to be started before anything else!
from omni.isaac.kit import SimulationApp
# Set the config for starting the application
CONFIG = {"renderer": "RayTracedLighting", "headless": False}
# Open Isaac sim with the GUI visible (headless==false)
simulation_app = SimulationApp(CONFIG)


########################
#        IMPORTS       #
########################
# from omni.isaac.core import World
# from omni.isaac.franka.controllers.rmpflow_controller import RMPFlowController
# from omni.isaac.franka.tasks import FollowTarget
# import omni.graph.core as og
from os.path import dirname, abspath
import numpy as np
import usdrt.Usd
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils import extensions, nucleus, prims, rotations, stage, viewports



########################
#   CONST DEFINITION   #
########################
# Get the package directory from the current script path
PKG_PATH = dirname(dirname(abspath(__file__)))
BACKGROUND_STAGE_PATH = "/spacelab_scene"
BACKGROUND_USD_PATH = "/models/usd/spacelab_scene.usd"
print(PKG_PATH + BACKGROUND_USD_PATH)


########################
#    INITIALIZATION    #
########################


# enable ROS2 bridge extension
extensions.enable_extension("omni.isaac.ros2_bridge")


########################
#   STAGE PREPARATION  #
########################
# Set the units of the stage to 1 meter
simulation_context = SimulationContext(stage_units_in_meters=1.0)

# Preparing stage cameras
viewports.set_camera_view(eye=np.array([1.2, 1.2, 0.8]), target=np.array([0, 0, 0.5]))

# Loading the environment
stage.add_reference_to_stage(PKG_PATH + BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH )


while simulation_app.is_running():

    # Run with a fixed step size
    simulation_context.step(render=True)
simulation_context.stop()
simulation_app.close()










# my_world = World(stage_units_in_meters=1.0)
# my_task = FollowTarget(name="follow_target_task")
# my_world.add_task(my_task)
# my_world.reset()
# task_params = my_world.get_task("follow_target_task").get_params()
# franka_name = task_params["robot_name"]["value"]
# target_name = task_params["target_name"]["value"]
# my_franka = my_world.scene.get_object(franka_name)
# my_controller = RMPFlowController(name="target_follower_controller", robot_articulation=my_franka)
# articulation_controller = my_franka.get_articulation_controller()
# while simulation_app.is_running():
#     my_world.step(render=True)
#     if my_world.is_playing():
#         if my_world.current_time_step_index == 0:
#             my_world.reset()
#             my_controller.reset()
#         observations = my_world.get_observations()
#         actions = my_controller.forward(
#             target_end_effector_position=observations[target_name]["position"],
#             target_end_effector_orientation=observations[target_name]["orientation"],
#         )
#         articulation_controller.apply_action(actions)

# simulation_app.close()
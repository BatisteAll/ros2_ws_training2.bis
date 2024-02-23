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
import omni.graph.core as og
import os
import ament_index_python
import numpy as np
import usdrt.Usd
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils import extensions, nucleus, prims, rotations, stage, viewports
from pxr import Gf



########################
#   CONST DEFINITION   #
########################
PKG_NAME = "isaacsim_rviz_training"
# Get the package directory from the current script path
# PKG_PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG_PATH = os.path.join(ament_index_python.packages.get_package_share_directory(PKG_NAME))
BACKGROUND_STAGE_PATH = "/World"
BACKGROUND_USD_PATH = "/models/usd/spacelab_scene2.usd"
SPACELAB_ROBOT_STAGE_PATH = "/spacelab_robot_prim"
SPACELAB_ROBOT_USD_PATH = "/models/usd/spacelab_robot.usd"


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
viewports.set_camera_view(eye=np.array([2.19528, 3.2686, 2.1152]), target=np.array([1.127179, 0, 2.58546]))

# Loading the environment
print("0000000000000000000000000000000000000000000000000000000000000000000000000000000000")
print(PKG_PATH + BACKGROUND_USD_PATH)
print(BACKGROUND_STAGE_PATH)
stage.add_reference_to_stage(PKG_PATH + BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH )

# Loading the franka robot USD
prims.create_prim(
    SPACELAB_ROBOT_STAGE_PATH,
    "Xform",
    position=np.array([0, 0, 0]),
    orientation=rotations.gf_rotation_to_np_array(Gf.Rotation(Gf.Vec3d(0, 0, 1), 90)),
    usd_path=PKG_PATH + SPACELAB_ROBOT_USD_PATH,
)
simulation_app.update()


# Creating a action graph with ROS component nodes
try:
    og.Controller.edit(
        {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
    )
except Exception as e:
    print(e)
simulation_app.update()


# need to initialize physics getting any articulation..etc
simulation_context.initialize_physics()
simulation_context.play()


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
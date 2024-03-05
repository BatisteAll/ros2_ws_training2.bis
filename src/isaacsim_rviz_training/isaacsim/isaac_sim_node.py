#! /usr/bin/env python3.10

########################
#     START ISAAC      #
########################
# Note: the omniverse app need to be started before anything else!
from omni.isaac.kit import SimulationApp
# Set the config for starting the application
CONFIG = {"renderer": "RayTracedLighting", "headless": False}
# Open Isaac sim with the GUI visible (headless==false)
simulation_app = SimulationApp(CONFIG)


########################
#        IMPORTS       #
########################
import omni.graph.core as og
import os
import ament_index_python
import numpy as np
import usdrt.Usd
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils import extensions, prims, rotations, stage, viewports
from pxr import Gf
from omni.isaac.sensor import Camera



########################
#   CONST DEFINITION   #
########################
PKG_NAME = "isaacsim_rviz_training"
# Get the package directory
PKG_PATH = os.path.join(ament_index_python.packages.get_package_share_directory(PKG_NAME))
# USD paths
BACKGROUND_USD_PATH = "/description/usd/spacelab_scene.usd"
SPACELAB_ROBOT_USD_PATH = "/description/usd/spacelab_robot.usd"
# ISAAC SIM prim path
BACKGROUND_STAGE_PATH = "/World/spacelab_scene"
SPACELAB_ROBOT_STAGE_PATH = "/World/spacelab_robot"


########################
#   VAR DEFINITION   #
########################
r_camrot = 10.0
x_camrot = 0.0
y_camrot = r_camrot
theta_camrot = 0.0


########################
#    INITIALIZATION    #
########################
# enable ROS2 bridge extension
extensions.enable_extension("omni.isaac.ros2_bridge")


########################
#       SENSORS        #
########################
assembly_view_cam = Camera(
    prim_path="/World/assembly_view",
    frequency=20,
    resolution=(256, 256),
)
rotating_view_cam = Camera(
    prim_path="/World/rotating_view",
    frequency=20,
    resolution=(10000, 10000),
)

########################
#   STAGE PREPARATION  #
########################
# Set the units of the stage to 1 meter
simulation_context = SimulationContext(stage_units_in_meters=1.0)

# Preparing stage cameras
viewports.set_camera_view(eye=np.array([2.0, 3.0, 2.0]), target=np.array([0.0, 0.0, 1.0]))
viewports.set_camera_view(eye=np.array([0.8, 0.0, 3.0]), target=np.array([0.8, 0.0, 0.5]), camera_prim_path="/World/assembly_view")
viewports.set_camera_view(eye=np.array([x_camrot, y_camrot, 2.0]), target=np.array([0.0, 0.0, 1.0]), camera_prim_path="/World/rotating_view")


########################
#     STAGE LOADING    #
########################
# Loading the environment
stage.add_reference_to_stage(PKG_PATH + BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH )

# Loading the spacelab robot USD
prims.create_prim(
    SPACELAB_ROBOT_STAGE_PATH,
    "Xform",
    position=np.array([0, 0, 0]),
    orientation=rotations.gf_rotation_to_np_array(Gf.Rotation(Gf.Vec3d(0, 0, 1), 0)),
    usd_path=PKG_PATH + SPACELAB_ROBOT_USD_PATH,
)
simulation_app.update()


########################
#     ACTION GRAPH     #
########################

# Creating a action graph with ROS component nodes
try:
    og.Controller.edit(
        {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),
                ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                ("PublishJointState", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
                ("SubscribeJointState", "omni.isaac.ros2_bridge.ROS2SubscribeJointState"),
                ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
                ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnImpulseEvent.outputs:execOut", "PublishJointState.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "SubscribeJointState.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "PublishClock.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "ArticulationController.inputs:execIn"),
                ("Context.outputs:context", "PublishJointState.inputs:context"),
                ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                (
                    "SubscribeJointState.outputs:positionCommand",
                    "ArticulationController.inputs:positionCommand",
                ),
                (
                    "SubscribeJointState.outputs:velocityCommand",
                    "ArticulationController.inputs:velocityCommand",
                ),
                ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
            ],
            og.Controller.Keys.SET_VALUES: [
                # Setting the /spacelab_robot target prim to Articulation Controller node
                ("ArticulationController.inputs:usePath", True),
                ("ArticulationController.inputs:robotPath", SPACELAB_ROBOT_STAGE_PATH),
                ("PublishJointState.inputs:topicName", "joint_states"),
                ("SubscribeJointState.inputs:topicName", "joint_commands"),
                ("PublishJointState.inputs:targetPrim", [usdrt.Sdf.Path(SPACELAB_ROBOT_STAGE_PATH)]),
                ("PublishTF.inputs:targetPrims", [usdrt.Sdf.Path(SPACELAB_ROBOT_STAGE_PATH)]),
                ("Context.inputs:domain_id", [26]),
            ],
        }
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

    # Tick the Publish/Subscribe JointState and Publish Clock nodes each frame
    og.Controller.set(og.Controller.attribute("/ActionGraph/OnImpulseEvent.state:enableImpulse"), True)
    

    # Rotate the camera to have a surrounding view
    theta_camrot = theta_camrot + 0.01
    x_camrot = r_camrot*np.cos(theta_camrot*np.pi/180)
    y_camrot = r_camrot*np.sin(theta_camrot*np.pi/180)
    viewports.set_camera_view(eye=np.array([x_camrot, y_camrot, 2.0]), target=np.array([0.0, 0.0, 1.0]), camera_prim_path="/World/rotating_view")

simulation_context.stop()
simulation_app.close()






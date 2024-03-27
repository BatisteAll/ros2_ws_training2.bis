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
import omni.kit.commands
# CAMERA class
from omni.isaac.sensor import Camera
# URDF importer
from omni.importer.urdf import _urdf
from pxr import Gf
# Physics Tools
from pxr import PhysxSchema, Usd, Sdf, UsdPhysics


########################
#   CONST DEFINITION   #
########################
PKG_NAME = "isaacsim_rviz_training"
# Get the package directory
PKG_PATH = os.path.join(ament_index_python.packages.get_package_share_directory(PKG_NAME))
# URDF path
SPACELAB_ROBOT_URDF_PATH = "/description/urdf/spacelab_robot.urdf"
# USD paths
BACKGROUND_USD_PATH = "/description/usd/spacelab_scene.usd"
SPACELAB_ROBOT_USD_PATH = "/description/usd/test.usd"       # ---> needed only if using robot usd import instead of urdf
# ISAAC SIM prim path
BACKGROUND_STAGE_PATH = "/spacelab_scene"
SPACELAB_ROBOT_STAGE_PATH = "/spacelab_robot"
ARTICULATED_ROOT_JOINT_PATH = SPACELAB_ROBOT_STAGE_PATH+"/world"

########################
#    VAR DEFINITION    #
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
# enable the joint GUI
extensions.enable_extension("omni.isaac.articulation_inspector")

# Check/set ROS domain ID
try:
    ros_domain_id = int(os.environ["ROS_DOMAIN_ID"])
    print("Using ROS_DOMAIN_ID: ", ros_domain_id)
except ValueError:
    print("Invalid ROS_DOMAIN_ID integer value. Setting value to 0")
    ros_domain_id = 0
except KeyError:
    print("ROS_DOMAIN_ID environment variable is not set. Setting value to 0")
    ros_domain_id = 0


########################
#     URDF IMPORT      #
########################
# Acquire the URDF extension interface
urdf_interface = _urdf.acquire_urdf_interface()
# Set the settings in the import config
import_config = _urdf.ImportConfig()
import_config.merge_fixed_joints = False
import_config.replace_cylinders_with_capsules = False
import_config.fix_base = True
import_config.import_inertia_tensor = False
import_config.distance_scale = 1
import_config.density = 0.0
import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
import_config.default_drive_strength = 10000 #1047.19751
import_config.default_position_drive_damping = 1000 #52.35988
# import_config.subdivision_scheme
import_config.convex_decomp = True                                                  # facetization method from visual mesh to create a collision mesh
import_config.self_collision = False
import_config.collision_from_visuals = False
import_config.create_physics_scene = True
import_config.make_instanceable = False
# import_config.instanceable_usd_path = "./instanceable_meshes.usd"
import_config.parse_mimic = False


########################
#       SENSORS        #
########################
assembly_view_cam = Camera(
    prim_path="/spacelab_scene/assembly_view",
    frequency=20,
    resolution=(256, 256),
)
rotating_view_cam = Camera(
    prim_path="/spacelab_scene/rotating_view",
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
viewports.set_camera_view(eye=np.array([0.8, 0.0, 3.0]), target=np.array([0.8, 0.0, 0.5]), camera_prim_path="/spacelab_scene/assembly_view")
viewports.set_camera_view(eye=np.array([x_camrot, y_camrot, 2.0]), target=np.array([0.0, 0.0, 1.0]), camera_prim_path="/spacelab_scene/rotating_view")


########################
#     SCENE LOADING    #
########################
# Loading the environment
stage.add_reference_to_stage(PKG_PATH + BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH )


########################
#     ROBOT LOADING    #
########################
# Loading the spacelab robot URDF
# https://docs.omniverse.nvidia.com/kit/docs/omniverse-urdf-importer/latest/source/extensions/omni.importer.urdf/docs/index.html#urdf-import-commands
result, prim_path = omni.kit.commands.execute( "URDFParseAndImportFile", urdf_path=PKG_PATH + SPACELAB_ROBOT_URDF_PATH, import_config=import_config)

# Remove Articulation root from base-link and set it to ancestor
# Indeed, as we are in a multi articulation chain (robot and gripper) we need the articulation root set to the ancestor of the chains
# Doc for PhysicsComponent: https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/extensions/ux/source/omni.physx.commands/docs/index.html
# Doc for GetPrimPath: https://docs.omniverse.nvidia.com/kit/docs/pxr-usd-api/latest/pxr/Usd.html

# Get the stage reference
context = omni.usd.get_context()
current_stage = context.get_stage()

# # Remove the articulation root from the base link
# omni.kit.commands.execute('RemovePhysicsComponent',
# 	usd_prim=current_stage.GetPrimAtPath('/spacelab_robot/world'),
# 	component='PhysicsArticulationRootAPI',
# 	multiple_api_token=None)
# omni.kit.commands.execute('UnapplyAPISchema',
# 	api=UsdPhysics.ArticulationRootAPI,
# 	prim=current_stage.GetPrimAtPath('/spacelab_robot/world'),
# 	api_prefix=None,
# 	multiple_api_token=None)
# omni.kit.commands.execute('UnapplyAPISchema',
# 	api=PhysxSchema.PhysxArticulationAPI,
# 	prim=current_stage.GetPrimAtPath('/spacelab_robot/world'),
# 	api_prefix=None,
# 	multiple_api_token=None)

# # Add the articulation root to the ancestor of the base link
# omni.kit.commands.execute("AddPhysicsComponent",
# 	usd_prim=current_stage.GetPrimAtPath('/spacelab_robot'),
# 	component="PhysicsArticulationRootAPI")
# omni.kit.commands.execute('ApplyAPISchema',
# 	api=UsdPhysics.ArticulationRootAPI,
# 	prim=current_stage.GetPrimAtPath('/spacelab_robot'))
# omni.kit.commands.execute('ApplyAPISchema',
# 	api=PhysxSchema.PhysxArticulationAPI,
# 	prim=current_stage.GetPrimAtPath('/spacelab_robot'))

# # Tune the articulation root api
# omni.kit.commands.execute('ChangeProperty',
# 	prop_path=current_stage.GetPrimAtPath('/spacelab_robot.physxArticulation:enabledSelfCollisions'),
# 	value=None,
# 	prev=True)
# omni.kit.commands.execute('ChangeProperty',
# 	prop_path=current_stage.GetPrimAtPath('/spacelab_robot.physxArticulation:solverPositionIterationCount'),
# 	value=64)
# omni.kit.commands.execute('ChangeProperty',
# 	prop_path=current_stage.GetPrimAtPath('/spacelab_robot.physxArticulation:solverVelocityIterationCount'),
# 	value=64)

# omni.kit.commands.execute('SetRelationshipTargets',
# 	relationship=current_stage.GetPrimAtPath('/spacelab_robot/root_joint').GetRelationship('physics:body1'),
# 	targets=[])
# omni.kit.commands.execute('SetRelationshipTargets',
# 	relationship=current_stage.GetPrimAtPath('/spacelab_robot/root_joint').GetRelationship('physics:body0'),
# 	targets=[Sdf.Path('/spacelab_robot/world')])


# Set the collision approximation to SDF Mesh for the EndEffector
# omni.kit.commands.execute('ChangeProperty',
# 	prop_path=current_stage.GetPrimAtPath('/spacelab_robot/link_EE/collisions.physics:approximation'),
# 	value='sdf',
# 	prev=None,
# 	target_layer=Sdf.Find('/home/spacefactory5/Desktop/spacelab_robot/spacelab_robot.usd'),
# 	usd_context_name=Usd.Stage.Open(rootLayer=Sdf.Find('/home/spacefactory5/Desktop/spacelab_robot/spacelab_robot.usd'), sessionLayer=Sdf.Find('anon:0x719e6411bc80'), pathResolverContext=<invalid repr>))


# <<<<<<<<<<<<<<<<<<<< DEBUG >>>>>>>>>>>>>>>>>>>>>>
# Uncomment this line and comment the lines above to load the usd instead of the urdf
# prims.create_prim(
#     SPACELAB_ROBOT_STAGE_PATH,
#     "Xform",
#     position=np.array([0, 0, 0]),
#     orientation=rotations.gf_rotation_to_np_array(Gf.Rotation(Gf.Vec3d(0, 0, 1), 0)),
#     usd_path=PKG_PATH + SPACELAB_ROBOT_USD_PATH,
# )

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
                ("ArticulationController.inputs:robotPath", ARTICULATED_ROOT_JOINT_PATH),
                ("PublishJointState.inputs:topicName", "/isaac_joint_states"),
                ("SubscribeJointState.inputs:topicName", "/joint_commands_trigger"),
                ("PublishJointState.inputs:targetPrim", [usdrt.Sdf.Path(ARTICULATED_ROOT_JOINT_PATH)]),
                ("Context.inputs:domain_id", ros_domain_id),
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
    viewports.set_camera_view(eye=np.array([x_camrot, y_camrot, 2.0]), target=np.array([0.0, 0.0, 1.0]), camera_prim_path="/spacelab_scene/rotating_view")

simulation_context.stop()
simulation_app.close()






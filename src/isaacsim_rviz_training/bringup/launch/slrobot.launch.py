"""
##################  Spacelab Robot Launch   ##################

Description
----------
Launch the isaac sim simulation of the spacelab robot:
    - parse urdf
    - publish urdf: instentiate robot state publisher
    - instentiate controller manager
    - instentiate the controllers
        -- joint_state_broadcaster
        -- joint_trajectory_controller
    - run ISAAC SIM node
    - run RVIZ node

        
Parameters
----------
NONE

"""


########################
#        IMPORTS       #
########################
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, RegisterEventHandler
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessStart, OnProcessExit


########################
#   CONST DEFINITION   #
########################
PKG_NAME = "isaacsim_rviz_training"
URDF_FILE = "spacelab_robot.urdf.xacro"
PARAM_FILE = 'slrobot_ros_params.yaml'
ROBOT_NAMESPACE = "slrobot"

########################
#   VAR DEFINITION   #
########################


def generate_launch_description():

    ########################
    #     URDF PARSING     #
    ########################
    print("[INFO]  [slrobot.launch.py] Parsing URDF ==>")
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(PKG_NAME), "description", "urdf", URDF_FILE]),
        ]
    )
    

    ########################
    #    ROBOT STATE PUB   #
    ########################
    # Instentiate the robot_state_publisher node
    robot_description = {"robot_description": robot_description_content}
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        # namespace=ROBOT_NAMESPACE
    )


    ########################
    #  CONTROLLER MANAGER  #
    ########################
    ## Get the path to the controller_manager parameters files
    controller_params_file = os.path.join(get_package_share_directory(PKG_NAME),'bringup','config',PARAM_FILE)
    ## Initiate the controller_manager node
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description,controller_params_file],
        output="both",
        # remappings=[('robot_description', ('/',ROBOT_NAMESPACE,'/robot_description'))]
    )
    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])
    # delay to wait for robot state publisher to be instentiated to start the controller manager


    ########################
    # INITIATE CONTROLLERS #
    ########################

    # JOINT STATE BROADCASTER 
    #   --> Publish the states of the joint on a defined topic as a /joint_states msg
    joint_state_broadcaster_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],    #Note: use --stopped as argument to only load the controller and not activate it automatically
        # namespace=ROBOT_NAMESPACE
    )

    delayed_joint_state_broadcaster_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_state_broadcaster_controller],
        )
    )

    # JOINT TRAJECTORY
    #   --> Interpolates a trajectory between a current location and a target location
    joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        # namespace=ROBOT_NAMESPACE
    )

    delayed_joint_trajectory_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_trajectory_controller],
        )
    )

    # GRIPPER CONTROLLER
    #   --> for now joint trajectory can be used to move the griper joints
    #   NOTE: A gripper controller exists already in ROS2 control library
    #         https://control.ros.org/master/doc/ros2_controllers/gripper_controllers/doc/userdoc.html


    ########################
    # ISAAC SIMULATION APP #
    ########################
    # Instentiate Isaac Sim
    isaacsim_node = Node(
        package="isaacsim_rviz_training",
        executable="isaac_sim_node.py",
        output="both", # {log, console, both} --> "both" is to see the output of the node in the log & the console
        namespace=ROBOT_NAMESPACE
    )
    delayed_isaacsim_node = TimerAction(period=4.0, actions=[isaacsim_node])



    ########################
    #      RVIZ NODE       #
    ########################
    # rviz_config_file = PathJoinSubstitution(
    #     [FindPackageShare("isaacsim_rviz_training"), "config", "example.rviz"]
    # )

    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output={'both': 'log'},
    #     arguments=["-d", rviz_config_file],
    # )

    ########################
    #      NODES CALL      #
    ########################
    nodes_to_start = [
        robot_state_publisher_node,
        delayed_controller_manager,
        delayed_joint_state_broadcaster_controller,
        delayed_joint_trajectory_controller,
        delayed_isaacsim_node,
        # rviz_node,
    ]


    return LaunchDescription(nodes_to_start)

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, RegisterEventHandler
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessStart, OnProcessExit



def generate_launch_description():

    ########################
    #     URDF PARSING     #
    ########################
    print("[INFO]  [sl_robot.launch.py] Parsing URDF ==>")
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("isaacsim_rviz_training"), "models", "urdf", "spacelab_robot.urdf.xacro"]),
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
    )


    ########################
    #   ISAAC/ROS2 BRIDGE  #
    ########################
    # Instentiate the sl_robot_isaac_ros2 node
    isaacsim_rviz_training = Node(
        package="isaacsim_rviz_training",
        executable="sl_robot_isaac_ros2.py",
        output="both", # {log, console, both} --> both is to see the output of the node in the log & the console
    )
    delayed_isaacsim_rviz_training = TimerAction(period=4.0, actions=[isaacsim_rviz_training])

    ########################
    # ISAAC SIMULATION APP #
    ########################
    # Instentiate Isaac Sim
    isaacsim_node = Node(
        package="isaacsim_rviz_training",
        executable="isaac_sim_urdf_import.py",
        output="both", # {log, console, both} --> both is to see the output of the node in the log & the console
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
        delayed_isaacsim_rviz_training,
        delayed_isaacsim_node,
        # rviz_node,
    ]


    return LaunchDescription(nodes_to_start)
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

    # ur5_traj_server = Node(
    #     package='ur5_isaac_simulation',
    #     name='ur5_controller',
    #     executable='ur5_controller'
    # )

    # gripper_traj_server = Node(
    #     package='ur5_isaac_simulation',
    #     name='gripper_controller',
    #     executable='gripper_controller'
    # )


    ########################
    #   ISAAC/ROS2 BRIDGE  #
    ########################
    # Instentiate the sl_robot_isaac_ros2 node
    isaacsim_rviz_training = Node(
        package="isaacsim_rviz_training",
        executable="sl_robot_isaac_ros2.py",
        output="both", # {log, console, both} --> both is to see the output of the node in the log & the console
    )
    # delayed_bis_isaacsim_rviz_training = RegisterEventHandler(
    #     event_handler=OnProcessStart(
    #         target_action=robot_state_publisher_node,
    #         on_start=[isaacsim_rviz_training],
    #     )
    # )
    delayed_isaacsim_rviz_training = TimerAction(period=4.0, actions=[isaacsim_rviz_training])

    # # Instentiate Isaac Sim from shell
    # omniverse_isaac_sim = ExecuteProcess(
    #     cmd=['~/.local/share/ov/pkg/isaac_sim-2023.1.1/isaac-sim.sh'],
    #     output='screen'
    # )


    ########################
    # ISAAC SIMULATION APP #
    ########################
    # Instentiate Isaac Sim
    isaacsim_node = Node(
        package="isaacsim_rviz_training",
        executable="isaac_sim_urdf_import.py",
        output="both", # {log, console, both} --> both is to see the output of the node in the log & the console
    )
    # delayed_bis_isaacsim_node = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=isaacsim_rviz_training,
    #         on_exit=[isaacsim_node],
    #     )
    # )
    delayed_isaacsim_node = TimerAction(period=4.0, actions=[isaacsim_node])


    nodes_to_start = [
        # ur5_traj_server,
        # gripper_traj_server,
        robot_state_publisher_node,
        # rviz_node,
        delayed_isaacsim_rviz_training,
        # omniverse_isaac_sim,
        delayed_isaacsim_node

    ]
    return LaunchDescription(nodes_to_start)

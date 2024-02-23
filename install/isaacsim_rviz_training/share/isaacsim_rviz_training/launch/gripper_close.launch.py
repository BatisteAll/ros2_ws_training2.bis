import os
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration


def generate_launch_description():


    # Declaration of the arguments that can be used in input for the configuration of the grasping controller
    config_dec= DeclareLaunchArgument('config', default_value='Grasp_template')
    config_path_dec= DeclareLaunchArgument('config_path', default_value=[LaunchConfiguration('config'), '.yaml'])
    config_package_description = "grasping_controller"
    config_path = PathJoinSubstitution([get_package_share_directory(config_package_description), 'config', LaunchConfiguration('config_path')])

    # Command to close the gripper
    Close_gripper = Node(
        package='gazebo_rviz_training',
        executable='Command_gripper.py',
        output="screen",
        # The arguments define the position of the gripper slider and the time to perform the grasping in sec.
        arguments=["-0.01","1"])
    
    # Load the parameters into the grasping controller node (e.g. ros2 param load /my_node my_node.yaml)
    Load_Configuration = ExecuteProcess(
        cmd=['ros2', 'param', 'load', '/grasping_lifecycle_node', config_path],
        output='screen'
    )

    # Configure grasping_controller lifecycle mode
    Grasping_controller_configure = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set',
             '/grasping_lifecycle_node','configure'],
        output='screen'
    )

    # Activate grasping_controller lifecycle mode
    Grasping_controller_activate = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set',
             '/grasping_lifecycle_node','activate'],
        output='screen'
    )

    return LaunchDescription([
        # Event handlers are used to sequence and make sure one action is executed at a time (e.g. the configuration is loaded only once the node instentiated) 
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=Close_gripper,
                on_exit=[Load_Configuration],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=Load_Configuration,
                on_exit=[Grasping_controller_configure],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=Grasping_controller_configure,
                on_exit=[Grasping_controller_activate],
            )
        ),
        config_dec,
        config_path_dec,
        Close_gripper,
    ])


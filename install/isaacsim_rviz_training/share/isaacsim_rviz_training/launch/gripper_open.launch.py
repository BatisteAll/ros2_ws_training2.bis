from launch import LaunchDescription

from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit


def generate_launch_description():

    # Command to open the gripper via the action client
    Open_gripper = Node(
        package='gazebo_rviz_training',
        executable='Command_gripper.py',
        output="screen",
        # The arguments define the position of the gripper slider and the time to perform the grasping in sec.
        arguments=["0.0","1"])

    # Deactivate grasping_controller lifecycle mode
    Grasping_controller_deactivate = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set',
             '/grasping_lifecycle_node','deactivate'],
        output='screen'
    )

    # Cleanup grasping_controller lifecycle mode
    Grasping_controller_cleanup = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set',
             '/grasping_lifecycle_node','cleanup'],
        output='screen'
    )

    return LaunchDescription([
        # Event handlers are used to sequence and make sure one action is executed at a time (e.g. the configuration is loaded only once the node instentiated) 
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=Open_gripper,
                on_exit=[Grasping_controller_deactivate],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=Grasping_controller_deactivate,
                on_exit=[Grasping_controller_cleanup],
            )
        ),
        Open_gripper,
        
    ])

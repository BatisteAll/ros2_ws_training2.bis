import random

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    # Position and orientation of the robot to spawn
    # [X, Y, Z]
    position = [0.0, 0.0, 0.0]
    # [Roll, Pitch, Yaw]
    orientation = [0.0, 0.0, 0.0]
    # Base Name or robot
    #robot_base_name = "test_robot"
    entity_name = "test_robot"
    
    # Used to increment the name of the robot. Can be used if multiple robots need to be loaded
    #entity_name = robot_base_name+"-"+str(int(random.random()*100000))

    # Spawn the robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-entity',
                   entity_name,
                   '-x', str(position[0]), '-y', str(position[1]
                                                     ), '-z', str(position[2]),
                   '-R', str(orientation[0]), '-P', str(orientation[1]
                                                        ), '-Y', str(orientation[2]),
                   '-topic', '/test_namespace/robot_description',
                   '-robot_namespace','/test_namespace',
                   ]
    )
    
    # Load the joint state controller with ros2 control command
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster','-c','/test_namespace/controller_manager'],
        output='screen'
    )

    # Load the joint trajectory controller with ros2 control command
    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_trajectory_controller','-c','/test_namespace/controller_manager'],
        output='screen'
    )

    # Run grasping controller with a dedicated script from a dedicated package
    run_grasping_controller = ExecuteProcess(
        cmd=['ros2', 'run', 'grasping_controller', 'grasping_controller'],
        output='screen'
    )

    # create and return launch description object
    return LaunchDescription(
        [
            # Event handlers are used to sequence and make sure one action is executed at a time (e.g. the configuration is loaded only once the node instentiated)
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_robot,
                    on_exit=[load_joint_state_controller],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_controller,
                    on_exit=[load_joint_trajectory_controller],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_trajectory_controller,
                    on_exit=[run_grasping_controller],
                )
            ),
            spawn_robot
        ]
    )

from launch import LaunchDescription

import launch.actions
import launch_ros.actions


def generate_launch_description():

    return LaunchDescription([
        launch_ros.actions.Node(
            package='gazebo_rviz_training',
            executable='Command_robot.py',
            output='screen',
            arguments=["1.5053, -1.2618, 1.8317, -2.1601, -1.5708, -0.0654","3"]),
    ])

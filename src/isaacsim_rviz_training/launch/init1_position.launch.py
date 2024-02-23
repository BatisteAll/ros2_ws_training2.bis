from launch import LaunchDescription

import launch.actions
import launch_ros.actions


def generate_launch_description():

    return LaunchDescription([
        launch_ros.actions.Node(
            package='gazebo_rviz_training',
            executable='Command_robot.py',
            output='screen',
            arguments=["-1.5708, -2.5, 2.5, -3.1415, 0.0, 0.0","3"])
    ])

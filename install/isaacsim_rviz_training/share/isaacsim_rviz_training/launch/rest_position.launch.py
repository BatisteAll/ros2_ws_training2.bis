from launch import LaunchDescription

import launch.actions
import launch_ros.actions


def generate_launch_description():

    return LaunchDescription([
        launch_ros.actions.Node(
            package='gazebo_rviz_training',
            executable='Command_robot.py',
            output='screen',
            arguments=["0.0, -2.1817, 2.1817, -1.5708, -1.5708, 0.0","3"]),
    ])

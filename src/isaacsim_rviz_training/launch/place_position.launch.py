from launch import LaunchDescription

import launch.actions
import launch_ros.actions


def generate_launch_description():

    return LaunchDescription([
        launch_ros.actions.Node(
            package='gazebo_rviz_training',
            executable='Command_robot.py',
            output='screen',
            arguments=["-0.1003, -1.1921, 1.8588, -2.2381, -1.5708, -0.0436","3"]),
    ])

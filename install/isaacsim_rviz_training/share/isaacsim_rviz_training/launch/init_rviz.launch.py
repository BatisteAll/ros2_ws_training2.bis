import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
import xacro

from launch import LaunchDescription

import launch.actions
import launch_ros.actions

# this is the function launch system will look for
def generate_launch_description():

    # Define the name of the rviz config file and the associated package
    rviz_config_file = 'example.rviz'
    package_description = "gazebo_rviz_training"

    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', rviz_config_file)

    # Launch Rviz with the associated config file
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])
            
    # Launch the import_3D_1.py code that will be used to generate markers 
    Import_3D_1 = Node(
        package='gazebo_rviz_training',
        executable='import_3D_1.py',
        output='screen')
        
    # Launch the import_3D_1.py code that will be used to generate markers 
    Import_3D_2 = Node(
        package='gazebo_rviz_training',
        executable='import_3D_2.py',
        output='screen')
        
    # Create and return launch description object
    return LaunchDescription(
        [
        Import_3D_1,
        Import_3D_2,
        rviz_node
        ]
    )

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
import xacro

# this is the function launch system will look for
def generate_launch_description():

    # Define the name of the URDF file and the associated package
    urdf_file = 'ros2_training.urdf.xacro'
    package_description = "gazebo_rviz_training"

    # Define the path where is located the URDF file
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "models", "urdf", urdf_file)

    # Launch xacro on the URDF file to parse every sub-urdf into the main one
    print("[INFO]  [publish_urdf.launch.py] Parsing URDF ==>")
    robot_description_config = xacro.process_file(robot_desc_path).toxml()

    params = {'robot_description': robot_description_config}

    # Launch the Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        namespace='test_namespace',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True}, params],
        output="screen"
    )
            
    # create and return launch description object
    return LaunchDescription(
        [          
            robot_state_publisher_node,
        ]
    )

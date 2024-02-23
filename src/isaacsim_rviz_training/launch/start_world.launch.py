import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_prefix
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

def generate_launch_description():

    # Get the path to the gazebo_ros package
    # This package will be used to find the gazebo.launch.py file that will be used later to launch gazebo
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # Get the path to the gazebo_rviz_training package
    pkg_training_gazebo = get_package_share_directory('gazebo_rviz_training')

    # We get the whole install dir
    # We do this to avoid having to copy or softlink manually the packages so that gazebo can find them
    description_package_name = "gazebo_rviz_training"
    install_dir = get_package_prefix(description_package_name)

    # Set the path to the WORLD model files (in environment variable). This is to find the models inside the models folder
    gazebo_models_path = os.path.join(pkg_training_gazebo, 'models', 'gazebo')
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share' + ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  install_dir + "/share" + ':' + gazebo_models_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    # Print the computed paths in the terminal for information
    print("[INFO] [start_world.launch.py] GAZEBO MODELS PATH=="+str(os.environ["GAZEBO_MODEL_PATH"]))
    print("[INFO] [start_world.launch.py] GAZEBO PLUGINS PATH=="+str(os.environ["GAZEBO_PLUGIN_PATH"]))

    # Get path to controller file
    gazebo_params_path = os.path.join(get_package_share_directory(description_package_name),'config','my_controllers.yaml')

    # Gazebo launch using gazebo.launch.py
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
           launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_path, 'pause': 'true'}.items()
    )    




    # The return section is the "main" section of the launch file where actions are executed
    return LaunchDescription([

        # DeclareLaunchArguments is a tool defining the parameters/arguments that can be pass to the launch file 
        # from the above launch file or from the console
        DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(pkg_training_gazebo, 'world', 'example.world'), ''],
          description='SDF world file'),

        # Launch gazebo with the example.world file
        gazebo
    ])

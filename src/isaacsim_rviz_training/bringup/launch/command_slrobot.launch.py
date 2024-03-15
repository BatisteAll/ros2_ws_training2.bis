"""
##################  Spacelab Robot - Send Command Launch   ##################

Description
----------
Send a command (target pose) to the simulated spacelab robot:
    - parse arguments to get the prim and the pose
    - instentiate the action node with proper arguments

        
Parameters
----------
goal_prim : arg string
    define the targeted prim to be controlled
    {'robot', 'gripper'}
target_pose : arg string
    define the pose to which the targeted prim needs to be moved
    each name corresponds to a pose in a small database
    {'rest', 'pick_far', 'pick', 'place_far', 'place', 'init', 'user_defined'}
goal_exec_time : arg string
    time in second to go from the current location to the targeted pose
full_sequence: boolean
    full pick/place sequence execution flag
    {true, false}

"""

########################
#        IMPORTS       #
########################
import numpy as np
import sys
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


########################
#   CONST DEFINITION   #
########################
PKG_NAME = "isaacsim_rviz_training"
TARGET_POSE_ROBOT_NAME_LIST = ['rest', 'pick_far', 'pick', 'place_far', 'place', 'init']
TARGET_POSE_ROBOT_LIST = np.array([[0.0, -2.1817, 2.1817, -1.5708, -1.5708, 0.0],           #rest
                                  [1.5053, -1.2618, 1.8317, -2.1601, -1.5708, -0.0654],     #pick_far
                                  [1.5053, -1.2095, 1.8841, -2.2383, -1.5708, -0.0654],     #pick
                                  [-0.1003, -1.2401, 1.8501, -2.1901, -1.5708, -0.0436],    #place_far
                                  [-0.1003, -1.1921, 1.8588, -2.2381, -1.5708, -0.0436],    #place
                                  [-1.5708, -2.5, 2.5, -3.1415, 0.0, 0.0]])                 #init
TARGET_POSE_GRIPPER_NAME_LIST = ['open', 'close']
TARGET_POSE_GRIPPER_LIST = np.array([[0.0, 0.0],     #open
                                    [-0.02, -0.02]])       #close



########################
#    ARGS DEFINITION   #
########################
# Declaration of the arguments that are called from the console with "arg_name:=value"
launch_args = [
    DeclareLaunchArgument('prim', default_value='robot', description='define the targeted prim to be controlled'),                      # {'robot', 'gripper'}
    DeclareLaunchArgument('target', default_value='init', description='define the pose to which the targeted prim needs to be moved'),  # {'rest', 'pick_far', 'pick', 'place_far', 'place', 'init', 'user_defined'}
    DeclareLaunchArgument('execution_time', default_value='10', description='time [s] to go from current location to targeted pose'),   # time in second
    DeclareLaunchArgument('full_sequence', default_value='false', description='executes the full sequence')                             # full pick/place sequence execution
]



# METHOD: setup the launch context in order to retrieve the value of the arguments 
#         for them to be used as python variables
#--------------------------------------------------------------
def launch_setup(context):

    ########################
    #  ARGUMENTS PARSING   #
    ########################
    # Retrieve the value of the arguments for them to be used as python variables
    goal_prim= LaunchConfiguration('prim').perform(context)
    target_pose_name= LaunchConfiguration('target').perform(context)
    goal_exec_time= LaunchConfiguration('execution_time').perform(context)
    full_sequence_flag= LaunchConfiguration('full_sequence').perform(context)

    ########################
    #   POSE DEFINITION    #
    ########################
    #ROBOT pose definition
    if (goal_prim == 'robot' and target_pose_name in TARGET_POSE_ROBOT_NAME_LIST):
        for i in range(0,len(TARGET_POSE_ROBOT_LIST)):
            if target_pose_name == TARGET_POSE_ROBOT_NAME_LIST[i]:
                target_joint_angles = TARGET_POSE_ROBOT_LIST[i,:]

    #GRIPPER pose definition
    elif(goal_prim == 'gripper' and target_pose_name in TARGET_POSE_GRIPPER_NAME_LIST):
        for i in range(0,len(TARGET_POSE_GRIPPER_LIST)):
            if target_pose_name == TARGET_POSE_GRIPPER_NAME_LIST[i]:
                target_joint_angles = TARGET_POSE_GRIPPER_LIST[i,:]

    #ERROR
    else:
        sys.exit(('[ERROR]: The prim needs to be set either to "robot" or to "gripper" to control on of these two systems.\n\
                        And the "target_pose_name" argument has to belong to the following list of pose:',\
                        TARGET_POSE_ROBOT_NAME_LIST,'\n', TARGET_POSE_GRIPPER_NAME_LIST ))
    
    # Convert target_joint_angles array to string in order to be passed as a node argument (requiring string or LaunchConfiguration types)
    # https://numpy.org/doc/stable/reference/generated/numpy.array2string.html
    target_joint_angles_str = (np.array2string(target_joint_angles, separator=',')).replace("[","").replace("]","") #remove the brackets of the array

    print('==================================')
    print('[command_slrobot.launch.py] [INFO]: The prim',goal_prim,'will perform the requested trajectory',\
           'during [',goal_exec_time,'s], from the current position to the',target_pose_name,'position :[',target_joint_angles_str,']')
    print('==================================')

    ########################
    #      ACTION NODE     #
    ########################
    # Command to move the robot or the gripper
    command_slrobot = Node(
        package=PKG_NAME,
        executable='command_slrobot.py',
        output="screen",
        # The arguments define the position of the gripper slider and the timtargete to perform the grasping in sec.
        arguments=[goal_prim, target_joint_angles_str, goal_exec_time, full_sequence_flag])

    return [command_slrobot]
#--------------------------------------------------------------


#METHOD: executes the nodes setup in the LaunchDescription
#--------------------------------------------------------------
def generate_launch_description():
    ############################
    # SETUP LAUNCH DESCRIPTION #
    ############################
    # setup the launch description
    # this is important for the args to be inscribed in the "context" variable of launch_setup method
    launch_description = LaunchDescription(launch_args)

    ########################
    #     NODES TO CALL    #
    ########################
    # Instentiate the OpaqueFunction named launch_setup to retrieve arguments value and setup the nodes to call
    nodes_to_call = OpaqueFunction(function = launch_setup)
    launch_description.add_action(nodes_to_call)

    return launch_description
#--------------------------------------------------------------

########################
#        IMPORTS       #
########################
import numpy as np
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


########################
#   CONST DEFINITION   #
########################
PKG_NAME = "isaacsim_rviz_training"
TARGET_POSE_ROBOT_NAME_LIST = ['rest', 'pick_far', 'pick', 'place_far', 'place', 'init']
TARGET_POSE_ROBOT_LIST = np.array[[0.0, -2.1817, 2.1817, -1.5708, -1.5708, 0.0],            #rest
                                  [1.5053, -1.2618, 1.8317, -2.1601, -1.5708, -0.0654],     #pick_far
                                  [1.5053, -1.2095, 1.8841, -2.2383, -1.5708, -0.0654],     #pick
                                  [-0.1003, -1.2401, 1.8501, -2.1901, -1.5708, -0.0436],    #place_far
                                  [-0.1003, -1.1921, 1.8588, -2.2381, -1.5708, -0.0436],    #place
                                  [-1.5708, -2.5, 2.5, -3.1415, 0.0, 0.0]]                  #init
TARGET_POSE_GRIPPER_NAME_LIST = ['open', 'close']
TARGET_POSE_GRIPPER_LIST = np.array[[0.02, 0.02],     #open
                                    [0.0, 0.0]]     #close


########################
#    VAR DEFINITION    #
########################


def generate_launch_description():

    ########################
    #  ARGUMENTS PARSING   #
    ########################
    # Declaration of the arguments that are called from the console with "arg_name:=value"
    goal_prim= DeclareLaunchArgument('prim', default_value='robot') # {'robot', 'gripper'}
    target_pose= DeclareLaunchArgument('target', default_value='user_defined')  #{'rest', 'pick_far', 'pick', 'place_far', 'place', 'init', 'user_defined'}
    goal_exec_time= DeclareLaunchArgument('execution_time', default_value=3)    # time in second


    ########################
    #   POSE DEFINITION    #
    ########################
    #ROBOT pose definition
    if (goal_prim == 'robot' and target_pose in TARGET_POSE_ROBOT_NAME_LIST):
        for i in range(0,len(TARGET_POSE_ROBOT_LIST)):
            if target_pose == TARGET_POSE_ROBOT_NAME_LIST[i]:
                target_joint_angles = TARGET_POSE_ROBOT_LIST[i,:]

    #GRIPPER pose definition
    elif(goal_prim == 'gripper' and target_pose in TARGET_POSE_GRIPPER_NAME_LIST):
        for i in range(0,len(TARGET_POSE_GRIPPER_LIST)):
            if target_pose == TARGET_POSE_GRIPPER_NAME_LIST[i]:
                target_joint_angles = TARGET_POSE_GRIPPER_LIST[i,:]

    #ERROR
    else:
        print('[ERROR]: The prim needs to be set either to "robot" or to "gripper" to control on of these two systems.\n\
                        And the "target_pose" argument has to belong to the following list of pose:',\
                        TARGET_POSE_ROBOT_NAME_LIST,'\n', TARGET_POSE_GRIPPER_NAME_LIST )
        exit
        

    ########################
    #      ACTION NODE     #
    ########################
    # Command to move the robot or the gripper
    command_slrobot = Node(
        package=PKG_NAME,
        executable='command_slrobot.py',
        output="screen",
        # The arguments define the position of the gripper slider and the time to perform the grasping in sec.
        arguments=[goal_prim, target_joint_angles, goal_exec_time])


    ########################
    #      NODES CALL      #
    ########################
    nodes_to_start = [
        command_slrobot,
    ]


    return LaunchDescription(nodes_to_start)
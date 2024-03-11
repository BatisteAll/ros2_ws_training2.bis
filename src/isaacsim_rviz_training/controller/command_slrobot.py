#! /usr/bin/env python3.10

"""
##################  Spacelab Robot - Send Command Node   ##################

Description
----------
Send a command (target pose) to the simulated spacelab robot:
        
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

"""

########################
#        IMPORTS       #
########################
import numpy as np
import rclpy
import sys
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

# Import Helper Functions form this current package
# from isaacsim_rviz_training.controller.helper_functions.load_ros_parameters import get_ros_parameters
import os
from ament_index_python.packages import get_package_share_directory
import yaml


########################
#   CONST DEFINITION   #
########################
PKG_NAME = "isaacsim_rviz_training"
NODE_NAME = "command_slrobot"
ROBOT_NAME = "Spacelab Robot"
PARAM_FILE_NAME = 'slrobot_ros_params.yaml'

########################
#    VAR DEFINITION    #
########################




#------------------------------------------load_yaml_file------------
class command_slrobot(Node):
    """Class to simulate Space Lab Robot in Isaac Sim"""



    #------------------------------------------------------
    def __init__(self):
        """Initialize the SpaceLab robot Isaac Sim simulation"""

        super().__init__(NODE_NAME)


        #########################
        # SLROBOT ACTION CLIENT #
        #########################
        # Instentiate the action client that will request the server the trajectory to a target position
        # This is done through the "topic?" '/namespace_if_existing/joint_trajectory_controller/follow_joint_trajectory'
        self._action_client = ActionClient(self,
                                           FollowJointTrajectory,
                                           '/joint_trajectory_controller/follow_joint_trajectory')

        #########################
        # GRIPPER ACTION CLIENT #
        #########################
        self._gripper_action_client = ActionClient(self,
                                                    FollowJointTrajectory,
                                                    '/joint_trajectory_controller/follow_joint_trajectory')
        
        ########################
        #   ROS2 PARAMETERS    #
        ########################
        # Get the parameters from the yaml file
        config_file = os.path.join(
            get_package_share_directory(PKG_NAME),
            'bringup',
            'config',
            PARAM_FILE_NAME
        )
        def load_yaml_file(filename) -> dict:
            """Load yaml file with the Parameters"""
            with open(filename, 'r', encoding='UTF-8') as file:
                data = yaml.safe_load(file)
            return data      
        config = load_yaml_file(config_file)
        self.ros_parameters = config[NODE_NAME]["ros__parameters"]
        self.robot_name = ROBOT_NAME
    #------------------------------------------------------       




    # [METHOD]  SEND GOAL ROBOT
    #------------------------------------------------------
    def send_goal_robot(
        self,
        target_pose: list,
        time_in_sec: int = 3
    ):
        """
        Send trajectory to the UR5 robot in Isaac Sim.

        Parameters
        ----------
        target_pose : list
            Target joint positions or
            target pose [x, y, z, roll, pitch, yaw] in degrees (if inv_kin
            is True).
        time_in_sec : int
            Duration desired to go from the current location to the target one.

        """

        if time_in_sec is not None:
            # Define the goal primary container for communication purposes
            self.goal_prim = self.robot_name
            # Get the list of actuated joints from the param list
            joint_names = self.ros_parameters['actuated_robot_joints']
            self.get_logger().info(f"Requested joints to be actuated: {joint_names}")

            # Define the goal of the requested "action" as a "FollowJointTrajectory.Goal"
            goal = FollowJointTrajectory.Goal()
            # goal.trajectory.joint_names = joint_names
            goal.trajectory.joint_names = ['joint_1_2', 'joint_2_3', 'joint_3_4', 'joint_4_5', 'joint_5_6', 'joint_6_7', 'joint_EE_Lgripper']
            duration = Duration(sec=time_in_sec)
            goal.trajectory.points.append(
                JointTrajectoryPoint(positions=[1.5053, -1.2618, 1.8317, -2.1601, -1.5708, -0.0654, 0.0],
                                     velocities=[0.0]*7,
                                     accelerations=[0.0]*7,
                                     time_from_start=duration))
            
            # Contact the server to send the requested goal (timeout if no server is found) (wait_for_server() is a method of the ActionClient() class)
            server_reached = self._action_client.wait_for_server(timeout_sec=10.0)
            if not server_reached:
                self.get_logger().error(f'Unable to connect to {self.goal_prim} action server. Timeout exceeded.')
                sys.exit()

            # Send the goal to the server
            send_goal_future =self._action_client.send_goal_async(goal,feedback_callback=self.feedback_callback)
            # Add a callback to the requested future to be executed/sent when the task is done
            send_goal_future.add_done_callback(self.goal_response_callback)
            # Display infos to the CLI/joint_trajectory_controller/follow_joint_trajectory
            self.get_logger().info(f"[{self.goal_prim}] Joint goal {target_pose} sent to the robot, expecting a [{time_in_sec}s] execution duration as expected.")

            return send_goal_future
        else:
            self.get_logger().error("Please, specify the time in seconds when sending a goal to a prim in node command_slrobot")
    #------------------------------------------------------


    # [METHOD]  SEND GOAL GRIPPER
    #------------------------------------------------------
    def send_goal_gripper(
        self,
        target_pose: list,
        time_in_sec: int = 3
    ):
        """
        Send trajectory to the UR5 gripper in Isaac Sim.

        Parameters
        ----------
        time_in_sec : float
            Time in seconds to complete the action.
        position : list
            List of joint positions.

        """
        if time_in_sec is not None:
            self.goal_prim = 'GRIPPER'

            # Get the list of actuated joints from the param list
            gripper_joint_names = self.ros_parameters['actuated_robot_joints']

            # Define the goal of the requested "action" as a "FollowJointTrajectory.Goal"
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = gripper_joint_names
            duration = Duration(sec=time_in_sec)
            n_joints = len(gripper_joint_names)
            goal.trajectory.points.append(
                JointTrajectoryPoint(positions=target_pose,
                                    velocities=[0.0]*n_joints,
                                    accelerations=[0.0]*n_joints,
                                    time_from_start=duration))
            
            # Contact the server to send the requested goal (timeout if no server is found) (wait_for_server() is a method of the ActionClient() class)
            self._gripper_action_client.wait_for_server(timeout_sec=10.0)

            # Send the goal to the server
            send_goal_future =self._gripper_action_client.send_goal_async(goal,feedback_callback=self.feedback_callback)
            send_goal_future.add_done_callback(self.goal_response_callback)
            self.get_logger().info(f"[{self.goal_prim}] Joint goal [{target_pose}] sent to the robot.")

            return send_goal_future
        else:
            self.get_logger().error("Please, specify the time in seconds when sending a goal to a prim in node command_slrobot")
    #------------------------------------------------------



    # [METHOD] ACTION CLIENT FEEDBACK
    #------------------------------------------------------
    def feedback_callback(self, feedback_msg):
        """
        Receives feedback from the action server and print in console.

        Parameters
        ----------
        feedback_msg : FollowJointTrajectory.Feedback
            Feedback message from the action server

        """
        self.get_logger().info("===========METHOD: feedback_callback")
        feedback = feedback_msg.feedback
        info = self.get_logger().info
        des_positions = np.round(feedback.desired.positions, 5)
        actual_positions = np.round(feedback.actual.positions, 5)
        error_positions = np.round(feedback.error.positions, 5)
        info(
            f'[Feedback] [{self.goal_prim}] Desired position: {des_positions}')
        info(
            f'[Feedback] [{self.goal_prim}] Reached position: {actual_positions}')
        info(
            f'[Feedback] [{self.goal_prim}] Error position by joint: {error_positions}')
        info(f"[Feedback] [{self.goal_prim}] Absolute error (radians): "f"{abs(sum(feedback.error.positions))}")
    #------------------------------------------------------


    # [METHOD]  ROBOT ACTION CLIENT RESPONSE
    #------------------------------------------------------
    def goal_response_callback(self, future):
        """
        Receives the response from the action server.

        Parameters
        ----------
        future : rclpy.task.Future
            Future object with the response from the action server

        """
        self.get_logger().info("===========METHOD: goal_response_callback")
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected :(')
            return

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)
    #------------------------------------------------------


    # [METHOD]  ROBOT ACTION CLIENT RESULT
    #------------------------------------------------------
    def get_result_callback(self, future):
        """
        Receives the result from the action server.

        Parameters
        ----------
        future : rclpy.task.Future
            Future object with the result from the action server

        """
        self.get_logger().info("===========METHOD: get_result_callback")
        result = future.result().result
        self.get_logger().info(f'[Feedback] [{self.goal_prim}] Result: {result.error_string}')
    #------------------------------------------------------



#------------------------------------------------------
def main(
        # goal_prim: str,
        # pose_joint_angles: list,
        # goal_exec_duration: int = 3,
        args=None
):
    """
    Main function to run the sl_robot Isaac Sim simulation.

    Parameters
    ----------
    goal_prim : str
        Prim to be actuated. (sys.argv[1])
    pose_joint_angles : list
        List of joint positions. (sys.argv[2])
    goal_exec_duration : int
        Time in seconds to complete the action. (sys.argv[3])

    """

    # Instentiate ROS
    rclpy.init(args=args)

    ########################
    #    PARSE ARGUMENTS   #
    ########################
    # Split and store command line argument
    goal_prim = sys.argv[1]
    pose_joint_angles_str = sys.argv[2].split(',')

    def convert_strings_to_floats(input_array):
        output_array = []
        for element in input_array:
            converted_float = float(element)
            output_array.append(converted_float)
        return output_array
    pose_joint_angles = convert_strings_to_floats(pose_joint_angles_str)
    goal_exec_duration = int(sys.argv[3])
    print('==================================')
    print('Initialization of the node command_slrobot')
    print(goal_prim,pose_joint_angles,goal_exec_duration)
    print('==================================')


    ########################
    #   INSTENTIATE NODE   #
    ########################            
    # Instentiate the container
    slrobot_client = command_slrobot()
    # Execute the initialization method of the node
    # the spin_once tool executes one action and stop at the first callback of either timeout or the end of the node execution
    rclpy.spin_once(slrobot_client, timeout_sec=0.5)



    ########################
    #       SEND GOAL      #
    ########################
    # send command to the robot and get the feedback
    if(goal_prim == 'robot'):
        future = slrobot_client.send_goal_robot(pose_joint_angles, goal_exec_duration)
    elif(goal_prim == 'gripper'):
        future = slrobot_client.send_goal_gripper(pose_joint_angles, goal_exec_duration)
    else:
        print('[ERROR]: The prim needs to be set either to "robot" or to "gripper" to control on of these two systems.')
        exit

    # Execute the node until its goal is achieved
    # spin_until_future_complete: tool that executes a command from a node until the future condition is verified
    rclpy.spin_until_future_complete(slrobot_client, future)
    print("The goal is reached, shutting down the action node.")

    ########################
    #     DESTROY NODE     #
    ########################
    slrobot_client.destroy_node()
    rclpy.shutdown()
#------------------------------------------------------






if __name__ == "__main__":
    main()

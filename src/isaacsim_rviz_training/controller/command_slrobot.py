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
from isaacsim_rviz_training.controller.helper_functions.load_ros_parameters import get_ros_parameters



########################
#   CONST DEFINITION   #
########################
NODE_NAME = "command_slrobot"
ROBOT_NAME = "Spacelab Robot"

########################
#    VAR DEFINITION    #
########################



#------------------------------------------------------
class command_slrobot(Node):
    """Class to simulate Space Lab Robot in Isaac Sim"""



    #------------------------------------------------------
    def __init__(self):
        """Initialize the SpaceLab robot Isaac Sim simulation"""

        super().__init__(NODE_NAME)


        #########################
        # SLROBOT ACTION CLIENT #
        #########################
        self._action_client = ActionClient(self,
                                           FollowJointTrajectory,
                                           'sl_robot/follow_joint_trajectory')

        #########################
        # GRIPPER ACTION CLIENT #
        #########################
        self._gripper_action_client = ActionClient(self,
                                                    FollowJointTrajectory,
                                                    'sl_robot_gripper/follow_joint_trajectory')
        
        ########################
        #   ROS2 PARAMETERS    #
        ########################
        # Call to the helper function to get the list of parameters
        self.ros_parameters = get_ros_parameters(NODE_NAME)
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

            # Define the goal of the requested "action" as a "FollowJointTrajectory.Goal"
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = joint_names
            duration = Duration(sec=time_in_sec)
            goal.trajectory.points.append(
                JointTrajectoryPoint(positions=target_pose, #[float(angle[0]), float(angle[1]), float(angle[2]), float(angle[3]), float(angle[4]), float(angle[5])]
                                     velocities=[0.0]*6,
                                     accelerations=[0.0]*6,
                                     time_from_start=duration))
            
            # Contact the server to send the requested goal (timeout if no server is found) (wait_for_server() is a method of the ActionClient() class)
            self._action_client.wait_for_server(timeout_sec=10.0)

            # Send the goal to the server
            send_goal_future =self._action_client.send_goal_async(goal,feedback_callback=self.feedback_callback)
            send_goal_future.add_done_callback(self.goal_response_callback)
            self.get_logger().info(f"[{self.goal_prim}] Joint goal [{target_pose}] sent to the robot.")
        else:
            self.get_logger().error("Please, specify the time in seconds")
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
        result = future.result().result
        self.get_logger().info(f'[Feedback] [{self.goal_prim}] Result: {result.error_string}')
    #------------------------------------------------------



#------------------------------------------------------
def main(
        self,
        target_pose: list,
        goal_duration_in_sec: int = 3,
        args=None
):
    """
    Main function to run the sl_robot Isaac Sim simulation.

    Parameters
    ----------
    time_in_sec : float
        Time in seconds to complete the action.
    position : list
        List of joint positions.

    """

    # Instentiate ROS
    rclpy.init(args=args)

    ########################
    #   INSTENTIATE NODE   #
    ########################            
    # Instentiate the container
    slrobot_client = command_slrobot()
    # Execute the initialization method of the node
    # the spin_once tool executes one action and stop at the first callback of either timeout or the end of the node execution
    rclpy.spin_once(slrobot_client, timeout_sec=0.5)


    ########################
    #    PARSE ARGUMENTS   #
    ########################
    # Split and store command line argument
    goal_prim = sys.argv[1]
    pose_joint_angles = sys.argv[2].split(',')
    goal_exec_duration = sys.argv[3]


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


    ########################
    #     DESTROY NODE     #
    ########################
    slrobot_client.destroy_node()
    rclpy.shutdown()
#------------------------------------------------------






if __name__ == "__main__":
    main()

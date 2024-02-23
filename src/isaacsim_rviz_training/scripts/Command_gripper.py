#! /usr/bin/env python3.10

import os
import sys
import rclpy
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

class CommandRobotActionClient(Node):

    # Initialization method
    def __init__(self):
        # super can be used to refer to parent classes without naming them explicitly
        # here it allows to inheritate the methods of the original CommandRobotActionClient() class
        super().__init__('command_robot_actionclient')

        # Create an action client, that will communicate with the server FollowJointTrajectory
        # Note: the FollowJointTrajectory action is defined by the control_msgs package and makes the robot move along a trajectory
        self._action_client = ActionClient(self, FollowJointTrajectory, '/test_namespace/joint_trajectory_controller/follow_joint_trajectory')
        
    # Define the action client goal
    def send_goal(self, gripper_pose, exec_time):
        actual_state = FollowJointTrajectory.Feedback()
        goal_msg = FollowJointTrajectory.Goal()

        # Define the name of the joints (to be defined like in /models/urdf/gripper.xacro)
        joint_names = ["joint_EE.Lgripper"]

        points = []
        # A JointTrajectoryPoint is created, it is the standard message type to communicate with FollowJointTrajectory
        point1 = JointTrajectoryPoint()
        # The time to reach this point is defined based on the input arguments (to_msg() convert the Duration format into standard msg format)
        point1.time_from_start = Duration(seconds=int(exec_time), nanoseconds=0).to_msg()
        # The point1 position is filled with gripper position defined as argument
        point1.positions = [float(gripper_pose)]

        # create list "points" and appends point1
        points.append(point1)
        
        # the goal entity is also parametrized with a duration, the joint names to actuate, and the list of points (trajectory)
        goal_msg.goal_time_tolerance = Duration(seconds=1, nanoseconds=0).to_msg()
        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.points = points

        # Wait for an action server to be ready (wait_for_server() is a method of the ActionClient() class)
        self._action_client.wait_for_server()
        
        # Send the goal and asynchronously get the result (send_goal_async() is a method of the ActionClient() class)
        return  self._action_client.send_goal_async(goal_msg)

def main(args=None):
    
    # rclpy library provides the canonical Python API for interacting with ROS 2.
    # the init() must be executed before any ROS2 node can be created
    rclpy.init()

    # Call the above class
    action_client = CommandRobotActionClient()

    # Split and store command line argument
    gripper_pose = sys.argv[1]
    exec_time = sys.argv[2]
    
    # the spin_once tool executes one action and stop at the first callback of either timeout or the end of the node execution
    rclpy.spin_once(action_client, timeout_sec=0.5)
    
    # send command to the gripper
    future = action_client.send_goal(gripper_pose,exec_time)

    # spin_until_future_complete tool executes a command from a node until the future condition is verified
    rclpy.spin_until_future_complete(action_client, future)


# this block ensures the main() is executed only when the file runs as a script, but not when it is imported as a module 
if __name__ == '__main__':
    main()

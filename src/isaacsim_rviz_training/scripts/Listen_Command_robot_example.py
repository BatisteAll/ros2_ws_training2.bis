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

    def __init__(self):
        super().__init__('command_robot_actionclient')
        # Create an action client that will communicate with the server FollowJointTrajectory
        self._action_client = ActionClient(self, FollowJointTrajectory, '/test_namespace/joint_trajectory_controller/follow_joint_trajectory')
        
        # Create a subscriber that will get the joint state from the topic /test_namespace/joint_states
        self.subscription = self.create_subscription(
            JointState,
            '/test_namespace/joint_states',
            self.listener_callback,
            10)
        self.subscription

    # Define the action client goal
    def send_goal(self, angle):
        actual_state = FollowJointTrajectory.Feedback()
        goal_msg = FollowJointTrajectory.Goal()

        # Define the name of the joints
        joint_names = ["joint_1",
                       "joint_2",
                       "joint_3",
                       "joint_4",
                       "joint_5",
                       "joint_6"]

        points = []
        # JointTrajectoryPoint is the standard message type to communicate with FollowJointTrajectory
        point1 = JointTrajectoryPoint()
        # The initial positions are populated with the joint states listened in the listener
        point1.positions = [joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]]
        
        # The final positions are populated with the angles written as argument of the script
        point2 = JointTrajectoryPoint()
        # The position will be reached in 5 secondes
        point2.time_from_start = Duration(seconds=5, nanoseconds=0).to_msg()
        point2.positions = [float(angle[0]), float(angle[1]), float(angle[2]), float(angle[3]), float(angle[4]), float(angle[5])]

        points.append(point1)
        points.append(point2)

        goal_msg.goal_time_tolerance = Duration(seconds=1, nanoseconds=0).to_msg()
        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.points = points

        self._action_client.wait_for_server()
        
        return  self._action_client.send_goal_async(goal_msg)
    
    # Definition of the listener
    def listener_callback(self, msg):
        # The joints variable are defined as global variable in order to be user in the send_goal function above
        global joints 
        joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # For each joints the value is stored in the joints[i] table
        for i,name in enumerate(msg.name):
            if name == "joint_1":
               joints[0] = msg.position[i]
            elif name == "joint_2":
               joints[1] = msg.position[i]
            elif name == "joint_3":
               joints[2] = msg.position[i]
            elif name == "joint_4":
               joints[3] = msg.position[i]
            elif name == "joint_5":
               joints[4] = msg.position[i]
            elif name == "joint_6":
               joints[5] = msg.position[i]
        return

def main(args=None):
    
    rclpy.init()

    action_client = CommandRobotActionClient()

    # Split and store command line argument
    angle = sys.argv[1].split(',')
    
    rclpy.spin_once(action_client, timeout_sec=0.5)
    
    future = action_client.send_goal(angle)

    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()

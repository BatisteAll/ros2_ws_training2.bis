#! /usr/bin/env python3.10

import numpy as np
import rclpy
import os
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from interactive_markers import InteractiveMarkerServer
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from ament_index_python.packages import get_package_share_directory
from trajectory_msgs.msg import JointTrajectoryPoint


class sl_robot_isaac_ros2(Node):
    """Class to simulate Space Lab Robot in Isaac Sim"""

    def __init__(self):
        """Initialize the SpaceLab robot Isaac Sim simulation"""

        node_name = "sl_robot_isaac_ros2"
        super().__init__(node_name)

        ###############################
        # TF LISTENER
        ###############################
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        ###############################
        # SL_ROBOT ACTION CLIENT
        ###############################
        self._action_client = ActionClient(self,
                                           FollowJointTrajectory,
                                           'sl_robot/follow_joint_trajectory')

        ###############################
        # GRIPPER ACTION CLIENT
        ###############################
        self._gripper_action_client =\
            ActionClient(self,
                         FollowJointTrajectory,
                         'sl_robot_gripper/follow_joint_trajectory')
        



def main(args=None):
    """Main function to run the sl_robot Isaac Sim simulation."""
    rclpy.init(args=args)

    ros2_publisher = sl_robot_isaac_ros2()
    rclpy.spin(ros2_publisher)

    ros2_publisher.destroy_node()
    rclpy.shutdown()







if __name__ == "__main__":
    main()

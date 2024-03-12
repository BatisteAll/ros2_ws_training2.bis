#! /usr/bin/env python3.10

"""
##################  Spacelab Robot - Msg Converter Node   ##################

Description
----------
Convert /joint_trajectory_controller/controller_state to sensor_msgs/msg/JointState
https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

Parameters
----------
goal_prim : arg string
    define the targeted prim to be controlled
    {'robot', 'gripper'}


"""

########################
#        IMPORTS       #
########################
import rclpy 
from rclpy.node import Node
# Publisher / Subscriber messages types
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectoryPoint
import time

########################
#   CONST DEFINITION   #
########################
PKG_NAME = "isaacsim_rviz_training"
NODE_NAME = "controlMsg2jointMsg_pubSub"
ROBOT_NAME = "Spacelab Robot"



class controlMsg2jointMsg_pubSub(Node):

    def __init__(self):
        super().__init__('pubSubConverter')


        ########################
        #       PUBLISHER      #
        ########################
        self.publisher_ = self.create_publisher(
            JointState,             # Msg type
            '/joint_commands',      # Topic
            10                      # QoS
        )
        # timer_period = 0.5  # seconds
        # self.writer_timer = self.create_timer(timer_period, self.listener2writer_callback)
        # self.pub_counter = 0 # counter starting at publisher init
     
        ########################
        #      SUBSCRIBER      #
        ########################
        # Define a subscriber subscribing to control_msgs/msg/JointTrajectoryControllerState message on the /joint_trajectory_controller/controller_state topic
        self.subscriber_ = self.create_subscription(
            JointTrajectoryControllerState,                      # Msg type
            '/joint_trajectory_controller/controller_state',     # Topic
            self.listener2writer_callback,                       # Function to call at reading
            10                                                   # QoS
        )
        # self.subscriber_ # prevent unused variable warning


    ########################
    #  LISTENER CALLBACK   #
    ########################
    def listener2writer_callback(self, listened_msg):
        """
        Description
        -----------
        Callback called whenever something is published on the subscribed topic, then write what has been heard.
        self.subscriber_callback_method is a bound method - it already contains a reference to the scope that 
        it is bound to (saved in the __self__ variable). So it is never needed to provide the self argument, 
        regardless of where the method is called.
        """
        
        # Recover the waypoint from JointTrajectoryController messages
        # listened_msg type: control_msgs.msg._joint_trajectory_controller_state.JointTrajectoryControllerState
        reference = listened_msg.reference                  #type: trajectory_msgs.msg._joint_trajectory_point.JointTrajectoryPoint 
        pose_joint_angles = reference.positions[:]          #type: array.array
        pose_joint_velocities = reference.velocities[:]     #type: array.array
        pose_joint_effort = reference.effort[:]             #type: array.array
        joint_names = listened_msg.joint_names
        # reference=trajectory_msgs.msg.JointTrajectoryPoint(
        #     positions=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.7040477e-317], 
        #     velocities=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
        #     accelerations=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
        #     effort=[], 
        #     time_from_start=builtin_interfaces.msg.Duration(sec=0, nanosec=0))
        
        # self.get_logger().info(listened_msg)

        published_msg = JointState()
        published_msg._name = joint_names                   #string[]
        published_msg._position = pose_joint_angles         #float64[]
        published_msg._velocity = pose_joint_velocities     #float64[]
        published_msg._effort = pose_joint_effort           #float64[]

        self.publisher_.publish(published_msg)
        # https://robotics.stackexchange.com/questions/101565/dynamically-create-subcription-callback-functions-python
        # https://stackoverflow.com/questions/70511324/typeerror-missing-one-required-positional-argument-event
        # setattr(self, 'listener2writer_callback', self.listener2writer_callback.__get__(self, self.__class__))
    
def main(args=None):

    # Init ROS2
    rclpy.init(args=args)

    # Create an instance of the class object
    pubSubConverter = controlMsg2jointMsg_pubSub()

    # Spin the node
    rclpy.spin(pubSubConverter)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pubSubConverter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

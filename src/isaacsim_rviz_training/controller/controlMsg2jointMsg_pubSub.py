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

########################
#   CONST DEFINITION   #
########################
PKG_NAME = "isaacsim_rviz_training"
NODE_NAME = "command_slrobot"
ROBOT_NAME = "Spacelab Robot"



class controlMsg2jointMsg_pubSub(Node):

    def __init__(self):
        super().__init__(NODE_NAME)

        self.get_logger().info('Initialization of controlMsg2jointMsg_pubSub node.')

        ########################
        #       PUBLISHER      #
        ########################
        self.publisher_ = self.create_publisher(
            JointState,             # Msg type
            '/joint_commands',      # Topic
            10                      # QoS
        )
        timer_period = 0.5  # seconds
        self.writer_timer = self.create_timer(timer_period, self.listener2writer_callback)
        self.pub_counter = 0 # counter starting at publisher init
     
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
        self.subscriber_ # prevent unused variable warning


    ########################
    #  LISTENER CALLBACK   #
    ########################
    def listener2writer_callback(self, listened_msg):
        """
        Description
        -----------
        Callback called whenever something is published on the subscribed topic, then write what has been heard.
        """
        
        # Recover the waypoint from JointTrajectoryController messages
        pose_joint_angles = listened_msg.reference.positions
        print(type(pose_joint_angles))
        pose_joint_velocities = listened_msg.reference.velocities
        pose_joint_accelerations = listened_msg.reference.accelerations
        pose_joint_effort = listened_msg.reference.effort
        self.get_logger().info("------------------------------------------------------------------------------------")
        self.get_logger().info(listened_msg)
        
        published_msg = JointState()
        self.publisher_.publish(published_msg)


    
def main(args=None):
    rclpy.init(args=args)
    pubSubConverter = controlMsg2jointMsg_pubSub()
    rclpy.spin(pubSubConverter)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pubSubConverter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

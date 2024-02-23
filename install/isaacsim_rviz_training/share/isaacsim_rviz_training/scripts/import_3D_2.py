#! /usr/bin/env python3.10

import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker


class MinimalPublisher(Node):

    def __init__(self):

        # super can be used to refer to parent classes without naming them explicitly
        # here it allows to inheritate the methods of the original MinimalPublisher() class
        super().__init__('minimal_publisher')
        
        # Definition of a publisher that will publish a marker message in the topic /visualization_marker_2
        self.publisher_ = self.create_publisher(Marker, "/visualization_marker_2", 2)
        timer_period = 0.5  # every seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        marker = Marker()
        marker.header.frame_id = "/world"  # The marker will be placed with regard to /world frame

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = 10
        marker.id = 0
        marker.mesh_resource = "package://gazebo_rviz_training/models/meshes/RADICART_FULL_PARTS.dae";

        # Set the scale of the marker
        marker.scale.x = float(1.0)
        marker.scale.y = float(1.0)
        marker.scale.z = float(1.0)

        # Set the color
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 1.0

        # Set the pose of the marker
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = -0.7071
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 0.7071
        
        self.publisher_.publish(marker)


def main(args=None):

    rclpy.init()
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

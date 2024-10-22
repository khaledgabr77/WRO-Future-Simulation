#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

class OdomToTFPublisher(Node):
    def __init__(self):
        super().__init__('odom_to_tf_publisher')
        
        # Declare parameters for frame IDs
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        
        # Get parameters
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value
        
        # Create a subscriber to the /odom topic
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Create a TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def odom_callback(self, msg):
        # Create a TransformStamped message
        transform = TransformStamped()

        # Set the header (including timestamp and frame ID)
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.frame_id
        transform.child_frame_id = self.child_frame_id

        # Set the translation
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z

        # Set the rotation
        transform.transform.rotation = msg.pose.pose.orientation

        # Publish the transform
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    
    node = OdomToTFPublisher()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

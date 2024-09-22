#!/usr/bin/env python3
########################################################################################################
# Name:             odom_to_tf.py
# Purpose:          provides the dynamic transforms from Odometry Frame to Base_Footprint 
# Description:      needed to link the robot wheels to the Nav2 system
# Related Files:         
# Author:           logan naidoo, south africa, 2024
########################################################################################################

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class OdomToTFNode(Node):
    def __init__(self):
        super().__init__('odom_to_tf_node')

        # Subscribe to the odom topic
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        # Create a transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

    def odom_callback(self, msg):
        # Create a TransformStamped message
        t = TransformStamped()

        # Fill in the header and frame information
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # Fill in the transform (from odometry)
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # Use quaternion from odometry for orientation
        t.transform.rotation = msg.pose.pose.orientation

        # Send the transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomToTFNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

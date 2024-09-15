#!/usr/bin/env python3
########################################################################################################
# Name:             jupiter_joint_states.py
# Purpose:          provides joint_states update of wheel encoder and velocity data
# Description:      uses the data from wheel_encoder and wheel_speeds topic published by the ESP32
# Related Files:    see ESP32 firmware that publishes the above topics     
# Author:           logan naidoo, south africa, 2024
########################################################################################################

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray, Float32MultiArray

class WheelJointStatePublisher(Node):
    def __init__(self):
        super().__init__('wheel_joint_state_publisher')

        # Create a publisher for joint_states
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)

        # Define joint names
        self.joint_names = ['front_left_wheel_joint', 'front_right_wheel_joint', 'rear_left_wheel_joint', 'rear_right_wheel_joint']

        # Initialize joint states
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = self.joint_names
        self.joint_state_msg.position = [0.0] * 4  # Placeholder for the encoder positions
        self.joint_state_msg.velocity = [0.0] * 4  # Placeholder for the encoder velocities

        # Initialize variables to store encoder data
        self.encoder_counts = [0, 0, 0, 0]
        self.encoder_rpms = [0.0, 0.0, 0.0, 0.0]

        # Subscribe to /wheel_encoders and /wheel_speeds topics
        self.encoder_subscriber = self.create_subscription(Int32MultiArray, '/wheel_encoders', self.encoder_callback, 10)
        self.speed_subscriber = self.create_subscription(Float32MultiArray, '/wheel_speeds', self.speed_callback, 10)

        # Ticks per revolution of the encoders
        self.ticks_per_revolution = 1320  # Replace with your actual ticks per revolution

        # Timer to periodically publish joint states
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.publish_joint_states)

    def encoder_callback(self, msg):
        # Update encoder counts from the message data
        self.encoder_counts = msg.data

    def speed_callback(self, msg):
        # Update encoder RPMs from the message data
        self.encoder_rpms = msg.data

    def publish_joint_states(self):
        # Get current time
        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        # Convert encoder counts to radians
        encoder_positions = [
            (2 * 3.14159 * self.encoder_counts[0]) / self.ticks_per_revolution,
            (2 * 3.14159 * self.encoder_counts[1]) / self.ticks_per_revolution,
            (2 * 3.14159 * self.encoder_counts[2]) / self.ticks_per_revolution,
            (2 * 3.14159 * self.encoder_counts[3]) / self.ticks_per_revolution
        ]

        # Convert RPM to radians per second
        encoder_velocities = [
            self.encoder_rpms[0] * (2 * 3.14159 / 60),
            self.encoder_rpms[1] * (2 * 3.14159 / 60),
            self.encoder_rpms[2] * (2 * 3.14159 / 60),
            self.encoder_rpms[3] * (2 * 3.14159 / 60)
        ]

        # Update joint states
        self.joint_state_msg.position = encoder_positions
        self.joint_state_msg.velocity = encoder_velocities

        # Publish the joint state message
        self.joint_state_publisher.publish(self.joint_state_msg)

def main(args=None):
    rclpy.init(args=args)
    wheel_joint_state_publisher = WheelJointStatePublisher()
    rclpy.spin(wheel_joint_state_publisher)
    wheel_joint_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

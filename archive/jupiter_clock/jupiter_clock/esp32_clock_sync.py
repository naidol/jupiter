#!/usr/bin/env python3
########################################################################################################
# Name:             esp32_clock_sync.py
# Purpose:          to sync the ROS-2 host pc clock with the ESP-32 micro-controller 
# Description:      enables system time to be stable between nodes on the micro-controller and host PC
# Related Files:    subscribers are found on the micro-controller firmware    
# Author:           logan naidoo, south africa, 2024
########################################################################################################
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from rclpy.clock import Clock

class ClockPublisher(Node):
    def __init__(self):
        super().__init__('clock_node')
        self.publisher_ = self.create_publisher(Time, '/clock', 10)
        self.timer = self.create_timer(0.02, self.publish_clock)  # Publish at 50 Hz = 0.02 secs
        self.clock = Clock()

    def publish_clock(self):
        current_time = self.clock.now().to_msg()
        self.publisher_.publish(current_time)
        #self.get_logger().info(f'Published time: {current_time.sec}.{current_time.nanosec}')

def main(args=None):
    rclpy.init(args=args)
    clock_publisher = ClockPublisher()
    rclpy.spin(clock_publisher)
    clock_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import time

class CameraStream(Node):

    def __init__(self):
        super().__init__('camera_node')
        self.get_logger().info("Camera node started.")
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.timer_ = self.create_timer(0.04, self.capture_image) # 25 FPS = 0.04
        self.bridge = CvBridge()

        # FPS timer variables
        self.start_time = time.time()
        self.num_frames = 0
        self.fps = 0

    def capture_image(self):
        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()
        cap.release()
        if ret:
            # Display FPS
            self.num_frames += 1
            elapsed_time = time.time() - self.start_time
            if elapsed_time >= 1:
                self.fps = self.num_frames / elapsed_time
                self.start_time = time.time()
                self.num_frames = 0
            cv2.putText(frame, f"FPS: {int(self.fps)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)
            #convert to from CV2 frame to ROS msg
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraStream()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

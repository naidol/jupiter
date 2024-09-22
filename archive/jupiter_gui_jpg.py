#!/usr/bin/env python3
########################################################################################################
# Name:             jupiter_gui.py
# Purpose:          GUI module for Jupiter Robot
# Description:      Module displays four windows on main display. Each window display various data.
#                   The GUI primarily uses TKinter which is part of the python system.
# Related Files:    
# Author:           logan naidoo, south africa, 2024
########################################################################################################
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from tkinter import Tk, Label, Frame
from PIL import Image as PILImage, ImageTk

class JupiterGuiNode(Node):
    def __init__(self, topics, speaking_image_path):
        super().__init__('gui_node')

        # Initialize Tkinter main window with fixed size of 1024x600
        self.root = Tk()
        self.root.title("Jupiter Robot")
        self.root.geometry("1024x600")

        # Set up frames with fixed dimensions and borders
        frame_width = 512
        frame_height = 300
        self.frames = []
        for i in range(2):
            for j in range(2):
                frame = Frame(self.root, borderwidth=5, relief="solid", width=frame_width, height=frame_height)
                frame.grid(row=i, column=j, padx=10, pady=10)
                frame.grid_propagate(False)
                frame.pack_propagate(False)
                self.frames.append(frame)

        # Track which topics are being subscribed to (max 4)
        assert len(topics) <= 4, "GUI supports a maximum of 4 topics."

        # Initialize subscribers for ROS topics
        self.labels = []
        self.speaking_image_active = False
        self.bridge = CvBridge()

        # Create a dictionary to store image and esp_led labels
        self.speaking_image_label = None
        self.esp_led_label = None

        for idx, topic in enumerate(topics):
            if topic == "/image_raw":
                self.image_label = Label(self.frames[idx], text="Waiting for image...", font=("Arial", 12), bg="gray")
                self.image_label.pack(expand=True, fill='both')
                self.create_subscription(Image, topic, self.create_image_callback(self.image_label), 10)
            elif topic == "/voice_tts":
                self.speaking_image_label = Label(self.frames[idx], text="Waiting for voice command...", font=("Arial", 12), bg="gray")
                self.speaking_image_label.pack(expand=True, fill='both')
                self.speaking_image_path = speaking_image_path  # Path to static jpg image
                self.create_subscription(String, topic, self.create_speaking_image_callback(self.speaking_image_label), 10)
            elif topic == "/esp_led":
                self.esp_led_label = Label(self.frames[idx], text="Waiting for ESP LED command...", font=("Arial", 12), bg="gray")
                self.esp_led_label.pack(expand=True, fill='both')
                self.create_subscription(String, topic, self.create_esp_led_callback(), 10)
            else:
                label = Label(self.frames[idx], text="Waiting for data...", font=("Arial", 12), wraplength=480, bg="gray")
                label.pack(expand=True, fill='both', padx=10, pady=10)
                self.labels.append(label)
                self.create_subscription(String, topic, self.create_text_listener_callback(label), 10)

    def create_text_listener_callback(self, label):
        def listener_callback(msg):
            label.config(text=str(msg.data))
        return listener_callback

    def create_image_callback(self, image_label):
        def image_callback(msg):
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                cv_image = cv2.resize(cv_image, (512, 300))
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                pil_image = PILImage.fromarray(cv_image)
                tk_image = ImageTk.PhotoImage(image=pil_image)
                image_label.config(image=tk_image, text="")
                image_label.image = tk_image
            except Exception as e:
                self.get_logger().error(f"Error processing image: {e}")
        return image_callback

    def create_speaking_image_callback(self, speaking_image_label):
        def speaking_image_callback(msg):
            if not self.speaking_image_active:
                try:
                    speaking_image = PILImage.open(self.speaking_image_path)
                    speaking_image = speaking_image.resize((512, 300))  # Resize image to fit the frame
                    tk_image = ImageTk.PhotoImage(speaking_image)
                    speaking_image_label.config(image=tk_image, text="")
                    speaking_image_label.image = tk_image
                    self.speaking_image_active = True
                    self.get_logger().info("Speaking image displayed.")
                except Exception as e:
                    self.get_logger().error(f"Error displaying speaking image: {e}")
        return speaking_image_callback

    def create_esp_led_callback(self):
        def esp_led_callback(msg):
            self.get_logger().info(f"Received message on /esp_led: {msg.data}")

            if msg.data == "listen":
                self.get_logger().info(f"Speaking image active status: {self.speaking_image_active}")

                if self.speaking_image_label:
                    self.speaking_image_label.config(image="", text="Listening for command...")  # Ensure the image is removed
                    self.speaking_image_label.image = None  # Reset the image attribute
                    self.get_logger().info("Speaking image removed from display.")

                self.speaking_image_active = False  # Stop showing the speaking image
            else:
                self.get_logger().info(f"Message not 'listen': {msg.data}")

            if self.esp_led_label:
                self.esp_led_label.config(text="Listening for command..." if msg.data == "listen" else "Waiting for ESP LED command...")
        return esp_led_callback

    def update_gui(self):
        rclpy.spin_once(self, timeout_sec=0.1)
        self.root.after(10, self.update_gui)

    def run_gui(self):
        self.update_gui()
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    topics = ["/topic1", "/esp_led", "/voice_tts", "/image_raw"]
    speaking_image_path = "/home/logan/Downloads/jupiter_talk.jpg"  # Static jpg image path
    node = JupiterGuiNode(topics, speaking_image_path)

    try:
        node.run_gui()
    except KeyboardInterrupt:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

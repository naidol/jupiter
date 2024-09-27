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
from PIL import Image as PILImage, ImageTk, ImageDraw, ImageFont
import os

class JupiterGuiNode(Node):
    def __init__(self, topics):
        super().__init__('gui_node')

        # Initialize Tkinter main window with fixed size of 1024x600
        self.root = Tk()
        self.root.title("Jupiter Robot")
        self.root.geometry("1024x600")      #was 1024x600

        # Set up 4 frames (2x2) with fixed dimensions and borders
        frame_width = 500       #was 512
        frame_height = 260      #was 300
        self.frames = []
        for i in range(2):
            for j in range(2):
                frame = Frame(self.root, borderwidth=2, relief="solid", width=frame_width, height=frame_height)
                frame.grid(row=i, column=j, padx=5, pady=5)  # was padx 10 pady 10
                frame.grid_propagate(False)
                frame.pack_propagate(False)
                self.frames.append(frame)

        # Track which topics are being subscribed to (max 4)
        assert len(topics) <= 4, "GUI supports a maximum of 4 topics."

        # Initialize subscribers for ROS topics
        self.labels = []
        self.bridge = CvBridge()

        # Create a dictionary to store image and esp_led labels
        self.speaking_image_label = None
        self.esp_led_label = None
        self.speaking_image_path = self.get_image_path("jupiter_face.jpg")
        self.listening_image_path = self.get_image_path("jupiter_listen.jpg")
        self.get_logger().info(f"speaking_image_path: {self.speaking_image_path}")

        for idx, topic in enumerate(topics):
            if topic == "/image_raw":
                self.camera_image_label = Label(self.frames[idx], text="Waiting for image...", font=("Arial", 12), bg="gray")
                self.camera_image_label.pack(expand=True, fill='both')
                self.create_subscription(Image, topic, self.create_camera_image_callback(self.camera_image_label), 10)
            elif topic == "/voice_tts":
                self.speaking_image_label = Label(self.frames[idx], text="Waiting for voice_tts command...", font=("Arial", 12), bg="gray")
                self.speaking_image_label.pack(expand=True, fill='both')
                self.create_subscription(String, topic, self.create_speaking_image_callback(self.speaking_image_label), 10)
                self.create_subscription(String, "/esp_led", self.create_esp_led_callback(), 10)
            elif topic == "/esp_led":
                pass
            else:
                label = Label(self.frames[idx], text="Waiting for data...", font=("Arial", 12), wraplength=480, bg="gray")
                label.pack(expand=True, fill='both', padx=10, pady=10)
                self.labels.append(label)
                self.create_subscription(String, topic, self.create_text_listener_callback(label), 10)

    def create_text_listener_callback(self, label):
        def listener_callback(msg):
            label.config(text=str(msg.data))
        return listener_callback

    def create_camera_image_callback(self, image_label):
        def camera_image_callback(msg):
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                cv_image = cv2.resize(cv_image, (500, 260))         #was 512 , 300
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                pil_image = PILImage.fromarray(cv_image)
                tk_image = ImageTk.PhotoImage(image=pil_image)
                image_label.config(image=tk_image, text="")
                image_label.image = tk_image
            except Exception as e:
                self.get_logger().error(f"Error processing image: {e}")
        return camera_image_callback

    def create_speaking_image_callback(self, speaking_image_label):
        def speaking_image_callback(msg):           # msg = /voice_tts msg.data
            try:
                # Open the JPG image
                speaking_image = PILImage.open(self.speaking_image_path)
                speaking_image = speaking_image.resize((500, 260))  # was 512 x 300 Resize image to fit the frame

                # Overlay the received text on the image
                draw = ImageDraw.Draw(speaking_image)
                font = ImageFont.load_default()  # Use default font
                # font = ImageFont.load("arial.pil")  # Use custom font
                text_position = (10, 230)  #  was 10, 260 ----- Position the text at the bottom of the image
                text_color = (255, 255, 255)  # White text
                draw.text(text_position, msg.data, font=font, fill=text_color)   # voice_tts contains the msg.data to print

                # Convert the PIL image with text to a Tkinter image
                tk_image = ImageTk.PhotoImage(speaking_image)
                speaking_image_label.config(image=tk_image, text="")        #was text=""
                speaking_image_label.image = tk_image

                # delay(3)
                self.get_logger().info(f"Speaking image with text displayed: {msg.data}")

            except Exception as e:
                self.get_logger().error(f"Error displaying speaking image: {e}")

        return speaking_image_callback

    def create_esp_led_callback(self):
        def esp_led_callback(msg):
            if msg.data == "listen":
                try:
                    self.speaking_image_label.config(image="", text="Listening for command...")  # Ensure the image is removed
                    self.speaking_image_label.image = None  # Reset the image attribute
                    self.get_logger().info("Speaking image removed from display.")   
                except Exception as e:
                    self.get_logger().error(f"Cannot remove speaking image: {e}")

            elif msg.data == "recognize":
                try:
                    self.speaking_image_label.config(image="", text="Please Wait ... recognizing command")  # Ensure the image is removed
                    self.speaking_image_label.image = None  # Reset the image attribute
                    self.get_logger().info("Speaking image removed from display.")   
                except Exception as e:
                    self.get_logger().error(f"Cannot remove speaking image: {e}")

            if self.esp_led_label:
                self.esp_led_label.config(text="Listening for command..." if msg.data == "listen" else "Waiting for ESP LED command...")
                
        return esp_led_callback

    # just get the os path to the package & folder containing the location of specified image
    def get_image_path(self,file_name):
        package_name = "jupiter_gui"  # Replace with your package name
        folder_name = "images"
        home_directory = os.path.expanduser("~")
        ros2_workspace_directory = None
        for root, dirs, files in os.walk(home_directory):
            if "install" in dirs and "src" in dirs:
                ros2_workspace_directory = os.path.join(root, "src/jupiter")
                break
        if ros2_workspace_directory is None:
            raise RuntimeError("Failed to find ROS2 workspace directory")
        package_directory = os.path.join(ros2_workspace_directory, package_name)
        return os.path.join(package_directory, folder_name, file_name)

    def update_gui(self):
        rclpy.spin_once(self, timeout_sec=0.1)
        self.root.after(10, self.update_gui)

    def run_gui(self):
        self.update_gui()
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    topics = ["/topic1", "/esp_led", "/voice_tts", "/image_raw"]
    node = JupiterGuiNode(topics)
    try:
        node.run_gui()
    except KeyboardInterrupt:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

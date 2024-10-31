#!/usr/bin/env python3
########################################################################################################
# Name:             cam_face_detect.py
# Purpose:          reads the cam stream and detects faces using the face_recognition package
# Description:      take new face pics and add to the /jupiter_camera/know_faces folder
# Related Files:    located at /jupiter_camera/known_faces      
# Author:           logan naidoo, south africa, 2024
########################################################################################################
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import face_recognition
import time
from std_msgs.msg import String
import os

class FaceRecognitionNode(Node):

    def __init__(self):
        super().__init__("face_recognition_node")
        self.get_logger().info("Face recognition node started.")

        # Load known face encodings
        self.known_face_encodings = []
        self.known_face_names = []
        
        # To store unknown face data for later use
        self.unknown_face_encoding = None
        self.unknown_face_image = None  # Store the frame with the unknown face
        self.unknown_face_location = None  # Store the location of the unknown face
        self.last_prompt_time = None  # Store the time of the last request for a name
        self.name_request_interval = 120  # 120 seconds cooldown for requesting unknown user's name

        # Get the path to the known_faces folder
        known_faces_dir = self.get_known_faces_path("")
        self.load_known_faces(known_faces_dir)

        self.bridge = CvBridge()

        # Initialize FPS timer
        self.start_time = time.time()
        self.num_frames = 0
        self.fps = 0

        # Face recognition timer
        self.remember_face_timer = time.time()
        self.faces_list = []

        # Create subscriber and publisher
        self.image_sub = self.create_subscription(Image, "/image_raw", self.image_callback, 10)
        self.image_pub = self.create_publisher(Image, "face_recognition/output", 10)
        self.tts_publisher = self.create_publisher(String, "/voice_tts", 10)
        self.user_id_publisher = self.create_publisher(String, "/user_id", 10)
        
        # Subscribe to the /new_user_id topic to get the new user's name
        self.new_user_sub = self.create_subscription(String, "/new_user_id", self.new_user_id_callback, 10)
        self.new_user_name = None

    def load_known_faces(self, known_faces_dir):
        """Load all the known faces from the known_faces directory."""
        for root, dirs, files in os.walk(known_faces_dir):
            for file in files:
                if file.endswith((".jpg", ".jpeg", ".png")):
                    name = os.path.splitext(file)[0]
                    image_path = os.path.join(root, file)
                    image = face_recognition.load_image_file(image_path)
                    encodings = face_recognition.face_encodings(image)
                    if encodings:
                        encoding = encodings[0]
                        self.known_face_encodings.append(encoding)
                        self.known_face_names.append(name)
                        self.get_logger().info(f"Loaded face encoding for {name}")

    def image_callback(self, msg):
        # Convert ROS message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        
        # Find faces in frame
        face_locations = face_recognition.face_locations(frame)
        face_encodings = face_recognition.face_encodings(frame, face_locations)
        
        # Recognize faces
        face_names = []
        for face_encoding, face_location in zip(face_encodings, face_locations):
            # Compare face encoding with known face encodings
            matches = face_recognition.compare_faces(self.known_face_encodings, face_encoding)
            name = "human"  # Default to "human" for unknown faces
            
            # Find best match
            face_distances = face_recognition.face_distance(self.known_face_encodings, face_encoding)
            best_match_index = face_distances.argmin() if len(face_distances) > 0 else None
            
            if best_match_index is not None and matches[best_match_index]:
                name = self.known_face_names[best_match_index]
            else:
                # Unknown face detected, request the name via greet_face
                self.handle_unknown_face(face_encoding, frame, face_location)
            
            face_names.append(name)
            self.publish_user_id(name)

            current_time = time.time()
            if self.last_prompt_time is None or (current_time - self.last_prompt_time > self.name_request_interval):
                self.greet_face(name)
                self.last_prompt_time = current_time  # Update the last prompt time
                  
        # Draw rectangles and names on frame
        for (top, right, bottom, left), name in zip(face_locations, face_names):
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
            cv2.putText(frame, name, (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)
        
        # Publish output
        output_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.image_pub.publish(output_msg)

    def handle_unknown_face(self, face_encoding, frame, face_location):
        """Handle the case when an unknown face is detected, ensuring a 60-second delay between prompts."""       
        self.unknown_face_encoding = face_encoding
        self.unknown_face_image = frame  # Save the frame with the unknown face
        self.unknown_face_location = face_location  # Save the location of the unknown face
        # self.greet_face("human")  # Ask for their name
            

    def greet_face(self, face_name):
        if face_name == "human":
            self.send_voice_tts("Hello! I don't know your name. Please tell me your name.")
        elif face_name in self.known_face_names:
            self.send_voice_tts(f"Welcome back {face_name}!")

    def send_voice_tts(self, text):
        tts_msg = String()
        tts_msg.data = text
        self.tts_publisher.publish(tts_msg)

    def publish_user_id(self, name):
        user_id_msg = String()
        user_id_msg.data = name
        self.user_id_publisher.publish(user_id_msg)

    def new_user_id_callback(self, msg):
        """Callback to receive new user ID from the /new_user_id topic."""
        self.new_user_name = msg.data
        
        # Store the new user's name and face encoding if we received a name
        if self.unknown_face_encoding is not None and self.new_user_name:
            self.known_face_encodings.append(self.unknown_face_encoding)
            self.known_face_names.append(self.new_user_name)

            # Save the face encoding and name into the known_faces folder
            self.save_new_user(self.unknown_face_image, self.unknown_face_location, self.new_user_name)
            self.get_logger().info(f"Stored new user: {self.new_user_name}")

            # Reset the unknown face encoding
            self.unknown_face_encoding = None
            self.unknown_face_image = None
            self.unknown_face_location = None
            self.new_user_name = None

    def save_new_user(self, frame, face_location, user_name):
        """Save the new user's face as a .jpg image in the known_faces folder."""
        known_faces_dir = self.get_known_faces_path("")
        new_face_image_path = os.path.join(known_faces_dir, f"{user_name}.jpg")
        
        # Crop the face from the frame based on face_location (top, right, bottom, left)
        top, right, bottom, left = face_location
        #face_image = frame[top:bottom, left:right]  # Crop the face
        face_image = frame  # Do not crop the image

        # Save the cropped face as a .jpg file
        cv2.imwrite(new_face_image_path, face_image)
        self.get_logger().info(f"Saved new face image for {user_name} at {new_face_image_path}")

    def get_known_faces_path(self, file_name):
        package_name = "jupiter_camera"
        folder_name = "known_faces"
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


def main(args=None):
    rclpy.init(args=args)
    node = FaceRecognitionNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
########################################################################################################
# Name:             cam_face_detect.py
# Purpose:          reads the cam stream and detects faces using the face_recognition package
# Description:      take new face pics and add to the /jupiter_camera/know_faces folder
# Related Files:    located at /jupiter_camera/known_faces      
# Author:           logan naidoo, south africa, 2024
########################################################################################################
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

        # Get the path to the known faces folder
        known_faces_dir = self.get_known_faces_path("")  # Get the directory for known faces

        # Iterate through the files in the known_faces folder
        for root, dirs, files in os.walk(known_faces_dir):
            for file in files:
                if file.endswith((".jpg", ".jpeg", ".png")):  # Process image files only
                    name = os.path.splitext(file)[0]  # Extract the name from the file name (without extension)
                    image_path = os.path.join(root, file)

                    # Load the image and encode the face
                    image = face_recognition.load_image_file(image_path)
                    encodings = face_recognition.face_encodings(image)

                    if encodings:  # Ensure at least one encoding is found
                        encoding = encodings[0]
                        self.known_face_encodings.append(encoding)
                        self.known_face_names.append(name)
                        self.get_logger().info(f"Loaded face encoding for {name}")
       
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
        self.user_id_publisher = self.create_publisher(String, "/user_id", 10)  # New publisher

    def image_callback(self, msg):
        # Convert ROS message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        
        # Find faces in frame
        face_locations = face_recognition.face_locations(frame)
        face_encodings = face_recognition.face_encodings(frame, face_locations)
        
        # Recognize faces
        face_names = []
        for face_encoding in face_encodings:
            # Compare face encoding with known face encodings
            matches = face_recognition.compare_faces(self.known_face_encodings, face_encoding)
            name = "Human"
            
            # Find best match
            face_distances = face_recognition.face_distance(self.known_face_encodings, face_encoding)
            best_match_index = face_distances.argmin()
            if matches[best_match_index]:
                name = self.known_face_names[best_match_index]
            face_names.append(name)
            
            # Publish recognized user ID
            self.publish_user_id(name)  # Publish the recognized name
            
            # Greet Face using the voice TTS
            self.greet_face(name)
        
        # Draw rectangles and names on frame
        for (top, right, bottom, left), name in zip(face_locations, face_names):
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
            cv2.putText(frame, name, (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)

        # Display FPS
        self.num_frames += 1
        elapsed_time = time.time() - self.start_time
        if elapsed_time >= 1:
            self.fps = self.num_frames / elapsed_time
            self.start_time = time.time()
            self.num_frames = 0
        cv2.putText(frame, f"FPS: {int(self.fps)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)
        
        # Publish output
        output_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.image_pub.publish(output_msg)

    def greet_face(self, face):
        if face in self.faces_list:
            elapsed_time = time.time() - self.remember_face_timer
            if elapsed_time > 300.0:
                self.send_voice_tts('Welcome back ' + face)
                self.remember_face_timer = time.time()
        else:
            self.faces_list.append(face)
            self.send_voice_tts('Hello ' + face)

    def send_voice_tts(self, text):
        tts_msg = String()
        tts_msg.data = text
        self.tts_publisher.publish(tts_msg)

    # Publish recognized user ID
    def publish_user_id(self, name):
        user_id_msg = String()
        user_id_msg.data = name
        self.user_id_publisher.publish(user_id_msg)  # Publishes the name of the recognized face

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

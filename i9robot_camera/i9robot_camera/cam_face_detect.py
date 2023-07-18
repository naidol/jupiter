#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import face_recognition #pip3 install face-recognition
import time
from std_msgs.msg import String
import os
#from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory

class FaceRecognitionNode(Node):

    def __init__(self):
        super().__init__("face_recognition_node")
        #self.counter_ = 0
        self.get_logger().info("Face recognition node started.")
        #self.create_timer(0.5, self.image_callback(self.msg))
        
        # Load known face encodings
        self.known_face_encodings = []
        self.known_face_names = []

        # Replace with your own known face images and names
        image1_path = self.get_known_faces_path("Logan/logan.jpg")
        self.get_logger().info(image1_path)
        image1 = face_recognition.load_image_file(image1_path)
        #image1 = face_recognition.load_image_file("/home/logan/i9robot_ws/src/i9robot_camera/known_faces/Logan/logan.jpg")
        encoding1 = face_recognition.face_encodings(image1)[0]
        self.known_face_encodings.append(encoding1)
        self.known_face_names.append("Logan")
        image2_path = self.get_known_faces_path("Indrani/indrani.jpg")
        image2 = face_recognition.load_image_file(image2_path)
        #image2 = face_recognition.load_image_file("/home/logan/i9robot_ws/src/i9robot_camera/known_faces/Indrani/indrani.jpg")
        encoding2 = face_recognition.face_encodings(image2)[0]
        self.known_face_encodings.append(encoding2)
        self.known_face_names.append("Indrani")
        
        # Initialize camera
        # self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge()
        
        # FPS timer variables
        self.start_time = time.time()
        self.num_frames = 0
        self.fps = 0

        # Initialise face_detect timer memory
        self.remember_face_timer = time.time()
        self.faces_list=[]
        
        # Create subscriber and publisher
        self.image_sub = self.create_subscription(Image,"camera/image_raw",self.image_callback,10)
        self.image_pub = self.create_publisher(Image,"face_recognition/output",10)
        self.publisher = self.create_publisher(String, "/voice_tts", 10)

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
            name = "Unknown"
            
            # Find best match
            face_distances = face_recognition.face_distance(self.known_face_encodings, face_encoding)
            best_match_index = face_distances.argmin()
            if matches[best_match_index]:
                name = self.known_face_names[best_match_index]
            face_names.append(name)
        
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
        #cv2.imshow("Face Recognition", frame)

    # send the message to convert text to voice via the /voice_tts topic
    def send_voice_tts(self, text):
        tts_msg = String()
        tts_msg.data = text
        self.publisher.publish(tts_msg)
        #self.get_logger().info(text)

    # once the face is recognised, then output a voice msg to greet the face
    def greet_face(self, face):
            if face in self.faces_list:
                elapsed_time = time.time() - self.remember_face_timer
                if elapsed_time > 300.0:         #adjust time so that faces are only greeted again after the set elapsed time
                    self.send_voice_tts('Welcome back ' + face)
                    #self.faces_list.remove(face)
                    self.remember_face_timer = time.time()
            else:
                self.faces_list.append(face)
                self.send_voice_tts('Hello ' + face)
    
    # just get the os path to the package containing the database of known faces
    def get_known_faces_path(self,file_name):
        package_name = "i9robot_camera"  # Replace with your package name
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
    node = FaceRecognitionNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="i9robot_camera",
            executable="face_detect",
            name="face_recognition_node",
            output="screen"
        )
    ])

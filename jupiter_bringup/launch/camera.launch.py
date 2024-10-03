########################################################################################################
# Name:             camera.launch.py
# Purpose:          USB Web-Cam launch file
# Description:      Jupiter robot vision for face detection
# Related Files:    /jupiter_camera folder contains the camera control programs
# Author:           logan naidoo, south africa, 2024
########################################################################################################

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    return LaunchDescription([

        # LAUNCH THE USB CAMERA SENSOR USING THE ROS2 PACKAGE 'usb_cam' WHICH PUBLISHES /image_raw TOPIC
        Node(
            name='camera_node',
            package='usb_cam',
            executable='usb_cam_node_exe',
            output='screen',
            # namespace='camera',
            parameters=[
                {'image_size': [640,480]}, # was 640,480
                {'time_per_frame': [1, 6]},
                {'camera_frame_id': 'camera_link'},
                {'video_device': '/dev/video0'} # was removed
            ]
        ),
    
        # LAUNCH THE FACE DETECTION NODE
        Node(
            name='camera_face_detect_node',
            package='jupiter_camera',
            executable='cam_face_detect'
            )
    ])
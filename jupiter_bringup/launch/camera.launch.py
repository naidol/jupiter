from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    camera_raw_node = Node(
            package='jupiter_camera',
            executable='cam_raw_stream'
         )
    
    camera_face_detect_node = Node(
            package='jupiter_camera',
            executable='cam_face_detect'
         )
    
    return LaunchDescription([
        camera_raw_node,
        camera_face_detect_node
    ])
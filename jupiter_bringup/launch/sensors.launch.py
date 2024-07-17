
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
        
    return LaunchDescription([
        
        # LAUNCH THE LD-LIDAR SENSOR 
        Node(
            name='ldlidar_node',
            package='ldlidar_stl_ros2',
            executable='ldlidar_stl_ros2_node',
            output='screen',
            parameters=[
                {'product_name': 'LDLiDAR_LD19'},
                {'topic_name': 'scan'},
                {'frame_id': 'base_laser'},
                {'port_name': '/dev/ttyUSB1'},  # LDLIDAR connected to USB1
                {'port_baudrate': 230400},
                {'laser_scan_dir': True},
                {'enable_angle_crop_func': False},
                {'angle_crop_min': 135.0},
                {'angle_crop_max': 225.0}
            ]
        ),

        # LAUNCH THE PS2 CAMERA SENSOR 
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            # output='screen',
            # namespace='camera',
            # parameters=[{
            #     'image_size': [640,480],
            #     'time_per_frame': [1, 6],
            #     'camera_frame_id': 'camera_link_optical'
            #     }]
        ),
    ])


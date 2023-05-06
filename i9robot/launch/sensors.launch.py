
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
        
    
    # DeclareLaunchArgument(
    #         name='sensor', 
    #         default_value='rplidar',
    #         description='Sensor to launch'
    #     ),
    # DeclareLaunchArgument(
    #         name='frame_id', 
    #         default_value='laser',
    #         description='Laser Frame ID'
    #     ),

    return LaunchDescription([
        
        # LAUNCH THE RP-LIDAR SENSOR 
        Node(
            #condition=LaunchConfigurationEquals('sensor', 'rplidar'),
            name='rplidar_composition',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            #remappings=[('scan', LaunchConfiguration('topic_name'))],
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,  # A1 / A2
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }],
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


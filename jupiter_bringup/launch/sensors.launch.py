########################################################################################################
# Name:             sensors.launch.py
# Purpose:          start up all attached sensors (e.g. lidar, imu, camera)
# Description:      purpose build for LD-Lidar | Bosch INO055 | USB camera
# Related Files:    please note the IMU code is found with the ESP-32 firmware software
# Author:           logan naidoo, south africa, 2024
########################################################################################################
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
                {'frame_id': 'laser_link'},      # Set the frame_id to the robot frame base_link - original = base_laser
                {'port_name': '/dev/ttyUSB1'},  # LDLIDAR connected to USB1
                {'port_baudrate': 230400},      # Set the optimum baud - original = 230400
                {'laser_scan_dir': True},
                {'enable_angle_crop_func': False},
                {'angle_crop_min': 135.0},
                {'angle_crop_max': 225.0}
            ]
        ),

        # # base_link to base_laser tf node
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_link_to_base_laser_ld19',
        #     arguments=['0.060','0.000','0.145','0','0','0','base_link','laser_link']
        # ),

        # LAUNCH THE USB CAMERA SENSOR 
        Node(
            name='camera_node',
            package='usb_cam',
            executable='usb_cam_node_exe',
            output='screen',
            # namespace='camera',
            parameters=[
                {'image_size': [640,480]},
                {'time_per_frame': [1, 6]},
                {'camera_frame_id': 'camera_link'}
            ]
        )
    ])


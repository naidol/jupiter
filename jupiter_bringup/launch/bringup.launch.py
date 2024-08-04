########################################################################################################
# Name:             bringup.launch.py
# Purpose:          main robot bringup launch - used to startup the jupiter robot
# Description:      ros2 humble "bringup" to be used with esp32 firmware and micro-ros
# Related Files:    located at /jupiter_bringup/launch     
# Author:           logan naidoo, south africa, 2024
########################################################################################################

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare('jupiter'), 'config', 'ekf.yaml']
    )
    microros_launch_path = PathJoinSubstitution(
        [FindPackageShare('jupiter_bringup'), 'launch', 'microros.launch.py']
    )
    sensors_launch_path = PathJoinSubstitution(
        [FindPackageShare('jupiter_bringup'), 'launch', 'sensors.launch.py']
    )
    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('jupiter_bringup'), 'launch', 'description.launch.py']
    )
    joystick_launch_path = PathJoinSubstitution(
        [FindPackageShare('jupiter_bringup'), 'launch', 'joystick.launch.py']
    )
    voice_launch_path = PathJoinSubstitution(
        [FindPackageShare('jupiter_bringup'), 'launch', 'voice.launch.py']
    )
    camera_launch_path = PathJoinSubstitution(
        [FindPackageShare('jupiter_bringup'), 'launch', 'camera.launch.py']
    )

    return LaunchDescription([
        
        DeclareLaunchArgument(
            name='base_serial_port', 
            default_value='/dev/ttyUSB0', # ESP32 micro-controller connected to USB0
            description='Base Serial Port'
        ),

        DeclareLaunchArgument(
            name='joy', 
            default_value='false',
            description='Use Joystick'
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_config_path
            ],
            remappings=[("odometry/filtered", "odom")]
        ),

        # LAUNCH MICRO-ROS AGENT TO COMMUNICATE WITH THE MICRO-CONTROLLER MOTOR DRIVER AND IMU HARDWARE
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(microros_launch_path),
            launch_arguments={
                'base_serial_port': LaunchConfiguration("base_serial_port")
            }.items()
        ),

        # LAUNCH THE ROBOT DESCRIPTION TO LOAD THE ROBOT URDF
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path)
        ),

        # LAUNCH THE SENSORS ATTACHED TO THE ROBOT (E.G. LIDAR, CAMERA, ETC.)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sensors_launch_path)
        ),

        # LAUNCH JOYSTICK, IF THE JOYSTICK IS ENABLED
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(joystick_launch_path),
            condition=IfCondition(LaunchConfiguration('joy')),
        ),

        # LAUNCH THE VOICE RECOGNITION SYSTEM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(voice_launch_path)
        ),
        # LAUNCH THE CAMERA FACE RECOGNITION SYSTEM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch_path)
        )
    ])

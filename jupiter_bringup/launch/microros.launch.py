########################################################################################################
# Name:             microros.launch.py
# Purpose:          launch the micro-ros agent to communicate with the micro-controller on USB0
# Description:      jupiter robot firmware running on the ESP-32 micro-controller will connect to this agent
# Related Files:    refer to the firmware (ESP-32) code built to use micro-ros
# Author:           logan naidoo, south africa, 2024
########################################################################################################
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    

    return LaunchDescription([
        DeclareLaunchArgument(
            name='base_serial_port', 
            default_value='/dev/ttyUSB0',  #USB port connected to ESP32 micro-controller
            description='Base Serial Port'
        ),
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=['serial', '--dev', LaunchConfiguration("base_serial_port")]
        )
        
    ])
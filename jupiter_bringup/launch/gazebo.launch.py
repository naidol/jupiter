########################################################################################################
# Name:             gazebo.launch.py
# Purpose:          launch the jupiter robot gazebo simulation
# Description:      Simulate the robot with gazebo sensors (imu, lidar, camera)
# Related Files:    look at all xacro files with the gazebo tags
# Author:           logan naidoo, south africa, 2024
########################################################################################################
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = True

    joy_launch_path = PathJoinSubstitution(
        [FindPackageShare('jupiter'), 'launch', 'joystick.launch.py']
    )

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("jupiter"), "config", "ekf.yaml"]
    )

    world_path = PathJoinSubstitution(
        [FindPackageShare("jupiter"), "worlds", "c82house_gz.world"]
    )

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('jupiter'), 'launch', 'description.launch.py']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='world', 
            default_value=world_path,
            description='Gazebo world'
        ),

        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so',  '-s', 'libgazebo_ros_init.so', LaunchConfiguration('world')],
            output='screen'
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=["-topic", "robot_description", "-entity", "jupiter"]
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time}, 
                ekf_config_path
            ],
            remappings=[("odometry/filtered", "odom")]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
            launch_arguments={
                'use_sim_time': str(use_sim_time),
                'publish_joints': 'false',      # was false
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(joy_launch_path),
        )
    ])

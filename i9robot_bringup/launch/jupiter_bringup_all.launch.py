from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare('i9robot'), 'config', 'ekf.yaml'])

    microros_launch_path = PathJoinSubstitution(
        [FindPackageShare('i9robot_bringup'), 'launch', 'microros.launch.py'])

    sensors_launch_path = PathJoinSubstitution(
        [FindPackageShare('i9robot'), 'launch', 'sensors.launch.py'])

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('i9robot'), 'launch', 'description.launch.py'])

    joystick_launch_path = PathJoinSubstitution(
        [FindPackageShare('i9robot'), 'launch', 'joystick.launch.py'])

    voice_recognition_node = Node(
            package='i9robot_voice',
            executable='voice_asr')
    
    voice_command_node = Node(
            package='i9robot_voice',
            executable='voice_cmd')
    
    voice_speech_node = Node(
            package='i9robot_voice',
            executable='voice_tts')

    camera_raw_node = Node(
            package='i9robot_camera',
            executable='cam_raw_stream')
    
    camera_face_detect_node = Node(
            package='i9robot_camera',
            executable='cam_face_detect')

    return LaunchDescription([
        
        DeclareLaunchArgument(
            name='base_serial_port', 
            default_value='/dev/ttyACM0',
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

        # LAUNCH THE VOICE RECOGNITION | COMMAND | TEXT-TO-SPEECH MODULES
        voice_recognition_node,
        voice_command_node,
        voice_speech_node,

        # LAUNCH THE CAMERA AND FACE-DETECTION MODULE
        # camera_raw_node,
        # camera_face_detect_node


    ])

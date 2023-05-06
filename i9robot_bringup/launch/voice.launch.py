from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    voice_recognition_node = Node(
            package='i9robot_voice',
            executable='voice_asr'
         )
    
    voice_command_node = Node(
            package='i9robot_voice',
            executable='voice_cmd'
         )
    
    voice_speech_node = Node(
            package='i9robot_voice',
            executable='voice_tts'
         )
    
    return LaunchDescription([
        voice_recognition_node,
        voice_command_node,
        voice_speech_node
    ])
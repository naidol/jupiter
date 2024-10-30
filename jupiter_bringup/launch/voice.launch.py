########################################################################################################
# Name:             voice.launch.py
# Purpose:          launch custom built voice recognition system for jupiter robot
# Description:      voice_wsp = automatic speech recognition | voice_etts = text-to-speech
#                   voice_cmd = process voice commands 
# Related Files:    see package /jupiter_voice
# Author:           logan naidoo, south africa, 2024
########################################################################################################
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    voice_recognition_node = Node(
            package='jupiter_voice',
            executable='voice_wsp'
         )
    
    voice_command_node = Node(
            package='jupiter_voice',
            executable='voice_cmd'
         )
    
    voice_speech_node = Node(
            package='jupiter_voice',
            executable='voice_etts'
         )

    return LaunchDescription([
        voice_recognition_node,
        voice_command_node,
        voice_speech_node
    ])
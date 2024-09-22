########################################################################################################
# Name:             clocl.launch.py
# Purpose:          launch the clock sync between host pc and esp32 micro-controller
# Description:      Jupiter robot clock sync is necessary to keep odometry headers in-sync
# Related Files:    check out the esp32 firmware for subscribers that read the /clock msgs
# Author:           logan naidoo, south africa, 2024
########################################################################################################

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    clock_sync_node = Node(
            package='jupiter_clock',
            executable='esp32_clock_sync'
         )
    
    return LaunchDescription([
        clock_sync_node
    ])
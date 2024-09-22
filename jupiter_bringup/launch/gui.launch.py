from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the ROS2 GUI Node
        Node(
            package='jupiter_gui',  # Replace with your package name
            executable='jupiter_gui',  # Name of your script/executable (after setup)
            name='gui_node',
            output='screen',
        ),
    ])

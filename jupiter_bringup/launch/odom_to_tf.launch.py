from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='odom_to_tf',
            executable='odom_to_tf_node',
            name='odom_to_tf_node'
        ),
    ])

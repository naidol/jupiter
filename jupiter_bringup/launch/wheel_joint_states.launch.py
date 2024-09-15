from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wheel_joint_state_publisher',
            executable='wheel_joint_state_publisher',
            name='wheel_joint_state_publisher_node',
            output='screen',
        )
    ])

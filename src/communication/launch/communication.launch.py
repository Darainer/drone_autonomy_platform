from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='communication',
            executable='communication_node',
            name='communication_node',
            output='screen',
            # DES-001: canonical topic wiring
            remappings=[('~/attitude_command', '/attitude_command')]
        )
    ])

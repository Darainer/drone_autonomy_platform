from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autonomy',
            executable='autonomy_node',
            name='autonomy_node',
            output='screen',
            # DES-001: canonical topic wiring
            remappings=[('~/mission', '/mission')]
        )
    ])

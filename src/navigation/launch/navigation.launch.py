from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navigation',
            executable='navigation_node',
            name='navigation_node',
            output='screen'
        )
    ])

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='safety',
            executable='safety_node',
            name='safety_node',
            output='screen'
        ),
        Node(
            package='safety',
            executable='battery_monitor',
            name='battery_monitor',
            output='screen'
        ),
    ])

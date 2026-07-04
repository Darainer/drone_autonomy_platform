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
            # DES-003: /mission_status lifecycle topic (transient_local QoS
            # is set on the publisher in autonomy_node, not here)
            remappings=[
                ('~/mission', '/mission'),
                ('~/mission_status', '/mission_status'),
            ]
        )
    ])

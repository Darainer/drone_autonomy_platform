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
            # is set on the publisher in autonomy_node, not here); /survey_request
            # is the GCS-facing entry point (behavior item 1) — the communication
            # package publishing to it is separate wiring, out of WP-1 scope
            remappings=[
                ('~/mission', '/mission'),
                ('~/mission_status', '/mission_status'),
                ('~/survey_request', '/survey_request'),
            ]
        )
    ])

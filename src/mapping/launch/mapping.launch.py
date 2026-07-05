"""Launch survey_recorder_node (DES-004).

DES-004 subscribes to fully-qualified topics (/oak/rgb/image_raw,
/oak/rgb/camera_info, /mavros/local_position/pose,
/mavros/global_position/global, /mission, /mission_status) — no remaps
needed here; this mirrors the DES-004 Interfaces table exactly.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'default_capture_rate_hz',
            default_value='2.0',
            description='Capture rate used when the arming Mission leaves capture_rate_hz at 0',
        ),
        DeclareLaunchArgument(
            'output_dir',
            default_value='/data/surveys',
            description='External data volume root for survey_<mission_id>/ datasets',
        ),
        DeclareLaunchArgument(
            'jpeg_quality',
            default_value='95',
            description='JPEG encode quality (DES-004 D2)',
        ),
        DeclareLaunchArgument(
            'min_free_mb',
            default_value='500',
            description='Refuse to arm below this much free space on the data volume (DES-004 D7)',
        ),
        Node(
            package='mapping',
            executable='survey_recorder_node',
            name='survey_recorder_node',
            output='screen',
            parameters=[{
                'default_capture_rate_hz': ParameterValue(
                    LaunchConfiguration('default_capture_rate_hz'), value_type=float),
                'output_dir': LaunchConfiguration('output_dir'),
                'jpeg_quality': ParameterValue(
                    LaunchConfiguration('jpeg_quality'), value_type=int),
                'min_free_mb': ParameterValue(
                    LaunchConfiguration('min_free_mb'), value_type=int),
            }],
        ),
    ])

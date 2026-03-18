"""Launch the full stack: RGB perception + stereo/IMU VSLAM + fusion node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('perception')

    return LaunchDescription([
        DeclareLaunchArgument(
            'engine_path',
            default_value='/home/dev/models/RF-DETR-SMALL.engine',
            description='Path to RF-DETR-S TensorRT FP16 engine',
        ),
        DeclareLaunchArgument(
            'confidence_threshold',
            default_value='0.5',
            description='Detection confidence threshold',
        ),
        DeclareLaunchArgument('camera_fps', default_value='15.0'),
        DeclareLaunchArgument('rgb_resolution', default_value='1080P'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_share, 'launch', 'oakd_full.launch.py'])
            ),
            launch_arguments={
                'camera_fps': LaunchConfiguration('camera_fps'),
                'rgb_resolution': LaunchConfiguration('rgb_resolution'),
            }.items(),
        ),
        Node(
            package='perception',
            executable='rfdetr_node',
            name='rfdetr_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([pkg_share, 'config', 'rfdetr.yaml']),
                {
                    'engine_path': LaunchConfiguration('engine_path'),
                    'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                },
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_share, 'launch', 'vslam.launch.py'])
            ),
        ),
        Node(
            package='perception',
            executable='perception_node',
            name='perception_node',
            output='screen',
        ),
    ])

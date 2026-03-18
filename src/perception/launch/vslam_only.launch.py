"""Launch VSLAM-only stack: OAK-D stereo/IMU sensors + VSLAM node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('perception')

    return LaunchDescription([
        DeclareLaunchArgument('camera_fps', default_value='15.0'),
        DeclareLaunchArgument('rgb_resolution', default_value='1080P'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_share, 'launch', 'oakd_vslam_sensors.launch.py'])
            ),
            launch_arguments={
                'camera_fps': LaunchConfiguration('camera_fps'),
                'rgb_resolution': LaunchConfiguration('rgb_resolution'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_share, 'launch', 'vslam.launch.py'])
            ),
        ),
    ])

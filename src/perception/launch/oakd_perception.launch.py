"""Launch a lean OAK-D RGB-only camera configuration for perception."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_fps', default_value='30.0'),
        DeclareLaunchArgument('rgb_resolution', default_value='1080P'),
        Node(
            package='depthai_ros_driver',
            executable='camera_node',
            name='oak',
            output='screen',
            parameters=[{
                'camera.i_usb_speed': 'SUPER_PLUS',
                # RF-DETR expects RGB input; avoid left/right mono streams here.
                'camera.i_enable_imu': False,
                'rgb.i_resolution': LaunchConfiguration('rgb_resolution'),
                'rgb.i_fps': LaunchConfiguration('camera_fps'),
                'left.i_publish_topic': False,
                'right.i_publish_topic': False,
                'stereo.i_publish_topic': False,
                'stereo.i_output_disparity': False,
                'stereo.i_lr_check': False,
            }],
        ),
    ])

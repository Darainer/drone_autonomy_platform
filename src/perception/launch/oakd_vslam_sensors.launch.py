"""Launch the OAK-D camera configuration needed for VSLAM."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_fps', default_value='15.0'),
        DeclareLaunchArgument('rgb_resolution', default_value='1080P'),
        Node(
            package='depthai_ros_driver',
            executable='camera_node',
            name='oak',
            output='screen',
            parameters=[{
                'camera.i_usb_speed': 'SUPER_PLUS',
                'camera.i_enable_imu': True,
                'rgb.i_resolution': LaunchConfiguration('rgb_resolution'),
                'rgb.i_fps': LaunchConfiguration('camera_fps'),
                'left.i_publish_topic': True,
                'right.i_publish_topic': True,
                'stereo.i_publish_topic': True,
                'stereo.i_depth_preset': 'HIGH_ACCURACY',
                'stereo.i_output_disparity': False,
                'stereo.i_lr_check': True,
            }],
        ),
    ])

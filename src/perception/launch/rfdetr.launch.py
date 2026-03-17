"""Launch RF-DETR-S detection with OAK-D camera and VSLAM.

Replaces the Isaac ROS RT-DETR pipeline with RF-DETR-S (Apache 2.0, NMS-free).
Publishes Detection2DArray on /detections — same interface as before.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('perception')

    return LaunchDescription([
        # --- Launch arguments ---
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
        DeclareLaunchArgument('camera_fps', default_value='30'),
        DeclareLaunchArgument('rgb_resolution', default_value='1080P'),

        # --- OAK-D camera driver ---
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
                'stereo.i_depth_preset': 'HIGH_ACCURACY',
                'stereo.i_output_disparity': False,
                'stereo.i_lr_check': True,
            }],
        ),

        # --- RF-DETR-S TensorRT inference node ---
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

        # --- Visual SLAM ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_share, 'launch', 'vslam.launch.py'])
            ),
        ),

        # --- Perception fusion node (unchanged) ---
        Node(
            package='perception',
            executable='perception_node',
            name='perception_node',
            output='screen',
        ),
    ])

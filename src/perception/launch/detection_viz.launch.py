"""
Launch the full RT-DETR perception pipeline + a live detection overlay viewer.

Topics:
  /oak/rgb/image_raw              — raw camera feed
  /detections                     — Detection2DArray from RT-DETR
  /visualization/detection_overlay — annotated image (view in rqt_image_view)

Usage:
  ros2 launch perception detection_viz.launch.py
  ros2 launch perception detection_viz.launch.py confidence_threshold:=0.5
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'confidence_threshold',
            default_value='0.5',
            description='RT-DETR confidence threshold (lower = more boxes shown)',
        ),
        DeclareLaunchArgument(
            'model_path',
            default_value='/home/dev/models/rtdetr_coco.plan',
            description='Path to TensorRT engine file',
        ),

        # Full RT-DETR pipeline — VSLAM disabled (not needed for viz)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('perception'), 'launch', 'perception.launch.py'
                ])
            ),
            launch_arguments={
                'enable_vslam': 'false',
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                'model_path': LaunchConfiguration('model_path'),
            }.items(),
        ),

        # Bounding box overlay publisher
        Node(
            package='perception',
            executable='detection_visualizer',
            name='detection_visualizer',
            output='screen',
        ),

        # Image viewer — opens a window showing the annotated feed
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='image_viewer',
            arguments=['/visualization/detection_overlay'],
            output='screen',
        ),
    ])

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Isaac ROS cuVSLAM — stereo visual odometry
        Node(
            package='isaac_ros_visual_slam',
            executable='visual_slam_node',
            name='visual_slam',
            output='screen',
            parameters=[{
                'use_stereo': True,
                'enable_imu_fusion': True,
                'gyro_noise_density': 0.000244,       # OAK-D IMU (BMI270)
                'gyro_random_walk': 0.000019393,
                'accel_noise_density': 0.001862,
                'accel_random_walk': 0.003,
                'enable_slam_visualization': False,    # disable for flight
                'enable_observations_view': False,
                'enable_landmarks_view': False,
                'publish_odom_to_base_tf': True,
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'map_frame': 'map',
            }],
            remappings=[
                ('stereo_camera/left/image', '/oak/left/image_rect'),
                ('stereo_camera/left/camera_info', '/oak/left/camera_info'),
                ('stereo_camera/right/image', '/oak/right/image_rect'),
                ('stereo_camera/right/camera_info', '/oak/right/camera_info'),
                ('visual_slam/imu', '/mavros/imu/data'),
            ],
        ),
    ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model_path = LaunchConfiguration('model_path')
    confidence = LaunchConfiguration('confidence_threshold')

    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value='/workspaces/isaac_ros-dev/models/rtdetr_l.plan',
            description='Path to TensorRT engine file',
        ),
        DeclareLaunchArgument(
            'confidence_threshold',
            default_value='0.7',
            description='RT-DETR detection confidence threshold',
        ),
        DeclareLaunchArgument('camera_fps', default_value='30'),
        DeclareLaunchArgument('rgb_resolution', default_value='1080P'),
        DeclareLaunchArgument('enable_vslam', default_value='true'),

        # --- OAK-D camera driver (shared by RT-DETR + VSLAM) ---
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

        # --- Isaac ROS RT-DETR inference pipeline ---
        ComposableNodeContainer(
            name='rtdetr_container',
            namespace='perception',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                # 1. Image -> tensor (resize + normalize)
                ComposableNode(
                    package='isaac_ros_dnn_image_encoder',
                    plugin='nvidia::isaac_ros::dnn_inference::DnnImageEncoderNode',
                    name='dnn_image_encoder',
                    parameters=[{
                        'input_image_width': 1920,
                        'input_image_height': 1080,
                        'network_image_width': 640,
                        'network_image_height': 640,
                        'image_mean': [0.485, 0.456, 0.406],
                        'image_stddev': [0.229, 0.224, 0.225],
                    }],
                    remappings=[
                        ('image', '/oak/rgb/image_raw'),
                        ('encoded_tensor', 'encoded_tensor'),
                    ],
                ),
                # 2. RT-DETR preprocessor
                ComposableNode(
                    package='isaac_ros_rtdetr',
                    plugin='nvidia::isaac_ros::rtdetr::RtDetrPreprocessorNode',
                    name='rtdetr_preprocessor',
                    parameters=[{
                        'image_height': 640,
                        'image_width': 640,
                    }],
                    remappings=[
                        ('encoded_tensor', 'encoded_tensor'),
                        ('tensor_pub', 'tensor_pub'),
                    ],
                ),
                # 3. TensorRT inference
                ComposableNode(
                    package='isaac_ros_tensor_rt',
                    plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
                    name='tensor_rt',
                    parameters=[{
                        'engine_file_path': model_path,
                        'input_tensor_names': ['images', 'orig_target_sizes'],
                        'input_binding_names': ['images', 'orig_target_sizes'],
                        'output_tensor_names': ['labels', 'boxes', 'scores'],
                        'output_binding_names': ['labels', 'boxes', 'scores'],
                        'verbose': False,
                    }],
                    remappings=[
                        ('tensor_pub', 'tensor_pub'),
                        ('tensor_sub', 'tensor_output'),
                    ],
                ),
                # 4. RT-DETR decoder -> Detection2DArray
                ComposableNode(
                    package='isaac_ros_rtdetr',
                    plugin='nvidia::isaac_ros::rtdetr::RtDetrDecoderNode',
                    name='rtdetr_decoder',
                    parameters=[{
                        'confidence_threshold': confidence,
                    }],
                    remappings=[
                        ('tensor_sub', 'tensor_output'),
                        ('detections', '/detections'),
                    ],
                ),
            ],
            output='screen',
        ),

        # --- Visual SLAM (separate launch file) ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('perception'), 'launch', 'vslam.launch.py'
                ])
            ),
        ),

        # --- Perception fusion node ---
        Node(
            package='perception',
            executable='perception_node',
            name='perception_node',
            output='screen',
        ),
    ])

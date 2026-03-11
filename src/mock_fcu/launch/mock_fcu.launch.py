"""
mock_fcu.launch.py

Launches the mock flight controller node for integration testing.
Include this in launch_testing suites instead of the real MAVROS stack.

Usage (standalone):
    ros2 launch mock_fcu mock_fcu.launch.py battery_percentage:=0.10

Usage (in launch_testing):
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from ament_index_python.packages import get_package_share_directory

    mock_fcu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mock_fcu'),
                         'launch', 'mock_fcu.launch.py')
        ),
        launch_arguments={'battery_percentage': '1.0', 'armed': 'false'}.items(),
    )
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('armed',                  default_value='false'),
        DeclareLaunchArgument('mode',                   default_value='STABILIZED'),
        DeclareLaunchArgument('battery_percentage',     default_value='1.0'),
        DeclareLaunchArgument('publish_hz',             default_value='10.0'),
        DeclareLaunchArgument('simulate_disarm_on_rtl', default_value='true'),

        Node(
            package='mock_fcu',
            executable='mock_fcu_node',
            name='mock_fcu',
            output='screen',
            parameters=[{
                'armed':                  LaunchConfiguration('armed'),
                'mode':                   LaunchConfiguration('mode'),
                'battery_percentage':     LaunchConfiguration('battery_percentage'),
                'publish_hz':             LaunchConfiguration('publish_hz'),
                'simulate_disarm_on_rtl': LaunchConfiguration('simulate_disarm_on_rtl'),
            }],
        ),
    ])

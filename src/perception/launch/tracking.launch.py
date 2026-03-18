"""Standalone launch file for the detector-agnostic multi-object tracker.

Can be used independently (pointed at any Detection2DArray topic) or
included from perception.launch.py via enable_tracking:=true.

Odometry source selection
-------------------------
odometry_source:=vslam   — subscribe to cuVSLAM odometry (default, requires VSLAM)
odometry_source:=mavros  — subscribe to MAVROS local position odometry
odometry_source:=none    — run tracker without ego-motion compensation

Detector source
---------------
detection_topic          — remap this to the Detection2DArray topic produced by
                           your detector (default: /detections, matches RT-DETR)

Tracker backend
---------------
tracker_type:=bytetrack  (default) — ByteTrack, best for aerial / fast-moving scenes
tracker_type:=sort       — SORT, lightweight, lower accuracy
tracker_type:=botsort    — BotSORT, includes built-in camera-motion compensation
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ── tracker backend ────────────────────────────────────────────────
        DeclareLaunchArgument(
            'tracker_type',
            default_value='bytetrack',
            description='Tracker backend: bytetrack | sort | botsort',
        ),

        # ── detector source (detector-agnostic: remap to any Detection2DArray) ──
        DeclareLaunchArgument(
            'detection_topic',
            default_value='/detections',
            description='vision_msgs/Detection2DArray topic from any detector',
        ),

        # ── odometry source ────────────────────────────────────────────────
        DeclareLaunchArgument(
            'odometry_source',
            default_value='vslam',
            description='Odometry source for ego-motion compensation: vslam | mavros | none',
        ),
        DeclareLaunchArgument(
            'vslam_odom_topic',
            default_value='/visual_slam/tracking/odometry',
            description='cuVSLAM odometry topic (used when odometry_source:=vslam)',
        ),
        DeclareLaunchArgument(
            'mavros_odom_topic',
            default_value='/mavros/local_position/odom',
            description='MAVROS odometry topic (used when odometry_source:=mavros)',
        ),

        # ── camera intrinsics for rotation compensation ────────────────────
        DeclareLaunchArgument(
            'camera_fx', default_value='860.0',
            description='Camera focal length X [px] — OAK-D 1080p default',
        ),
        DeclareLaunchArgument(
            'camera_fy', default_value='860.0',
            description='Camera focal length Y [px]',
        ),
        DeclareLaunchArgument(
            'camera_cx', default_value='960.0',
            description='Camera principal point X [px]',
        ),
        DeclareLaunchArgument(
            'camera_cy', default_value='540.0',
            description='Camera principal point Y [px]',
        ),

        # ── ByteTrack tuning ───────────────────────────────────────────────
        DeclareLaunchArgument(
            'bt_track_activation_threshold', default_value='0.25',
            description='ByteTrack: min confidence to activate a new track',
        ),
        DeclareLaunchArgument(
            'bt_lost_track_buffer', default_value='30',
            description='ByteTrack: frames before a lost track is removed',
        ),
        DeclareLaunchArgument(
            'bt_minimum_matching_threshold', default_value='0.8',
            description='ByteTrack: IoU threshold for detection–track matching',
        ),
        DeclareLaunchArgument(
            'bt_frame_rate', default_value='15',
            description='ByteTrack: assumed frame rate for velocity model',
        ),
        DeclareLaunchArgument(
            'bt_minimum_consecutive_frames', default_value='1',
            description='ByteTrack: frames before a track is confirmed',
        ),

        # ── SORT tuning ────────────────────────────────────────────────────
        DeclareLaunchArgument(
            'sort_min_hits', default_value='3',
            description='SORT: min hits before a track is confirmed',
        ),
        DeclareLaunchArgument(
            'sort_iou_threshold', default_value='0.3',
            description='SORT: IoU threshold for association',
        ),
        DeclareLaunchArgument(
            'sort_max_age', default_value='1',
            description='SORT: frames before a lost track is removed',
        ),

        # ── tracker node ───────────────────────────────────────────────────
        Node(
            package='perception',
            executable='multi_object_tracker',
            name='multi_object_tracker',
            output='screen',
            parameters=[{
                'tracker_type':                     LaunchConfiguration('tracker_type'),
                'detection_topic':                  LaunchConfiguration('detection_topic'),
                'odometry_source':                  LaunchConfiguration('odometry_source'),
                'vslam_odom_topic':                 LaunchConfiguration('vslam_odom_topic'),
                'mavros_odom_topic':                LaunchConfiguration('mavros_odom_topic'),
                'camera_fx':                        LaunchConfiguration('camera_fx'),
                'camera_fy':                        LaunchConfiguration('camera_fy'),
                'camera_cx':                        LaunchConfiguration('camera_cx'),
                'camera_cy':                        LaunchConfiguration('camera_cy'),
                'bt_track_activation_threshold':    LaunchConfiguration('bt_track_activation_threshold'),
                'bt_lost_track_buffer':             LaunchConfiguration('bt_lost_track_buffer'),
                'bt_minimum_matching_threshold':    LaunchConfiguration('bt_minimum_matching_threshold'),
                'bt_frame_rate':                    LaunchConfiguration('bt_frame_rate'),
                'bt_minimum_consecutive_frames':    LaunchConfiguration('bt_minimum_consecutive_frames'),
                'sort_min_hits':                    LaunchConfiguration('sort_min_hits'),
                'sort_iou_threshold':               LaunchConfiguration('sort_iou_threshold'),
                'sort_max_age':                     LaunchConfiguration('sort_max_age'),
            }],
        ),
    ])

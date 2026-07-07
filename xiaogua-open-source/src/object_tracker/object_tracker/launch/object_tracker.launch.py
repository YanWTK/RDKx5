from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('detection_topic', default_value='/yolo_detector/detections'),
        DeclareLaunchArgument('selected_detection_topic',
                              default_value='/vlm_target_selector/selected_detection'),
        DeclareLaunchArgument('image_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('track_high_thresh', default_value='0.15'),
        DeclareLaunchArgument('track_low_thresh', default_value='0.02'),
        DeclareLaunchArgument('new_track_thresh', default_value='0.20'),
        DeclareLaunchArgument('match_thresh', default_value='0.7'),
        DeclareLaunchArgument('second_match_thresh', default_value='0.5'),
        DeclareLaunchArgument('selected_match_thresh', default_value='0.3'),
        DeclareLaunchArgument('track_buffer', default_value='45'),
        DeclareLaunchArgument('max_new_tracks_per_frame', default_value='3'),
        DeclareLaunchArgument('class_aware', default_value='true'),
        DeclareLaunchArgument('enable_debug_image', default_value='true'),
        DeclareLaunchArgument('draw_lost_tracks', default_value='false'),
        DeclareLaunchArgument('selected_only_after_lock', default_value='true'),
        DeclareLaunchArgument('clear_selection_on_lost', default_value='true'),
        DeclareLaunchArgument('selected_lost_clear_frames', default_value='0'),
        DeclareLaunchArgument('use_appearance_match', default_value='true'),
        DeclareLaunchArgument('appearance_weight', default_value='0.35'),
        DeclareLaunchArgument('appearance_match_thresh', default_value='0.45'),
        DeclareLaunchArgument('appearance_update_rate', default_value='0.15'),
        Node(
            package='object_tracker',
            executable='object_tracker_node',
            output='screen',
            parameters=[{
                'detection_topic': LaunchConfiguration('detection_topic'),
                'selected_detection_topic': LaunchConfiguration('selected_detection_topic'),
                'image_topic': LaunchConfiguration('image_topic'),
                'track_high_thresh': LaunchConfiguration('track_high_thresh'),
                'track_low_thresh': LaunchConfiguration('track_low_thresh'),
                'new_track_thresh': LaunchConfiguration('new_track_thresh'),
                'match_thresh': LaunchConfiguration('match_thresh'),
                'second_match_thresh': LaunchConfiguration('second_match_thresh'),
                'selected_match_thresh': LaunchConfiguration('selected_match_thresh'),
                'track_buffer': LaunchConfiguration('track_buffer'),
                'max_new_tracks_per_frame': LaunchConfiguration('max_new_tracks_per_frame'),
                'class_aware': LaunchConfiguration('class_aware'),
                'enable_debug_image': LaunchConfiguration('enable_debug_image'),
                'draw_lost_tracks': LaunchConfiguration('draw_lost_tracks'),
                'selected_only_after_lock': LaunchConfiguration('selected_only_after_lock'),
                'clear_selection_on_lost': LaunchConfiguration('clear_selection_on_lost'),
                'selected_lost_clear_frames': LaunchConfiguration('selected_lost_clear_frames'),
                'use_appearance_match': LaunchConfiguration('use_appearance_match'),
                'appearance_weight': LaunchConfiguration('appearance_weight'),
                'appearance_match_thresh': LaunchConfiguration('appearance_match_thresh'),
                'appearance_update_rate': LaunchConfiguration('appearance_update_rate'),
            }],
        ),
    ])

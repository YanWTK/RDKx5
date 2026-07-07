from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('detection_topic', default_value='/yolo_detector/detections'),
        DeclareLaunchArgument('use_local_vlm', default_value='true'),
        DeclareLaunchArgument('vlm_url', default_value='http://127.0.0.1:8000/analyze'),
        DeclareLaunchArgument('bailian_model', default_value='qwen3-vl-plus'),
        DeclareLaunchArgument('bailian_base_url',
                              default_value='https://dashscope.aliyuncs.com/compatible-mode/v1'),
        DeclareLaunchArgument('bailian_api_key_env', default_value='DASHSCOPE_API_KEY'),
        DeclareLaunchArgument('bailian_enable_thinking', default_value='false'),
        DeclareLaunchArgument('service_name', default_value='/vlm_target_selector/select_target'),
        DeclareLaunchArgument('log_dir', default_value='/opt/xiaogua/ros2_ws/vlm_logs'),
        DeclareLaunchArgument('request_timeout_sec', default_value='20.0'),
        DeclareLaunchArgument('max_data_age_sec', default_value='2.0'),
        DeclareLaunchArgument('max_pair_time_diff_sec', default_value='0.5'),
        DeclareLaunchArgument('selection_frame_count', default_value='10'),
        DeclareLaunchArgument('selection_timeout_sec', default_value='1.0'),
        DeclareLaunchArgument('always_save_debug_images', default_value='true'),
        DeclareLaunchArgument('task_gated', default_value='false'),
        Node(
            package='vlm_target_selector',
            executable='vlm_target_selector_node',
            output='screen',
            parameters=[{
                'image_topic': LaunchConfiguration('image_topic'),
                'detection_topic': LaunchConfiguration('detection_topic'),
                'use_local_vlm': LaunchConfiguration('use_local_vlm'),
                'vlm_url': LaunchConfiguration('vlm_url'),
                'bailian_model': LaunchConfiguration('bailian_model'),
                'bailian_base_url': LaunchConfiguration('bailian_base_url'),
                'bailian_api_key_env': LaunchConfiguration('bailian_api_key_env'),
                'bailian_enable_thinking': LaunchConfiguration('bailian_enable_thinking'),
                'service_name': LaunchConfiguration('service_name'),
                'log_dir': LaunchConfiguration('log_dir'),
                'request_timeout_sec': LaunchConfiguration('request_timeout_sec'),
                'max_data_age_sec': LaunchConfiguration('max_data_age_sec'),
                'max_pair_time_diff_sec': LaunchConfiguration('max_pair_time_diff_sec'),
                'selection_frame_count': LaunchConfiguration('selection_frame_count'),
                'selection_timeout_sec': LaunchConfiguration('selection_timeout_sec'),
                'always_save_debug_images': LaunchConfiguration('always_save_debug_images'),
                'task_gated': LaunchConfiguration('task_gated'),
            }],
        ),
    ])

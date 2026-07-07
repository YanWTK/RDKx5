from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('status_topic', default_value='/object_tracker/status'),
        DeclareLaunchArgument('result_topic',
                              default_value='/vlm_target_selector/reselector_status'),
        DeclareLaunchArgument('service_name',
                              default_value='/vlm_target_selector/select_target'),
        DeclareLaunchArgument('target_name', default_value=''),
        DeclareLaunchArgument('target_name_topic',
                              default_value='/vlm_target_selector/current_target_name'),
        DeclareLaunchArgument('lost_reselect_delay_sec', default_value='1.0'),
        DeclareLaunchArgument('max_lost_no_object_sec', default_value='8.0'),
        DeclareLaunchArgument('reselect_cooldown_sec', default_value='5.0'),
        DeclareLaunchArgument('save_debug_images', default_value='true'),
        DeclareLaunchArgument('trigger_on_lost_final', default_value='true'),
        Node(
            package='vlm_target_selector',
            executable='lost_reselector_node',
            output='screen',
            parameters=[{
                'status_topic': LaunchConfiguration('status_topic'),
                'result_topic': LaunchConfiguration('result_topic'),
                'service_name': LaunchConfiguration('service_name'),
                'target_name': LaunchConfiguration('target_name'),
                'target_name_topic': LaunchConfiguration('target_name_topic'),
                'lost_reselect_delay_sec': LaunchConfiguration('lost_reselect_delay_sec'),
                'max_lost_no_object_sec': LaunchConfiguration('max_lost_no_object_sec'),
                'reselect_cooldown_sec': LaunchConfiguration('reselect_cooldown_sec'),
                'save_debug_images': LaunchConfiguration('save_debug_images'),
                'trigger_on_lost_final': LaunchConfiguration('trigger_on_lost_final'),
            }],
        ),
    ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rosbridge_url', default_value='ws://127.0.0.1:9090'),
        DeclareLaunchArgument('input_topic', default_value='/object_tracker/selected_detection'),
        DeclareLaunchArgument('ros1_output_topic', default_value='/tracked_yolov8/detections'),
        DeclareLaunchArgument('publish_empty_on_no_target', default_value='true'),
        DeclareLaunchArgument('empty_publish_timeout_sec', default_value='0.5'),
        Node(
            package='vision_ros1_tf_bridge',
            executable='selected_detection_bridge_node',
            output='screen',
            parameters=[{
                'rosbridge_url': LaunchConfiguration('rosbridge_url'),
                'input_topic': LaunchConfiguration('input_topic'),
                'ros1_output_topic': LaunchConfiguration('ros1_output_topic'),
                'publish_empty_on_no_target': LaunchConfiguration('publish_empty_on_no_target'),
                'empty_publish_timeout_sec': LaunchConfiguration('empty_publish_timeout_sec'),
            }],
        ),
    ])

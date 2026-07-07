from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rosbridge_url', default_value='ws://127.0.0.1:9090'),
        DeclareLaunchArgument('input_topic', default_value='/vision/target_point_local'),
        DeclareLaunchArgument('parent_frame', default_value='camera_link'),
        DeclareLaunchArgument('child_frame', default_value='vision_target'),
        DeclareLaunchArgument('input_frame_mode', default_value='optical_to_camera_link'),
        DeclareLaunchArgument('publish_ros1_point', default_value='true'),
        DeclareLaunchArgument('ros1_point_topic', default_value='/vision/target_point_camera_link'),
        Node(
            package='vision_ros1_tf_bridge',
            executable='vision_tf_bridge_node',
            name='vision_tf_bridge',
            output='screen',
            parameters=[{
                'rosbridge_url': LaunchConfiguration('rosbridge_url'),
                'input_topic': LaunchConfiguration('input_topic'),
                'parent_frame': LaunchConfiguration('parent_frame'),
                'child_frame': LaunchConfiguration('child_frame'),
                'input_frame_mode': LaunchConfiguration('input_frame_mode'),
                'publish_ros1_point': LaunchConfiguration('publish_ros1_point'),
                'ros1_point_topic': LaunchConfiguration('ros1_point_topic'),
            }],
        ),
    ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rosbridge_url', default_value='ws://127.0.0.1:9090'),
        DeclareLaunchArgument('bridge_tf', default_value='true'),
        DeclareLaunchArgument('bridge_tf_static', default_value='true'),
        Node(
            package='vision_ros1_tf_bridge',
            executable='ros1_tf_to_ros2_bridge_node',
            name='ros1_tf_to_ros2_bridge',
            output='screen',
            parameters=[{
                'rosbridge_url': LaunchConfiguration('rosbridge_url'),
                'bridge_tf': LaunchConfiguration('bridge_tf'),
                'bridge_tf_static': LaunchConfiguration('bridge_tf_static'),
            }],
        ),
    ])

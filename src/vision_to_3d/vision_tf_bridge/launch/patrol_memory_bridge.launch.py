from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rosbridge_url', default_value='ws://127.0.0.1:9090'),
        DeclareLaunchArgument('bridge_markers', default_value='true'),
        Node(
            package='vision_ros1_tf_bridge',
            executable='patrol_memory_bridge_node',
            name='patrol_memory_bridge',
            output='screen',
            parameters=[{
                'rosbridge_url': LaunchConfiguration('rosbridge_url'),
                'bridge_markers': LaunchConfiguration('bridge_markers'),
            }],
        ),
    ])

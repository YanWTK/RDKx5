from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rosbridge_url', default_value='ws://127.0.0.1:9090'),
        DeclareLaunchArgument('forward_vad', default_value='true'),
        Node(
            package='doa_ros1_bridge',
            executable='bridge_node',
            name='doa_to_ros1_bridge',
            output='screen',
            parameters=[{
                'rosbridge_url': LaunchConfiguration('rosbridge_url'),
                'forward_vad': LaunchConfiguration('forward_vad'),
            }],
        ),
    ])

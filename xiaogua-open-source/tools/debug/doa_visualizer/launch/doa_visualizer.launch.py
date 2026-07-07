from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value='/opt/xiaogua/ros2_ws/src/doa_visualizer/config/default.yaml',
            description='Path to the doa_visualizer YAML config file.'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value='/opt/xiaogua/ros2_ws/src/doa_visualizer/config/default.yaml',
            description='Alias of config_file for symmetry with other launch files.'
        ),

        Node(
            package='doa_visualizer',
            executable='doa_visualizer_node',
            name='doa_visualizer_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
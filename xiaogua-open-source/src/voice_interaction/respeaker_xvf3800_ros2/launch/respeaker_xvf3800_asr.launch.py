from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="respeaker_xvf3800_ros2",
                executable="respeaker_xvf3800_node",
                name="respeaker_xvf3800_node",
                output="screen",
                parameters=[
                    PathJoinSubstitution(
                        [FindPackageShare("respeaker_xvf3800_ros2"), "config", "default.yaml"]
                    )
                ],
            ),
            Node(
                package="respeaker_xvf3800_ros2",
                executable="respeaker_xvf3800_asr_node",
                name="respeaker_xvf3800_asr_node",
                output="screen",
            ),
        ]
    )

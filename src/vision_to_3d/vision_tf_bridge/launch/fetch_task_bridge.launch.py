#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("rosbridge_url", default_value="ws://127.0.0.1:9090"),
        DeclareLaunchArgument(
            "ros1_to_ros2_topics",
            default_value="/task_understanding/query,/object_memory/query,/target_confirm/confirm_cmd,/memory_target_selector/select_cmd",
        ),
        DeclareLaunchArgument(
            "ros2_to_ros1_topics",
            default_value="/task_understanding/result,/object_memory/query_result,/target_confirm/result,/memory_target_selector/result,/object_tracker/status,/vlm_target_selector/reselector_status",
        ),
        Node(
            package="vision_ros1_tf_bridge",
            executable="fetch_task_bridge_node",
            name="fetch_task_bridge",
            output="screen",
            parameters=[{
                "rosbridge_url": LaunchConfiguration("rosbridge_url"),
                "ros1_to_ros2_topics": LaunchConfiguration("ros1_to_ros2_topics"),
                "ros2_to_ros1_topics": LaunchConfiguration("ros2_to_ros1_topics"),
            }],
        ),
    ])

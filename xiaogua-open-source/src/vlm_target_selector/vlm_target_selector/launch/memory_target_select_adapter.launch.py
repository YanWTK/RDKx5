#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("memory_path", default_value=""),
        DeclareLaunchArgument("cmd_topic", default_value="/memory_target_selector/select_cmd"),
        DeclareLaunchArgument("result_topic", default_value="/memory_target_selector/result"),
        DeclareLaunchArgument("selector_service", default_value="/vlm_target_selector/select_target"),
        DeclareLaunchArgument("wait_service_timeout_sec", default_value="2.0"),
        DeclareLaunchArgument("save_debug_images", default_value="true"),
        Node(
            package="vlm_target_selector",
            executable="memory_target_select_adapter_node",
            name="memory_target_select_adapter",
            output="screen",
            parameters=[{
                "memory_path": LaunchConfiguration("memory_path"),
                "cmd_topic": LaunchConfiguration("cmd_topic"),
                "result_topic": LaunchConfiguration("result_topic"),
                "selector_service": LaunchConfiguration("selector_service"),
                "wait_service_timeout_sec": LaunchConfiguration("wait_service_timeout_sec"),
                "save_debug_images": LaunchConfiguration("save_debug_images"),
            }],
        ),
    ])

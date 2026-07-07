#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("query_topic", default_value="/task_understanding/query"),
        DeclareLaunchArgument("result_topic", default_value="/task_understanding/result"),
        DeclareLaunchArgument("use_local_llm", default_value="true"),
        DeclareLaunchArgument("llm_url", default_value="http://127.0.0.1:8000/analyze"),
        DeclareLaunchArgument("bailian_model", default_value="qwen3.6-flash"),
        DeclareLaunchArgument("bailian_base_url", default_value=""),
        DeclareLaunchArgument("bailian_api_key_env", default_value="DASHSCOPE_API_KEY"),
        DeclareLaunchArgument("bailian_enable_thinking", default_value="false"),
        DeclareLaunchArgument(
            "location_config_path",
            default_value="/opt/xiaogua/legacy_ws/yahboomcar_ws/src/nav_pkg/config/patrol_points.json",
        ),
        Node(
            package="vlm_target_selector",
            executable="task_understanding_node",
            name="task_understanding",
            output="screen",
            parameters=[{
                "query_topic": LaunchConfiguration("query_topic"),
                "result_topic": LaunchConfiguration("result_topic"),
                "use_local_llm": LaunchConfiguration("use_local_llm"),
                "llm_url": LaunchConfiguration("llm_url"),
                "bailian_model": LaunchConfiguration("bailian_model"),
                "bailian_base_url": LaunchConfiguration("bailian_base_url"),
                "bailian_api_key_env": LaunchConfiguration("bailian_api_key_env"),
                "bailian_enable_thinking": LaunchConfiguration("bailian_enable_thinking"),
                "location_config_path": LaunchConfiguration("location_config_path"),
            }],
        ),
    ])

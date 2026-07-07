#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("memory_path", default_value=""),
        DeclareLaunchArgument("query_topic", default_value="/object_memory/query"),
        DeclareLaunchArgument("result_topic", default_value="/object_memory/query_result"),
        DeclareLaunchArgument("min_score", default_value="10.0"),
        DeclareLaunchArgument("use_llm_selection", default_value="true"),
        DeclareLaunchArgument("allow_rule_fallback", default_value="true"),
        DeclareLaunchArgument("use_local_llm", default_value="true"),
        DeclareLaunchArgument("llm_url", default_value="http://127.0.0.1:8000/analyze"),
        DeclareLaunchArgument("bailian_model", default_value="qwen3.6-flash"),
        DeclareLaunchArgument("bailian_base_url", default_value=""),
        DeclareLaunchArgument("bailian_api_key_env", default_value="DASHSCOPE_API_KEY"),
        DeclareLaunchArgument("bailian_enable_thinking", default_value="false"),
        Node(
            package="vlm_target_selector",
            executable="object_memory_query_node",
            name="object_memory_query",
            output="screen",
            parameters=[{
                "memory_path": LaunchConfiguration("memory_path"),
                "query_topic": LaunchConfiguration("query_topic"),
                "result_topic": LaunchConfiguration("result_topic"),
                "min_score": LaunchConfiguration("min_score"),
                "use_llm_selection": LaunchConfiguration("use_llm_selection"),
                "allow_rule_fallback": LaunchConfiguration("allow_rule_fallback"),
                "use_local_llm": LaunchConfiguration("use_local_llm"),
                "llm_url": LaunchConfiguration("llm_url"),
                "bailian_model": LaunchConfiguration("bailian_model"),
                "bailian_base_url": LaunchConfiguration("bailian_base_url"),
                "bailian_api_key_env": LaunchConfiguration("bailian_api_key_env"),
                "bailian_enable_thinking": LaunchConfiguration("bailian_enable_thinking"),
            }],
        ),
    ])

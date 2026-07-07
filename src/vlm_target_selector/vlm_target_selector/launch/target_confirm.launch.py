#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


CUSTOM_MODEL = "/opt/xiaogua/models/yolo_model.bin"
CUSTOM_CLASSES = "person,cell phone,mouse,remote,book,bottle,cup,bowl,apple,banana,teddy bear,bag_wrapper,box"


def generate_launch_description():
    args = [
        DeclareLaunchArgument("image_topic", default_value="/camera/color/image_raw"),
        DeclareLaunchArgument("depth_topic", default_value="/camera/depth/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/depth/camera_info"),
        DeclareLaunchArgument("confirm_cmd_topic", default_value="/target_confirm/confirm_cmd"),
        DeclareLaunchArgument("confirm_result_topic", default_value="/target_confirm/result"),
        DeclareLaunchArgument("numbered_image_topic", default_value="/target_confirm/numbered_image"),
        DeclareLaunchArgument("final_image_topic", default_value="/target_confirm/final_image"),
        DeclareLaunchArgument("object_memory_path", default_value=""),
        DeclareLaunchArgument(
            "model_path",
            default_value=CUSTOM_MODEL,
        ),
        DeclareLaunchArgument("class_names", default_value=CUSTOM_CLASSES),
        DeclareLaunchArgument("preprocess_mode", default_value="letterbox"),
        DeclareLaunchArgument("conf_threshold", default_value="0.07"),
        DeclareLaunchArgument("nms_threshold", default_value="0.7"),
        DeclareLaunchArgument("capture_count", default_value="5"),
        DeclareLaunchArgument("capture_interval_sec", default_value="0.25"),
        DeclareLaunchArgument("max_image_age_sec", default_value="2.0"),
        DeclareLaunchArgument("request_timeout_sec", default_value="20.0"),
        DeclareLaunchArgument("use_local_vlm", default_value="false"),
        DeclareLaunchArgument("vlm_url", default_value="http://127.0.0.1:8000/analyze"),
        DeclareLaunchArgument("bailian_model", default_value="qwen3-vl-plus"),
        DeclareLaunchArgument("bailian_base_url", default_value=""),
        DeclareLaunchArgument("bailian_api_key_env", default_value="DASHSCOPE_API_KEY"),
        DeclareLaunchArgument("bailian_enable_thinking", default_value="false"),
        DeclareLaunchArgument("task_gated", default_value="false"),
    ]

    node = Node(
        package="vlm_target_selector",
        executable="target_confirm_node",
        name="target_confirm",
        output="screen",
        parameters=[
            {
                "image_topic": LaunchConfiguration("image_topic"),
                "depth_topic": LaunchConfiguration("depth_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "confirm_cmd_topic": LaunchConfiguration("confirm_cmd_topic"),
                "confirm_result_topic": LaunchConfiguration("confirm_result_topic"),
                "numbered_image_topic": LaunchConfiguration("numbered_image_topic"),
                "final_image_topic": LaunchConfiguration("final_image_topic"),
                "object_memory_path": LaunchConfiguration("object_memory_path"),
                "model_path": LaunchConfiguration("model_path"),
                "class_names": LaunchConfiguration("class_names"),
                "preprocess_mode": LaunchConfiguration("preprocess_mode"),
                "conf_threshold": LaunchConfiguration("conf_threshold"),
                "nms_threshold": LaunchConfiguration("nms_threshold"),
                "capture_count": LaunchConfiguration("capture_count"),
                "capture_interval_sec": LaunchConfiguration("capture_interval_sec"),
                "max_image_age_sec": LaunchConfiguration("max_image_age_sec"),
                "request_timeout_sec": LaunchConfiguration("request_timeout_sec"),
                "use_local_vlm": LaunchConfiguration("use_local_vlm"),
                "vlm_url": LaunchConfiguration("vlm_url"),
                "bailian_model": LaunchConfiguration("bailian_model"),
                "bailian_base_url": LaunchConfiguration("bailian_base_url"),
                "bailian_api_key_env": LaunchConfiguration("bailian_api_key_env"),
                "bailian_enable_thinking": LaunchConfiguration("bailian_enable_thinking"),
                "task_gated": LaunchConfiguration("task_gated"),
            }
        ],
    )

    return LaunchDescription(args + [node])

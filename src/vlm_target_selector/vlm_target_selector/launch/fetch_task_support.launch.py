#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


CUSTOM_MODEL = "/opt/xiaogua/models/yolo_model.bin"
CUSTOM_CLASSES = "person,cell phone,mouse,remote,book,bottle,cup,bowl,apple,banana,teddy bear,bag_wrapper,box"


def generate_launch_description():
    args = [
        DeclareLaunchArgument("rosbridge_url", default_value="ws://127.0.0.1:9090"),
        DeclareLaunchArgument("memory_path", default_value=""),
        DeclareLaunchArgument("image_topic", default_value="/camera/color/image_raw"),
        DeclareLaunchArgument("depth_topic", default_value="/camera/depth/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/depth/camera_info"),
        DeclareLaunchArgument("person_3d_detection_topic", default_value="/yolo_detector/detections"),
        DeclareLaunchArgument("person_3d_output_topic", default_value="/vision/target_point_local"),
        DeclareLaunchArgument("person_3d_target_class", default_value="person"),
        DeclareLaunchArgument("person_3d_debug_image_topic", default_value="/vision/person_distance_image"),
        DeclareLaunchArgument("source_image_width", default_value="640"),
        DeclareLaunchArgument("source_image_height", default_value="480"),
        DeclareLaunchArgument("vision_tf_parent_frame", default_value="camera_link"),
        DeclareLaunchArgument("vision_tf_child_frame", default_value="vision_target"),
        DeclareLaunchArgument("vision_tf_input_frame_mode", default_value="optical_to_camera_link"),
        DeclareLaunchArgument("yolo_model_path", default_value=CUSTOM_MODEL),
        DeclareLaunchArgument("yolo_class_names", default_value=CUSTOM_CLASSES),
        DeclareLaunchArgument("yolo_preprocess_mode", default_value="letterbox"),
        DeclareLaunchArgument("conf_threshold", default_value="0.07"),
        DeclareLaunchArgument("capture_count", default_value="5"),
        DeclareLaunchArgument("capture_interval_sec", default_value="0.25"),
        DeclareLaunchArgument("use_local_vlm", default_value="false"),
        DeclareLaunchArgument("vlm_url", default_value="http://127.0.0.1:8000/analyze"),
        DeclareLaunchArgument("use_local_llm", default_value="true"),
        DeclareLaunchArgument("llm_url", default_value="http://127.0.0.1:8000/analyze"),
        DeclareLaunchArgument("task_model", default_value="qwen3.6-flash"),
        DeclareLaunchArgument("bailian_model", default_value="qwen3-vl-plus"),
        DeclareLaunchArgument("bailian_base_url", default_value=""),
        DeclareLaunchArgument("bailian_api_key_env", default_value="DASHSCOPE_API_KEY"),
        DeclareLaunchArgument("bailian_enable_thinking", default_value="false"),
        DeclareLaunchArgument("vision_task_gated", default_value="false"),
        DeclareLaunchArgument(
            "speech_profile_path",
            default_value="/opt/xiaogua/legacy_ws/yahboomcar_ws/src/nav_pkg/config/voice_persona.json",
        ),
        DeclareLaunchArgument("speech_profile", default_value=""),
        DeclareLaunchArgument(
            "location_config_path",
            default_value="/opt/xiaogua/legacy_ws/yahboomcar_ws/src/nav_pkg/config/patrol_points.json",
        ),
    ]

    return LaunchDescription(args + [
        Node(
            package="vlm_target_selector",
            executable="task_understanding_node",
            name="task_understanding",
            output="screen",
            parameters=[{
                "use_local_llm": LaunchConfiguration("use_local_llm"),
                "llm_url": LaunchConfiguration("llm_url"),
                "bailian_model": LaunchConfiguration("task_model"),
                "bailian_base_url": LaunchConfiguration("bailian_base_url"),
                "bailian_api_key_env": LaunchConfiguration("bailian_api_key_env"),
                "bailian_enable_thinking": LaunchConfiguration("bailian_enable_thinking"),
                "speech_profile_path": LaunchConfiguration("speech_profile_path"),
                "speech_profile": LaunchConfiguration("speech_profile"),
                "location_config_path": LaunchConfiguration("location_config_path"),
            }],
        ),
        Node(
            package="vlm_target_selector",
            executable="object_memory_query_node",
            name="object_memory_query",
            output="screen",
            parameters=[{
                "memory_path": LaunchConfiguration("memory_path"),
                "use_llm_selection": True,
                "allow_rule_fallback": True,
                "use_local_llm": LaunchConfiguration("use_local_llm"),
                "llm_url": LaunchConfiguration("llm_url"),
                "bailian_model": LaunchConfiguration("task_model"),
                "bailian_base_url": LaunchConfiguration("bailian_base_url"),
                "bailian_api_key_env": LaunchConfiguration("bailian_api_key_env"),
                "bailian_enable_thinking": LaunchConfiguration("bailian_enable_thinking"),
            }],
        ),
        Node(
            package="vlm_target_selector",
            executable="target_confirm_node",
            name="target_confirm",
            output="screen",
            parameters=[{
                "image_topic": LaunchConfiguration("image_topic"),
                "depth_topic": LaunchConfiguration("depth_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "object_memory_path": LaunchConfiguration("memory_path"),
                "model_path": LaunchConfiguration("yolo_model_path"),
                "class_names": LaunchConfiguration("yolo_class_names"),
                "preprocess_mode": LaunchConfiguration("yolo_preprocess_mode"),
                "conf_threshold": LaunchConfiguration("conf_threshold"),
                "capture_count": LaunchConfiguration("capture_count"),
                "capture_interval_sec": LaunchConfiguration("capture_interval_sec"),
                "use_local_vlm": LaunchConfiguration("use_local_vlm"),
                "vlm_url": LaunchConfiguration("vlm_url"),
                "bailian_model": LaunchConfiguration("bailian_model"),
                "bailian_base_url": LaunchConfiguration("bailian_base_url"),
                "bailian_api_key_env": LaunchConfiguration("bailian_api_key_env"),
                "bailian_enable_thinking": LaunchConfiguration("bailian_enable_thinking"),
                "task_gated": LaunchConfiguration("vision_task_gated"),
            }],
        ),
        Node(
            package="vlm_target_selector",
            executable="memory_target_select_adapter_node",
            name="memory_target_select_adapter",
            output="screen",
            parameters=[{
                "memory_path": LaunchConfiguration("memory_path"),
            }],
        ),
        Node(
            package="vision_ros1_tf_bridge",
            executable="fetch_task_bridge_node",
            name="fetch_task_bridge",
            output="screen",
            parameters=[{
                "rosbridge_url": LaunchConfiguration("rosbridge_url"),
            }],
        ),
        Node(
            package="vision_ros1_tf_bridge",
            executable="selected_detection_bridge_node",
            name="selected_detection_bridge",
            output="screen",
            parameters=[{
                "rosbridge_url": LaunchConfiguration("rosbridge_url"),
                "input_topic": "/object_tracker/selected_detection",
                "ros1_output_topic": "/tracked_yolov8/detections",
            }],
        ),
        Node(
            package="vision_to_3d_local",
            executable="vision_to_3d_local_node",
            name="person_vision_to_3d",
            output="screen",
            parameters=[{
                "image_topic": LaunchConfiguration("image_topic"),
                "depth_topic": LaunchConfiguration("depth_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "detection_topic": LaunchConfiguration("person_3d_detection_topic"),
                "output_topic": LaunchConfiguration("person_3d_output_topic"),
                "target_class": LaunchConfiguration("person_3d_target_class"),
                "publish_debug_image": False,
                "debug_image_topic": LaunchConfiguration("person_3d_debug_image_topic"),
                "source_image_width": LaunchConfiguration("source_image_width"),
                "source_image_height": LaunchConfiguration("source_image_height"),
                "task_gated": LaunchConfiguration("vision_task_gated"),
            }],
        ),
        Node(
            package="vision_ros1_tf_bridge",
            executable="vision_tf_bridge_node",
            name="person_vision_tf_bridge",
            output="screen",
            parameters=[{
                "rosbridge_url": LaunchConfiguration("rosbridge_url"),
                "input_topic": LaunchConfiguration("person_3d_output_topic"),
                "parent_frame": LaunchConfiguration("vision_tf_parent_frame"),
                "child_frame": LaunchConfiguration("vision_tf_child_frame"),
                "input_frame_mode": LaunchConfiguration("vision_tf_input_frame_mode"),
                "publish_ros1_point": True,
                "ros1_point_topic": "/vision/person_point_camera_link",
            }],
        ),
    ])

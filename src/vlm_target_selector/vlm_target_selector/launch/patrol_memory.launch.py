from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


CUSTOM_MODEL = "/opt/xiaogua/models/yolo_model.bin"
CUSTOM_CLASSES = "person,cell phone,mouse,remote,book,bottle,cup,bowl,apple,banana,teddy bear,bag_wrapper,box"
PATROL_CLASSES = CUSTOM_CLASSES


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("start_yolo", default_value="true"),
        DeclareLaunchArgument("image_topic", default_value="/camera/color/image_raw"),
        DeclareLaunchArgument("depth_topic", default_value="/camera/depth/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/depth/camera_info"),
        DeclareLaunchArgument("target_classes", default_value=PATROL_CLASSES),
        DeclareLaunchArgument("yolo_model_path", default_value=CUSTOM_MODEL),
        DeclareLaunchArgument("yolo_class_names", default_value=CUSTOM_CLASSES),
        DeclareLaunchArgument("conf_threshold", default_value="0.07"),
        DeclareLaunchArgument("yolo_nms_threshold", default_value="0.7"),
        DeclareLaunchArgument("yolo_max_fps", default_value="15.0"),
        DeclareLaunchArgument("yolo_preprocess_mode", default_value="letterbox"),
        DeclareLaunchArgument("yolo_task_gated", default_value="false"),
        DeclareLaunchArgument("process_only_when_scanning", default_value="false"),
        DeclareLaunchArgument("capture_count", default_value="5"),
        DeclareLaunchArgument("capture_interval_sec", default_value="0.25"),
        DeclareLaunchArgument("marker_text_scale", default_value="0.10"),
        DeclareLaunchArgument("marker_text_z_offset", default_value="0.14"),
        DeclareLaunchArgument("memory_path", default_value=""),
        DeclareLaunchArgument("clear_memory_on_start", default_value="false"),
        DeclareLaunchArgument("use_local_vlm", default_value="true"),
        DeclareLaunchArgument("vlm_url", default_value="http://127.0.0.1:8000/analyze"),
        DeclareLaunchArgument("bailian_model", default_value="qwen3-vl-plus"),
        DeclareLaunchArgument("bailian_enable_thinking", default_value="false"),

        Node(
            condition=IfCondition(LaunchConfiguration("start_yolo")),
            package="yolo_detector",
            executable="yolo_detector_node",
            name="yolo_detector",
            output="screen",
            parameters=[{
                "image_topic": LaunchConfiguration("image_topic"),
                "target_classes": LaunchConfiguration("target_classes"),
                "model_path": LaunchConfiguration("yolo_model_path"),
                "class_names": LaunchConfiguration("yolo_class_names"),
                "conf_threshold": LaunchConfiguration("conf_threshold"),
                "nms_threshold": LaunchConfiguration("yolo_nms_threshold"),
                "preprocess_mode": LaunchConfiguration("yolo_preprocess_mode"),
                "max_fps": LaunchConfiguration("yolo_max_fps"),
                "task_gated": LaunchConfiguration("yolo_task_gated"),
            }],
        ),
        Node(
            package="vlm_target_selector",
            executable="patrol_scan_adapter_node",
            name="patrol_scan_adapter",
            output="screen",
            parameters=[{
                "image_topic": LaunchConfiguration("image_topic"),
                "depth_topic": LaunchConfiguration("depth_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "target_classes": LaunchConfiguration("target_classes"),
                "capture_count": LaunchConfiguration("capture_count"),
                "capture_interval_sec": LaunchConfiguration("capture_interval_sec"),
                "marker_text_scale": LaunchConfiguration("marker_text_scale"),
                "marker_text_z_offset": LaunchConfiguration("marker_text_z_offset"),
                "memory_path": LaunchConfiguration("memory_path"),
                "clear_memory_on_start": LaunchConfiguration("clear_memory_on_start"),
                "use_local_vlm": LaunchConfiguration("use_local_vlm"),
                "vlm_url": LaunchConfiguration("vlm_url"),
                "bailian_model": LaunchConfiguration("bailian_model"),
                "bailian_enable_thinking": LaunchConfiguration("bailian_enable_thinking"),
                "process_only_when_scanning": LaunchConfiguration("process_only_when_scanning"),
            }],
        ),
    ])

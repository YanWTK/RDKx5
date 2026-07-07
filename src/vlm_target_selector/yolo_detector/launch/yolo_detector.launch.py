from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


CUSTOM_MODEL = '/opt/xiaogua/models/yolo_model.bin'
CUSTOM_CLASSES = 'person,cell phone,mouse,remote,book,bottle,cup,bowl,apple,banana,teddy bear,bag_wrapper,box'


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('target_classes', default_value=CUSTOM_CLASSES),
        DeclareLaunchArgument('conf_threshold', default_value='0.07'),
        DeclareLaunchArgument('nms_threshold', default_value='0.7'),
        DeclareLaunchArgument('max_fps', default_value='15.0'),
        DeclareLaunchArgument('task_gated', default_value='false'),
        DeclareLaunchArgument('class_names', default_value=CUSTOM_CLASSES),
        DeclareLaunchArgument('preprocess_mode', default_value='letterbox'),
        DeclareLaunchArgument('model_path',
                              default_value=CUSTOM_MODEL),
        Node(
            package='yolo_detector',
            executable='yolo_detector_node',
            output='screen',
            parameters=[{
                'image_topic': LaunchConfiguration('image_topic'),
                'target_classes': LaunchConfiguration('target_classes'),
                'conf_threshold': LaunchConfiguration('conf_threshold'),
                'nms_threshold': LaunchConfiguration('nms_threshold'),
                'model_path': LaunchConfiguration('model_path'),
                'class_names': LaunchConfiguration('class_names'),
                'preprocess_mode': LaunchConfiguration('preprocess_mode'),
                'max_fps': LaunchConfiguration('max_fps'),
                'task_gated': LaunchConfiguration('task_gated'),
            }],
        ),
    ])

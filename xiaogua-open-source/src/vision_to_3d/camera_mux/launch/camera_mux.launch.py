from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('depth_image_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('usb_image_topic', default_value='/usb_camera/image_raw'),
        DeclareLaunchArgument('output_image_topic', default_value='/active_camera/image_raw'),
        DeclareLaunchArgument('vision_output_image_topic', default_value='/vision_camera/image_raw'),
        DeclareLaunchArgument('depth_input_topic', default_value='/camera/depth/image_raw'),
        DeclareLaunchArgument('vision_output_depth_topic', default_value='/vision_camera/depth/image_raw'),
        DeclareLaunchArgument('camera_info_input_topic', default_value='/camera/depth/camera_info'),
        DeclareLaunchArgument('vision_output_camera_info_topic', default_value='/vision_camera/depth/camera_info'),
        DeclareLaunchArgument('vision_active_topic', default_value='/yolo_detector/active'),
        DeclareLaunchArgument('initial_source', default_value='depth'),
        DeclareLaunchArgument('status_period_sec', default_value='1.0'),
        Node(
            package='camera_mux',
            executable='camera_mux_node',
            name='camera_mux',
            output='screen',
            parameters=[{
                'depth_image_topic': LaunchConfiguration('depth_image_topic'),
                'usb_image_topic': LaunchConfiguration('usb_image_topic'),
                'output_image_topic': LaunchConfiguration('output_image_topic'),
                'vision_output_image_topic': LaunchConfiguration('vision_output_image_topic'),
                'depth_input_topic': LaunchConfiguration('depth_input_topic'),
                'vision_output_depth_topic': LaunchConfiguration('vision_output_depth_topic'),
                'camera_info_input_topic': LaunchConfiguration('camera_info_input_topic'),
                'vision_output_camera_info_topic': LaunchConfiguration('vision_output_camera_info_topic'),
                'vision_active_topic': LaunchConfiguration('vision_active_topic'),
                'initial_source': LaunchConfiguration('initial_source'),
                'status_period_sec': LaunchConfiguration('status_period_sec'),
            }],
        ),
    ])

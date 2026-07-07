import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    config_file_path = os.path.join(
        get_package_prefix('hobot_usb_cam'),
        'lib/hobot_usb_cam/config/usb_camera_calibration.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'usb_camera_calibration_file_path',
            default_value=TextSubstitution(text=str(config_file_path)),
        ),
        DeclareLaunchArgument('usb_frame_id', default_value='usb_camera'),
        DeclareLaunchArgument('usb_framerate', default_value='30'),
        DeclareLaunchArgument('usb_image_height', default_value='480'),
        DeclareLaunchArgument('usb_image_width', default_value='960'),
        DeclareLaunchArgument('usb_io_method', default_value='mmap'),
        DeclareLaunchArgument('usb_pixel_format', default_value='mjpeg'),
        DeclareLaunchArgument('usb_video_device', default_value='/dev/video8'),
        DeclareLaunchArgument('usb_zero_copy', default_value='False'),
        DeclareLaunchArgument('image_topic', default_value='/usb_camera/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/usb_camera/camera_info'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('hobot_shm'),
                    'launch/hobot_shm.launch.py',
                )
            )
        ),
        Node(
            package='hobot_usb_cam',
            executable='hobot_usb_cam',
            name='usb_camera',
            output='screen',
            parameters=[{
                'camera_calibration_file_path': LaunchConfiguration(
                    'usb_camera_calibration_file_path'
                ),
                'frame_id': LaunchConfiguration('usb_frame_id'),
                'framerate': LaunchConfiguration('usb_framerate'),
                'image_height': LaunchConfiguration('usb_image_height'),
                'image_width': LaunchConfiguration('usb_image_width'),
                'io_method': LaunchConfiguration('usb_io_method'),
                'pixel_format': LaunchConfiguration('usb_pixel_format'),
                'video_device': LaunchConfiguration('usb_video_device'),
                'zero_copy': LaunchConfiguration('usb_zero_copy'),
            }],
            remappings=[
                ('image', LaunchConfiguration('image_topic')),
                ('camera_info', LaunchConfiguration('camera_info_topic')),
            ],
            arguments=['--ros-args', '--log-level', 'warn'],
        ),
    ])

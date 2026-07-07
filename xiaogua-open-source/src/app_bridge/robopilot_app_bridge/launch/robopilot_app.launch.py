from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path = PathJoinSubstitution(
        [FindPackageShare("robopilot_app_bridge"), "config", "robopilot_app.yaml"]
    )

    topics_glob_default = (
        "[/cmd_vel,/voltage,/odom,/scan,/cartographer_map,/map,"
        "/imu/imu_data,/diagnostics,/robot_status,/robot_pose,/voice_cmd,"
        "/voice_persona/set,/voice_persona/status,/move_base_simple/goal,"
        "/camera/rgb/image_raw,/camera/color/image_raw,/robopilot/mapping/control,"
        "/mode/status]"
    )
    services_glob_default = "[/mapping/start,/mapping/save,/mapping/stop,/mode/switch_to_mapping,/mode/switch_to_navigation,/mode/switch_to_patrol,/mode/get_status]"

    return LaunchDescription(
        [
            DeclareLaunchArgument("publish_mock_topics", default_value="true"),
            DeclareLaunchArgument("publish_mock_camera", default_value="true"),
            DeclareLaunchArgument("enable_ros1_bridge", default_value="true"),
            DeclareLaunchArgument("use_ros1_camera_stream", default_value="false"),
            DeclareLaunchArgument("rosbridge_address", default_value="0.0.0.0"),
            DeclareLaunchArgument("rosbridge_port", default_value="9090"),
            DeclareLaunchArgument("stream_port", default_value="8081"),
            DeclareLaunchArgument("stream_topic", default_value="/camera/rgb/image_raw"),
            DeclareLaunchArgument("camera_source_topic", default_value="/camera/color/image_raw"),
            DeclareLaunchArgument("ros1_bridge_url", default_value="ws://127.0.0.1:19090"),
            DeclareLaunchArgument("ros1_bridge_reconnect_sec", default_value="2.0"),
            DeclareLaunchArgument("ros1_topic_timeout_sec", default_value="2.0"),
            DeclareLaunchArgument(
                "ros1_carto_configuration_directory",
                default_value="/opt/xiaogua/runtime_ws/software/carto_ws/src/cartographer_ros/cartographer_ros/configuration_files",
            ),
            DeclareLaunchArgument("ros1_carto_configuration_basename", default_value="yahboomcar.lua"),
            DeclareLaunchArgument(
                "ros1_write_state_filename",
                default_value="/opt/xiaogua/runtime_ws/yahboomcar_ws/src/yahboomcar_nav/maps/app_map/cartographer_state.pbstream",
            ),
            DeclareLaunchArgument("topics_glob", default_value=topics_glob_default),
            DeclareLaunchArgument("services_glob", default_value=services_glob_default),
            DeclareLaunchArgument("params_glob", default_value=""),
            DeclareLaunchArgument(
                "save_dir",
                default_value="/opt/xiaogua/legacy_ws/yahboomcar_ws/src/yahboomcar_nav/maps/app_map",
            ),
            DeclareLaunchArgument("save_basename", default_value="cartographer_map"),
            Node(
                package="rosbridge_server",
                executable="rosbridge_websocket",
                name="rosbridge_websocket",
                output="screen",
                parameters=[
                    {
                        "address": LaunchConfiguration("rosbridge_address"),
                        "port": ParameterValue(LaunchConfiguration("rosbridge_port"), value_type=int),
                        "default_call_service_timeout": 5.0,
                        "call_services_in_new_thread": True,
                        "send_action_goals_in_new_thread": True,
                        "topics_glob": ParameterValue(
                            LaunchConfiguration("topics_glob"), value_type=str
                        ),
                        "services_glob": ParameterValue(
                            LaunchConfiguration("services_glob"), value_type=str
                        ),
                        "params_glob": ParameterValue(
                            LaunchConfiguration("params_glob"), value_type=str
                        ),
                    }
                ],
            ),
            Node(
                package="rosapi",
                executable="rosapi_node",
                name="rosapi",
                output="screen",
                parameters=[
                    {
                        "topics_glob": ParameterValue(
                            LaunchConfiguration("topics_glob"), value_type=str
                        ),
                        "services_glob": ParameterValue(
                            LaunchConfiguration("services_glob"), value_type=str
                        ),
                        "params_glob": ParameterValue(
                            LaunchConfiguration("params_glob"), value_type=str
                        ),
                    }
                ],
            ),
            Node(
                package="robopilot_app_bridge",
                executable="robopilot_mapping_service_node",
                name="robopilot_mapping_service",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_ros1_bridge")),
                parameters=[
                    config_path,
                    {
                        "save_dir": LaunchConfiguration("save_dir"),
                        "save_basename": LaunchConfiguration("save_basename"),
                        "map_frame": "map",
                        "map_width_cells": 240,
                        "map_height_cells": 240,
                        "map_resolution": 0.05,
                        "control_topic": "/robopilot/mapping/control",
                    },
                ],
            ),
            Node(
                package="robopilot_app_bridge",
                executable="robopilot_app_bridge_node",
                name="robopilot_app_bridge",
                output="screen",
                condition=UnlessCondition(LaunchConfiguration("enable_ros1_bridge")),
                parameters=[
                    config_path,
                    {
                        "publish_mock_topics": ParameterValue(
                            LaunchConfiguration("publish_mock_topics"), value_type=bool
                        ),
                        "save_dir": LaunchConfiguration("save_dir"),
                        "save_basename": LaunchConfiguration("save_basename"),
                    },
                ],
            ),
            Node(
                package="robopilot_app_bridge",
                executable="robopilot_mock_camera_node",
                name="robopilot_mock_camera",
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            LaunchConfiguration("publish_mock_camera"),
                            "' == 'true' and '",
                            LaunchConfiguration("enable_ros1_bridge"),
                            "' == 'false'",
                        ]
                    )
                ),
                parameters=[
                    config_path,
                    {
                        "topic": LaunchConfiguration("camera_source_topic"),
                    },
                ],
            ),
            Node(
                package="robopilot_app_bridge",
                executable="robopilot_mjpeg_server",
                name="robopilot_mjpeg_server",
                output="screen",
                parameters=[
                    config_path,
                    {
                        "port": ParameterValue(LaunchConfiguration("stream_port"), value_type=int),
                        "stream_topic": LaunchConfiguration("stream_topic"),
                        "source_topic": LaunchConfiguration("camera_source_topic"),
                    },
                ],
            ),
        ]
    )

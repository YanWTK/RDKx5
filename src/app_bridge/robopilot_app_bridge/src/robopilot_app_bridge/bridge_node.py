from __future__ import annotations

import json
import math
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from .common import (
    build_static_map,
    draw_blob,
    occupancy_grid_message,
    pose_to_cell,
    quaternion_from_yaw,
    save_occupancy_grid,
)


def _make_transform(
    parent: str,
    child: str,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    yaw: float = 0.0,
) -> TransformStamped:
    transform = TransformStamped()
    transform.header.frame_id = parent
    transform.child_frame_id = child
    transform.transform.translation.x = float(x)
    transform.transform.translation.y = float(y)
    transform.transform.translation.z = float(z)
    transform.transform.rotation = quaternion_from_yaw(yaw)
    return transform


class AppBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("robopilot_app_bridge")

        self.declare_parameter("publish_mock_topics", True)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("map_width_cells", 240)
        self.declare_parameter("map_height_cells", 240)
        self.declare_parameter("map_resolution", 0.05)
        self.declare_parameter("save_dir", "/opt/xiaogua/legacy_ws/yahboomcar_ws/src/yahboomcar_nav/maps/app_map")
        self.declare_parameter("save_basename", "cartographer_map")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("base_link_frame", "base_link")
        self.declare_parameter("laser_frame", "laser")
        self.declare_parameter("imu_frame", "imu_link")
        self.declare_parameter("camera_frame", "camera_rgb_frame")
        self.declare_parameter("initial_voltage", 12.4)
        self.declare_parameter("placeholder_map_message", "mock mapping layer active")

        self.publish_mock_topics = bool(self.get_parameter("publish_mock_topics").value)
        self.publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.map_width_cells = int(self.get_parameter("map_width_cells").value)
        self.map_height_cells = int(self.get_parameter("map_height_cells").value)
        self.map_resolution = float(self.get_parameter("map_resolution").value)
        self.save_dir = Path(str(self.get_parameter("save_dir").value))
        self.save_basename = str(self.get_parameter("save_basename").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.base_link_frame = str(self.get_parameter("base_link_frame").value)
        self.laser_frame = str(self.get_parameter("laser_frame").value)
        self.imu_frame = str(self.get_parameter("imu_frame").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.initial_voltage = float(self.get_parameter("initial_voltage").value)
        self.placeholder_map_message = str(self.get_parameter("placeholder_map_message").value)

        self.origin_x = -0.5 * self.map_width_cells * self.map_resolution
        self.origin_y = -0.5 * self.map_height_cells * self.map_resolution

        self._state_lock = threading.Lock()
        self._last_tick = time.monotonic()
        self._pose_x = 0.0
        self._pose_y = 0.0
        self._pose_yaw = 0.0
        self._cmd_linear_x = 0.0
        self._cmd_angular_z = 0.0
        self._last_voice_cmd = ""
        self._last_goal = None
        self._mapping_active = False
        self._voltage = self.initial_voltage
        self._latest_saved_map = ""
        self._trail = deque(maxlen=300)
        self._scan_phase = 0.0
        self._base_map = build_static_map(self.map_width_cells, self.map_height_cells)
        self._live_map = self._base_map.copy()

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        topic_qos = 10

        self._voltage_pub = self.create_publisher(Float32, "/voltage", topic_qos)
        self._odom_pub = self.create_publisher(Odometry, "/odom", topic_qos)
        self._scan_pub = self.create_publisher(LaserScan, "/scan", topic_qos)
        self._map_pub = self.create_publisher(OccupancyGrid, "/map", map_qos)
        self._cartographer_map_pub = self.create_publisher(
            OccupancyGrid, "/cartographer_map", map_qos
        )
        self._imu_pub = self.create_publisher(Imu, "/imu/imu_data", topic_qos)
        self._diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", topic_qos)
        self._status_pub = self.create_publisher(String, "/robot_status", topic_qos)

        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, topic_qos)
        self.create_subscription(String, "/voice_cmd", self._on_voice_cmd, topic_qos)
        self.create_subscription(PoseStamped, "/move_base_simple/goal", self._on_goal, topic_qos)

        self.create_service(Trigger, "/mapping/start", self._on_mapping_start)
        self.create_service(Trigger, "/mapping/save", self._on_mapping_save)
        self.create_service(Trigger, "/mapping/stop", self._on_mapping_stop)

        self._tf_broadcaster = TransformBroadcaster(self)
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_transforms()

        self._timer = self.create_timer(1.0 / self.publish_rate_hz, self._on_timer)
        self._publish_snapshot()
        self.get_logger().info(
            "Robopilot bridge ready: mock_topics=%s" % ("true" if self.publish_mock_topics else "false")
        )

    def _publish_static_transforms(self) -> None:
        transforms = [
            _make_transform(self.map_frame, self.odom_frame),
            _make_transform(self.base_frame, self.base_link_frame, 0.0, 0.0, 0.03),
            _make_transform(self.base_link_frame, self.laser_frame, 0.14, 0.0, 0.10),
            _make_transform(self.laser_frame, "laser_link", 0.0, 0.0, 0.0),
            _make_transform(self.base_link_frame, self.imu_frame, 0.0, 0.0, 0.12),
            _make_transform(self.base_link_frame, "camera_link", 0.08, 0.0, 0.15),
            _make_transform("camera_link", self.camera_frame, 0.0, 0.0, 0.0),
            _make_transform(self.camera_frame, "camera_color_frame", 0.0, 0.0, 0.0),
            _make_transform("camera_color_frame", "camera_color_optical_frame", 0.0, 0.0, 0.0),
            _make_transform(self.camera_frame, "camera_depth_frame", 0.0, 0.0, 0.0),
            _make_transform("camera_depth_frame", "camera_depth_optical_frame", 0.0, 0.0, 0.0),
            _make_transform(self.camera_frame, "camera_ir_frame", 0.0, 0.0, 0.0),
            _make_transform("camera_ir_frame", "camera_ir_optical_frame", 0.0, 0.0, 0.0),
            _make_transform(self.camera_frame, "camera_rgb_optical_frame", 0.0, 0.0, 0.0),
        ]
        self._static_tf_broadcaster.sendTransform(transforms)

    def _on_cmd_vel(self, msg: Twist) -> None:
        with self._state_lock:
            self._cmd_linear_x = float(msg.linear.x)
            self._cmd_angular_z = float(msg.angular.z)

    def _on_voice_cmd(self, msg: String) -> None:
        with self._state_lock:
            self._last_voice_cmd = str(msg.data)

    def _on_goal(self, msg: PoseStamped) -> None:
        with self._state_lock:
            self._last_goal = {
                "frame_id": msg.header.frame_id,
                "x": float(msg.pose.position.x),
                "y": float(msg.pose.position.y),
                "z": float(msg.pose.position.z),
            }

    def _on_mapping_start(self, request, response):
        with self._state_lock:
            self._mapping_active = True
            self._live_map = self._base_map.copy()
            self._trail.clear()
        response.success = True
        response.message = "mapping started"
        return response

    def _on_mapping_stop(self, request, response):
        with self._state_lock:
            self._mapping_active = False
        response.success = True
        response.message = "mapping stopped"
        return response

    def _on_mapping_save(self, request, response):
        with self._state_lock:
            saved = save_occupancy_grid(
                self._live_map,
                self.save_dir,
                self.save_basename,
                self.map_resolution,
                self.origin_x,
                self.origin_y,
            )
            self._latest_saved_map = str(saved)

        response.success = True
        response.message = f"saved map to {self._latest_saved_map}.yaml"
        return response

    def _integrate_motion(self, dt: float) -> None:
        if dt <= 0.0:
            return
        with self._state_lock:
            linear_x = self._cmd_linear_x
            angular_z = self._cmd_angular_z

            self._pose_yaw += angular_z * dt
            self._pose_yaw = math.atan2(math.sin(self._pose_yaw), math.cos(self._pose_yaw))
            self._pose_x += linear_x * math.cos(self._pose_yaw) * dt
            self._pose_y += linear_x * math.sin(self._pose_yaw) * dt

            x_min = self.origin_x + self.map_resolution
            y_min = self.origin_y + self.map_resolution
            x_max = self.origin_x + (self.map_width_cells - 1) * self.map_resolution
            y_max = self.origin_y + (self.map_height_cells - 1) * self.map_resolution
            self._pose_x = max(x_min, min(x_max, self._pose_x))
            self._pose_y = max(y_min, min(y_max, self._pose_y))

            speed = abs(linear_x) + abs(angular_z) * 0.25
            self._voltage = max(10.5, self._voltage - speed * dt * 0.05)

            if self._mapping_active:
                row, col = pose_to_cell(
                    self._pose_x,
                    self._pose_y,
                    self.origin_x,
                    self.origin_y,
                    self.map_resolution,
                    self.map_width_cells,
                    self.map_height_cells,
                )
                self._trail.append((row, col))
                self._live_map = self._base_map.copy()
                for trail_row, trail_col in self._trail:
                    draw_blob(self._live_map, trail_row, trail_col, 2, 0)
                front_x = self._pose_x + 0.9 * math.cos(self._pose_yaw)
                front_y = self._pose_y + 0.9 * math.sin(self._pose_yaw)
                front_row, front_col = pose_to_cell(
                    front_x,
                    front_y,
                    self.origin_x,
                    self.origin_y,
                    self.map_resolution,
                    self.map_width_cells,
                    self.map_height_cells,
                )
                draw_blob(self._live_map, front_row, front_col, 2, 100)

    def _publish_snapshot(self) -> None:
        now = self.get_clock().now().to_msg()

        status_data = self._build_status_payload()
        self._status_pub.publish(String(data=json.dumps(status_data, ensure_ascii=False)))

        diag = DiagnosticArray()
        diag.header.stamp = now
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK
        status.name = "robopilot_app_bridge/mock"
        status.message = "mock layer active" if self.publish_mock_topics else "bridge only"
        status.values = [
            KeyValue(key="mode", value="mock" if self.publish_mock_topics else "bridge"),
            KeyValue(key="mapping", value="active" if self._mapping_active else "idle"),
            KeyValue(key="voltage", value=f"{self._voltage:.2f}"),
            KeyValue(key="voice_cmd", value=self._last_voice_cmd or ""),
        ]
        diag.status = [status]
        self._diagnostics_pub.publish(diag)

        if self.publish_mock_topics:
            voltage = Float32()
            voltage.data = float(self._voltage)
            self._voltage_pub.publish(voltage)

            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = self.odom_frame
            odom.child_frame_id = self.base_frame
            odom.pose.pose.position.x = float(self._pose_x)
            odom.pose.pose.position.y = float(self._pose_y)
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation = quaternion_from_yaw(self._pose_yaw)
            odom.twist.twist.linear.x = float(self._cmd_linear_x)
            odom.twist.twist.angular.z = float(self._cmd_angular_z)
            self._odom_pub.publish(odom)

            scan = LaserScan()
            scan.header.stamp = now
            scan.header.frame_id = self.laser_frame
            scan.angle_min = -math.pi
            scan.angle_max = math.pi
            scan.angle_increment = 2.0 * math.pi / 180.0
            scan.scan_time = 0.1
            scan.time_increment = scan.scan_time / 180.0
            scan.range_min = 0.15
            scan.range_max = 4.0
            ranges = []
            self._scan_phase += 0.15
            for index in range(180):
                distance = 2.3 + 0.8 * math.sin(self._scan_phase + index * 0.12)
                if 80 <= index <= 100:
                    distance = min(distance, 1.0 + 0.2 * math.cos(self._scan_phase * 1.7))
                ranges.append(max(scan.range_min, min(scan.range_max, distance)))
            scan.ranges = ranges
            self._scan_pub.publish(scan)

            imu = Imu()
            imu.header.stamp = now
            imu.header.frame_id = self.imu_frame
            imu.orientation = quaternion_from_yaw(self._pose_yaw)
            imu.angular_velocity.z = float(self._cmd_angular_z)
            self._imu_pub.publish(imu)

            map_msg = occupancy_grid_message(
                self._base_map,
                self.map_frame,
                self.map_resolution,
                self.origin_x,
                self.origin_y,
                now,
            )
            cartographer_map_msg = occupancy_grid_message(
                self._live_map,
                self.map_frame,
                self.map_resolution,
                self.origin_x,
                self.origin_y,
                now,
            )
            self._map_pub.publish(map_msg)
            self._cartographer_map_pub.publish(cartographer_map_msg)

            tf_msg = TransformStamped()
            tf_msg.header.stamp = now
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = float(self._pose_x)
            tf_msg.transform.translation.y = float(self._pose_y)
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation = quaternion_from_yaw(self._pose_yaw)
            self._tf_broadcaster.sendTransform(tf_msg)

    def _build_status_payload(self) -> dict:
        with self._state_lock:
            payload = {
                "mode": "mock" if self.publish_mock_topics else "bridge",
                "mapping": "active" if self._mapping_active else "idle",
                "voltage": round(self._voltage, 2),
                "pose": {
                    "x": round(self._pose_x, 3),
                    "y": round(self._pose_y, 3),
                    "yaw": round(self._pose_yaw, 3),
                },
                "cmd_vel": {
                    "linear_x": round(self._cmd_linear_x, 3),
                    "angular_z": round(self._cmd_angular_z, 3),
                },
                "voice_cmd": self._last_voice_cmd,
                "goal": self._last_goal,
                "latest_saved_map": self._latest_saved_map,
                "message": self.placeholder_map_message,
            }
        return payload

    def _on_timer(self) -> None:
        now = time.monotonic()
        dt = now - self._last_tick
        self._last_tick = now
        self._integrate_motion(dt)
        self._publish_snapshot()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AppBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

from __future__ import annotations

import array
import json
import math
import threading
import time
import uuid
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from websockets.sync.client import connect as websocket_connect

from .common import (
    build_static_map,
    draw_blob,
    occupancy_grid_message,
    pose_to_cell,
    quaternion_from_yaw,
    save_occupancy_grid,
)


@dataclass
class _RemoteMessage:
    msg: Any
    received_at: float


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


def _jsonable(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): _jsonable(val) for key, val in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    if isinstance(value, array.array):
        return list(value)
    if isinstance(value, np.generic):
        return value.item()
    return value


def _message_to_jsonable_dict(msg: Any) -> dict[str, Any]:
    return _jsonable(message_to_ordereddict(msg))


def _message_type_name(msg_type: type) -> str:
    package = str(msg_type.__module__.split(".")[0])
    return f"{package}/{msg_type.__name__}"


def _diagnostic_array_from_payload(payload: dict[str, Any]) -> DiagnosticArray:
    message = DiagnosticArray()
    header = payload.get("header")
    if isinstance(header, dict):
        set_message_fields(message.header, header, expand_time_now=True)
    statuses = []
    for item in payload.get("status", []):
        status = DiagnosticStatus()
        status.level = bytes([int(item.get("level", 0)) & 0xFF])
        status.name = str(item.get("name", ""))
        status.message = str(item.get("message", ""))
        status.hardware_id = str(item.get("hardware_id", ""))
        values = []
        for kv in item.get("values", []):
            key_value = KeyValue()
            key_value.key = str(kv.get("key", ""))
            key_value.value = str(kv.get("value", ""))
            values.append(key_value)
        status.values = values
        statuses.append(status)
    message.status = statuses
    return message


def _message_from_payload(msg_type: type, payload: dict[str, Any]) -> Any:
    if msg_type is DiagnosticArray:
        return _diagnostic_array_from_payload(payload)
    message = msg_type()
    set_message_fields(message, payload, expand_header_auto=True, expand_time_now=True)
    return message


class RosbridgeClient:
    def __init__(
        self,
        url: str,
        logger,
        reconnect_delay_sec: float = 2.0,
        connect_timeout_sec: float = 5.0,
    ) -> None:
        self.url = url
        self._logger = logger
        self._reconnect_delay_sec = max(0.2, float(reconnect_delay_sec))
        self._connect_timeout_sec = max(0.5, float(connect_timeout_sec))
        self._subscriptions: dict[str, tuple[type, Any]] = {}
        self._publish_topics: dict[str, type] = {}
        self._service_waiters: dict[str, tuple[threading.Event, dict[str, Any]]] = {}
        self._send_lock = threading.Lock()
        self._registry_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._connected_event = threading.Event()
        self._ws = None
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def is_connected(self) -> bool:
        return self._connected_event.is_set()

    def close(self) -> None:
        self._stop_event.set()
        ws = self._ws
        self._ws = None
        if ws is not None:
            try:
                ws.close()
            except Exception:
                pass

    def register_subscription(self, topic: str, msg_type: type, callback) -> None:
        with self._registry_lock:
            self._subscriptions[topic] = (msg_type, callback)
        self._send_if_connected(
            {"op": "subscribe", "topic": topic, "type": _message_type_name(msg_type)}
        )

    def register_publish_topic(self, topic: str, msg_type: type) -> None:
        with self._registry_lock:
            self._publish_topics[topic] = msg_type
        self._send_if_connected(
            {"op": "advertise", "topic": topic, "type": _message_type_name(msg_type)}
        )

    def publish(self, topic: str, msg: Any) -> bool:
        payload = _message_to_jsonable_dict(msg)
        return self._send(
            {
                "op": "publish",
                "topic": topic,
                "msg": payload,
            }
        )

    def call_service(
        self, service: str, args: dict[str, Any], timeout_sec: float = 5.0
    ) -> tuple[bool, dict[str, Any] | None, str]:
        request_id = f"{service}:{uuid.uuid4().hex}"
        event = threading.Event()
        response_holder = {"response": None}
        with self._registry_lock:
            self._service_waiters[request_id] = (event, response_holder)

        if not self._connected_event.wait(timeout=min(timeout_sec, self._connect_timeout_sec)):
            with self._registry_lock:
                self._service_waiters.pop(request_id, None)
            return False, None, "rosbridge not connected"

        if not self._send(
            {
                "op": "call_service",
                "service": service,
                "args": _jsonable(args),
                "id": request_id,
            }
        ):
            with self._registry_lock:
                self._service_waiters.pop(request_id, None)
            return False, None, "failed to send service request"

        if not event.wait(timeout=timeout_sec):
            with self._registry_lock:
                self._service_waiters.pop(request_id, None)
            return False, None, f"timeout calling {service}"

        response = response_holder["response"] or {}
        result = bool(response.get("result", False))
        values = response.get("values") or {}
        message = str(response.get("msg") or response.get("error") or "")
        return result, values, message

    def _run(self) -> None:
        while not self._stop_event.is_set():
            try:
                ws = websocket_connect(
                    self.url,
                    open_timeout=self._connect_timeout_sec,
                    ping_interval=None,
                    max_size=None,
                    proxy=None,
                )
                self._ws = ws
                self._connected_event.set()
                self._logger.info(f"connected to ROS1 rosbridge at {self.url}")
                self._flush_registry()

                while not self._stop_event.is_set():
                    try:
                        raw = ws.recv(timeout=1.0)
                    except TimeoutError:
                        continue
                    if not raw:
                        continue
                    self._handle_message(raw)
            except Exception as exc:
                self._logger.warn(f"rosbridge connection error: {exc}")
            finally:
                self._connected_event.clear()
                ws = self._ws
                self._ws = None
                if ws is not None:
                    try:
                        ws.close()
                    except Exception:
                        pass
                if not self._stop_event.is_set():
                    time.sleep(self._reconnect_delay_sec)

    def _flush_registry(self) -> None:
        with self._registry_lock:
            subscriptions = list(self._subscriptions.items())
            publish_topics = list(self._publish_topics.items())
        for topic, (msg_type, _) in subscriptions:
            self._send_if_connected(
                {"op": "subscribe", "topic": topic, "type": _message_type_name(msg_type)}
            )
        for topic, msg_type in publish_topics:
            self._send_if_connected(
                {"op": "advertise", "topic": topic, "type": _message_type_name(msg_type)}
            )

    def _handle_message(self, raw: str) -> None:
        try:
            message = json.loads(raw)
        except json.JSONDecodeError:
            return

        op = message.get("op")
        if op == "publish":
            topic = str(message.get("topic", ""))
            payload = message.get("msg") or {}
            with self._registry_lock:
                subscription = self._subscriptions.get(topic)
            if subscription is not None:
                _, callback = subscription
                try:
                    callback(topic, payload)
                except Exception as exc:
                    self._logger.warn(f"rosbridge topic callback failed for {topic}: {exc}")
        elif op == "service_response":
            request_id = str(message.get("id", ""))
            with self._registry_lock:
                waiter = self._service_waiters.pop(request_id, None)
            if waiter is not None:
                event, response_holder = waiter
                response_holder["response"] = message
                event.set()

    def _send_if_connected(self, payload: dict[str, Any]) -> bool:
        if not self.is_connected():
            return False
        return self._send(payload)

    def _send(self, payload: dict[str, Any]) -> bool:
        ws = self._ws
        if ws is None:
            return False
        try:
            data = json.dumps(payload, ensure_ascii=False)
        except TypeError:
            data = json.dumps(_jsonable(payload), ensure_ascii=False)
        with self._send_lock:
            try:
                ws.send(data)
            except Exception as exc:
                self._logger.warn(f"rosbridge send failed: {exc}")
                return False
        return True


class Ros1BridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("robopilot_ros1_bridge")

        self.declare_parameter("publish_mock_topics", True)
        self.declare_parameter("enable_ros1_bridge", True)
        self.declare_parameter("ros1_bridge_url", "ws://127.0.0.1:19090")
        self.declare_parameter("ros1_bridge_reconnect_sec", 2.0)
        self.declare_parameter("ros1_topic_timeout_sec", 2.0)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("map_width_cells", 240)
        self.declare_parameter("map_height_cells", 240)
        self.declare_parameter("map_resolution", 0.05)
        self.declare_parameter("save_dir", "/opt/xiaogua/legacy_ws/yahboomcar_ws/src/yahboomcar_nav/maps/app_map")
        self.declare_parameter("save_basename", "cartographer_map")
        self.declare_parameter(
            "ros1_carto_configuration_directory",
            "/opt/xiaogua/runtime_ws/software/carto_ws/src/cartographer_ros/cartographer_ros/configuration_files",
        )
        self.declare_parameter("ros1_carto_configuration_basename", "yahboomcar.lua")
        self.declare_parameter(
            "ros1_write_state_filename",
            "/opt/xiaogua/runtime_ws/yahboomcar_ws/src/yahboomcar_nav/maps/app_map/cartographer_state.pbstream",
        )
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("base_link_frame", "base_link")
        self.declare_parameter("laser_frame", "laser")
        self.declare_parameter("imu_frame", "imu_link")
        self.declare_parameter("camera_frame", "camera_rgb_frame")
        self.declare_parameter("initial_voltage", 12.4)
        self.declare_parameter("placeholder_map_message", "ros1 bridge active")

        self.publish_mock_topics = bool(self.get_parameter("publish_mock_topics").value)
        self.enable_ros1_bridge = bool(self.get_parameter("enable_ros1_bridge").value)
        self.ros1_bridge_url = str(self.get_parameter("ros1_bridge_url").value)
        self.ros1_bridge_reconnect_sec = float(self.get_parameter("ros1_bridge_reconnect_sec").value)
        self.ros1_topic_timeout_sec = float(self.get_parameter("ros1_topic_timeout_sec").value)
        self.publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.map_width_cells = int(self.get_parameter("map_width_cells").value)
        self.map_height_cells = int(self.get_parameter("map_height_cells").value)
        self.map_resolution = float(self.get_parameter("map_resolution").value)
        self.save_dir = Path(str(self.get_parameter("save_dir").value))
        self.save_basename = str(self.get_parameter("save_basename").value)
        self.ros1_carto_configuration_directory = str(
            self.get_parameter("ros1_carto_configuration_directory").value
        )
        self.ros1_carto_configuration_basename = str(
            self.get_parameter("ros1_carto_configuration_basename").value
        )
        self.ros1_write_state_filename = str(self.get_parameter("ros1_write_state_filename").value)
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
        self._remote_lock = threading.Lock()
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
        self._remote_messages: dict[str, _RemoteMessage] = {}
        self._remote_robot_status = ""
        self._ros1_trajectory_id: int | None = None

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

        self._bridge_client: RosbridgeClient | None = None
        if self.enable_ros1_bridge:
            self._start_ros1_bridge()

        self._timer = self.create_timer(1.0 / self.publish_rate_hz, self._on_timer)
        self._publish_snapshot()
        self.get_logger().info(
            "Robopilot ROS1 bridge ready: bridge=%s mock=%s"
            % ("true" if self.enable_ros1_bridge else "false", "true" if self.publish_mock_topics else "false")
        )

    def _start_ros1_bridge(self) -> None:
        client = RosbridgeClient(
            self.ros1_bridge_url,
            self.get_logger(),
            reconnect_delay_sec=self.ros1_bridge_reconnect_sec,
        )
        self._bridge_client = client

        bridge_topics = {
            "/voltage": Float32,
            "/odom": Odometry,
            "/scan": LaserScan,
            "/map": OccupancyGrid,
            "/cartographer_map": OccupancyGrid,
            "/imu/imu_data": Imu,
            "/diagnostics": DiagnosticArray,
            "/robot_status": String,
        }
        for topic, msg_type in bridge_topics.items():
            client.register_subscription(topic, msg_type, self._on_ros1_topic)

        for topic, msg_type in {
            "/cmd_vel": Twist,
            "/voice_cmd": String,
            "/move_base_simple/goal": PoseStamped,
        }.items():
            client.register_publish_topic(topic, msg_type)

    def _on_ros1_topic(self, topic: str, payload: dict[str, Any]) -> None:
        message_type = {
            "/voltage": Float32,
            "/odom": Odometry,
            "/scan": LaserScan,
            "/map": OccupancyGrid,
            "/cartographer_map": OccupancyGrid,
            "/imu/imu_data": Imu,
            "/diagnostics": DiagnosticArray,
            "/robot_status": String,
        }.get(topic)
        if message_type is None:
            return

        try:
            message = _message_from_payload(message_type, payload)
        except Exception as exc:
            self.get_logger().warn(f"failed to decode ROS1 topic {topic}: {exc}")
            return

        with self._remote_lock:
            self._remote_messages[topic] = _RemoteMessage(message, time.monotonic())
            if topic == "/robot_status":
                self._remote_robot_status = str(getattr(message, "data", ""))

    def _remote_message(self, topic: str) -> Any | None:
        with self._remote_lock:
            cached = self._remote_messages.get(topic)
        if cached is None:
            return None
        if topic == "/robot_status":
            return cached.msg
        if (time.monotonic() - cached.received_at) <= self.ros1_topic_timeout_sec:
            return cached.msg
        return None

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
        if self._bridge_client is not None:
            self._bridge_client.publish("/cmd_vel", msg)

    def _on_voice_cmd(self, msg: String) -> None:
        with self._state_lock:
            self._last_voice_cmd = str(msg.data)
        if self._bridge_client is not None:
            self._bridge_client.publish("/voice_cmd", msg)

    def _on_goal(self, msg: PoseStamped) -> None:
        with self._state_lock:
            self._last_goal = {
                "frame_id": msg.header.frame_id,
                "x": float(msg.pose.position.x),
                "y": float(msg.pose.position.y),
                "z": float(msg.pose.position.z),
            }
        if self._bridge_client is not None:
            goal_msg = PoseStamped()
            goal_msg.header = msg.header
            goal_msg.pose = msg.pose
            if goal_msg.header.stamp.sec == 0 and goal_msg.header.stamp.nanosec == 0:
                goal_msg.header.stamp = self.get_clock().now().to_msg()
            self._bridge_client.publish("/move_base_simple/goal", goal_msg)

    def _call_mode_service(self, service_name: str, timeout_sec: float = 15.0) -> tuple[bool, str]:
        """Call a mode manager service on ROS1 via rosbridge."""
        if self._bridge_client is None or not self._bridge_client.is_connected():
            return False, "rosbridge not connected"
        ok, values, message = self._bridge_client.call_service(service_name, {}, timeout_sec=timeout_sec)
        remote_message = str((values or {}).get("message", "") or "") if values else ""
        return ok, remote_message or message

    def _on_mapping_start(self, request, response):
        ok, message = self._call_mode_service("/mode/switch_to_mapping", timeout_sec=15.0)

        with self._state_lock:
            self._mapping_active = True
            self._live_map = self._base_map.copy()
            self._trail.clear()

        response.success = True
        if ok and message:
            response.message = f"mapping started: {message}"
        elif ok:
            response.message = "mapping started"
        else:
            response.message = f"mapping started (mode switch note: {message})" if message else "mapping started"
        return response

    def _on_mapping_stop(self, request, response):
        ok, message = self._call_mode_service("/mode/switch_to_navigation", timeout_sec=20.0)

        with self._state_lock:
            self._mapping_active = False

        response.success = True
        if ok and message:
            response.message = f"mapping stopped: {message}"
        elif ok:
            response.message = "mapping stopped"
        else:
            response.message = f"mapping stopped (mode switch note: {message})" if message else "mapping stopped"
        return response

    def _latest_grid_for_save(self) -> np.ndarray:
        remote_grid = self._remote_message("/cartographer_map")
        if remote_grid is not None:
            try:
                return np.array(remote_grid.data, dtype=np.int8).reshape(
                    int(remote_grid.info.height), int(remote_grid.info.width)
                )
            except Exception:
                pass
        remote_map = self._remote_message("/map")
        if remote_map is not None:
            try:
                return np.array(remote_map.data, dtype=np.int8).reshape(
                    int(remote_map.info.height), int(remote_map.info.width)
                )
            except Exception:
                pass
        with self._state_lock:
            return self._live_map.copy()

    def _on_mapping_save(self, request, response):
        # Save map locally as backup
        saved = save_occupancy_grid(
            self._latest_grid_for_save(),
            self.save_dir,
            self.save_basename,
            self.map_resolution,
            self.origin_x,
            self.origin_y,
        )
        with self._state_lock:
            self._latest_saved_map = str(saved)

        # Switch to navigation mode (mode manager will save map + start nav)
        ok, message = self._call_mode_service("/mode/switch_to_navigation", timeout_sec=20.0)

        with self._state_lock:
            self._mapping_active = False

        response.success = True
        if ok and message:
            response.message = f"saved map to {self._latest_saved_map}.yaml; {message}"
        elif ok:
            response.message = f"saved map to {self._latest_saved_map}.yaml; switched to navigation"
        else:
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

    def _build_status_payload(self) -> dict[str, Any]:
        remote_odom = self._remote_message("/odom")
        remote_voltage = self._remote_message("/voltage")
        remote_status = self._remote_message("/robot_status")
        remote_cartographer_map = self._remote_message("/cartographer_map")
        remote_map = self._remote_message("/map")
        with self._state_lock:
            pose = {
                "x": round(self._pose_x, 3),
                "y": round(self._pose_y, 3),
                "yaw": round(self._pose_yaw, 3),
            }
            if remote_odom is not None:
                pose = {
                    "x": round(float(remote_odom.pose.pose.position.x), 3),
                    "y": round(float(remote_odom.pose.pose.position.y), 3),
                    "yaw": round(
                        math.atan2(
                            2.0
                            * (
                                remote_odom.pose.pose.orientation.w
                                * remote_odom.pose.pose.orientation.z
                                + remote_odom.pose.pose.orientation.x
                                * remote_odom.pose.pose.orientation.y
                            ),
                            1.0
                            - 2.0
                            * (
                                remote_odom.pose.pose.orientation.y
                                * remote_odom.pose.pose.orientation.y
                                + remote_odom.pose.pose.orientation.z
                                * remote_odom.pose.pose.orientation.z
                            ),
                        ),
                        3,
                    ),
                }

            voltage = self._voltage
            if remote_voltage is not None:
                voltage = float(remote_voltage.data)

            payload = {
                "mode": "ros1_bridge" if self.enable_ros1_bridge else "mock",
                "bridge_connected": self._bridge_client.is_connected()
                if self._bridge_client is not None
                else False,
                "bridge_url": self.ros1_bridge_url if self.enable_ros1_bridge else "",
                "mapping": "active" if self._mapping_active else "idle",
                "voltage": round(voltage, 2),
                "pose": pose,
                "cmd_vel": {
                    "linear_x": round(self._cmd_linear_x, 3),
                    "angular_z": round(self._cmd_angular_z, 3),
                },
                "voice_cmd": self._last_voice_cmd,
                "goal": self._last_goal,
                "latest_saved_map": self._latest_saved_map,
                "message": self.placeholder_map_message,
                "sources": {
                    "odom": "ros1" if remote_odom is not None else "mock",
                    "voltage": "ros1" if remote_voltage is not None else "mock",
                    "map": "ros1" if remote_map is not None else "mock",
                    "cartographer_map": "ros1" if remote_cartographer_map is not None else "mock",
                },
            }
            if self._remote_robot_status:
                payload["ros1_robot_status"] = self._remote_robot_status
        return payload

    def _build_diagnostics_message(self, stamp) -> DiagnosticArray:
        remote_diag = self._remote_message("/diagnostics")
        if remote_diag is not None:
            return remote_diag

        diag = DiagnosticArray()
        diag.header.stamp = stamp
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK
        status.name = "robopilot_app_bridge/ros1_bridge"
        if self.enable_ros1_bridge and self._bridge_client is not None:
            status.message = (
                "ros1 bridge connected" if self._bridge_client.is_connected() else "ros1 bridge reconnecting"
            )
        else:
            status.message = "mock layer active"
        status.values = [
            KeyValue(
                key="mode",
                value="ros1_bridge" if self.enable_ros1_bridge else "mock",
            ),
            KeyValue(
                key="mapping",
                value="active" if self._mapping_active else "idle",
            ),
            KeyValue(key="voltage", value=f"{self._voltage:.2f}"),
            KeyValue(key="voice_cmd", value=self._last_voice_cmd or ""),
            KeyValue(
                key="bridge_connected",
                value="true" if (self._bridge_client is not None and self._bridge_client.is_connected()) else "false",
            ),
        ]
        diag.status = [status]
        return diag

    def _publish_snapshot(self) -> None:
        now = self.get_clock().now().to_msg()

        status_data = self._build_status_payload()
        self._status_pub.publish(String(data=json.dumps(status_data, ensure_ascii=False)))
        self._diagnostics_pub.publish(self._build_diagnostics_message(now))

        remote_voltage = self._remote_message("/voltage")
        if remote_voltage is not None:
            self._voltage_pub.publish(remote_voltage)
        elif self.publish_mock_topics:
            voltage = Float32()
            voltage.data = float(self._voltage)
            self._voltage_pub.publish(voltage)

        remote_odom = self._remote_message("/odom")
        if remote_odom is not None:
            self._odom_pub.publish(remote_odom)
        elif self.publish_mock_topics:
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

        remote_scan = self._remote_message("/scan")
        if remote_scan is not None:
            self._scan_pub.publish(remote_scan)
        elif self.publish_mock_topics:
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

        remote_imu = self._remote_message("/imu/imu_data")
        if remote_imu is not None:
            self._imu_pub.publish(remote_imu)
        elif self.publish_mock_topics:
            imu = Imu()
            imu.header.stamp = now
            imu.header.frame_id = self.imu_frame
            imu.orientation = quaternion_from_yaw(self._pose_yaw)
            imu.angular_velocity.z = float(self._cmd_angular_z)
            self._imu_pub.publish(imu)

        remote_map = self._remote_message("/map")
        if remote_map is not None:
            self._map_pub.publish(remote_map)
        elif self.publish_mock_topics:
            map_msg = occupancy_grid_message(
                self._base_map,
                self.map_frame,
                self.map_resolution,
                self.origin_x,
                self.origin_y,
                now,
            )
            self._map_pub.publish(map_msg)

        remote_cartographer_map = self._remote_message("/cartographer_map")
        if remote_cartographer_map is not None:
            self._cartographer_map_pub.publish(remote_cartographer_map)
        elif self.publish_mock_topics:
            cartographer_map_msg = occupancy_grid_message(
                self._live_map,
                self.map_frame,
                self.map_resolution,
                self.origin_x,
                self.origin_y,
                now,
            )
            self._cartographer_map_pub.publish(cartographer_map_msg)

        if self.publish_mock_topics:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = now
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = float(self._pose_x)
            tf_msg.transform.translation.y = float(self._pose_y)
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation = quaternion_from_yaw(self._pose_yaw)
            self._tf_broadcaster.sendTransform(tf_msg)

    def _on_timer(self) -> None:
        now = time.monotonic()
        dt = now - self._last_tick
        self._last_tick = now
        self._integrate_motion(dt)
        self._publish_snapshot()

    def destroy_node(self) -> bool:
        if self._bridge_client is not None:
            self._bridge_client.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Ros1BridgeNode()
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

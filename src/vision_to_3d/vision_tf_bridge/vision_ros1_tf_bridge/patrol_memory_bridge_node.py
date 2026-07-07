#!/usr/bin/env python3
"""Bridge patrol memory topics between ROS2 and ROS1 rosbridge websocket."""

import json
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

try:
    import websocket
except ImportError:
    print("ERROR: websocket-client not installed. pip3 install websocket-client")
    raise


class PatrolMemoryBridge(Node):
    def __init__(self):
        super().__init__('patrol_memory_bridge')

        self.declare_parameter('rosbridge_url', 'ws://127.0.0.1:9090')
        self.declare_parameter('reconnect_interval', 2.0)
        self.declare_parameter('scan_cmd_topic', '/patrol_scan_cmd')
        self.declare_parameter('scan_done_topic', '/patrol_scan_done')
        self.declare_parameter('marker_topic', '/semantic_object_markers')
        self.declare_parameter('bridge_markers', True)

        self.url = self.get_parameter('rosbridge_url').value
        self.reconnect_interval = float(self.get_parameter('reconnect_interval').value)
        self.scan_cmd_topic = self.get_parameter('scan_cmd_topic').value
        self.scan_done_topic = self.get_parameter('scan_done_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.bridge_markers = self._as_bool(self.get_parameter('bridge_markers').value)

        self.ws = None
        self.ws_lock = threading.RLock()
        self.connected = False
        self._advertised = False
        self._subscribed = False

        self._scan_cmd_pub = self.create_publisher(String, self.scan_cmd_topic, 10)
        self.create_subscription(String, self.scan_done_topic, self._on_scan_done, 10)
        if self.bridge_markers:
            self.create_subscription(MarkerArray, self.marker_topic, self._on_markers, 10)

        threading.Thread(target=self._io_loop, daemon=True).start()
        self.get_logger().info(
            f'patrol_memory_bridge started rosbridge={self.url} '
            f'cmd={self.scan_cmd_topic} done={self.scan_done_topic} '
            f'markers={self.marker_topic if self.bridge_markers else "<disabled>"}'
        )

    def _io_loop(self):
        while rclpy.ok():
            try:
                self._connect()
                while rclpy.ok() and self.connected:
                    raw = self.ws.recv()
                    if not raw:
                        continue
                    self._handle_rosbridge_message(raw)
            except Exception as exc:
                self.connected = False
                self._advertised = False
                self._subscribed = False
                self.get_logger().warn(f'rosbridge loop failed: {exc}')
                time.sleep(self.reconnect_interval)

    def _connect(self):
        with self.ws_lock:
            if self.ws:
                try:
                    self.ws.close()
                except Exception:
                    pass
            self.ws = websocket.WebSocket()
            self.ws.settimeout(5)
            self.ws.connect(self.url)
            self.ws.settimeout(None)
            self.connected = True
            self._advertised = False
            self._subscribed = False
            self._advertise_and_subscribe()
            self.get_logger().info(f'connected to ROS1 rosbridge: {self.url}')

    def _advertise_and_subscribe(self):
        if not (self.ws and self.connected):
            return
        if not self._advertised:
            self.ws.send(json.dumps({
                'op': 'advertise',
                'topic': self.scan_done_topic,
                'type': 'std_msgs/String',
            }))
            if self.bridge_markers:
                self.ws.send(json.dumps({
                    'op': 'advertise',
                    'topic': self.marker_topic,
                    'type': 'visualization_msgs/MarkerArray',
                }))
            self._advertised = True
        if not self._subscribed:
            self.ws.send(json.dumps({
                'op': 'subscribe',
                'topic': self.scan_cmd_topic,
                'type': 'std_msgs/String',
            }))
            self._subscribed = True

    def _handle_rosbridge_message(self, raw: str):
        try:
            body = json.loads(raw)
        except Exception:
            return
        if body.get('op') != 'publish':
            return
        if body.get('topic') != self.scan_cmd_topic:
            return
        data = body.get('msg', {}).get('data', '')
        msg = String()
        msg.data = str(data)
        self._scan_cmd_pub.publish(msg)
        self.get_logger().info(f'bridged ROS1 scan cmd to ROS2: {msg.data}')

    def _on_scan_done(self, msg: String):
        self._publish_ros1(self.scan_done_topic, 'std_msgs/String', {'data': msg.data})

    def _on_markers(self, msg: MarkerArray):
        if not self.bridge_markers:
            return
        marker_msg = {'markers': [_marker_to_dict(marker) for marker in msg.markers]}
        self._publish_ros1(self.marker_topic, 'visualization_msgs/MarkerArray', marker_msg)

    def _publish_ros1(self, topic: str, msg_type: str, msg: dict):
        try:
            with self.ws_lock:
                if not (self.ws and self.connected):
                    return
                self._advertise_and_subscribe()
                self.ws.send(json.dumps({
                    'op': 'publish',
                    'topic': topic,
                    'type': msg_type,
                    'msg': msg,
                }))
        except Exception as exc:
            self.connected = False
            self._advertised = False
            self._subscribed = False
            self.get_logger().warn(f'publish to ROS1 failed: {exc}')

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    def destroy_node(self):
        with self.ws_lock:
            if self.ws:
                try:
                    self.ws.close()
                except Exception:
                    pass
        super().destroy_node()


def _marker_to_dict(marker) -> dict:
    return {
        'header': _header_to_dict(marker.header),
        'ns': marker.ns,
        'id': int(marker.id),
        'type': int(marker.type),
        'action': int(marker.action),
        'pose': {
            'position': _point_to_dict(marker.pose.position),
            'orientation': _quat_to_dict(marker.pose.orientation),
        },
        'scale': _vector3_to_dict(marker.scale),
        'color': _color_to_dict(marker.color),
        'lifetime': {
            'secs': int(marker.lifetime.sec),
            'nsecs': int(marker.lifetime.nanosec),
        },
        'frame_locked': bool(marker.frame_locked),
        'points': [_point_to_dict(point) for point in marker.points],
        'colors': [_color_to_dict(color) for color in marker.colors],
        'text': marker.text,
        'mesh_resource': marker.mesh_resource,
        'mesh_use_embedded_materials': bool(marker.mesh_use_embedded_materials),
    }


def _header_to_dict(header) -> dict:
    return {
        'seq': 0,
        'stamp': {
            'secs': int(header.stamp.sec),
            'nsecs': int(header.stamp.nanosec),
        },
        'frame_id': header.frame_id,
    }


def _point_to_dict(point) -> dict:
    return {'x': float(point.x), 'y': float(point.y), 'z': float(point.z)}


def _quat_to_dict(quat) -> dict:
    return {
        'x': float(quat.x),
        'y': float(quat.y),
        'z': float(quat.z),
        'w': float(quat.w),
    }


def _vector3_to_dict(vec) -> dict:
    return {'x': float(vec.x), 'y': float(vec.y), 'z': float(vec.z)}


def _color_to_dict(color) -> dict:
    return {
        'r': float(color.r),
        'g': float(color.g),
        'b': float(color.b),
        'a': float(color.a),
    }


def main(args=None):
    rclpy.init(args=args)
    node = PatrolMemoryBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Bridge a ROS2 PointStamped target into ROS1 /tf through rosbridge."""

import json
import threading
import time
from typing import Tuple

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node

try:
    import websocket
except ImportError:
    print("ERROR: websocket-client not installed. pip3 install websocket-client")
    raise


class VisionTfBridge(Node):
    def __init__(self):
        super().__init__('vision_tf_bridge')

        self.declare_parameter('rosbridge_url', 'ws://127.0.0.1:9090')
        self.declare_parameter('reconnect_interval', 2.0)
        self.declare_parameter('input_topic', '/vision/target_point_local')
        self.declare_parameter('parent_frame', 'camera_link')
        self.declare_parameter('child_frame', 'vision_target')
        self.declare_parameter('input_frame_mode', 'optical_to_camera_link')
        self.declare_parameter('publish_ros1_point', True)
        self.declare_parameter('ros1_point_topic', '/vision/target_point_camera_link')

        self.url = self.get_parameter('rosbridge_url').value
        self.reconnect_interval = float(self.get_parameter('reconnect_interval').value)
        self.input_topic = self.get_parameter('input_topic').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        self.input_frame_mode = self.get_parameter('input_frame_mode').value
        self.publish_ros1_point = self._as_bool(self.get_parameter('publish_ros1_point').value)
        self.ros1_point_topic = self.get_parameter('ros1_point_topic').value
        self._warned_unknown_frame_mode = False

        self.ws = None
        self.ws_lock = threading.RLock()
        self.connected = False
        self._advertised = False
        self._last_point_log = 0.0

        self.create_subscription(PointStamped, self.input_topic, self._on_point, 10)

        threading.Thread(target=self._connect_loop, daemon=True).start()
        self.get_logger().info(
            f'vision_tf_bridge started input={self.input_topic} '
            f'rosbridge={self.url} tf={self.parent_frame}->{self.child_frame} '
            f'mode={self.input_frame_mode}'
        )

    def _connect_loop(self):
        while rclpy.ok():
            if not self.connected:
                self._try_connect()
            time.sleep(self.reconnect_interval)

    def _try_connect(self):
        try:
            with self.ws_lock:
                if self.ws:
                    try:
                        self.ws.close()
                    except Exception:
                        pass
                self.ws = websocket.WebSocket()
                self.ws.settimeout(5)
                self.ws.connect(self.url)
                self.connected = True
                self._advertised = False
                self.get_logger().info(f'connected to ROS1 rosbridge: {self.url}')
        except Exception as exc:
            self.connected = False
            self.get_logger().warn(f'rosbridge connection failed: {exc}')

    def _advertise(self):
        if self._advertised:
            return
        with self.ws_lock:
            if not (self.ws and self.connected):
                return
            self.ws.send(json.dumps({
                'op': 'advertise',
                'topic': '/tf',
                'type': 'tf2_msgs/TFMessage',
            }))
            if self.publish_ros1_point:
                self.ws.send(json.dumps({
                    'op': 'advertise',
                    'topic': self.ros1_point_topic,
                    'type': 'geometry_msgs/PointStamped',
                }))
            self._advertised = True

    def _on_point(self, msg: PointStamped):
        point = self._convert_point(msg.point.x, msg.point.y, msg.point.z)
        stamp = self._stamp_dict(msg)
        now_s = time.monotonic()
        if now_s - self._last_point_log > 2.0:
            self.get_logger().info(
                f'forward target {msg.header.frame_id or "<empty>"} '
                f'({msg.point.x:.3f}, {msg.point.y:.3f}, {msg.point.z:.3f}) '
                f'as {self.parent_frame}->{self.child_frame} '
                f'({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f})'
            )
            self._last_point_log = now_s

        tf_msg = {
            'transforms': [{
                'header': {
                    'stamp': stamp,
                    'frame_id': self.parent_frame,
                },
                'child_frame_id': self.child_frame,
                'transform': {
                    'translation': {
                        'x': point[0],
                        'y': point[1],
                        'z': point[2],
                    },
                    'rotation': {
                        'x': 0.0,
                        'y': 0.0,
                        'z': 0.0,
                        'w': 1.0,
                    },
                },
            }],
        }

        try:
            self._advertise()
            with self.ws_lock:
                if not (self.ws and self.connected):
                    return
                self.ws.send(json.dumps({
                    'op': 'publish',
                    'topic': '/tf',
                    'type': 'tf2_msgs/TFMessage',
                    'msg': tf_msg,
                }))
                if self.publish_ros1_point:
                    self.ws.send(json.dumps({
                        'op': 'publish',
                        'topic': self.ros1_point_topic,
                        'type': 'geometry_msgs/PointStamped',
                        'msg': {
                            'header': {
                                'stamp': stamp,
                                'frame_id': self.parent_frame,
                            },
                            'point': {
                                'x': point[0],
                                'y': point[1],
                                'z': point[2],
                            },
                        },
                    }))
        except Exception as exc:
            self.connected = False
            self._advertised = False
            self.get_logger().warn(f'publish to ROS1 failed: {exc}')

    def _convert_point(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        if self.input_frame_mode == 'identity':
            return float(x), float(y), float(z)
        if self.input_frame_mode == 'optical_to_camera_link':
            return float(z), float(-x), float(-y)
        if not self._warned_unknown_frame_mode:
            self.get_logger().warn(
                f'unknown input_frame_mode={self.input_frame_mode}, using identity'
            )
            self._warned_unknown_frame_mode = True
        return float(x), float(y), float(z)

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    def _stamp_dict(self, msg: PointStamped):
        sec = int(msg.header.stamp.sec)
        nanosec = int(msg.header.stamp.nanosec)
        if sec == 0 and nanosec == 0:
            now = self.get_clock().now().to_msg()
            sec = int(now.sec)
            nanosec = int(now.nanosec)
        return {'secs': sec, 'nsecs': nanosec}

    def destroy_node(self):
        with self.ws_lock:
            if self.ws:
                try:
                    self.ws.close()
                except Exception:
                    pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionTfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except RCLError:
            pass


if __name__ == '__main__':
    main()

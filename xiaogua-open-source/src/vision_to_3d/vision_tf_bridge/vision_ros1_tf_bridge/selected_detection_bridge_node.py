#!/usr/bin/env python3
"""Bridge ROS2 selected detections into the ROS1 JSON topic used by auto_aim.py."""

import json
import threading
import time

import rclpy
from ai_msgs.msg import PerceptionTargets
from rclpy.node import Node

try:
    import websocket
except ImportError:
    print("ERROR: websocket-client not installed. pip3 install websocket-client")
    raise


class SelectedDetectionBridge(Node):
    def __init__(self):
        super().__init__('selected_detection_bridge')

        self.declare_parameter('rosbridge_url', 'ws://127.0.0.1:9090')
        self.declare_parameter('reconnect_interval', 2.0)
        self.declare_parameter('input_topic', '/object_tracker/selected_detection')
        self.declare_parameter('ros1_output_topic', '/tracked_yolov8/detections')
        self.declare_parameter('publish_empty_on_no_target', True)
        self.declare_parameter('empty_publish_timeout_sec', 0.5)

        self.url = self.get_parameter('rosbridge_url').value
        self.reconnect_interval = float(self.get_parameter('reconnect_interval').value)
        self.input_topic = self.get_parameter('input_topic').value
        self.ros1_output_topic = self.get_parameter('ros1_output_topic').value
        self.publish_empty_on_no_target = self._as_bool(
            self.get_parameter('publish_empty_on_no_target').value
        )
        self.empty_publish_timeout = float(
            self.get_parameter('empty_publish_timeout_sec').value
        )

        self.ws = None
        self.ws_lock = threading.RLock()
        self.connected = False
        self._advertised = False
        self._last_publish_time = 0.0
        self._last_detection_time = 0.0

        self.create_subscription(PerceptionTargets, self.input_topic, self._on_targets, 10)
        self.create_timer(0.2, self._on_timer)

        threading.Thread(target=self._connect_loop, daemon=True).start()
        self.get_logger().info(
            f'selected_detection_bridge started input={self.input_topic} '
            f'rosbridge={self.url} ros1_output={self.ros1_output_topic}'
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
                'topic': self.ros1_output_topic,
                'type': 'std_msgs/String',
            }))
            self._advertised = True

    def _on_targets(self, msg: PerceptionTargets):
        detections = []
        for target in msg.targets:
            if not target.rois:
                continue
            roi = target.rois[0]
            rect = roi.rect
            width = int(rect.width)
            height = int(rect.height)
            if width <= 0 or height <= 0:
                continue

            x1 = int(rect.x_offset)
            y1 = int(rect.y_offset)
            x2 = x1 + width
            y2 = y1 + height
            confidence = float(getattr(roi, 'confidence', 0.0))
            detections.append({
                'name': target.type or 'selected',
                'conf': confidence,
                'cx': int(round(x1 + width * 0.5)),
                'cy': int(round(y1 + height * 0.5)),
                'x1': x1,
                'y1': y1,
                'x2': x2,
                'y2': y2,
                'w': width,
                'h': height,
                'track_id': int(getattr(target, 'track_id', 0)),
                'selected': True,
            })

        self._last_detection_time = time.monotonic()
        self._publish_ros1_string(json.dumps(detections, ensure_ascii=False))
        if detections:
            det = detections[0]
            self.get_logger().info(
                f'bridged selected target {det["name"]} '
                f'cx={det["cx"]} conf={det["conf"]:.3f}'
            )

    def _on_timer(self):
        if not self.publish_empty_on_no_target:
            return
        now = time.monotonic()
        if self._last_detection_time <= 0.0:
            return
        if now - self._last_detection_time < self.empty_publish_timeout:
            return
        if now - self._last_publish_time < self.empty_publish_timeout:
            return
        self._publish_ros1_string('[]')

    def _publish_ros1_string(self, data: str) -> None:
        try:
            self._advertise()
            with self.ws_lock:
                if not (self.ws and self.connected):
                    return
                self.ws.send(json.dumps({
                    'op': 'publish',
                    'topic': self.ros1_output_topic,
                    'type': 'std_msgs/String',
                    'msg': {'data': data},
                }))
                self._last_publish_time = time.monotonic()
        except Exception as exc:
            self.connected = False
            self._advertised = False
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


def main(args=None):
    rclpy.init(args=args)
    node = SelectedDetectionBridge()
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

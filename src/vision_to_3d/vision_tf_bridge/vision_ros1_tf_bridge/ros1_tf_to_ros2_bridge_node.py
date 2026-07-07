#!/usr/bin/env python3
"""Bridge ROS1 /tf and /tf_static from rosbridge into ROS2."""

import json
import threading
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

try:
    import websocket
except ImportError:
    print("ERROR: websocket-client not installed. pip3 install websocket-client")
    raise


class Ros1TfToRos2Bridge(Node):
    def __init__(self):
        super().__init__('ros1_tf_to_ros2_bridge')

        self.declare_parameter('rosbridge_url', 'ws://127.0.0.1:9090')
        self.declare_parameter('reconnect_interval', 2.0)
        self.declare_parameter('bridge_tf', True)
        self.declare_parameter('bridge_tf_static', True)

        self.url = self.get_parameter('rosbridge_url').value
        self.reconnect_interval = float(self.get_parameter('reconnect_interval').value)
        self.bridge_tf = self._as_bool(self.get_parameter('bridge_tf').value)
        self.bridge_tf_static = self._as_bool(self.get_parameter('bridge_tf_static').value)

        tf_qos = QoSProfile(depth=100)
        tf_qos.reliability = ReliabilityPolicy.RELIABLE

        static_qos = QoSProfile(depth=1)
        static_qos.reliability = ReliabilityPolicy.RELIABLE
        static_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._tf_pub = self.create_publisher(TFMessage, '/tf', tf_qos)
        self._tf_static_pub = self.create_publisher(TFMessage, '/tf_static', static_qos)

        self.ws = None
        self.ws_lock = threading.RLock()
        self.connected = False
        self._subscribed = False
        self._last_log = 0.0

        threading.Thread(target=self._io_loop, daemon=True).start()
        self.get_logger().info(
            f'ros1_tf_to_ros2_bridge started rosbridge={self.url} '
            f'tf={self.bridge_tf} tf_static={self.bridge_tf_static}'
        )

    def _io_loop(self):
        while rclpy.ok():
            try:
                self._connect()
                while rclpy.ok() and self.connected:
                    raw = self.ws.recv()
                    if raw:
                        self._handle_message(raw)
            except Exception as exc:
                self.connected = False
                self._subscribed = False
                self.get_logger().warn(f'rosbridge TF loop failed: {exc}')
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
            self._subscribed = False
            self._subscribe()
            self.get_logger().info(f'connected to ROS1 rosbridge: {self.url}')

    def _subscribe(self):
        if self._subscribed or not (self.ws and self.connected):
            return
        if self.bridge_tf:
            self.ws.send(json.dumps({
                'op': 'subscribe',
                'topic': '/tf',
                'type': 'tf2_msgs/TFMessage',
                'queue_length': 20,
                'throttle_rate': 0,
            }))
        if self.bridge_tf_static:
            self.ws.send(json.dumps({
                'op': 'subscribe',
                'topic': '/tf_static',
                'type': 'tf2_msgs/TFMessage',
                'queue_length': 1,
                'throttle_rate': 0,
            }))
        self._subscribed = True

    def _handle_message(self, raw: str):
        try:
            body = json.loads(raw)
        except Exception:
            return
        if body.get('op') != 'publish':
            return

        topic = body.get('topic')
        if topic not in ('/tf', '/tf_static'):
            return

        msg = self._tf_message_from_dict(body.get('msg', {}))
        if not msg.transforms:
            return

        if topic == '/tf':
            self._tf_pub.publish(msg)
        else:
            self._tf_static_pub.publish(msg)

        now = time.monotonic()
        if now - self._last_log > 5.0:
            first = msg.transforms[0]
            self.get_logger().info(
                f'bridged {topic}: {len(msg.transforms)} transforms, '
                f'first={first.header.frame_id}->{first.child_frame_id}'
            )
            self._last_log = now

    def _tf_message_from_dict(self, data: dict) -> TFMessage:
        out = TFMessage()
        transforms = data.get('transforms', [])
        if not isinstance(transforms, list):
            return out
        for item in transforms:
            try:
                out.transforms.append(_transform_from_dict(item))
            except Exception as exc:
                self.get_logger().debug(f'skip bad transform: {exc}')
        return out

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


def _transform_from_dict(data: dict) -> TransformStamped:
    msg = TransformStamped()
    header = data.get('header', {})
    stamp = header.get('stamp', {})
    msg.header.stamp.sec = int(stamp.get('secs', stamp.get('sec', 0)) or 0)
    msg.header.stamp.nanosec = int(stamp.get('nsecs', stamp.get('nanosec', 0)) or 0)
    msg.header.frame_id = str(header.get('frame_id', ''))
    msg.child_frame_id = str(data.get('child_frame_id', ''))

    transform = data.get('transform', {})
    translation = transform.get('translation', {})
    rotation = transform.get('rotation', {})
    msg.transform.translation.x = float(translation.get('x', 0.0) or 0.0)
    msg.transform.translation.y = float(translation.get('y', 0.0) or 0.0)
    msg.transform.translation.z = float(translation.get('z', 0.0) or 0.0)
    msg.transform.rotation.x = float(rotation.get('x', 0.0) or 0.0)
    msg.transform.rotation.y = float(rotation.get('y', 0.0) or 0.0)
    msg.transform.rotation.z = float(rotation.get('z', 0.0) or 0.0)
    msg.transform.rotation.w = float(rotation.get('w', 1.0) or 1.0)
    return msg


def main(args=None):
    rclpy.init(args=args)
    node = Ros1TfToRos2Bridge()
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

#!/usr/bin/env python3
"""ROS2 → ROS1 桥接节点：声源定位（DOA）转发"""

import json
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool

try:
    import websocket
except ImportError:
    print("ERROR: websocket-client not installed. pip3 install websocket-client")
    exit(1)


class DoaBridge(Node):
    def __init__(self):
        super().__init__('doa_to_ros1_bridge')

        self.declare_parameter('rosbridge_url', 'ws://127.0.0.1:9090')
        self.declare_parameter('reconnect_interval', 5.0)
        self.declare_parameter('forward_vad', True)

        self.url = self.get_parameter('rosbridge_url').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value
        self.forward_vad = self.get_parameter('forward_vad').value

        self.ws = None
        self.ws_lock = threading.RLock()
        self.connected = False
        self._advertised = False
        self._tts_playing = False
        self._tts_lock = threading.Lock()

        # ROS2 订阅
        self.create_subscription(Float32, '/xvf3800/doa_deg', self._on_doa, 10)
        self.create_subscription(Bool, '/tts_playing', self._on_tts_playing, 10)
        if self.forward_vad:
            self.create_subscription(Bool, '/xvf3800/vad', self._on_vad, 10)

        self.get_logger().info(f'DOA桥接启动 url={self.url}')
        threading.Thread(target=self._connect_loop, daemon=True).start()

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
                self.get_logger().info(f'已连接 rosbridge: {self.url}')
        except Exception as e:
            self.get_logger().warn(f'连接失败: {e}')
            self.connected = False

    def _advertise(self):
        """在ROS1侧注册话题（首次发送前调用一次）"""
        if self._advertised:
            return
        try:
            with self.ws_lock:
                if self.ws and self.connected:
                    self.ws.send(json.dumps({
                        'op': 'advertise',
                        'topic': '/xvf3800/doa_deg',
                        'type': 'std_msgs/Float32',
                    }))
                    if self.forward_vad:
                        self.ws.send(json.dumps({
                            'op': 'advertise',
                            'topic': '/xvf3800/vad',
                            'type': 'std_msgs/Bool',
                        }))
                    self.ws.send(json.dumps({
                        'op': 'advertise',
                        'topic': '/tts_playing',
                        'type': 'std_msgs/Bool',
                    }))
                    self._advertised = True
        except Exception:
            self.connected = False

    def _send_ros1(self, topic, msg_type, data):
        """发送消息到ROS1话题"""
        try:
            self._advertise()
            with self.ws_lock:
                if self.ws and self.connected:
                    self.ws.send(json.dumps({
                        'op': 'publish',
                        'topic': topic,
                        'type': msg_type,
                        'msg': {'data': data},
                    }))
                    return True
        except Exception:
            self.connected = False
            self._advertised = False
        return False

    def _is_tts_playing(self):
        with self._tts_lock:
            return self._tts_playing

    def _on_tts_playing(self, msg):
        playing = bool(msg.data)
        with self._tts_lock:
            self._tts_playing = playing
        self._send_ros1('/tts_playing', 'std_msgs/Bool', playing)
        if playing and self.forward_vad:
            self._send_ros1('/xvf3800/vad', 'std_msgs/Bool', False)

    def _on_doa(self, msg):
        """收到DOA角度，转发到ROS1"""
        if self._is_tts_playing():
            return
        self._send_ros1('/xvf3800/doa_deg', 'std_msgs/Float32', float(msg.data))

    def _on_vad(self, msg):
        """收到VAD状态，转发到ROS1"""
        if self._is_tts_playing():
            return
        self._send_ros1('/xvf3800/vad', 'std_msgs/Bool', bool(msg.data))

    def destroy_node(self):
        with self.ws_lock:
            if self.ws:
                try:
                    self.ws.close()
                except Exception:
                    pass
        super().destroy_node()


def main():
    rclpy.init()
    node = DoaBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

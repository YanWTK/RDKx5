#!/usr/bin/env python3
"""Minimal MJPEG server for viewing ROS2 Image topics from a browser."""

import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image


def _encode_jpeg(frame, quality: int) -> bytes:
    ok, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        return b''
    return buf.tobytes()


def _placeholder(width: int, height: int, lines: list[str]) -> bytes:
    frame = np.full((height, width, 3), 235, dtype=np.uint8)
    cv2.putText(frame, 'ROS2 MJPEG', (24, 52), cv2.FONT_HERSHEY_SIMPLEX,
                1.2, (40, 40, 40), 2)
    y = 105
    for line in lines:
        cv2.putText(frame, line, (24, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, (70, 70, 70), 2)
        y += 36
    return _encode_jpeg(frame, 80)


class _Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, addr, handler, node):
        super().__init__(addr, handler)
        self.node = node
        self.stop_event = threading.Event()
        self.frame_lock = threading.Lock()
        self.latest_jpeg = node.placeholder_jpeg

    def set_jpeg(self, jpeg: bytes) -> None:
        if not jpeg:
            return
        with self.frame_lock:
            self.latest_jpeg = jpeg

    def get_jpeg(self) -> bytes:
        with self.frame_lock:
            return self.latest_jpeg or self.node.placeholder_jpeg


class _Handler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.0'

    def log_message(self, fmt, *args):  # noqa: A003
        return

    def do_GET(self):
        if self.path in ('/', '/index.html'):
            self._index()
            return
        if not self.path.startswith('/stream'):
            self.send_error(HTTPStatus.NOT_FOUND, 'not found')
            return

        self.send_response(HTTPStatus.OK)
        self.send_header('Age', '0')
        self.send_header('Cache-Control', 'no-cache, private')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()

        interval = 1.0 / max(1.0, self.server.node.fps)
        while not self.server.stop_event.is_set():
            try:
                jpeg = self.server.get_jpeg()
                self.wfile.write(b'--frame\r\n')
                self.wfile.write(b'Content-Type: image/jpeg\r\n')
                self.wfile.write(f'Content-Length: {len(jpeg)}\r\n\r\n'.encode('ascii'))
                self.wfile.write(jpeg)
                self.wfile.write(b'\r\n')
                self.wfile.flush()
                time.sleep(interval)
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                break

    def _index(self):
        topic = self.server.node.source_topic
        body = (
            '<html><body>'
            '<h3>ROS2 MJPEG Stream</h3>'
            f'<p>topic: {topic}</p>'
            '<p><img src="/stream" style="max-width: 100%; height: auto;"></p>'
            '</body></html>'
        ).encode('utf-8')
        self.send_response(HTTPStatus.OK)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)


class ImageMjpegServer(Node):
    def __init__(self):
        super().__init__('image_mjpeg_server')

        self.declare_parameter('address', '0.0.0.0')
        self.declare_parameter('port', 8082)
        self.declare_parameter('source_topic', '/yolo_detector/image')
        self.declare_parameter('source_type', 'image')
        self.declare_parameter('fps', 10.0)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('placeholder_width', 640)
        self.declare_parameter('placeholder_height', 480)

        self.address = str(self.get_parameter('address').value)
        self.port = int(self.get_parameter('port').value)
        self.source_topic = str(self.get_parameter('source_topic').value)
        self.source_type = str(self.get_parameter('source_type').value).strip().lower()
        self.fps = float(self.get_parameter('fps').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        width = int(self.get_parameter('placeholder_width').value)
        height = int(self.get_parameter('placeholder_height').value)

        self._bridge = CvBridge()
        self.placeholder_jpeg = _placeholder(width, height, [
            'waiting for frames',
            f'topic: {self.source_topic}',
        ])

        qos = QoSProfile(
            depth=2,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        if self.source_type in ('compressed', 'compressed_image', 'compressedimage'):
            self.create_subscription(
                CompressedImage, self.source_topic, self._on_compressed_image, qos
            )
            type_label = 'sensor_msgs/msg/CompressedImage'
        else:
            self.create_subscription(Image, self.source_topic, self._on_image, qos)
            type_label = 'sensor_msgs/msg/Image'

        self._server = _Server((self.address, self.port), _Handler, self)
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()
        self.get_logger().info(
            f'MJPEG serving {self.source_topic} ({type_label}) '
            f'on http://{self.address}:{self.port}/stream'
        )

    def _on_image(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if frame.ndim == 2:
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            elif frame.shape[-1] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            elif msg.encoding.lower() == 'rgb8':
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        self._server.set_jpeg(_encode_jpeg(frame, self.jpeg_quality))

    def _on_compressed_image(self, msg: CompressedImage):
        data = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
        if frame is not None:
            self._server.set_jpeg(_encode_jpeg(frame, self.jpeg_quality))

    def shutdown_server(self):
        self._server.stop_event.set()
        self._server.shutdown()
        self._server.server_close()


def main(args=None):
    rclpy.init(args=args)
    node = ImageMjpegServer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown_server()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

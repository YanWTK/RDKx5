from __future__ import annotations

import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from .common import encode_jpeg, render_placeholder_frame


class _RobopilotMjpegHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, server_address, RequestHandlerClass, node: "MjpegServerNode"):
        super().__init__(server_address, RequestHandlerClass)
        self.node = node
        self.latest_jpeg = node.placeholder_jpeg
        self.frame_lock = threading.Lock()
        self.client_lock = threading.Lock()
        self.client_count = 0
        self.stop_event = threading.Event()

    def set_latest_jpeg(self, jpeg: bytes) -> None:
        with self.frame_lock:
            self.latest_jpeg = jpeg

    def get_latest_jpeg(self) -> bytes:
        with self.frame_lock:
            return self.latest_jpeg or self.node.placeholder_jpeg

    def add_client(self) -> None:
        with self.client_lock:
            self.client_count += 1

    def remove_client(self) -> None:
        with self.client_lock:
            self.client_count = max(0, self.client_count - 1)

    def get_client_count(self) -> int:
        with self.client_lock:
            return self.client_count


class _MjpegRequestHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.0"

    def log_message(self, format, *args):  # noqa: A003
        return

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/":
            self._send_index()
            return
        if parsed.path != "/stream":
            self.send_error(HTTPStatus.NOT_FOUND, "not found")
            return

        params = parse_qs(parsed.query)
        topic = params.get("topic", [""])[0] or self.server.node.stream_topic
        if topic not in {self.server.node.stream_topic, self.server.node.source_topic}:
            self.send_error(HTTPStatus.BAD_REQUEST, "unexpected topic")
            return

        self.send_response(HTTPStatus.OK)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()

        frame_interval = 1.0 / max(1.0, self.server.node.stream_fps)
        self.server.add_client()
        try:
            while not self.server.stop_event.is_set():
                jpeg = self.server.get_latest_jpeg()
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii"))
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
                time.sleep(frame_interval)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass
        finally:
            self.server.remove_client()

    def _send_index(self) -> None:
        body = (
            "<html><body>"
            "<p>Robopilot MJPEG stream</p>"
            f"<p>Use /stream?topic={self.server.node.stream_topic}</p>"
            "</body></html>"
        ).encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


class MjpegServerNode(Node):
    def __init__(self) -> None:
        super().__init__("robopilot_mjpeg_server")

        self.declare_parameter("address", "0.0.0.0")
        self.declare_parameter("port", 8081)
        self.declare_parameter("stream_topic", "/camera/rgb/image_raw")
        self.declare_parameter("source_topic", "/camera/rgb/image_raw")
        self.declare_parameter("stream_fps", 15.0)
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter("stream_width", 0)
        self.declare_parameter("stream_height", 0)
        self.declare_parameter("placeholder_width", 640)
        self.declare_parameter("placeholder_height", 480)
        self.declare_parameter("placeholder_text", "RoboPilot video stream waiting for frames")

        self.address = str(self.get_parameter("address").value)
        self.port = int(self.get_parameter("port").value)
        self.stream_topic = str(self.get_parameter("stream_topic").value)
        self.source_topic = str(self.get_parameter("source_topic").value)
        self.stream_fps = max(1.0, float(self.get_parameter("stream_fps").value))
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.stream_width = max(0, int(self.get_parameter("stream_width").value))
        self.stream_height = max(0, int(self.get_parameter("stream_height").value))
        self.placeholder_width = int(self.get_parameter("placeholder_width").value)
        self.placeholder_height = int(self.get_parameter("placeholder_height").value)
        self.placeholder_text = str(self.get_parameter("placeholder_text").value)

        self._bridge = CvBridge()
        self._last_encode_time = 0.0
        self.placeholder_jpeg = encode_jpeg(
            render_placeholder_frame(
                self.placeholder_width,
                self.placeholder_height,
                [
                    self.placeholder_text,
                    f"topic: {self.source_topic}",
                    "stream endpoint: /stream?topic=/camera/rgb/image_raw",
                ],
            ),
            quality=self.jpeg_quality,
        )
        self._server = _RobopilotMjpegHTTPServer((self.address, self.port), _MjpegRequestHandler, self)
        self._server_thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._server_thread.start()
        image_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._subscription = self.create_subscription(
            Image, self.source_topic, self._on_image, image_qos
        )
        self.get_logger().info(f"MJPEG stream serving on http://{self.address}:{self.port}/stream")

    def _on_image(self, msg: Image) -> None:
        if self._server.get_client_count() <= 0:
            return
        now = time.monotonic()
        if now - self._last_encode_time < 1.0 / self.stream_fps:
            return
        self._last_encode_time = now
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            if frame.ndim == 2:
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            elif frame.shape[-1] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            elif msg.encoding.lower() == "rgb8":
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if self.stream_width > 0 and self.stream_height > 0:
            frame = cv2.resize(
                frame,
                (self.stream_width, self.stream_height),
                interpolation=cv2.INTER_AREA,
            )
        jpeg = encode_jpeg(frame, quality=self.jpeg_quality)
        self._server.set_latest_jpeg(jpeg)

    def shutdown_server(self) -> None:
        if hasattr(self, "_server") and self._server is not None:
            self._server.stop_event.set()
            self._server.shutdown()
            self._server.server_close()
            self._server = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MjpegServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_server()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()

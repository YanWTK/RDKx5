from __future__ import annotations

import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class MockCameraNode(Node):
    def __init__(self) -> None:
        super().__init__("robopilot_mock_camera")

        self.declare_parameter("topic", "/camera/rgb/image_raw")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 20.0)
        self.declare_parameter("frame_id", "camera_rgb_frame")
        self.declare_parameter("title", "RoboPilot Mock Camera")

        self.topic = str(self.get_parameter("topic").value)
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.fps = max(1.0, float(self.get_parameter("fps").value))
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.title = str(self.get_parameter("title").value)

        self._publisher = self.create_publisher(Image, self.topic, 10)
        self._bridge = CvBridge()
        self._index = 0
        self._start = time.monotonic()
        self._timer = self.create_timer(1.0 / self.fps, self._on_timer)
        self.get_logger().info(f"Mock camera publishing {self.topic}")

    def _render_frame(self) -> np.ndarray:
        phase = self._index * 0.05
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        x_grad = np.linspace(20, 180, self.width, dtype=np.uint8)
        y_grad = np.linspace(30, 170, self.height, dtype=np.uint8)[:, None]
        frame[:, :, 0] = (x_grad[None, :] + int(50 * math.sin(phase))) % 255
        frame[:, :, 1] = (y_grad + int(40 * math.cos(phase * 0.7))) % 255
        frame[:, :, 2] = ((x_grad[None, :] // 2 + y_grad // 2 + int(phase * 18)) % 255)

        center_x = int((math.sin(phase * 0.8) * 0.35 + 0.5) * self.width)
        center_y = int((math.cos(phase * 0.6) * 0.25 + 0.5) * self.height)
        radius = max(18, min(self.width, self.height) // 10)

        cv2.circle(frame, (center_x, center_y), radius, (30, 230, 255), 3)
        cv2.rectangle(frame, (30, 30), (self.width - 30, self.height - 30), (255, 255, 255), 1)
        cv2.line(frame, (0, center_y), (self.width - 1, center_y), (255, 255, 255), 1)
        cv2.line(frame, (center_x, 0), (center_x, self.height - 1), (255, 255, 255), 1)

        elapsed = time.monotonic() - self._start
        lines = [
            self.title,
            f"topic: {self.topic}",
            f"frame: {self._index}",
            f"time: {elapsed:.1f}s",
        ]
        y = 48
        for line in lines:
            cv2.putText(
                frame,
                line,
                (24, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (245, 245, 245),
                2,
                cv2.LINE_AA,
            )
            y += 32

        return frame

    def _on_timer(self) -> None:
        frame = self._render_frame()
        msg = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self._publisher.publish(msg)
        self._index += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockCameraNode()
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

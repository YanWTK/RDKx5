#!/usr/bin/env python3
import argparse
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class CalibImageCapture(Node):
    def __init__(self, args):
        super().__init__("capture_calib_images")
        self.topic = args.topic
        self.output_dir = args.output_dir
        self.count = args.count
        self.interval = args.interval
        self.prefix = args.prefix
        self.jpeg_quality = args.jpeg_quality
        self.saved = 0
        self.last_save = 0.0
        os.makedirs(self.output_dir, exist_ok=True)
        self.sub = self.create_subscription(Image, self.topic, self._on_image, 10)
        self.get_logger().info(
            f"capturing {self.count} images from {self.topic} to {self.output_dir}, "
            f"interval={self.interval:.2f}s"
        )

    def _on_image(self, msg):
        now = time.time()
        if self.saved >= self.count:
            return
        if now - self.last_save < self.interval:
            return

        image = self._msg_to_bgr(msg)
        if image is None:
            return

        path = os.path.join(self.output_dir, f"{self.prefix}_{self.saved:04d}.jpg")
        ok = cv2.imwrite(path, image, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            self.get_logger().error(f"failed to write {path}")
            return

        self.saved += 1
        self.last_save = now
        self.get_logger().info(f"saved {self.saved}/{self.count}: {path}")
        if self.saved >= self.count:
            self.get_logger().info("capture complete")
            rclpy.shutdown()

    def _msg_to_bgr(self, msg):
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding in ("rgb8", "bgr8"):
            if msg.step < msg.width * 3:
                self.get_logger().error(
                    f"invalid step={msg.step} for {msg.encoding} {msg.width}x{msg.height}"
                )
                return None
            image = arr.reshape(msg.height, msg.step // 3, 3)[:, : msg.width, :]
            if msg.encoding == "rgb8":
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            return image

        if msg.encoding in ("mono8", "8UC1"):
            image = arr.reshape(msg.height, msg.step)[:, : msg.width]
            return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

        self.get_logger().error(f"unsupported image encoding: {msg.encoding}")
        return None


def parse_args():
    parser = argparse.ArgumentParser(description="Capture calibration images from a ROS2 image topic.")
    parser.add_argument("--topic", default="/camera/color/image_raw")
    parser.add_argument("--output-dir", default="/opt/xiaogua/data/calib_images_scene")
    parser.add_argument("--count", type=int, default=100)
    parser.add_argument("--interval", type=float, default=0.3)
    parser.add_argument("--prefix", default="scene")
    parser.add_argument("--jpeg-quality", type=int, default=95)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = CalibImageCapture(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f"stopped, saved {node.saved}/{node.count}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

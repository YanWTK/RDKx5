#!/usr/bin/env python3
"""Bridge fetch-task String topics between ROS1 rosbridge and ROS2."""

from __future__ import annotations

import json
import threading
import time
from typing import Iterable

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import websocket
except ImportError:
    print("ERROR: websocket-client not installed. pip3 install websocket-client")
    raise


DEFAULT_ROS1_TO_ROS2 = [
    "/voice_fetch/state",
    "/task_understanding/query",
    "/object_memory/query",
    "/target_confirm/confirm_cmd",
    "/memory_target_selector/select_cmd",
]

DEFAULT_ROS2_TO_ROS1 = [
    "/task_understanding/result",
    "/object_memory/query_result",
    "/target_confirm/result",
    "/memory_target_selector/result",
    "/object_tracker/status",
    "/vlm_target_selector/reselector_status",
]


class FetchTaskBridge(Node):
    def __init__(self) -> None:
        super().__init__("fetch_task_bridge")

        self.declare_parameter("rosbridge_url", "ws://127.0.0.1:9090")
        self.declare_parameter("reconnect_interval", 2.0)
        self.declare_parameter(
            "ros1_to_ros2_topics",
            ",".join(DEFAULT_ROS1_TO_ROS2),
        )
        self.declare_parameter(
            "ros2_to_ros1_topics",
            ",".join(DEFAULT_ROS2_TO_ROS1),
        )

        self.url = str(self.get_parameter("rosbridge_url").value)
        self.reconnect_interval = float(self.get_parameter("reconnect_interval").value)
        self.ros1_to_ros2_topics = _parse_topics(
            self.get_parameter("ros1_to_ros2_topics").value,
            DEFAULT_ROS1_TO_ROS2,
        )
        self.ros2_to_ros1_topics = _parse_topics(
            self.get_parameter("ros2_to_ros1_topics").value,
            DEFAULT_ROS2_TO_ROS1,
        )

        self.ws = None
        self.ws_lock = threading.RLock()
        self.connected = False
        self._advertised: set[str] = set()
        self._subscribed: set[str] = set()
        self._ros2_publishers = {
            topic: self.create_publisher(String, topic, 10)
            for topic in self.ros1_to_ros2_topics
        }
        for topic in self.ros2_to_ros1_topics:
            self.create_subscription(
                String,
                topic,
                lambda msg, topic=topic: self._on_ros2_string(topic, msg),
                10,
            )

        threading.Thread(target=self._io_loop, daemon=True).start()
        self.get_logger().info(
            "fetch_task_bridge started | "
            f"rosbridge={self.url} | "
            f"ROS1->ROS2={self.ros1_to_ros2_topics} | "
            f"ROS2->ROS1={self.ros2_to_ros1_topics}"
        )

    def _io_loop(self) -> None:
        while rclpy.ok():
            try:
                self._connect()
                while rclpy.ok() and self.connected:
                    raw = self.ws.recv()
                    if raw:
                        self._handle_rosbridge_message(raw)
            except Exception as exc:
                self.connected = False
                self._advertised.clear()
                self._subscribed.clear()
                self.get_logger().warn(f"rosbridge loop failed: {exc}")
                time.sleep(self.reconnect_interval)

    def _connect(self) -> None:
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
            self._advertised.clear()
            self._subscribed.clear()
            self._advertise_and_subscribe()
            self.get_logger().info(f"connected to ROS1 rosbridge: {self.url}")

    def _advertise_and_subscribe(self) -> None:
        if not (self.ws and self.connected):
            return
        for topic in self.ros2_to_ros1_topics:
            if topic in self._advertised:
                continue
            self.ws.send(json.dumps({
                "op": "advertise",
                "topic": topic,
                "type": "std_msgs/String",
            }))
            self._advertised.add(topic)
        for topic in self.ros1_to_ros2_topics:
            if topic in self._subscribed:
                continue
            self.ws.send(json.dumps({
                "op": "subscribe",
                "topic": topic,
                "type": "std_msgs/String",
            }))
            self._subscribed.add(topic)

    def _handle_rosbridge_message(self, raw: str) -> None:
        try:
            body = json.loads(raw)
        except Exception:
            return
        if body.get("op") != "publish":
            return
        topic = str(body.get("topic") or "")
        publisher = self._ros2_publishers.get(topic)
        if publisher is None:
            return
        data = body.get("msg", {}).get("data", "")
        publisher.publish(String(data=str(data)))
        self.get_logger().info(f"bridged ROS1 -> ROS2 {topic}: {str(data)[:180]}")

    def _on_ros2_string(self, topic: str, msg: String) -> None:
        self._publish_ros1(topic, msg.data)

    def _publish_ros1(self, topic: str, data: str) -> None:
        try:
            with self.ws_lock:
                if not (self.ws and self.connected):
                    return
                self._advertise_and_subscribe()
                self.ws.send(json.dumps({
                    "op": "publish",
                    "topic": topic,
                    "type": "std_msgs/String",
                    "msg": {"data": data},
                }))
        except Exception as exc:
            self.connected = False
            self._advertised.clear()
            self._subscribed.clear()
            self.get_logger().warn(f"publish to ROS1 failed: {exc}")

    def destroy_node(self) -> None:
        with self.ws_lock:
            if self.ws:
                try:
                    self.ws.close()
                except Exception:
                    pass
        super().destroy_node()


def _parse_topics(value, default: Iterable[str]) -> list[str]:
    if isinstance(value, (list, tuple)):
        topics = [str(item).strip() for item in value]
    else:
        text = str(value or "").strip()
        if not text:
            topics = list(default)
        elif text.startswith("["):
            try:
                decoded = json.loads(text)
                topics = [str(item).strip() for item in decoded]
            except Exception:
                topics = [item.strip().strip("'\"") for item in text.strip("[]").split(",")]
        else:
            topics = [item.strip() for item in text.split(",")]
    return [topic for topic in topics if topic]


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FetchTaskBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

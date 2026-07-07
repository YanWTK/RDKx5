#!/usr/bin/env python3
"""Forward one selected camera image topic to a stable output topic."""

import json
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on', 'depth')
    return bool(value)


class CameraMuxNode(Node):
    """Subscribe to one camera at a time and republish it as active camera."""

    DEPTH = 'depth'
    USB = 'usb'

    def __init__(self):
        super().__init__('camera_mux')

        self.declare_parameter('depth_image_topic', '/camera/color/image_raw')
        self.declare_parameter('usb_image_topic', '/usb_camera/image_raw')
        self.declare_parameter('output_image_topic', '/active_camera/image_raw')
        self.declare_parameter('vision_output_image_topic', '/vision_camera/image_raw')
        self.declare_parameter('depth_input_topic', '/camera/depth/image_raw')
        self.declare_parameter('vision_output_depth_topic', '/vision_camera/depth/image_raw')
        self.declare_parameter('camera_info_input_topic', '/camera/depth/camera_info')
        self.declare_parameter('vision_output_camera_info_topic', '/vision_camera/depth/camera_info')
        self.declare_parameter('vision_active_topic', '/yolo_detector/active')
        self.declare_parameter('status_topic', '/camera_mux/status')
        self.declare_parameter('select_depth_service', '/camera_mux/select_depth')
        self.declare_parameter('toggle_service', '/camera_mux/toggle')
        self.declare_parameter('initial_source', 'depth')
        self.declare_parameter('status_period_sec', 1.0)

        self.depth_image_topic = self.get_parameter('depth_image_topic').value
        self.usb_image_topic = self.get_parameter('usb_image_topic').value
        self.output_image_topic = self.get_parameter('output_image_topic').value
        self.vision_output_image_topic = self.get_parameter('vision_output_image_topic').value
        self.depth_input_topic = self.get_parameter('depth_input_topic').value
        self.vision_output_depth_topic = self.get_parameter('vision_output_depth_topic').value
        self.camera_info_input_topic = self.get_parameter('camera_info_input_topic').value
        self.vision_output_camera_info_topic = self.get_parameter(
            'vision_output_camera_info_topic'
        ).value
        self.vision_active_topic = self.get_parameter('vision_active_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        self.select_depth_service = self.get_parameter('select_depth_service').value
        self.toggle_service = self.get_parameter('toggle_service').value
        self.status_period_sec = float(self.get_parameter('status_period_sec').value)

        initial = str(self.get_parameter('initial_source').value).strip().lower()
        if initial not in (self.DEPTH, self.USB):
            self.get_logger().warn(
                f'unknown initial_source={initial!r}; using depth'
            )
            initial = self.DEPTH

        self._image_count = 0
        self._last_stamp_sec = 0.0
        self._last_receive_time = 0.0
        self._active_source = initial
        self._active_sub = None
        self._vision_active = False
        self._latest_camera_info = None

        self._image_qos = QoSProfile(
            depth=2,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._image_pub = self.create_publisher(
            Image, self.output_image_topic, self._image_qos
        )
        self._vision_image_pub = self.create_publisher(
            Image, self.vision_output_image_topic, self._image_qos
        )
        self._vision_depth_pub = self.create_publisher(
            Image, self.vision_output_depth_topic, self._image_qos
        )
        self._vision_camera_info_pub = self.create_publisher(
            CameraInfo, self.vision_output_camera_info_topic, 1
        )
        self._status_pub = self.create_publisher(String, self.status_topic, 10)
        self.create_service(SetBool, self.select_depth_service, self._on_select_depth)
        self.create_service(Trigger, self.toggle_service, self._on_toggle)

        self._subscribe_active_source()
        self.create_subscription(
            Image, self.depth_input_topic, self._on_depth, self._image_qos
        )
        self.create_subscription(
            CameraInfo, self.camera_info_input_topic, self._on_camera_info, 10
        )
        active_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Bool, self.vision_active_topic, self._on_vision_active, active_qos
        )
        self.create_timer(self.status_period_sec, self._publish_status)

        self.get_logger().info(
            'camera_mux started | '
            f'depth={self.depth_image_topic} | usb={self.usb_image_topic} | '
            f'output={self.output_image_topic} | '
            f'vision_output={self.vision_output_image_topic} | active={self._active_source}'
        )

    def _topic_for_source(self, source: str) -> str:
        return self.depth_image_topic if source == self.DEPTH else self.usb_image_topic

    def _subscribe_active_source(self) -> None:
        if self._active_sub is not None:
            self.destroy_subscription(self._active_sub)
            self._active_sub = None

        topic = self._topic_for_source(self._active_source)
        self._active_sub = self.create_subscription(
            Image, topic, self._on_image, self._image_qos
        )
        self._image_count = 0
        self._last_stamp_sec = 0.0
        self._last_receive_time = 0.0
        self.get_logger().info(
            f'camera_mux active source={self._active_source} topic={topic}'
        )

    def _set_source(self, source: str) -> None:
        if source == self._active_source:
            return
        self._active_source = source
        self._subscribe_active_source()
        self._publish_status()

    def _on_image(self, msg: Image) -> None:
        self._image_pub.publish(msg)
        if self._vision_active:
            self._vision_image_pub.publish(msg)
        self._image_count += 1
        self._last_receive_time = time.monotonic()
        self._last_stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

    def _on_depth(self, msg: Image) -> None:
        if self._vision_active:
            self._vision_depth_pub.publish(msg)

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._latest_camera_info = msg
        if self._vision_active:
            self._vision_camera_info_pub.publish(msg)

    def _on_vision_active(self, msg: Bool) -> None:
        active = bool(msg.data)
        if active == self._vision_active:
            return
        self._vision_active = active
        if active and self._latest_camera_info is not None:
            self._vision_camera_info_pub.publish(self._latest_camera_info)
        self.get_logger().info(
            f"vision image relay {'enabled' if active else 'paused'}"
        )
        self._publish_status()

    def _on_select_depth(self, request: SetBool.Request, response: SetBool.Response):
        source = self.DEPTH if _as_bool(request.data) else self.USB
        self._set_source(source)
        response.success = True
        response.message = (
            f'active_source={self._active_source} '
            f'topic={self._topic_for_source(self._active_source)}'
        )
        return response

    def _on_toggle(self, request: Trigger.Request, response: Trigger.Response):
        del request
        next_source = self.USB if self._active_source == self.DEPTH else self.DEPTH
        self._set_source(next_source)
        response.success = True
        response.message = (
            f'active_source={self._active_source} '
            f'topic={self._topic_for_source(self._active_source)}'
        )
        return response

    def _publish_status(self) -> None:
        now = time.monotonic()
        age = now - self._last_receive_time if self._last_receive_time else -1.0
        status = {
            'active_source': self._active_source,
            'active_topic': self._topic_for_source(self._active_source),
            'output_topic': self.output_image_topic,
            'vision_active': self._vision_active,
            'vision_output_topic': self.vision_output_image_topic,
            'vision_depth_output_topic': self.vision_output_depth_topic,
            'image_count_since_switch': self._image_count,
            'last_image_age_sec': round(age, 3) if age >= 0.0 else None,
            'last_image_stamp_sec': self._last_stamp_sec,
        }
        self._status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = CameraMuxNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

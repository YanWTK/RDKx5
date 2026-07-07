#!/usr/bin/env python3
"""vision_to_3d_local_node — 像素 (u,v) + 深度 → 相机局部 3D 坐标。

订阅:
  - 目标检测结果 (ai_msgs/PerceptionTargets)
  - 彩色图 (sensor_msgs/Image)
  - 深度图 (sensor_msgs/Image, 16UC1 或 32FC1)
  - 相机内参 (sensor_msgs/CameraInfo, 获取一次后取消订阅)

发布:
  - /vision/target_point_local (geometry_msgs/PointStamped)
  - /vision/distance_image (sensor_msgs/Image)
"""

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool


def _as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


class VisionTo3DLocal(Node):
    """接收 2D 检测 + 深度图，发布相机光轴坐标系下的 3D 点。"""

    def __init__(self):
        super().__init__('vision_to_3d_local')

        # ---- 参数 ----
        self.declare_parameter('detection_topic', '/yolo_detector/detections')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/depth/camera_info')
        self.declare_parameter('output_topic', '/vision/target_point_local')
        self.declare_parameter('target_class', 'person')
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('debug_image_topic', '/vision/distance_image')
        self.declare_parameter('task_gated', False)
        self.declare_parameter('active_topic', '/yolo_detector/active')
        self.declare_parameter('source_image_width', 640)
        self.declare_parameter('source_image_height', 480)
        self.declare_parameter('depth_patch_bbox_fraction', 0.25)
        self.declare_parameter('depth_patch_min_size', 7)
        self.declare_parameter('depth_patch_max_size', 31)
        self.declare_parameter('depth_trim_fraction', 0.10)
        self.declare_parameter('min_depth_m', 0.05)
        self.declare_parameter('max_depth_m', 5.0)
        self._target_class = self.get_parameter('target_class').get_parameter_value().string_value.strip()
        self._publish_debug_image = bool(self.get_parameter('publish_debug_image').value)
        self._task_gated = _as_bool(self.get_parameter('task_gated').value)
        self._task_active = not self._task_gated
        self._source_width = max(1, int(self.get_parameter('source_image_width').value))
        self._source_height = max(1, int(self.get_parameter('source_image_height').value))
        self._patch_fraction = max(0.01, float(self.get_parameter('depth_patch_bbox_fraction').value))
        self._patch_min = max(1, int(self.get_parameter('depth_patch_min_size').value))
        self._patch_max = max(self._patch_min, int(self.get_parameter('depth_patch_max_size').value))
        self._trim_fraction = min(0.45, max(0.0, float(self.get_parameter('depth_trim_fraction').value)))
        self._min_depth_m = float(self.get_parameter('min_depth_m').value)
        self._max_depth_m = float(self.get_parameter('max_depth_m').value)
        if self._target_class:
            self.get_logger().info(f'只检测类别: {self._target_class}')
        else:
            self.get_logger().info('检测所有类别（不过滤）')

        # ---- 相机内参（未获取时为 None）----
        self._fx = None
        self._fy = None
        self._cx = None
        self._cy = None
        self._depth_frame_id = None  # 深度图的 frame_id
        self._info_sub = None        # 用于取消订阅

        # ---- 深度图缓存 ----
        self._depth_cache = None     # np.ndarray, 单位: 米
        self._color_cache = None
        self._bridge = CvBridge()

        # ---- 发布者 ----
        output_topic = self.get_parameter('output_topic').value
        self._pub = self.create_publisher(
            PointStamped, output_topic, 10,
        )
        self.get_logger().info(f'3D 目标点输出: {output_topic}')
        self._debug_pub = None
        if self._publish_debug_image:
            debug_topic = self.get_parameter('debug_image_topic').value
            self._debug_pub = self.create_publisher(Image, debug_topic, 10)
            self.get_logger().info(f'测距可视化图像: {debug_topic}')

        # ---- 订阅深度图 ----
        # 深度图用 sensor QoS（best-effort 常见于相机驱动）
        self._sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._depth_topic = self.get_parameter('depth_topic').value
        self._image_topic = self.get_parameter('image_topic').value
        self._depth_sub = None
        self._color_sub = None
        self._set_sensor_subscriptions(self._task_active)
        if self._task_gated:
            self.create_subscription(
                Bool,
                self.get_parameter('active_topic').value,
                self._on_active,
                QoSProfile(
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                ),
            )

        # ---- 订阅相机内参（获取一次后取消）----
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self._info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self._on_camera_info, 10,
        )
        self.get_logger().info(f'已订阅深度相机内参: {camera_info_topic}')

        # ---- 订阅检测结果 ----
        det_topic = self.get_parameter('detection_topic').value
        try:
            from ai_msgs.msg import PerceptionTargets
            self.create_subscription(
                PerceptionTargets, det_topic, self._on_detection, 10,
            )
            self.get_logger().info(f'已订阅检测话题: {det_topic}')
        except ImportError:
            self.get_logger().error(
                '未找到 ai_msgs，请确认 tros 环境已 source。'
                '节点将仅接收深度图，无法处理检测结果。'
            )

        self.get_logger().info('vision_to_3d_local 节点已启动')

    # ==================== 回调函数 ====================

    def _on_camera_info(self, msg: CameraInfo):
        """只取一次内参，然后取消订阅。"""
        self._fx = msg.k[0]  # fx
        self._fy = msg.k[4]  # fy
        self._cx = msg.k[2]  # cx
        self._cy = msg.k[5]  # cy
        self.get_logger().info(
            f'相机内参已获取: fx={self._fx:.1f} fy={self._fy:.1f} '
            f'cx={self._cx:.1f} cy={self._cy:.1f}'
        )
        # 取消订阅，节省资源
        if self._info_sub is not None:
            self.destroy_subscription(self._info_sub)
            self._info_sub = None

    def _on_active(self, msg: Bool):
        if not self._task_gated:
            return
        active = bool(msg.data)
        if self._task_active == active:
            return
        self._task_active = active
        self._set_sensor_subscriptions(active)
        if not active:
            self._depth_cache = None
            self._color_cache = None
        self.get_logger().info(
            f'vision_to_3d sensor processing {"enabled" if active else "paused"}'
        )

    def _set_sensor_subscriptions(self, enabled: bool):
        if enabled:
            if self._depth_sub is None:
                self._depth_sub = self.create_subscription(
                    Image, self._depth_topic, self._on_depth, self._sensor_qos
                )
                self.get_logger().info(f'已订阅深度图像: {self._depth_topic}')
            if self._publish_debug_image and self._color_sub is None:
                self._color_sub = self.create_subscription(
                    Image, self._image_topic, self._on_color, self._sensor_qos
                )
                self.get_logger().info(f'已订阅彩色图像: {self._image_topic}')
        else:
            if self._depth_sub is not None:
                self.destroy_subscription(self._depth_sub)
                self._depth_sub = None
            if self._color_sub is not None:
                self.destroy_subscription(self._color_sub)
                self._color_sub = None

    def _on_depth(self, msg: Image):
        """缓存最新深度图，统一转换为 float32 米。"""
        try:
            cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge 转换深度图失败: {e}')
            return

        # 记录 frame_id（用于 PointStamped header）
        self._depth_frame_id = msg.header.frame_id

        # 16UC1 → 毫米转米；32FC1 → 直接就是米
        if cv_img.dtype == np.uint16:
            self._depth_cache = cv_img.astype(np.float32) / 1000.0
        elif cv_img.dtype == np.float32:
            self._depth_cache = cv_img
        else:
            self.get_logger().warn(f'不支持的深度图编码: {cv_img.dtype}')
            self._depth_cache = None

    def _on_color(self, msg: Image):
        """缓存最新彩色图，用于绘制测距结果。"""
        if not self._publish_debug_image:
            return
        try:
            self._color_cache = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self._source_height, self._source_width = self._color_cache.shape[:2]
        except Exception as e:
            self.get_logger().warn(f'cv_bridge 转换彩色图失败: {e}')
            self._color_cache = None

    def _on_detection(self, msg):
        """收到检测结果时，取中心像素 → 查深度 → 转 3D → 发布。"""
        if self._depth_cache is None:
            self.get_logger().warn('深度图尚未就绪，跳过本次检测结果')
            return
        if self._fx is None:
            self.get_logger().warn('相机内参尚未就绪，跳过本次检测结果')
            return

        depth_img = self._depth_cache
        h, w = depth_img.shape[:2]
        debug_img = None
        if self._publish_debug_image and self._color_cache is not None:
            debug_img = self._color_cache.copy()

        candidates = []
        for target in msg.targets:
            if not target.rois:
                continue

            # 类别过滤
            target_type = target.type or 'unknown'
            if self._target_class and target_type != self._target_class:
                continue

            roi = target.rois[0]  # 取第一个 ROI
            rect = roi.rect

            depth_sample = self._sample_bbox_depth(depth_img, rect)
            if depth_sample is None:
                self.get_logger().warn(
                    f'目标 {target_type} bbox 深度无效，跳过'
                )
                continue
            u, v, z, patch_box = depth_sample

            rgb_u = int(rect.x_offset + rect.width / 2)
            rgb_v = int(rect.y_offset + rect.height / 2)

            # 越界检查
            if u < 0 or u >= w or v < 0 or v >= h:
                self.get_logger().warn(
                    f'像素 ({u},{v}) 超出深度图范围 ({w}x{h})，跳过'
                )
                continue

            # 像素 → 相机局部 3D 坐标
            x_c = (u - self._cx) * z / self._fx
            y_c = (v - self._cy) * z / self._fy
            z_c = z

            distance = math.sqrt(x_c * x_c + y_c * y_c + z_c * z_c)
            candidates.append({
                'target_type': target_type,
                'rect': rect,
                'rgb_u': rgb_u,
                'rgb_v': rgb_v,
                'u': u,
                'v': v,
                'z': z,
                'patch_box': patch_box,
                'x_c': x_c,
                'y_c': y_c,
                'z_c': z_c,
                'distance': distance,
            })

        if not candidates:
            if debug_img is not None and self._debug_pub is not None:
                try:
                    self._debug_pub.publish(self._bridge.cv2_to_imgmsg(debug_img, encoding='bgr8'))
                except Exception as e:
                    self.get_logger().warn(f'发布测距可视化图像失败: {e}')
            return

        selected = min(candidates, key=lambda item: item['distance'])

        for candidate in candidates:
            rect = candidate['rect']
            target_type = candidate['target_type']
            z_c = candidate['z_c']
            rgb_u = candidate['rgb_u']
            rgb_v = candidate['rgb_v']
            is_selected = candidate is selected
            if debug_img is not None:
                x1 = int(max(0, rect.x_offset))
                y1 = int(max(0, rect.y_offset))
                x2 = int(min(debug_img.shape[1] - 1, rect.x_offset + rect.width))
                y2 = int(min(debug_img.shape[0] - 1, rect.y_offset + rect.height))
                label = f'{target_type} {z_c:.2f}m'
                color = (0, 255, 0) if is_selected else (128, 128, 128)
                thickness = 2 if is_selected else 1
                cv2.rectangle(debug_img, (x1, y1), (x2, y2), color, thickness)
                cv2.circle(debug_img, (rgb_u, rgb_v), 4, (0, 0, 255), -1)
                cv2.putText(
                    debug_img,
                    label,
                    (x1, max(20, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    color,
                    2,
                    cv2.LINE_AA,
                )

        # 构造 PointStamped，只发布最近的目标，避免远处的人覆盖 vision_target。
        pt = PointStamped()
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.header.frame_id = self._depth_frame_id or 'camera_depth_optical_frame'
        pt.point.x = selected['x_c']
        pt.point.y = selected['y_c']
        pt.point.z = selected['z_c']

        self._pub.publish(pt)

        self.get_logger().info(
            f'[{selected["target_type"]}] selected nearest of {len(candidates)} '
            f'rgb_pixel=({selected["rgb_u"]},{selected["rgb_v"]}) '
            f'depth_pixel=({selected["u"]},{selected["v"]}) '
            f'patch={selected["patch_box"]} depth={selected["z"]:.3f}m '
            f'distance={selected["distance"]:.3f}m '
            f'→ 3D=({selected["x_c"]:.3f}, {selected["y_c"]:.3f}, {selected["z_c"]:.3f})'
        )

        if debug_img is not None and self._debug_pub is not None:
            try:
                self._debug_pub.publish(self._bridge.cv2_to_imgmsg(debug_img, encoding='bgr8'))
            except Exception as e:
                self.get_logger().warn(f'发布测距可视化图像失败: {e}')

    def _sample_bbox_depth(self, depth_img, rect):
        h, w = depth_img.shape[:2]
        src_w = max(1.0, float(self._source_width))
        src_h = max(1.0, float(self._source_height))
        scale_x = float(w) / src_w
        scale_y = float(h) / src_h

        center_x = (float(rect.x_offset) + float(rect.width) * 0.5) * scale_x
        center_y = (float(rect.y_offset) + float(rect.height) * 0.5) * scale_y
        u = int(round(center_x))
        v = int(round(center_y))
        if u < 0 or u >= w or v < 0 or v >= h:
            return None

        patch_w = int(round(float(rect.width) * scale_x * self._patch_fraction))
        patch_h = int(round(float(rect.height) * scale_y * self._patch_fraction))
        patch_w = _clamp_odd(patch_w, self._patch_min, self._patch_max)
        patch_h = _clamp_odd(patch_h, self._patch_min, self._patch_max)

        half_w = patch_w // 2
        half_h = patch_h // 2
        x1 = max(0, u - half_w)
        x2 = min(w, u + half_w + 1)
        y1 = max(0, v - half_h)
        y2 = min(h, v + half_h + 1)
        patch = depth_img[y1:y2, x1:x2]
        valid = patch[np.isfinite(patch)]
        valid = valid[(valid > self._min_depth_m) & (valid < self._max_depth_m)]
        if valid.size == 0:
            return None

        if self._trim_fraction > 0.0 and valid.size >= 10:
            valid = np.sort(valid)
            trim = int(valid.size * self._trim_fraction)
            if trim > 0 and valid.size > trim * 2:
                valid = valid[trim:-trim]

        return u, v, float(np.median(valid)), [int(x1), int(y1), int(x2), int(y2)]


def _clamp_odd(value: int, minimum: int, maximum: int) -> int:
    value = max(int(minimum), min(int(maximum), int(value)))
    if value % 2 == 0:
        value += 1
    if value > maximum:
        value -= 2
    return max(1, value)


def main(args=None):
    rclpy.init(args=args)
    node = VisionTo3DLocal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

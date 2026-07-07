#!/usr/bin/env python3
"""Depth bbox center projection and TF conversion into map frame."""

from __future__ import annotations

import math
from typing import Optional

import numpy as np
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener


class DepthToMapProjector:
    def __init__(
        self,
        node,
        target_frame: str = "map",
        patch_size: int = 10,
        patch_bbox_fraction: float = 0.25,
        min_patch_size: int = 7,
        max_patch_size: int = 31,
        trim_fraction: float = 0.10,
        source_width: int = 640,
        source_height: int = 480,
        min_depth_m: float = 0.05,
        max_depth_m: float = 5.0,
        tf_timeout_sec: float = 0.2,
        enable_tf: bool = True,
    ) -> None:
        self._node = node
        self._target_frame = target_frame
        self._patch_size = max(1, int(patch_size))
        self._patch_bbox_fraction = max(0.01, float(patch_bbox_fraction))
        self._min_patch_size = max(1, int(min_patch_size))
        self._max_patch_size = max(self._min_patch_size, int(max_patch_size))
        self._trim_fraction = min(0.45, max(0.0, float(trim_fraction)))
        self._source_width = max(1, int(source_width))
        self._source_height = max(1, int(source_height))
        self._min_depth_m = float(min_depth_m)
        self._max_depth_m = float(max_depth_m)
        self._tf_timeout = float(tf_timeout_sec)
        self._enable_tf = bool(enable_tf)
        self._bridge = CvBridge()
        self._tf_buffer = None
        self._tf_listener = None

        self._depth = None
        self._depth_frame_id = ""
        self._fx = None
        self._fy = None
        self._cx = None
        self._cy = None

    def release_tf_listener(self) -> None:
        if self._tf_listener is None:
            return
        try:
            self._tf_listener.unregister()
        except Exception as exc:
            self._node.get_logger().debug(f"failed to unregister TF listener: {exc}")
        self._tf_listener = None
        self._tf_buffer = None

    def update_source_image_shape(self, image_shape) -> None:
        if not image_shape or len(image_shape) < 2:
            return
        self._source_height = max(1, int(image_shape[0]))
        self._source_width = max(1, int(image_shape[1]))

    @property
    def depth_frame_id(self) -> str:
        return self._depth_frame_id

    def update_camera_info(self, msg: CameraInfo) -> None:
        self._fx = float(msg.k[0])
        self._fy = float(msg.k[4])
        self._cx = float(msg.k[2])
        self._cy = float(msg.k[5])
        if msg.header.frame_id:
            self._depth_frame_id = msg.header.frame_id

    def update_depth(self, msg: Image) -> None:
        cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if cv_img.dtype == np.uint16:
            self._depth = cv_img.astype(np.float32) / 1000.0
        elif cv_img.dtype in (np.float32, np.float64):
            self._depth = cv_img.astype(np.float32)
        else:
            raise ValueError(f"unsupported depth dtype: {cv_img.dtype}")
        self._depth_frame_id = msg.header.frame_id or self._depth_frame_id

    def project_bbox_to_map(self, box: tuple[int, int, int, int]) -> Optional[list[float]]:
        if not self._enable_tf:
            return None
        camera_point = self.project_bbox_to_camera(box)
        if camera_point is None:
            return None
        x_c, y_c, z_c = camera_point
        if not self._depth_frame_id:
            return None

        self._ensure_tf_listener()
        if self._tf_buffer is None:
            return None

        try:
            transform = self._tf_buffer.lookup_transform(
                self._target_frame,
                self._depth_frame_id,
                self._node.get_clock().now(),
                timeout=Duration(seconds=self._tf_timeout),
            )
        except TransformException:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._target_frame,
                    self._depth_frame_id,
                    Time(),
                    timeout=Duration(seconds=self._tf_timeout),
                )
            except TransformException:
                return None

        x_m, y_m, z_m = _apply_transform(x_c, y_c, z_c, transform.transform)
        return [round(x_m, 3), round(y_m, 3), round(z_m, 3)]

    def _ensure_tf_listener(self) -> None:
        if not self._enable_tf or self._tf_listener is not None:
            return
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self._node)

    def project_bbox_to_camera(self, box: tuple[int, int, int, int]) -> Optional[list[float]]:
        if self._depth is None or self._fx is None or self._fy is None:
            return None

        depth_img = self._depth
        h, w = depth_img.shape[:2]
        x_min, y_min, x_max, y_max = box
        scale_x = float(w) / max(1.0, float(self._source_width))
        scale_y = float(h) / max(1.0, float(self._source_height))
        u = int(round(((x_min + x_max) / 2.0) * scale_x))
        v = int(round(((y_min + y_max) / 2.0) * scale_y))
        if u < 0 or u >= w or v < 0 or v >= h:
            return None

        box_w = max(1.0, float(x_max - x_min) * scale_x)
        box_h = max(1.0, float(y_max - y_min) * scale_y)
        z = self._median_depth(depth_img, u, v, box_w, box_h)
        if z is None:
            return None

        x_c = (u - self._cx) * z / self._fx
        y_c = (v - self._cy) * z / self._fy
        z_c = z
        return [round(x_c, 3), round(y_c, 3), round(z_c, 3)]

    def _median_depth(self, depth_img, u: int, v: int, box_w: float, box_h: float) -> Optional[float]:
        patch_w = _clamp_odd(
            int(round(max(float(self._patch_size), box_w * self._patch_bbox_fraction))),
            self._min_patch_size,
            self._max_patch_size,
        )
        patch_h = _clamp_odd(
            int(round(max(float(self._patch_size), box_h * self._patch_bbox_fraction))),
            self._min_patch_size,
            self._max_patch_size,
        )
        half_w = patch_w // 2
        half_h = patch_h // 2
        y1 = max(0, v - half_h)
        y2 = min(depth_img.shape[0], v + half_h + 1)
        x1 = max(0, u - half_w)
        x2 = min(depth_img.shape[1], u + half_w + 1)
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
        return float(np.median(valid))


def _apply_transform(x: float, y: float, z: float, transform) -> tuple[float, float, float]:
    q = transform.rotation
    tx = float(transform.translation.x)
    ty = float(transform.translation.y)
    tz = float(transform.translation.z)
    rx, ry, rz = _rotate_vector_by_quaternion(
        float(x), float(y), float(z), float(q.x), float(q.y), float(q.z), float(q.w)
    )
    return rx + tx, ry + ty, rz + tz


def _clamp_odd(value: int, minimum: int, maximum: int) -> int:
    value = max(int(minimum), min(int(maximum), int(value)))
    if value % 2 == 0:
        value += 1
    if value > maximum:
        value -= 2
    return max(1, value)


def _rotate_vector_by_quaternion(
    vx: float,
    vy: float,
    vz: float,
    qx: float,
    qy: float,
    qz: float,
    qw: float,
) -> tuple[float, float, float]:
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 0.0:
        return vx, vy, vz
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)

    rx = vx + qw * tx + (qy * tz - qz * ty)
    ry = vy + qw * ty + (qz * tx - qx * tz)
    rz = vz + qw * tz + (qx * ty - qy * tx)
    return rx, ry, rz

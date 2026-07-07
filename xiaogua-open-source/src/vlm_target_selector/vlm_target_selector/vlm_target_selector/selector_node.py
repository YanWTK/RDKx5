#!/usr/bin/env python3
"""Select one YOLO detection by asking a VLM to choose a numbered box."""

import base64
import copy
import os
import re
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
import rclpy
import requests
from rclpy._rclpy_pybind11 import RCLError
from ai_msgs.msg import PerceptionTargets
from cv_bridge import CvBridge
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .bailian_vlm_client import (
    BailianVLMClient,
    BailianVLMError,
    DEFAULT_BASE_URL as BAILIAN_DEFAULT_BASE_URL,
    DEFAULT_MODEL as BAILIAN_DEFAULT_MODEL,
)
from vlm_target_msgs.srv import SelectTarget


@dataclass
class Candidate:
    selected_id: int
    target: object
    target_type: str
    confidence: float
    box: tuple[int, int, int, int]


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


def _stamp_key(header) -> tuple[int, int]:
    return (int(header.stamp.sec), int(header.stamp.nanosec))


def _stamp_seconds(header) -> Optional[float]:
    key = _stamp_key(header)
    if key == (0, 0):
        return None
    return float(key[0]) + float(key[1]) * 1e-9


def _safe_filename(text: str) -> str:
    safe = re.sub(r'[^A-Za-z0-9_.-]+', '_', text).strip('_')
    return safe or 'target'


def _parse_selected_id(raw_reply, max_index: int) -> int:
    text = str(raw_reply or '').strip()
    if not text:
        return 0

    conclusion_patterns = (
        r'(?:答案|最终答案|选择|选|编号|物品编号|目标编号)\s*(?:是|为|:|：)?\s*(\d+)',
        r'(?:^|\n|\r)\s*(\d+)\s*$',
    )
    for pattern in conclusion_patterns:
        matches = re.findall(pattern, text, flags=re.IGNORECASE)
        for value in reversed(matches):
            selected = int(value)
            if 0 <= selected <= max_index:
                return selected

    numbers = [int(value) for value in re.findall(r'\d+', text)]
    valid = [value for value in numbers if 0 <= value <= max_index]
    if not valid:
        return 0
    return valid[-1]


class VlmTargetSelector(Node):
    """ROS2 service node for VLM arbitration over YOLO candidate boxes."""

    def __init__(self):
        super().__init__('vlm_target_selector')

        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('detection_topic', '/yolo_detector/detections')
        self.declare_parameter('use_local_vlm', True)
        self.declare_parameter('vlm_url', 'http://127.0.0.1:8000/analyze')
        self.declare_parameter('bailian_model', BAILIAN_DEFAULT_MODEL)
        self.declare_parameter('bailian_base_url', '')
        self.declare_parameter('bailian_api_key_env', 'DASHSCOPE_API_KEY')
        self.declare_parameter('bailian_enable_thinking', False)
        self.declare_parameter('service_name', '/vlm_target_selector/select_target')
        self.declare_parameter('current_target_topic', '/vlm_target_selector/current_target_name')
        self.declare_parameter('log_dir', '/opt/xiaogua/ros2_ws/vlm_logs')
        self.declare_parameter('request_timeout_sec', 20.0)
        self.declare_parameter('max_data_age_sec', 2.0)
        self.declare_parameter('max_pair_time_diff_sec', 0.5)
        self.declare_parameter('image_cache_size', 20)
        self.declare_parameter('selection_frame_count', 10)
        self.declare_parameter('selection_timeout_sec', 1.0)
        self.declare_parameter('always_save_debug_images', True)
        self.declare_parameter('task_gated', False)
        self.declare_parameter('fetch_state_topic', '/voice_fetch/state')
        self.declare_parameter(
            'prompt_template',
            '画面中有多个带编号的物品，用户需要【{target_name}】，'
            '只输出最终物品的纯数字编号，不要解释，不要列举分析。'
            '如果没有，请只输出 0。'
        )

        self._image_topic = self.get_parameter('image_topic').value
        self._detection_topic = self.get_parameter('detection_topic').value
        self._use_local_vlm = _as_bool(self.get_parameter('use_local_vlm').value)
        self._vlm_url = self.get_parameter('vlm_url').value
        self._bailian_model = self.get_parameter('bailian_model').value
        self._bailian_base_url = self.get_parameter('bailian_base_url').value.strip()
        self._bailian_api_key_env = self.get_parameter('bailian_api_key_env').value
        self._bailian_enable_thinking = _as_bool(
            self.get_parameter('bailian_enable_thinking').value
        )
        self._service_name = self.get_parameter('service_name').value
        self._current_target_topic = self.get_parameter('current_target_topic').value
        self._log_dir = self.get_parameter('log_dir').value
        self._timeout = float(self.get_parameter('request_timeout_sec').value)
        self._max_age = float(self.get_parameter('max_data_age_sec').value)
        self._max_pair_diff = float(self.get_parameter('max_pair_time_diff_sec').value)
        self._cache_size = int(self.get_parameter('image_cache_size').value)
        self._selection_frame_count = max(
            1, int(self.get_parameter('selection_frame_count').value)
        )
        self._selection_timeout = max(
            0.0, float(self.get_parameter('selection_timeout_sec').value)
        )
        self._always_save = _as_bool(self.get_parameter('always_save_debug_images').value)
        self._task_gated = _as_bool(self.get_parameter('task_gated').value)
        self._task_active = not self._task_gated
        self._prompt_template = self.get_parameter('prompt_template').value

        self._bridge = CvBridge()
        self._images = deque(maxlen=max(1, self._cache_size))
        self._image_lock = threading.Lock()
        self._detection_frames = deque(maxlen=max(self._selection_frame_count * 3, 10))
        self._frame_seq = 0
        self._frame_condition = threading.Condition()
        self._latest_detection = None
        self._latest_detection_received = None
        self._callback_group = ReentrantCallbackGroup()
        self._bailian_client = None

        self._image_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._image_sub = None
        self._detection_sub = None
        self._set_data_subscriptions(self._task_active)
        if self._task_gated:
            self.create_subscription(
                String,
                str(self.get_parameter('fetch_state_topic').value),
                self._on_fetch_state,
                10,
                callback_group=self._callback_group,
            )

        self._selected_pub = self.create_publisher(
            PerceptionTargets, '/vlm_target_selector/selected_detection', 10
        )
        current_target_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._current_target_pub = self.create_publisher(
            String, self._current_target_topic, current_target_qos
        )
        self._prompt_image_pub = self.create_publisher(
            Image, '/vlm_target_selector/prompt_image', 2
        )
        self._final_image_pub = self.create_publisher(
            Image, '/vlm_target_selector/final_image', 2
        )
        self.create_service(
            SelectTarget, self._service_name, self._on_select_target,
            callback_group=self._callback_group,
        )

        self.get_logger().info(
            'vlm_target_selector started | '
            f'image={self._image_topic} | detections={self._detection_topic} | '
            f'service={self._service_name} | '
            f'vlm_mode={"local" if self._use_local_vlm else "bailian"} | '
            f'vlm_url={self._vlm_url} | bailian_model={self._bailian_model} | '
            f'selection_frames={self._selection_frame_count} | '
            f'selection_timeout={self._selection_timeout:.2f}s | '
            f'task_gated={self._task_gated}'
        )

    def _on_image(self, msg: Image) -> None:
        if not self._task_active:
            return
        try:
            cv_image = self._image_to_bgr(msg)
        except Exception as exc:
            self.get_logger().warn(f'image conversion failed: {exc}')
            return

        with self._image_lock:
            self._images.append({
                'key': _stamp_key(msg.header),
                'header': copy.deepcopy(msg.header),
                'image': cv_image,
                'received': self.get_clock().now(),
            })

    def _on_detection(self, msg: PerceptionTargets) -> None:
        if not self._task_active:
            return
        received = self.get_clock().now()
        detection_msg = copy.deepcopy(msg)
        self._latest_detection = detection_msg
        self._latest_detection_received = received

        image_entry, error = self._find_image_for_detection(detection_msg)
        if error:
            self.get_logger().debug(f'skip detection frame cache: {error}')
            return

        candidates = self._extract_candidates(detection_msg)
        frame = {
            'seq': 0,
            'received': received,
            'image_entry': image_entry,
            'detection_msg': detection_msg,
            'candidates': candidates,
            'candidate_count': len(candidates),
            'confidence_sum': sum(c.confidence for c in candidates),
        }
        with self._frame_condition:
            self._frame_seq += 1
            frame['seq'] = self._frame_seq
            self._detection_frames.append(frame)
            self._frame_condition.notify_all()

    def _on_select_target(self, request, response):
        target_name = request.target_name.strip()
        if not target_name:
            return self._fail(response, 'target_name is empty')
        self._current_target_pub.publish(String(data=target_name))
        self._set_task_active(True)

        frame, error = self._select_best_detection_frame()
        if error:
            return self._fail(response, error)

        image_entry = frame['image_entry']
        detection_msg = frame['detection_msg']
        candidates = frame['candidates']
        if not candidates:
            return self._fail(response, 'no YOLO candidates available')

        original = image_entry['image']
        annotated = self._draw_prompt_image(original, candidates)
        self._publish_image(self._prompt_image_pub, annotated, image_entry['header'])

        save_debug = self._always_save or bool(request.save_debug_images)
        annotated_path = ''
        final_path = ''
        if save_debug:
            annotated_path = self._save_image(annotated, target_name, 'annotated_for_vlm')

        start_time = time.time()
        selected_id, raw_reply, vlm_error = self._ask_vlm(
            annotated,
            target_name,
            len(candidates),
        )
        elapsed = time.time() - start_time
        if vlm_error:
            return self._fail(response, vlm_error, annotated_path=annotated_path)

        if selected_id <= 0:
            msg = f'VLM returned {selected_id}; target not found. raw_reply={raw_reply!r}'
            return self._fail(response, msg, annotated_path=annotated_path)

        if selected_id > len(candidates):
            msg = (
                f'VLM returned id {selected_id}, but only {len(candidates)} '
                f'candidates exist. raw_reply={raw_reply!r}'
            )
            return self._fail(response, msg, annotated_path=annotated_path)

        selected = candidates[selected_id - 1]
        final_image = self._draw_final_image(original, selected, target_name)
        self._publish_image(self._final_image_pub, final_image, image_entry['header'])
        self._publish_selected_detection(detection_msg, selected)

        if save_debug:
            final_path = self._save_image(final_image, target_name, 'vlm_final_decision')

        x_min, y_min, x_max, y_max = selected.box
        response.success = True
        response.selected_id = selected.selected_id
        response.selected_type = selected.target_type
        response.confidence = float(selected.confidence)
        response.x_min = int(x_min)
        response.y_min = int(y_min)
        response.x_max = int(x_max)
        response.y_max = int(y_max)
        response.annotated_image_path = annotated_path
        response.final_image_path = final_path
        response.message = (
            f'selected id={selected.selected_id} type={selected.target_type} '
            f'conf={selected.confidence:.3f} '
            f'frame_seq={frame["seq"]} frame_candidates={frame["candidate_count"]} '
            f'frame_conf_sum={frame["confidence_sum"]:.3f} '
            f'elapsed={elapsed:.2f}s raw_reply={raw_reply!r}'
        )
        self.get_logger().info(response.message)
        return response

    def _on_fetch_state(self, msg: String) -> None:
        state = msg.data.strip()
        if state == 'idle':
            self._set_task_active(False)
        elif state == 'fetch_vision_active' or state in {
            'query_memory',
            'navigate_to_object_point',
            'confirm_target',
            'select_tracking_target',
            'wait_tracker_lock',
            'auto_aim',
        }:
            self._set_task_active(True)

    def _set_task_active(self, active: bool) -> None:
        if not self._task_gated:
            return
        if self._task_active == active:
            return
        self._task_active = active
        self._set_data_subscriptions(active)
        if not active:
            with self._image_lock:
                self._images.clear()
            with self._frame_condition:
                self._detection_frames.clear()
                self._latest_detection = None
                self._latest_detection_received = None
        self.get_logger().info(
            f'VLM selector image processing {"enabled" if active else "paused"}'
        )

    def _set_data_subscriptions(self, enabled: bool) -> None:
        if enabled:
            if self._image_sub is None:
                self._image_sub = self.create_subscription(
                    Image,
                    self._image_topic,
                    self._on_image,
                    self._image_qos,
                    callback_group=self._callback_group,
                )
            if self._detection_sub is None:
                self._detection_sub = self.create_subscription(
                    PerceptionTargets,
                    self._detection_topic,
                    self._on_detection,
                    10,
                    callback_group=self._callback_group,
                )
        else:
            if self._image_sub is not None:
                self.destroy_subscription(self._image_sub)
                self._image_sub = None
            if self._detection_sub is not None:
                self.destroy_subscription(self._detection_sub)
                self._detection_sub = None

    def _select_best_detection_frame(self):
        start_time = time.monotonic()
        with self._frame_condition:
            start_seq = self._frame_seq
            deadline = start_time + self._selection_timeout
            while True:
                frames = [
                    frame for frame in self._detection_frames
                    if frame['seq'] > start_seq
                ]
                if len(frames) >= self._selection_frame_count:
                    break
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self._frame_condition.wait(timeout=remaining)

            if not frames:
                frames = list(self._detection_frames)[-self._selection_frame_count:]

        if not frames:
            return None, 'no YOLO detection frames have been received'

        now = self.get_clock().now()
        fresh_frames = []
        for frame in frames:
            age = (now - frame['received']).nanoseconds * 1e-9
            if self._max_age <= 0.0 or age <= self._max_age:
                fresh_frames.append(frame)
        if not fresh_frames:
            return None, f'no fresh YOLO detection frames within {self._max_age:.2f}s'

        best = max(
            fresh_frames,
            key=lambda frame: (
                frame['candidate_count'],
                frame['confidence_sum'],
                frame['seq'],
            ),
        )
        wait_elapsed = time.monotonic() - start_time
        self.get_logger().info(
            f'selected VLM prompt frame seq={best["seq"]} '
            f'candidates={best["candidate_count"]} '
            f'conf_sum={best["confidence_sum"]:.3f} '
            f'from {len(fresh_frames)} frames in {wait_elapsed:.2f}s'
        )
        return best, None

    def _find_image_for_detection(self, detection_msg):
        with self._image_lock:
            if not self._images:
                return None, 'no color image has been received'

            images = list(self._images)

        latest_image = images[-1]

        det_key = _stamp_key(detection_msg.header)
        matched = None
        if det_key != (0, 0):
            for entry in reversed(images):
                if entry['key'] == det_key:
                    matched = entry
                    break
        image_entry = matched or latest_image

        image_stamp = _stamp_seconds(image_entry['header'])
        det_stamp = _stamp_seconds(detection_msg.header)
        if (
            matched is None
            and self._max_pair_diff > 0.0
            and image_stamp is not None
            and det_stamp is not None
        ):
            diff = abs(image_stamp - det_stamp)
            if diff > self._max_pair_diff:
                return None, (
                    f'image/detection timestamp mismatch: {diff:.3f}s '
                    f'(limit {self._max_pair_diff:.3f}s)'
                )

        return image_entry, None

    def _image_to_bgr(self, msg: Image):
        if msg.encoding in ('rgb8', 'bgr8'):
            return self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if msg.encoding in ('mono8', '8UC1'):
            mono = self._bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            return cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)
        if msg.encoding in ('16UC1', 'mono16'):
            raw = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if raw.max() > 0:
                norm = (raw.astype(np.float32) / raw.max() * 255).astype(np.uint8)
            else:
                norm = raw.astype(np.uint8)
            return cv2.cvtColor(norm, cv2.COLOR_GRAY2BGR)
        return self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _extract_candidates(self, msg: PerceptionTargets) -> list[Candidate]:
        candidates = []
        for target in msg.targets:
            if not target.rois:
                continue
            roi = target.rois[0]
            rect = roi.rect
            width = int(rect.width)
            height = int(rect.height)
            if width <= 0 or height <= 0:
                continue

            x_min = int(rect.x_offset)
            y_min = int(rect.y_offset)
            x_max = x_min + width
            y_max = y_min + height
            candidates.append(Candidate(
                selected_id=len(candidates) + 1,
                target=copy.deepcopy(target),
                target_type=target.type or 'unknown',
                confidence=float(getattr(roi, 'confidence', 0.0)),
                box=(x_min, y_min, x_max, y_max),
            ))
        return candidates

    def _draw_prompt_image(self, image_bgr, candidates: list[Candidate]):
        annotated = image_bgr.copy()
        h, w = annotated.shape[:2]
        for candidate in candidates:
            x_min, y_min, x_max, y_max = self._clamp_box(candidate.box, w, h)
            cv2.rectangle(annotated, (x_min, y_min), (x_max, y_max), (0, 0, 255), 3)
            label = str(candidate.selected_id)
            label_w = max(46, 22 * len(label) + 18)
            top = max(0, y_min - 38)
            cv2.rectangle(annotated, (x_min, top), (min(w - 1, x_min + label_w), y_min),
                          (0, 0, 255), -1)
            cv2.putText(annotated, label, (x_min + 10, max(24, y_min - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 3)
        return annotated

    def _draw_final_image(self, image_bgr, selected: Candidate, target_name: str):
        final = image_bgr.copy()
        h, w = final.shape[:2]
        x_min, y_min, x_max, y_max = self._clamp_box(selected.box, w, h)
        cv2.rectangle(final, (x_min, y_min), (x_max, y_max), (0, 255, 0), 4)
        label = f'Confirmed: {target_name}'
        cv2.putText(final, label, (x_min, max(28, y_min - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        return final

    def _ask_vlm(self, annotated_bgr, target_name: str, max_index: int):
        if self._use_local_vlm:
            return self._ask_local_vlm(annotated_bgr, target_name, max_index)
        return self._ask_bailian_vlm(annotated_bgr, target_name, max_index)

    def _ask_local_vlm(self, annotated_bgr, target_name: str, max_index: int):
        ok, buffer = cv2.imencode('.jpg', annotated_bgr)
        if not ok:
            return 0, '', 'failed to JPEG-encode prompt image'

        image_b64 = base64.b64encode(buffer).decode('utf-8')
        prompt = self._prompt_template.format(target_name=target_name)
        payload = {
            'prompt': prompt,
            'image': image_b64,
            'image_name': 'prompt_vision.jpg',
        }

        try:
            response = requests.post(self._vlm_url, json=payload, timeout=self._timeout)
        except Exception as exc:
            return 0, '', f'VLM request failed: {exc}'

        if response.status_code != 200:
            return 0, response.text[:200], f'VLM server returned HTTP {response.status_code}'

        try:
            body = response.json()
        except Exception as exc:
            return 0, response.text[:200], f'VLM response is not JSON: {exc}'

        raw_reply = str(
            body.get('ai_response')
            or body.get('response')
            or body.get('text')
            or ''
        ).strip()
        return _parse_selected_id(raw_reply, max_index), raw_reply, None

    def _ask_bailian_vlm(self, annotated_bgr, target_name: str, max_index: int):
        prompt = self._prompt_template.format(target_name=target_name)
        try:
            if self._bailian_client is None:
                self._bailian_client = BailianVLMClient(
                    api_key_env=self._bailian_api_key_env,
                    base_url=self._bailian_base_url or BAILIAN_DEFAULT_BASE_URL,
                    model=self._bailian_model,
                    timeout=self._timeout,
                    enable_thinking=self._bailian_enable_thinking,
                    component="vlm_target_selector",
                    log_callback=self.get_logger().info,
                )
            raw_reply = self._bailian_client.select_target_id(annotated_bgr, prompt)
        except BailianVLMError as exc:
            return 0, '', f'Bailian VLM request failed: {exc}'
        except Exception as exc:
            return 0, '', f'Unexpected Bailian VLM failure: {exc}'

        raw_reply = str(raw_reply).strip()
        return _parse_selected_id(raw_reply, max_index), raw_reply, None

    def _publish_selected_detection(self, detection_msg: PerceptionTargets, selected: Candidate) -> None:
        selected_msg = PerceptionTargets()
        selected_msg.header = detection_msg.header
        selected_msg.fps = detection_msg.fps
        selected_msg.perfs = copy.deepcopy(detection_msg.perfs)
        selected_msg.targets = [copy.deepcopy(selected.target)]
        self._selected_pub.publish(selected_msg)

    def _publish_image(self, publisher, image_bgr, header) -> None:
        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        msg = self._bridge.cv2_to_imgmsg(image_rgb, encoding='rgb8')
        msg.header = header
        publisher.publish(msg)

    def _save_image(self, image_bgr, target_name: str, stem: str) -> str:
        os.makedirs(self._log_dir, exist_ok=True)
        stamp = time.strftime('%Y%m%d_%H%M%S')
        path = os.path.join(self._log_dir, f'{stamp}_{_safe_filename(target_name)}_{stem}.jpg')
        if not cv2.imwrite(path, image_bgr):
            self.get_logger().warn(f'failed to save debug image: {path}')
            return ''
        return path

    @staticmethod
    def _clamp_box(box, width: int, height: int):
        x_min, y_min, x_max, y_max = box
        x_min = max(0, min(width - 1, int(x_min)))
        y_min = max(0, min(height - 1, int(y_min)))
        x_max = max(0, min(width - 1, int(x_max)))
        y_max = max(0, min(height - 1, int(y_max)))
        return x_min, y_min, x_max, y_max

    @staticmethod
    def _fail(response, message: str, annotated_path: str = ''):
        response.success = False
        response.selected_id = 0
        response.selected_type = ''
        response.confidence = 0.0
        response.x_min = 0
        response.y_min = 0
        response.x_max = 0
        response.y_max = 0
        response.annotated_image_path = annotated_path
        response.final_image_path = ''
        response.message = message
        return response


def main(args=None):
    rclpy.init(args=args)
    node = VlmTargetSelector()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        try:
            executor.shutdown()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

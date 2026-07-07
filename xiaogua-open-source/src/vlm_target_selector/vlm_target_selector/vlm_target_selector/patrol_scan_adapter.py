#!/usr/bin/env python3
"""Patrol scan adapter: YOLO detections + crop VLM + object memory + markers."""

from __future__ import annotations

import base64
import copy
import json
import re
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

import cv2
import numpy as np
import rclpy
import requests
from ai_msgs.msg import PerceptionTargets
from cv_bridge import CvBridge
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from .bailian_vlm_client import (
    BailianVLMClient,
    BailianVLMError,
    DEFAULT_BASE_URL as BAILIAN_DEFAULT_BASE_URL,
    DEFAULT_MODEL as BAILIAN_DEFAULT_MODEL,
)
from .depth_to_map_projector import DepthToMapProjector
from .memory_manager import ObjectMemoryManager
from .semantic_marker_publisher import SemanticMarkerPublisher


PATROL_WHITELIST = [
    "person",
    "cell phone",
    "mouse",
    "remote",
    "book",
    "bottle",
    "cup",
    "bowl",
    "apple",
    "banana",
    "teddy bear",
    "bag_wrapper",
    "box",
]

DEFAULT_BACKUP_YOLO_CLASSES = {
    "person": [],
    "cell phone": ["remote"],
    "mouse": ["remote"],
    "remote": ["cell phone"],
    "book": [],
    "bottle": ["cup"],
    "cup": ["bottle"],
    "bowl": ["cup"],
    "apple": ["banana"],
    "banana": ["apple"],
    "teddy bear": [],
    "bag_wrapper": ["box"],
    "box": ["bag_wrapper"],
}

DEFAULT_UNCERTAIN_NAMES = {
    "person": "不确定人物",
    "cell phone": "不确定手机",
    "mouse": "不确定鼠标",
    "remote": "不确定遥控器",
    "book": "不确定书本",
    "bottle": "不确定瓶子",
    "cup": "不确定杯子",
    "bowl": "不确定碗",
    "apple": "不确定苹果",
    "banana": "不确定香蕉",
    "teddy bear": "不确定玩偶",
    "bag_wrapper": "不确定包装袋",
    "box": "不确定盒子",
}

ITEM_PROMPT = (
    "请识别这张图片中的单个家庭物品。只输出 JSON。字段为 main_name 和 possible_names。"
    "main_name 和 possible_names 必须使用中文。"
    "如果不能确定具体物品，请输出不确定瓶子、不确定盒子等，不要编造过于确定的品牌。"
)


@dataclass
class Candidate:
    yolo_class: str
    confidence: float
    box: tuple[int, int, int, int]


class PatrolScanAdapter(Node):
    def __init__(self) -> None:
        super().__init__("patrol_scan_adapter")

        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/depth/camera_info")
        self.declare_parameter("detection_topic", "/yolo_detector/detections")
        self.declare_parameter("scan_cmd_topic", "/patrol_scan_cmd")
        self.declare_parameter("scan_done_topic", "/patrol_scan_done")
        self.declare_parameter("semantic_marker_topic", "/semantic_object_markers")
        self.declare_parameter("marker_text_scale", 0.10)
        self.declare_parameter("marker_text_z_offset", 0.14)
        self.declare_parameter("memory_path", "")
        self.declare_parameter("clear_memory_on_start", False)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("target_classes", ",".join(PATROL_WHITELIST))
        self.declare_parameter("min_detection_conf", 0.05)
        self.declare_parameter("max_data_age_sec", 2.0)
        self.declare_parameter("scan_settle_sec", 0.5)
        self.declare_parameter("capture_count", 5)
        self.declare_parameter("capture_interval_sec", 0.25)
        self.declare_parameter("process_only_when_scanning", False)
        self.declare_parameter("request_timeout_sec", 20.0)
        self.declare_parameter("use_local_vlm", True)
        self.declare_parameter("vlm_url", "http://127.0.0.1:8000/analyze")
        self.declare_parameter("bailian_model", BAILIAN_DEFAULT_MODEL)
        self.declare_parameter("bailian_base_url", "")
        self.declare_parameter("bailian_api_key_env", "DASHSCOPE_API_KEY")
        self.declare_parameter("bailian_enable_thinking", False)
        self.declare_parameter("prompt_template", ITEM_PROMPT)
        self.declare_parameter(
            "backup_yolo_classes_json",
            json.dumps(DEFAULT_BACKUP_YOLO_CLASSES, ensure_ascii=False),
        )
        self.declare_parameter(
            "uncertain_names_json",
            json.dumps(DEFAULT_UNCERTAIN_NAMES, ensure_ascii=False),
        )

        self._bridge = CvBridge()
        self._callback_group = ReentrantCallbackGroup()
        self._cache_lock = threading.RLock()
        self._scan_lock = threading.Lock()

        self._latest_image = None
        self._latest_image_header = None
        self._latest_image_received = 0.0
        self._latest_detection = None
        self._latest_detection_received = 0.0
        self._scan_active = False

        self._target_classes = _parse_classes(self.get_parameter("target_classes").value)
        self._min_conf = float(self.get_parameter("min_detection_conf").value)
        self._max_age = float(self.get_parameter("max_data_age_sec").value)
        self._scan_settle = float(self.get_parameter("scan_settle_sec").value)
        self._capture_count = max(1, int(self.get_parameter("capture_count").value))
        self._capture_interval = max(0.0, float(self.get_parameter("capture_interval_sec").value))
        self._process_only_when_scanning = _as_bool(
            self.get_parameter("process_only_when_scanning").value
        )
        self._timeout = float(self.get_parameter("request_timeout_sec").value)
        self._use_local_vlm = _as_bool(self.get_parameter("use_local_vlm").value)
        self._vlm_url = str(self.get_parameter("vlm_url").value)
        self._prompt_template = str(self.get_parameter("prompt_template").value)
        self._backup_yolo_classes = _parse_string_list_map(
            self.get_parameter("backup_yolo_classes_json").value,
            DEFAULT_BACKUP_YOLO_CLASSES,
        )
        self._uncertain_names = _parse_string_map(
            self.get_parameter("uncertain_names_json").value,
            DEFAULT_UNCERTAIN_NAMES,
        )

        memory_path = str(self.get_parameter("memory_path").value).strip()
        if not memory_path:
            memory_path = str(Path.home() / ".ros" / "patrol_memory" / "object_memory.json")
        self._memory = ObjectMemoryManager(memory_path)
        self._clear_memory_on_start = _as_bool(
            self.get_parameter("clear_memory_on_start").value
        )
        if self._clear_memory_on_start:
            backup_path = self._memory.backup_and_clear()
            if backup_path is not None:
                self.get_logger().info(
                    f"backed up object memory to {backup_path} and cleared {self._memory.path}"
                )
            else:
                self.get_logger().info(f"cleared object memory on start: {self._memory.path}")

        self._projector = DepthToMapProjector(
            self,
            target_frame=str(self.get_parameter("map_frame").value),
        )
        self._markers = SemanticMarkerPublisher(
            self,
            topic=str(self.get_parameter("semantic_marker_topic").value),
            frame_id=str(self.get_parameter("map_frame").value),
            text_scale=float(self.get_parameter("marker_text_scale").value),
            text_z_offset=float(self.get_parameter("marker_text_z_offset").value),
        )

        self._bailian_client = None
        self._done_pub = self.create_publisher(
            String, str(self.get_parameter("scan_done_topic").value), 10
        )

        self._image_topic = str(self.get_parameter("image_topic").value)
        self._depth_topic = str(self.get_parameter("depth_topic").value)
        self._detection_topic = str(self.get_parameter("detection_topic").value)
        self._sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._image_sub = None
        self._depth_sub = None
        self._detection_sub = None
        self._camera_info_ready = False
        self._data_subscriptions_wanted = not self._process_only_when_scanning
        self._set_data_subscriptions(not self._process_only_when_scanning)
        self._camera_info_sub = self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self._on_camera_info,
            10,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("scan_cmd_topic").value),
            self._on_scan_cmd,
            10,
            callback_group=self._callback_group,
        )

        self._markers.publish(self._memory.objects)
        self.get_logger().info(
            "patrol_scan_adapter started | "
            f"classes={','.join(self._target_classes)} | "
            f"capture_count={self._capture_count} | "
            f"memory={self._memory.path} | "
            f"vlm_mode={'local' if self._use_local_vlm else 'bailian'} | "
            f"scan_gated={self._process_only_when_scanning}"
        )

    def _on_image(self, msg: Image) -> None:
        if self._process_only_when_scanning and not self._scan_active:
            return
        try:
            image = self._image_to_bgr(msg)
        except Exception as exc:
            self.get_logger().warn(f"image conversion failed: {exc}")
            return
        self._projector.update_source_image_shape(image.shape)
        with self._cache_lock:
            self._latest_image = image
            self._latest_image_header = copy.deepcopy(msg.header)
            self._latest_image_received = time.monotonic()

    def _on_depth(self, msg: Image) -> None:
        if self._process_only_when_scanning and not self._scan_active:
            return
        try:
            self._projector.update_depth(msg)
        except Exception as exc:
            self.get_logger().warn(f"depth update failed: {exc}")

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if self._camera_info_ready:
            return
        try:
            self._projector.update_camera_info(msg)
            self._camera_info_ready = True
        except Exception as exc:
            self.get_logger().warn(f"camera info update failed: {exc}")

    def _on_detection(self, msg: PerceptionTargets) -> None:
        if self._process_only_when_scanning and not self._scan_active:
            return
        with self._cache_lock:
            self._latest_detection = copy.deepcopy(msg)
            self._latest_detection_received = time.monotonic()

    def _on_scan_cmd(self, msg: String) -> None:
        try:
            cmd = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warn(f"invalid patrol scan cmd JSON: {exc}")
            self._publish_done("", False, 0, 0, f"invalid JSON: {exc}")
            return

        if not self._scan_lock.acquire(blocking=False):
            point_id = str(cmd.get("point_id") or "")
            self._publish_done(point_id, False, 0, 0, "scan already running")
            return

        self._set_scan_active(True)
        threading.Thread(target=self._run_scan_cmd, args=(cmd,), daemon=True).start()

    def _run_scan_cmd(self, cmd: dict[str, Any]) -> None:
        point_id = str(cmd.get("point_id") or "").strip()
        try:
            if not point_id:
                raise ValueError("point_id is empty")

            if str(cmd.get("command") or "").strip() == "start_patrol":
                self._reset_memory_for_patrol()
                self._publish_done(point_id, True, 0, 0, "")
                return

            if _as_bool(cmd.get("reset_memory", False)):
                self._reset_memory_for_patrol()

            if "scan_angles" in cmd:
                self.get_logger().info(
                    "scan_angles is ignored; patrol memory uses fixed robot pose "
                    "and selects the best frame from repeated captures"
                )
            capture_count = _positive_int(cmd.get("capture_count"), self._capture_count)
            result = self._scan_once(point_id, capture_count)
            seen = result["seen"]
            updated = result["updated"]

            self._markers.publish(self._memory.objects)
            self._publish_done(point_id, True, seen, updated, "")
        except Exception as exc:
            self.get_logger().warn(f"patrol scan failed: {exc}")
            self._publish_done(point_id, False, 0, 0, str(exc))
        finally:
            self._set_scan_active(False)
            self._scan_lock.release()

    def _reset_memory_for_patrol(self) -> None:
        backup_path = self._memory.backup_and_clear()
        self._markers.publish(self._memory.objects)
        if backup_path is not None:
            self.get_logger().info(
                f"patrol started: backed up object memory to {backup_path} "
                f"and cleared {self._memory.path}"
            )
        else:
            self.get_logger().info(
                f"patrol started: cleared object memory {self._memory.path}"
            )

    def _set_scan_active(self, active: bool) -> None:
        self._scan_active = active
        if self._process_only_when_scanning:
            self._data_subscriptions_wanted = active
            self._set_data_subscriptions(active)
            if not active:
                self._projector.release_tf_listener()
        if active:
            with self._cache_lock:
                self._latest_image = None
                self._latest_image_header = None
                self._latest_image_received = 0.0
                self._latest_detection = None
                self._latest_detection_received = 0.0
        self.get_logger().info(
            f"patrol image processing {'enabled' if active else 'paused'}"
        )

    def _set_data_subscriptions(self, enabled: bool) -> None:
        if self._image_sub is None:
            self._image_sub = self.create_subscription(
                Image,
                self._image_topic,
                self._on_image,
                self._sensor_qos,
                callback_group=self._callback_group,
            )
        if self._depth_sub is None:
            self._depth_sub = self.create_subscription(
                Image,
                self._depth_topic,
                self._on_depth,
                self._sensor_qos,
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

    def _scan_once(self, point_id: str, capture_count: int) -> dict[str, int]:
        if self._scan_settle > 0.0:
            time.sleep(self._scan_settle)

        frame = self._select_best_capture(capture_count)
        if frame is None:
            self.get_logger().warn(
                f"point={point_id}: no usable whitelist detections from "
                f"{capture_count} fixed-pose captures"
            )
            return {"seen": 0, "updated": 0}

        image = frame["image"]
        candidates = frame["candidates"]
        if not candidates:
            self.get_logger().info(f"point={point_id}: no whitelist detections")
            return {"seen": 0, "updated": 0}

        seen = 0
        updated = 0
        for candidate in candidates:
            try:
                crop = _crop_image(image, candidate.box)
                if crop is None:
                    continue

                vlm = self._ask_vlm_for_item(crop, candidate.yolo_class)
                map_position = self._projector.project_bbox_to_map(candidate.box)
                item = {
                    "main_name": vlm["main_name"],
                    "possible_names": vlm["possible_names"],
                    "yolo_class": candidate.yolo_class,
                    "backup_yolo_classes": self._backup_yolo_classes.get(candidate.yolo_class, []),
                    "point_id": point_id,
                    "detected_at_point": point_id,
                    "_marker_position": map_position,
                }
                _, created = self._memory.upsert(item)
                seen += 1
                if not created:
                    updated += 1
            except Exception as exc:
                self.get_logger().warn(
                    f"object memory update failed class={candidate.yolo_class}: {exc}"
                )

        self._markers.publish(self._memory.objects)
        self.get_logger().info(
            f"point={point_id}: selected capture={frame['index'] + 1}/{capture_count} "
            f"score={frame['score']:.3f} candidates={len(candidates)} "
            f"remembered={seen} updated={updated}"
        )
        return {"seen": seen, "updated": updated}

    def _select_best_capture(self, capture_count: int) -> Optional[dict[str, Any]]:
        best = None
        for index in range(max(1, capture_count)):
            if index > 0 and self._capture_interval > 0.0:
                time.sleep(self._capture_interval)

            image, detection, error = self._snapshot()
            if error:
                self.get_logger().warn(f"capture {index + 1}/{capture_count} skipped: {error}")
                continue

            candidates = self._extract_candidates(detection, image.shape)
            sharpness = _image_sharpness(image)
            confidence_sum = sum(c.confidence for c in candidates)
            score = len(candidates) * 100.0 + confidence_sum * 10.0 + sharpness * 0.001
            frame = {
                "index": index,
                "image": image,
                "candidates": candidates,
                "score": score,
                "sharpness": sharpness,
                "confidence_sum": confidence_sum,
            }
            if best is None or frame["score"] > best["score"]:
                best = frame

        if best is not None:
            self.get_logger().info(
                f"best fixed-pose capture {best['index'] + 1}/{capture_count}: "
                f"candidates={len(best['candidates'])} "
                f"conf_sum={best['confidence_sum']:.3f} "
                f"sharpness={best['sharpness']:.1f}"
            )
        return best

    def _snapshot(self):
        with self._cache_lock:
            image = None if self._latest_image is None else self._latest_image.copy()
            detection = copy.deepcopy(self._latest_detection)
            image_age = time.monotonic() - self._latest_image_received
            det_age = time.monotonic() - self._latest_detection_received

        if image is None:
            return None, None, "no RGB image has been received"
        if detection is None:
            return None, None, "no YOLO detections have been received"
        if self._max_age > 0.0 and image_age > self._max_age:
            return None, None, f"RGB image is stale: {image_age:.2f}s"
        if self._max_age > 0.0 and det_age > self._max_age:
            return None, None, f"YOLO detection is stale: {det_age:.2f}s"
        return image, detection, ""

    def _extract_candidates(self, msg: PerceptionTargets, image_shape) -> list[Candidate]:
        h, w = image_shape[:2]
        candidates: list[Candidate] = []
        for target in msg.targets:
            try:
                yolo_class = str(target.type or "").strip()
                if yolo_class not in self._target_classes:
                    continue
                if not target.rois:
                    continue
                roi = target.rois[0]
                conf = float(getattr(roi, "confidence", 0.0))
                if conf < self._min_conf:
                    continue
                rect = roi.rect
                x1 = max(0, min(w - 1, int(rect.x_offset)))
                y1 = max(0, min(h - 1, int(rect.y_offset)))
                x2 = max(0, min(w - 1, int(rect.x_offset + rect.width)))
                y2 = max(0, min(h - 1, int(rect.y_offset + rect.height)))
                if x2 <= x1 or y2 <= y1:
                    continue
                candidates.append(Candidate(yolo_class, conf, (x1, y1, x2, y2)))
            except Exception as exc:
                self.get_logger().warn(f"bad YOLO target skipped: {exc}")
        return candidates

    def _ask_vlm_for_item(self, crop_bgr, yolo_class: str) -> dict[str, Any]:
        try:
            raw_reply = self._ask_local_vlm(crop_bgr) if self._use_local_vlm else self._ask_bailian_vlm(crop_bgr)
            parsed = _parse_vlm_item_json(raw_reply)
            if parsed is not None:
                return parsed
            self.get_logger().warn(f"VLM reply is not expected JSON: {raw_reply[:160]!r}")
        except Exception as exc:
            self.get_logger().warn(f"VLM item recognition failed: {exc}")

        fallback = self._uncertain_names.get(yolo_class, "不确定物品")
        return {"main_name": fallback, "possible_names": [fallback, yolo_class]}

    def _ask_local_vlm(self, crop_bgr) -> str:
        ok, buffer = cv2.imencode(".jpg", crop_bgr)
        if not ok:
            raise RuntimeError("failed to JPEG-encode crop")
        payload = {
            "prompt": self._prompt_template,
            "image": base64.b64encode(buffer).decode("ascii"),
            "image_name": "patrol_crop.jpg",
        }
        response = requests.post(self._vlm_url, json=payload, timeout=self._timeout)
        if response.status_code != 200:
            raise RuntimeError(f"local VLM HTTP {response.status_code}: {response.text[:160]}")
        body = response.json()
        return str(body.get("ai_response") or body.get("response") or body.get("text") or "")

    def _ask_bailian_vlm(self, crop_bgr) -> str:
        if self._bailian_client is None:
            self._bailian_client = BailianVLMClient(
                api_key_env=str(self.get_parameter("bailian_api_key_env").value),
                base_url=str(self.get_parameter("bailian_base_url").value).strip()
                or BAILIAN_DEFAULT_BASE_URL,
                model=str(self.get_parameter("bailian_model").value),
                timeout=self._timeout,
                enable_thinking=_as_bool(self.get_parameter("bailian_enable_thinking").value),
            )
        try:
            return self._bailian_client.select_target_id(crop_bgr, self._prompt_template)
        except BailianVLMError:
            raise

    def _publish_done(
        self,
        point_id: str,
        success: bool,
        objects_seen: int,
        objects_updated: int,
        error: str,
    ) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                "point_id": point_id,
                "success": bool(success),
                "objects_seen": int(objects_seen),
                "objects_updated": int(objects_updated),
                "error": error,
            },
            ensure_ascii=False,
        )
        self._done_pub.publish(msg)

    def _image_to_bgr(self, msg: Image):
        if msg.encoding in ("rgb8", "bgr8"):
            return self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if msg.encoding in ("mono8", "8UC1"):
            mono = self._bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
            return cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)
        return self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")


def _parse_classes(value: Any) -> set[str]:
    if isinstance(value, list):
        return {str(v).strip() for v in value if str(v).strip()}
    return {v.strip() for v in str(value).split(",") if v.strip()}


def _parse_string_list_map(value: Any, default: dict[str, list[str]]) -> dict[str, list[str]]:
    try:
        data = json.loads(value) if isinstance(value, str) else value
    except Exception:
        return {key: list(vals) for key, vals in default.items()}
    if not isinstance(data, dict):
        return {key: list(vals) for key, vals in default.items()}
    result: dict[str, list[str]] = {}
    for key, vals in data.items():
        name = str(key).strip()
        if not name:
            continue
        if isinstance(vals, list):
            result[name] = [str(v).strip() for v in vals if str(v).strip()]
        else:
            result[name] = []
    return result


def _parse_string_map(value: Any, default: dict[str, str]) -> dict[str, str]:
    try:
        data = json.loads(value) if isinstance(value, str) else value
    except Exception:
        return dict(default)
    if not isinstance(data, dict):
        return dict(default)
    result: dict[str, str] = {}
    for key, val in data.items():
        name = str(key).strip()
        text = str(val).strip()
        if name and text:
            result[name] = text
    return result or dict(default)


def _positive_int(value: Any, default: int) -> int:
    try:
        return max(1, int(value))
    except Exception:
        return max(1, int(default))


def _image_sharpness(image_bgr) -> float:
    try:
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        return float(cv2.Laplacian(gray, cv2.CV_64F).var())
    except Exception:
        return 0.0


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def _crop_image(image, box: tuple[int, int, int, int]) -> Optional[np.ndarray]:
    x1, y1, x2, y2 = box
    if x2 <= x1 or y2 <= y1:
        return None
    return image[y1:y2, x1:x2].copy()


def _parse_vlm_item_json(text: str) -> Optional[dict[str, Any]]:
    raw = (text or "").strip()
    if raw.startswith("```"):
        raw = re.sub(r"^```(?:json)?", "", raw, flags=re.IGNORECASE).strip()
        raw = re.sub(r"```$", "", raw).strip()
    match = re.search(r"\{.*\}", raw, flags=re.DOTALL)
    if match:
        raw = match.group(0)

    try:
        data = json.loads(raw)
    except Exception:
        return None
    main_name = str(data.get("main_name") or "").strip()
    possible = data.get("possible_names")
    if not main_name or not isinstance(possible, list):
        return None
    possible_names = [str(v).strip() for v in possible if str(v).strip()]
    if not possible_names:
        possible_names = [main_name]
    return {"main_name": main_name, "possible_names": possible_names}


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PatrolScanAdapter()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            executor.shutdown()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Confirm a remembered target at the current patrol point with YOLO + VLM."""

from __future__ import annotations

import copy
import json
import threading
import time
from pathlib import Path
from typing import Any

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String

from yolo_detector.yolo_engine import COCO_CLASSES, YOLOEngine

from .bailian_vlm_client import (
    BailianVLMClient,
    DEFAULT_BASE_URL as BAILIAN_DEFAULT_BASE_URL,
    DEFAULT_MODEL as BAILIAN_DEFAULT_MODEL,
)
from .depth_to_map_projector import DepthToMapProjector
from .numbered_candidate_drawer import draw_numbered_boxes, draw_selected_box
from .vlm_number_selector import DEFAULT_NUMBER_PROMPT, VlmNumberSelector


DEFAULT_MODEL_PATH = "/opt/xiaogua/models/yolo_model.bin"
DEFAULT_CLASS_NAMES = "person,cell phone,mouse,remote,book,bottle,cup,bowl,apple,banana,teddy bear,bag_wrapper,box"


class TargetConfirmNode(Node):
    def __init__(self) -> None:
        super().__init__("target_confirm")

        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/depth/camera_info")
        self.declare_parameter("confirm_cmd_topic", "/target_confirm/confirm_cmd")
        self.declare_parameter("confirm_result_topic", "/target_confirm/result")
        self.declare_parameter("numbered_image_topic", "/target_confirm/numbered_image")
        self.declare_parameter("final_image_topic", "/target_confirm/final_image")
        self.declare_parameter("object_memory_path", "")
        self.declare_parameter("model_path", DEFAULT_MODEL_PATH)
        self.declare_parameter("class_names", DEFAULT_CLASS_NAMES)
        self.declare_parameter("preprocess_mode", "letterbox")
        self.declare_parameter("conf_threshold", 0.07)
        self.declare_parameter("nms_threshold", 0.7)
        self.declare_parameter("capture_count", 5)
        self.declare_parameter("capture_interval_sec", 0.25)
        self.declare_parameter("max_image_age_sec", 2.0)
        self.declare_parameter("request_timeout_sec", 20.0)
        self.declare_parameter("use_local_vlm", False)
        self.declare_parameter("vlm_url", "http://127.0.0.1:8000/analyze")
        self.declare_parameter("bailian_model", BAILIAN_DEFAULT_MODEL)
        self.declare_parameter("bailian_base_url", "")
        self.declare_parameter("bailian_api_key_env", "DASHSCOPE_API_KEY")
        self.declare_parameter("bailian_enable_thinking", False)
        self.declare_parameter("prompt_template", DEFAULT_NUMBER_PROMPT)
        self.declare_parameter("task_gated", False)
        self.declare_parameter("fetch_state_topic", "/voice_fetch/state")
        self.declare_parameter("active_topic", "/yolo_detector/active")

        self._bridge = CvBridge()
        self._callback_group = ReentrantCallbackGroup()
        self._cache_lock = threading.RLock()
        self._confirm_lock = threading.Lock()
        self._latest_image = None
        self._latest_image_header = None
        self._latest_image_received = 0.0
        self._task_gated = _as_bool(self.get_parameter("task_gated").value)
        self._task_active = not self._task_gated

        self._model_path = str(self.get_parameter("model_path").value)
        self._class_names_param = str(self.get_parameter("class_names").value)
        self._preprocess_mode = str(self.get_parameter("preprocess_mode").value)
        self._conf = float(self.get_parameter("conf_threshold").value)
        self._nms = float(self.get_parameter("nms_threshold").value)
        self._capture_count = max(1, int(self.get_parameter("capture_count").value))
        self._capture_interval = max(0.0, float(self.get_parameter("capture_interval_sec").value))
        self._max_image_age = float(self.get_parameter("max_image_age_sec").value)
        self._timeout = float(self.get_parameter("request_timeout_sec").value)
        self._use_local_vlm = _as_bool(self.get_parameter("use_local_vlm").value)
        self._vlm_url = str(self.get_parameter("vlm_url").value)
        self._prompt_template = str(self.get_parameter("prompt_template").value)

        memory_path = str(self.get_parameter("object_memory_path").value).strip()
        if not memory_path:
            memory_path = str(Path.home() / ".ros" / "patrol_memory" / "object_memory.json")
        self._memory_path = Path(memory_path).expanduser()

        self._engine = YOLOEngine(
            self._model_path,
            self._conf,
            self._nms,
            class_names=self._class_names_param,
            preprocess_mode=self._preprocess_mode,
        )
        self._projector = DepthToMapProjector(self, target_frame="map", enable_tf=False)
        self._bailian_client = None

        self._result_pub = self.create_publisher(
            String, str(self.get_parameter("confirm_result_topic").value), 10
        )
        self._numbered_image_pub = self.create_publisher(
            Image, str(self.get_parameter("numbered_image_topic").value), 2
        )
        self._final_image_pub = self.create_publisher(
            Image, str(self.get_parameter("final_image_topic").value), 2
        )

        self._image_topic = str(self.get_parameter("image_topic").value)
        self._depth_topic = str(self.get_parameter("depth_topic").value)
        self._sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._image_sub = None
        self._depth_sub = None
        self._set_sensor_subscriptions(True)
        self._camera_info_sub = self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self._on_camera_info,
            10,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("confirm_cmd_topic").value),
            self._on_confirm_cmd,
            10,
            callback_group=self._callback_group,
        )
        if self._task_gated:
            self.create_subscription(
                Bool,
                str(self.get_parameter("active_topic").value),
                self._on_active,
                QoSProfile(
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                ),
                callback_group=self._callback_group,
            )
            self.create_subscription(
                String,
                str(self.get_parameter("fetch_state_topic").value),
                self._on_fetch_state,
                10,
                callback_group=self._callback_group,
            )

        self.get_logger().info(
            "target_confirm started | "
            f"image={self.get_parameter('image_topic').value} | "
            f"model={self._model_path} | preprocess={self._preprocess_mode} | "
            f"vlm_mode={'local' if self._use_local_vlm else 'bailian'} | "
            f"memory={self._memory_path} | task_gated={self._task_gated}"
        )

    def _on_image(self, msg: Image) -> None:
        if not self._task_active:
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
        if not self._task_active:
            return
        try:
            self._projector.update_depth(msg)
        except Exception as exc:
            self.get_logger().warn(f"depth update failed: {exc}")

    def _on_camera_info(self, msg: CameraInfo) -> None:
        try:
            self._projector.update_camera_info(msg)
        except Exception as exc:
            self.get_logger().warn(f"camera info update failed: {exc}")

    def _on_confirm_cmd(self, msg: String) -> None:
        try:
            cmd = json.loads(msg.data)
        except Exception as exc:
            self._publish_result(_fail(f"invalid JSON: {exc}"))
            return

        if not self._confirm_lock.acquire(blocking=False):
            self._publish_result(_fail("confirmation already running"))
            return
        activated_now = self._set_task_active(True)
        threading.Thread(
            target=self._run_confirm,
            args=(cmd, activated_now),
            daemon=True,
        ).start()

    def _on_active(self, msg: Bool) -> None:
        if self._task_gated and not bool(msg.data):
            self._set_task_active(False)

    def _on_fetch_state(self, msg: String) -> None:
        state = msg.data.strip()
        if state in {
            "idle",
            "received_command",
            "turn_to_speaker",
            "record_person_pose",
            "understand_command",
            "query_memory",
            "navigate_to_object_point",
        }:
            self._set_task_active(False)
        elif state == "fetch_vision_active" or state in {
            "confirm_target",
            "select_tracking_target",
            "wait_tracker_lock",
            "auto_aim",
        }:
            self._set_task_active(True)

    def _set_task_active(self, active: bool) -> bool:
        if not self._task_gated:
            return False
        if self._task_active == active:
            return False
        self._task_active = active
        if not active:
            with self._cache_lock:
                self._latest_image = None
                self._latest_image_header = None
                self._latest_image_received = 0.0
        self.get_logger().info(
            f"target confirm image processing {'enabled' if active else 'paused'}"
        )
        return True

    def _set_sensor_subscriptions(self, enabled: bool) -> None:
        if not enabled:
            return
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

    def _wait_for_first_image(self, timeout_sec: float = 0.75) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            with self._cache_lock:
                if self._latest_image is not None:
                    return
            time.sleep(0.02)

    def _run_confirm(self, cmd: dict[str, Any], warmup_needed: bool = False) -> None:
        try:
            if warmup_needed:
                self._wait_for_first_image()
            target_obj = self._resolve_target_obj(cmd)
            target_name = str(cmd.get("target_name") or cmd.get("user_query") or target_obj.get("main_name") or "").strip()
            if not target_name:
                target_name = str(target_obj.get("id") or "target")

            target_classes = _target_classes_from_obj(target_obj)
            class_ids, unknown = _resolve_class_ids(target_classes, self._engine.class_names)
            if unknown:
                self.get_logger().warn(f"unknown YOLO classes ignored: {unknown}")
            if not class_ids:
                self._publish_result(_fail("target object has no valid YOLO classes"))
                return

            capture_count = _positive_int(cmd.get("capture_count"), self._capture_count)
            frame = self._select_best_frame(capture_count, class_ids)
            if frame is None or not frame["detections"]:
                self._publish_result({
                    "target_found": False,
                    "selected_index": 0,
                    "reason": "YOLO has no valid candidate",
                    "target_name": target_name,
                    "target_classes": target_classes,
                    "point_id": target_obj.get("point_id"),
                })
                return

            numbered_image, indexed = draw_numbered_boxes(frame["image"], frame["detections"])
            self._publish_image(self._numbered_image_pub, numbered_image, frame["header"])
            if not indexed:
                self._publish_result(_fail("numbered candidate drawing produced no valid boxes"))
                return

            selector = self._number_selector()
            try:
                selected_index, raw_reply = selector.select_target_by_vlm(
                    numbered_image,
                    target_name,
                    max_index=len(indexed),
                )
            except Exception as exc:
                self._publish_result({
                    "target_found": False,
                    "selected_index": 0,
                    "reason": f"VLM selection failed: {exc}",
                    "target_name": target_name,
                    "target_classes": target_classes,
                    "candidate_count": len(indexed),
                })
                return

            if selected_index <= 0 or selected_index > len(indexed):
                self._publish_result({
                    "target_found": False,
                    "selected_index": 0,
                    "reason": "VLM returned 0 or no valid candidate",
                    "raw_vlm_reply": raw_reply,
                    "target_name": target_name,
                    "target_classes": target_classes,
                    "candidate_count": len(indexed),
                })
                return

            selected = indexed[selected_index - 1]
            final_image = draw_selected_box(frame["image"], selected, target_name)
            self._publish_image(self._final_image_pub, final_image, frame["header"])
            object_3d = self._projector.project_bbox_to_camera(tuple(selected["bbox"]))

            result = {
                "target_found": True,
                "selected_index": int(selected_index),
                "selected_bbox": selected["bbox"],
                "selected_yolo_class": selected.get("yolo_class"),
                "confidence": float(selected.get("confidence", 0.0)),
                "target_name": target_name,
                "target_id": target_obj.get("id"),
                "point_id": target_obj.get("point_id"),
                "object_3d_position": object_3d,
                "object_3d_frame": self._projector.depth_frame_id,
                "raw_vlm_reply": raw_reply,
            }
            self._publish_result(result)
        except Exception as exc:
            self.get_logger().warn(f"target confirmation failed: {exc}")
            self._publish_result(_fail(str(exc)))
        finally:
            self._confirm_lock.release()

    def _resolve_target_obj(self, cmd: dict[str, Any]) -> dict[str, Any]:
        target_obj = cmd.get("target_obj")
        if isinstance(target_obj, dict):
            return target_obj
        if "id" in cmd and "yolo_class" in cmd:
            return cmd

        target_id = str(cmd.get("target_id") or "").strip()
        if not target_id:
            raise ValueError("confirm command needs target_obj or target_id")
        if not self._memory_path.exists():
            raise ValueError(f"object memory file does not exist: {self._memory_path}")
        with self._memory_path.open("r", encoding="utf-8") as f:
            data = json.load(f)
        if not isinstance(data, list):
            raise ValueError("object memory JSON is not a list")
        for obj in data:
            if str(obj.get("id") or "") == target_id:
                return obj
        raise ValueError(f"target_id not found in memory: {target_id}")

    def _select_best_frame(self, capture_count: int, class_ids: list[int]) -> dict[str, Any] | None:
        best = None
        for index in range(max(1, capture_count)):
            if index > 0 and self._capture_interval > 0.0:
                time.sleep(self._capture_interval)
            image, header, error = self._snapshot()
            if error:
                self.get_logger().warn(f"capture {index + 1}/{capture_count} skipped: {error}")
                continue
            try:
                raw_dets = self._engine.detect(image, allowed_class_ids=class_ids)
            except Exception as exc:
                self.get_logger().warn(f"YOLO detect failed: {exc}")
                continue
            detections = [_normalize_det(det) for det in raw_dets]
            sharpness = _image_sharpness(image)
            confidence_sum = sum(float(det.get("confidence", 0.0)) for det in detections)
            score = len(detections) * 100.0 + confidence_sum * 10.0 + sharpness * 0.001
            frame = {
                "index": index,
                "image": image,
                "header": header,
                "detections": detections,
                "score": score,
            }
            if best is None or frame["score"] > best["score"]:
                best = frame
        if best is not None:
            self.get_logger().info(
                f"best confirmation capture {best['index'] + 1}/{capture_count}: "
                f"candidates={len(best['detections'])} score={best['score']:.3f}"
            )
        return best

    def _snapshot(self):
        with self._cache_lock:
            image = None if self._latest_image is None else self._latest_image.copy()
            header = copy.deepcopy(self._latest_image_header)
            age = time.monotonic() - self._latest_image_received
        if image is None:
            return None, None, "no RGB image has been received"
        if self._max_image_age > 0.0 and age > self._max_image_age:
            return None, None, f"RGB image is stale: {age:.2f}s"
        return image, header, ""

    def _number_selector(self) -> VlmNumberSelector:
        client = None
        if not self._use_local_vlm:
            if self._bailian_client is None:
                self._bailian_client = BailianVLMClient(
                    api_key_env=str(self.get_parameter("bailian_api_key_env").value),
                    base_url=str(self.get_parameter("bailian_base_url").value).strip()
                    or BAILIAN_DEFAULT_BASE_URL,
                    model=str(self.get_parameter("bailian_model").value),
                    timeout=self._timeout,
                    enable_thinking=_as_bool(self.get_parameter("bailian_enable_thinking").value),
                    component="target_confirm",
                    log_callback=self.get_logger().info,
                )
            client = self._bailian_client
        return VlmNumberSelector(
            use_local_vlm=self._use_local_vlm,
            vlm_url=self._vlm_url,
            timeout=self._timeout,
            prompt_template=self._prompt_template,
            bailian_client=client,
        )

    def _publish_result(self, result: dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps(result, ensure_ascii=False)
        self._result_pub.publish(msg)
        self.get_logger().info(msg.data)

    def _publish_image(self, publisher, image_bgr, header) -> None:
        try:
            image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
            msg = self._bridge.cv2_to_imgmsg(image_rgb, encoding="rgb8")
            if header is not None:
                msg.header = header
            publisher.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f"failed to publish debug image: {exc}")

    def _image_to_bgr(self, msg: Image):
        if msg.encoding in ("rgb8", "bgr8"):
            return self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if msg.encoding in ("mono8", "8UC1"):
            mono = self._bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
            return cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)
        return self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")


def _target_classes_from_obj(obj: dict[str, Any]) -> list[str]:
    classes = []
    primary = str(obj.get("yolo_class") or "").strip()
    if primary:
        classes.append(primary)
    backup = obj.get("backup_yolo_classes", [])
    if isinstance(backup, list):
        for item in backup:
            text = str(item).strip()
            if text and text not in classes:
                classes.append(text)
    return classes


def _resolve_class_ids(
    classes: list[str], class_names: list[str] | tuple[str, ...] | None = None
) -> tuple[list[int], list[str]]:
    name_to_id = {name: idx for idx, name in enumerate(class_names or COCO_CLASSES)}
    ids = []
    unknown = []
    for name in classes:
        if name in name_to_id:
            ids.append(name_to_id[name])
        else:
            unknown.append(name)
    return ids, unknown


def _normalize_det(det: dict[str, Any]) -> dict[str, Any]:
    return {
        "bbox": [int(v) for v in det.get("box", [0, 0, 0, 0])],
        "yolo_class": str(det.get("class_name") or "unknown"),
        "confidence": float(det.get("score", 0.0)),
    }


def _fail(reason: str) -> dict[str, Any]:
    return {"target_found": False, "selected_index": 0, "reason": reason}


def _positive_int(value: Any, default: int) -> int:
    try:
        return max(1, int(value))
    except Exception:
        return max(1, int(default))


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def _image_sharpness(image_bgr) -> float:
    try:
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        return float(cv2.Laplacian(gray, cv2.CV_64F).var())
    except Exception:
        return 0.0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TargetConfirmNode()
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

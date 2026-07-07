#!/usr/bin/env python3
"""Read a remembered target by id and call the existing VLM selector service."""

from __future__ import annotations

import json
import threading
from pathlib import Path
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from vlm_target_msgs.srv import SelectTarget


class MemoryTargetSelectAdapter(Node):
    def __init__(self) -> None:
        super().__init__("memory_target_select_adapter")

        self.declare_parameter("memory_path", "")
        self.declare_parameter("cmd_topic", "/memory_target_selector/select_cmd")
        self.declare_parameter("result_topic", "/memory_target_selector/result")
        self.declare_parameter("selector_service", "/vlm_target_selector/select_target")
        self.declare_parameter("wait_service_timeout_sec", 2.0)
        self.declare_parameter("save_debug_images", True)

        memory_path = str(self.get_parameter("memory_path").value).strip()
        if not memory_path:
            memory_path = str(Path.home() / ".ros" / "patrol_memory" / "object_memory.json")
        self._memory_path = Path(memory_path).expanduser()
        self._selector_service = str(self.get_parameter("selector_service").value)
        self._wait_timeout = float(self.get_parameter("wait_service_timeout_sec").value)
        self._save_debug_images = _as_bool(self.get_parameter("save_debug_images").value)
        self._busy_lock = threading.Lock()

        self._client = self.create_client(SelectTarget, self._selector_service)
        self._result_pub = self.create_publisher(
            String,
            str(self.get_parameter("result_topic").value),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("cmd_topic").value),
            self._on_cmd,
            10,
        )

        self.get_logger().info(
            "memory_target_select_adapter started | "
            f"memory={self._memory_path} | service={self._selector_service}"
        )

    def _on_cmd(self, msg: String) -> None:
        try:
            cmd = json.loads(msg.data)
        except Exception as exc:
            self._publish_result({"success": False, "reason": f"invalid JSON: {exc}"})
            return

        if not self._busy_lock.acquire(blocking=False):
            self._publish_result({"success": False, "reason": "adapter is busy"})
            return

        threading.Thread(target=self._run, args=(cmd,), daemon=True).start()

    def _run(self, cmd: dict[str, Any]) -> None:
        try:
            target_id = str(cmd.get("target_id") or "").strip()
            target_obj = cmd.get("target_obj")
            if isinstance(target_obj, dict):
                target_obj = dict(target_obj)
                if target_id and not str(target_obj.get("id") or "").strip():
                    target_obj["id"] = target_id
                target_id = str(target_obj.get("id") or target_id or "temporary_target").strip()
            elif not target_id:
                self._publish_result({"success": False, "reason": "target_id is required"})
                self._busy_lock.release()
                return
            else:
                target_obj = self._load_target(target_id)

            target_name = _resolve_target_name(cmd, target_obj)
            target_classes = _target_classes_from_obj(target_obj)

            if target_classes:
                self.get_logger().info(
                    "target_id=%s needs YOLO target_classes: %s"
                    % (target_id, ",".join(target_classes))
                )

            if not self._client.wait_for_service(timeout_sec=self._wait_timeout):
                self._publish_result({
                    "success": False,
                    "target_id": target_id,
                    "target_name": target_name,
                    "target_classes": target_classes,
                    "reason": f"selector service not available: {self._selector_service}",
                })
                self._busy_lock.release()
                return

            request = SelectTarget.Request()
            request.target_name = target_name
            request.save_debug_images = _as_bool(cmd.get("save_debug_images", self._save_debug_images))

            future = self._client.call_async(request)
            future.add_done_callback(
                lambda done: self._on_select_done(done, target_obj, target_name, target_classes)
            )
        except Exception as exc:
            self._publish_result({"success": False, "reason": str(exc)})
            self._busy_lock.release()

    def _load_target(self, target_id: str) -> dict[str, Any]:
        if not self._memory_path.exists():
            raise ValueError(f"memory file does not exist: {self._memory_path}")
        with self._memory_path.open("r", encoding="utf-8") as f:
            data = json.load(f)
        if not isinstance(data, list):
            raise ValueError("memory JSON must be a list")

        for item in data:
            if isinstance(item, dict) and str(item.get("id") or "") == target_id:
                return item
        raise ValueError(f"target_id not found: {target_id}")

    def _on_select_done(
        self,
        future,
        target_obj: dict[str, Any],
        target_name: str,
        target_classes: list[str],
    ) -> None:
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    "VLM selector selected id=%d type=%s conf=%.3f box=[%d,%d,%d,%d]"
                    % (
                        response.selected_id,
                        response.selected_type,
                        response.confidence,
                        response.x_min,
                        response.y_min,
                        response.x_max,
                        response.y_max,
                    )
                )
            else:
                self.get_logger().warn(f"VLM selector failed: {response.message}")

            self._publish_result({
                "success": bool(response.success),
                "target_id": target_obj.get("id"),
                "target_name": target_name,
                "target_classes": target_classes,
                "selector_selected_id": int(response.selected_id),
                "selector_selected_type": response.selected_type,
                "selector_confidence": float(response.confidence),
                "selector_bbox": [
                    int(response.x_min),
                    int(response.y_min),
                    int(response.x_max),
                    int(response.y_max),
                ],
                "selector_message": response.message,
            })
        except Exception as exc:
            self._publish_result({
                "success": False,
                "target_id": target_obj.get("id"),
                "target_name": target_name,
                "target_classes": target_classes,
                "reason": f"selector call failed: {exc}",
            })
        finally:
            self._busy_lock.release()

    def _publish_result(self, payload: dict[str, Any]) -> None:
        text = json.dumps(payload, ensure_ascii=False)
        self.get_logger().info(text)
        self._result_pub.publish(String(data=text))


def _resolve_target_name(cmd: dict[str, Any], target_obj: dict[str, Any]) -> str:
    for key in ("target_name", "user_query"):
        value = str(cmd.get(key) or "").strip()
        if value:
            return value
    main_name = str(target_obj.get("main_name") or "").strip()
    if main_name:
        return main_name
    possible = target_obj.get("possible_names")
    if isinstance(possible, list):
        for item in possible:
            value = str(item).strip()
            if value:
                return value
    return str(target_obj.get("id") or "target")


def _target_classes_from_obj(target_obj: dict[str, Any]) -> list[str]:
    classes: list[str] = []
    primary = str(target_obj.get("yolo_class") or "").strip()
    if primary:
        classes.append(primary)
    backup = target_obj.get("backup_yolo_classes")
    if isinstance(backup, list):
        for item in backup:
            value = str(item).strip()
            if value and value not in classes:
                classes.append(value)
    return classes


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MemoryTargetSelectAdapter()
    executor = MultiThreadedExecutor(num_threads=2)
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

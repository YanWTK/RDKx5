#!/usr/bin/env python3
"""RViz MarkerArray publisher for semantic object memory."""

from __future__ import annotations

from typing import Any

from visualization_msgs.msg import Marker, MarkerArray


class SemanticMarkerPublisher:
    def __init__(
        self,
        node,
        topic: str = "/semantic_object_markers",
        frame_id: str = "map",
        text_scale: float = 0.10,
        text_z_offset: float = 0.14,
    ) -> None:
        self._node = node
        self._frame_id = frame_id
        self._text_scale = max(0.02, float(text_scale))
        self._text_z_offset = float(text_z_offset)
        self._pub = node.create_publisher(MarkerArray, topic, 10)

    def publish(self, objects: list[dict[str, Any]]) -> None:
        marker_array = MarkerArray()
        stamp = self._node.get_clock().now().to_msg()

        for obj in objects:
            pos = obj.get("_marker_position") or obj.get("map_position")
            if not _valid_position(pos):
                continue

            stable_num = _object_number(obj.get("id"))
            marker_array.markers.append(
                self._sphere_marker(stamp, stable_num * 2, pos)
            )
            marker_array.markers.append(
                self._text_marker(stamp, stable_num * 2 + 1, pos, obj)
            )

        self._pub.publish(marker_array)

    def _sphere_marker(self, stamp, marker_id: int, pos) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._frame_id
        marker.header.stamp = stamp
        marker.ns = "semantic_objects"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(pos[0])
        marker.pose.position.y = float(pos[1])
        marker.pose.position.z = float(pos[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.scale.z = 0.08
        marker.color.r = 0.1
        marker.color.g = 0.8
        marker.color.b = 0.2
        marker.color.a = 0.9
        return marker

    def _text_marker(self, stamp, marker_id: int, pos, obj: dict[str, Any]) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._frame_id
        marker.header.stamp = stamp
        marker.ns = "semantic_objects"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = float(pos[0])
        marker.pose.position.y = float(pos[1])
        marker.pose.position.z = float(pos[2]) + self._text_z_offset
        marker.pose.orientation.w = 1.0
        marker.scale.z = self._text_scale
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = _rviz_ascii_label(obj)
        return marker


def _object_number(obj_id: Any) -> int:
    text = str(obj_id or "")
    if text.startswith("obj_"):
        try:
            return max(1, int(text.split("_", 1)[1]))
        except Exception:
            pass
    return abs(hash(text)) % 100000 + 1


def _valid_position(pos: Any) -> bool:
    if not isinstance(pos, list) or len(pos) != 3:
        return False
    try:
        float(pos[0])
        float(pos[1])
        float(pos[2])
    except Exception:
        return False
    return True


def _rviz_ascii_label(obj: dict[str, Any]) -> str:
    obj_id = _ascii_token(obj.get("id")) or "obj"
    yolo_class = _ascii_token(obj.get("yolo_class")) or "object"
    return f"{obj_id} {yolo_class}".strip()


def _ascii_token(value: Any) -> str:
    text = str(value or "").strip().replace(" ", "_")
    return "".join(ch for ch in text if 32 <= ord(ch) < 127)

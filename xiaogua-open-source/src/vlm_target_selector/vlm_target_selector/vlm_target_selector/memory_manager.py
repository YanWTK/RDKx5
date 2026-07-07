#!/usr/bin/env python3
"""Persistent object memory helpers for patrol scanning."""

from __future__ import annotations

import json
import shutil
from pathlib import Path
from typing import Any


class ObjectMemoryManager:
    """Load, upsert, and save the simplified object_memory.json format."""

    def __init__(self, memory_path: str | Path) -> None:
        self.path = Path(memory_path).expanduser()
        self.objects: list[dict[str, Any]] = []
        self.load()

    def load(self) -> list[dict[str, Any]]:
        if not self.path.exists():
            self.objects = []
            return self.objects

        try:
            with self.path.open("r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception:
            self.objects = []
            return self.objects

        self.objects = data if isinstance(data, list) else []
        return self.objects

    def save(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        serializable = [
            {key: value for key, value in obj.items() if not str(key).startswith("_")}
            for obj in self.objects
        ]
        with self.path.open("w", encoding="utf-8") as f:
            json.dump(serializable, f, ensure_ascii=False, indent=2)
            f.write("\n")

    def clear(self) -> None:
        self.objects = []
        self.save()

    def backup_and_clear(self) -> Path | None:
        backup_path = self.path.with_name(f"{self.path.stem}.backup{self.path.suffix}")
        if self.path.exists():
            self.path.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(self.path, backup_path)
        self.clear()
        return backup_path if backup_path.exists() else None

    def upsert(self, item: dict[str, Any]) -> tuple[dict[str, Any], bool]:
        """Insert or update by point/yolo/main-name/possible-name rules.

        Returns (object, created).
        """
        possible = _string_set(item.get("possible_names"))
        for existing in self.objects:
            if not self._is_same_object(existing, item, possible):
                continue

            existing_possible = _string_set(existing.get("possible_names"))
            merged = sorted(existing_possible | possible)
            existing["possible_names"] = merged

            old_name = str(existing.get("main_name") or "")
            new_name = str(item.get("main_name") or "")
            if old_name.startswith("不确定") and new_name and not new_name.startswith("不确定"):
                existing["main_name"] = new_name

            existing["backup_yolo_classes"] = item.get("backup_yolo_classes", [])
            existing["detected_at_point"] = item.get(
                "detected_at_point",
                item.get("point_id", existing.get("point_id")),
            )
            if item.get("_marker_position") is not None:
                existing["_marker_position"] = item.get("_marker_position")
            self.save()
            return existing, False

        stored = dict(item)
        stored["id"] = self._next_id()
        stored["possible_names"] = sorted(possible)
        marker_position = stored.pop("_marker_position", None)
        stored.pop("map_position", None)
        if marker_position is not None:
            stored["_marker_position"] = marker_position
        self.objects.append(stored)
        self.save()
        return stored, True

    def _is_same_object(
        self,
        existing: dict[str, Any],
        item: dict[str, Any],
        possible: set[str],
    ) -> bool:
        if existing.get("point_id") != item.get("point_id"):
            return False
        if existing.get("yolo_class") != item.get("yolo_class"):
            return False

        old_main = str(existing.get("main_name") or "")
        new_main = str(item.get("main_name") or "")
        if old_main and new_main and old_main == new_main:
            return True

        return bool(_string_set(existing.get("possible_names")) & possible)

    def _next_id(self) -> str:
        max_num = 0
        for obj in self.objects:
            obj_id = str(obj.get("id") or "")
            if not obj_id.startswith("obj_"):
                continue
            try:
                max_num = max(max_num, int(obj_id.split("_", 1)[1]))
            except Exception:
                continue
        return f"obj_{max_num + 1:03d}"


def _string_set(value: Any) -> set[str]:
    if isinstance(value, str):
        return {value.strip()} if value.strip() else set()
    if not isinstance(value, list):
        return set()
    result: set[str] = set()
    for item in value:
        text = str(item).strip()
        if text:
            result.add(text)
    return result

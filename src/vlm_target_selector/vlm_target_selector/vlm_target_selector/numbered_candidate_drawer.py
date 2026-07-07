#!/usr/bin/env python3
"""Draw numbered YOLO candidate boxes for VLM selection."""

from __future__ import annotations

from typing import Any

import cv2


def draw_numbered_boxes(image_bgr, detections: list[dict[str, Any]]):
    """Return numbered image and detections with 1-based index fields."""
    numbered = image_bgr.copy()
    h, w = numbered.shape[:2]
    indexed = []

    for idx, det in enumerate(detections, start=1):
        box = det.get("bbox") or det.get("box")
        if not box or len(box) != 4:
            continue
        x1, y1, x2, y2 = _clamp_box(box, w, h)
        if x2 <= x1 or y2 <= y1:
            continue

        item = dict(det)
        item["index"] = idx
        item["bbox"] = [x1, y1, x2, y2]
        indexed.append(item)

        cv2.rectangle(numbered, (x1, y1), (x2, y2), (0, 0, 255), 3)
        label = str(idx)
        label_w = max(46, 22 * len(label) + 18)
        top = max(0, y1 - 38)
        cv2.rectangle(
            numbered,
            (x1, top),
            (min(w - 1, x1 + label_w), y1),
            (0, 0, 255),
            -1,
        )
        cv2.putText(
            numbered,
            label,
            (x1 + 10, max(24, y1 - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (255, 255, 255),
            3,
            cv2.LINE_AA,
        )

    return numbered, indexed


def draw_selected_box(image_bgr, selected: dict[str, Any], target_name: str = ""):
    final = image_bgr.copy()
    h, w = final.shape[:2]
    x1, y1, x2, y2 = _clamp_box(selected.get("bbox", [0, 0, 0, 0]), w, h)
    cv2.rectangle(final, (x1, y1), (x2, y2), (0, 255, 0), 4)
    label = f"selected {selected.get('index', '')}".strip()
    if target_name:
        label = f"{label}: {target_name}"
    cv2.putText(
        final,
        label,
        (x1, max(28, y1 - 10)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )
    return final


def _clamp_box(box, width: int, height: int):
    x1, y1, x2, y2 = [int(v) for v in box]
    x1 = max(0, min(width - 1, x1))
    y1 = max(0, min(height - 1, y1))
    x2 = max(0, min(width - 1, x2))
    y2 = max(0, min(height - 1, y2))
    return x1, y1, x2, y2

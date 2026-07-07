from __future__ import annotations

import math
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import OccupancyGrid


def quaternion_from_yaw(yaw: float) -> Quaternion:
    half = yaw * 0.5
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


def build_static_map(width: int, height: int) -> np.ndarray:
    grid = np.zeros((height, width), dtype=np.int8)
    wall = max(2, min(width, height) // 40)

    grid[:wall, :] = 100
    grid[-wall:, :] = 100
    grid[:, :wall] = 100
    grid[:, -wall:] = 100

    mid_x = width // 2
    mid_y = height // 2
    band = max(3, min(width, height) // 60)
    gap = max(8, min(width, height) // 8)

    grid[mid_y - band : mid_y + band, wall : width - wall] = 100
    grid[wall : height - wall, mid_x - band : mid_x + band] = 100

    grid[mid_y - band : mid_y + band, mid_x - gap : mid_x - gap + band * 2] = 0
    grid[mid_y - band : mid_y + band, mid_x + gap - band * 2 : mid_x + gap] = 0
    grid[mid_y - gap : mid_y - gap + band * 2, mid_x - band : mid_x + band] = 0
    grid[mid_y + gap - band * 2 : mid_y + gap, mid_x - band : mid_x + band] = 0

    shelf = max(8, min(width, height) // 12)
    grid[wall + shelf : wall + shelf + band, wall + shelf : width // 3] = 100
    grid[height - wall - shelf - band : height - wall - shelf, width * 2 // 3 : -wall - shelf] = 100

    return grid


def pose_to_cell(
    x: float,
    y: float,
    origin_x: float,
    origin_y: float,
    resolution: float,
    width: int,
    height: int,
) -> tuple[int, int]:
    col = int((x - origin_x) / resolution)
    row = int((y - origin_y) / resolution)
    col = max(0, min(width - 1, col))
    row = max(0, min(height - 1, row))
    return row, col


def draw_blob(grid: np.ndarray, row: int, col: int, radius: int, value: int) -> None:
    height, width = grid.shape
    r0 = max(0, row - radius)
    r1 = min(height, row + radius + 1)
    c0 = max(0, col - radius)
    c1 = min(width, col + radius + 1)
    grid[r0:r1, c0:c1] = value


def occupancy_grid_message(
    grid: np.ndarray,
    frame_id: str,
    resolution: float,
    origin_x: float,
    origin_y: float,
    stamp,
) -> OccupancyGrid:
    msg = OccupancyGrid()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.info.resolution = float(resolution)
    msg.info.width = int(grid.shape[1])
    msg.info.height = int(grid.shape[0])
    msg.info.origin.position.x = float(origin_x)
    msg.info.origin.position.y = float(origin_y)
    msg.info.origin.position.z = 0.0
    msg.info.origin.orientation.w = 1.0
    msg.data = grid.astype(np.int8).reshape(-1).tolist()
    return msg


def save_occupancy_grid(
    grid: np.ndarray,
    save_dir: str | Path,
    basename: str,
    resolution: float,
    origin_x: float,
    origin_y: float,
) -> Path:
    target_dir = Path(save_dir)
    target_dir.mkdir(parents=True, exist_ok=True)

    stem = target_dir / basename
    pgm_path = stem.with_suffix(".pgm")
    yaml_path = stem.with_suffix(".yaml")

    image = np.full(grid.shape, 205, dtype=np.uint8)
    image[grid <= 0] = 254
    image[grid >= 65] = 0
    image = np.flipud(image)

    with pgm_path.open("wb") as handle:
        header = f"P5\n{image.shape[1]} {image.shape[0]}\n255\n".encode("ascii")
        handle.write(header)
        handle.write(image.tobytes())

    yaml_lines = [
        f"image: {pgm_path.name}",
        f"resolution: {resolution}",
        f"origin: [{origin_x}, {origin_y}, 0.0]",
        "negate: 0",
        "occupied_thresh: 0.65",
        "free_thresh: 0.196",
        "mode: trinary",
    ]
    yaml_path.write_text("\n".join(yaml_lines) + "\n", encoding="utf-8")
    return stem


def render_placeholder_frame(
    width: int,
    height: int,
    lines: Iterable[str],
    accent: tuple[int, int, int] = (52, 166, 255),
) -> np.ndarray:
    frame = np.full((height, width, 3), 28, dtype=np.uint8)
    cv2.rectangle(frame, (0, 0), (width - 1, height - 1), accent, 2)
    cv2.rectangle(frame, (10, 10), (width - 11, height - 11), (50, 50, 50), 1)

    y = 60
    for line in lines:
        cv2.putText(
            frame,
            str(line),
            (24, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (235, 235, 235),
            2,
            cv2.LINE_AA,
        )
        y += 32

    return frame


def encode_jpeg(frame: np.ndarray, quality: int = 80) -> bytes:
    quality = max(5, min(95, int(quality)))
    ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        raise RuntimeError("failed to encode JPEG")
    return encoded.tobytes()

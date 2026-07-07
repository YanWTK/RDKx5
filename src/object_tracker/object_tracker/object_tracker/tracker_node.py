#!/usr/bin/env python3
"""Lightweight ByteTrack-style tracker for ai_msgs/PerceptionTargets."""

import copy
import json
import time
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
import rclpy
from ai_msgs.msg import PerceptionTargets
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


@dataclass
class Detection:
    box: np.ndarray
    score: float
    class_name: str
    target: object


class Track:
    def __init__(self, track_id: int, det: Detection):
        self.track_id = track_id
        self.box = det.box.astype(np.float32)
        self.velocity = np.zeros(4, dtype=np.float32)
        self.score = float(det.score)
        self.class_name = det.class_name
        self.target = copy.deepcopy(det.target)
        self.hits = 1
        self.age = 1
        self.lost_frames = 0
        self.updated_in_frame = True
        self.last_update_time = time.monotonic()

    def predicted_box(self) -> np.ndarray:
        if self.lost_frames > 0:
            return self.box
        return self.box + self.velocity

    def update(self, det: Detection) -> None:
        new_box = det.box.astype(np.float32)
        self.velocity = new_box - self.box
        self.box = new_box
        self.score = float(det.score)
        self.class_name = det.class_name
        self.target = copy.deepcopy(det.target)
        self.hits += 1
        self.age += 1
        self.lost_frames = 0
        self.updated_in_frame = True
        self.last_update_time = time.monotonic()

    def mark_lost(self) -> None:
        self.box = self.box + self.velocity * 0.25
        self.velocity *= 0.25
        self.age += 1
        self.lost_frames += 1
        self.updated_in_frame = False


def _bbox_iou(a: np.ndarray, b: np.ndarray) -> float:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    inter_x1 = max(ax1, bx1)
    inter_y1 = max(ay1, by1)
    inter_x2 = min(ax2, bx2)
    inter_y2 = min(ay2, by2)
    inter_w = max(0.0, inter_x2 - inter_x1)
    inter_h = max(0.0, inter_y2 - inter_y1)
    inter = inter_w * inter_h
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = area_a + area_b - inter
    if union <= 0.0:
        return 0.0
    return float(inter / union)


def _clip_box(box: np.ndarray, width: int, height: int) -> tuple[int, int, int, int]:
    x1, y1, x2, y2 = [int(round(v)) for v in box]
    x1 = max(0, min(width - 1, x1))
    y1 = max(0, min(height - 1, y1))
    x2 = max(0, min(width, x2))
    y2 = max(0, min(height, y2))
    return x1, y1, x2, y2


def _greedy_match(
    tracks: list[Track],
    detections: list[Detection],
    threshold: float,
    class_aware: bool,
) -> tuple[list[tuple[int, int]], list[int], list[int]]:
    pairs = []
    for ti, track in enumerate(tracks):
        for di, det in enumerate(detections):
            if class_aware and track.class_name != det.class_name:
                continue
            iou = _bbox_iou(track.predicted_box(), det.box)
            if iou >= threshold:
                pairs.append((iou, ti, di))

    pairs.sort(reverse=True, key=lambda item: item[0])
    matched_tracks = set()
    matched_dets = set()
    matches = []
    for _, ti, di in pairs:
        if ti in matched_tracks or di in matched_dets:
            continue
        matched_tracks.add(ti)
        matched_dets.add(di)
        matches.append((ti, di))

    unmatched_tracks = [i for i in range(len(tracks)) if i not in matched_tracks]
    unmatched_dets = [i for i in range(len(detections)) if i not in matched_dets]
    return matches, unmatched_tracks, unmatched_dets


class ObjectTrackerNode(Node):
    """ByteTrack-style two-stage tracker with VLM-selected target locking."""

    def __init__(self):
        super().__init__('object_tracker')

        self.declare_parameter('detection_topic', '/yolo_detector/detections')
        self.declare_parameter('selected_detection_topic', '/vlm_target_selector/selected_detection')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('tracks_topic', '/object_tracker/tracks')
        self.declare_parameter('selected_track_topic', '/object_tracker/selected_detection')
        self.declare_parameter('status_topic', '/object_tracker/status')
        self.declare_parameter('debug_image_topic', '/object_tracker/image')
        self.declare_parameter('track_high_thresh', 0.15)
        self.declare_parameter('track_low_thresh', 0.02)
        self.declare_parameter('new_track_thresh', 0.20)
        self.declare_parameter('match_thresh', 0.7)
        self.declare_parameter('second_match_thresh', 0.5)
        self.declare_parameter('selected_match_thresh', 0.3)
        self.declare_parameter('track_buffer', 45)
        self.declare_parameter('max_new_tracks_per_frame', 3)
        self.declare_parameter('min_box_area', 16.0)
        self.declare_parameter('class_aware', True)
        self.declare_parameter('enable_debug_image', True)
        self.declare_parameter('draw_lost_tracks', False)
        self.declare_parameter('selected_only_after_lock', True)
        self.declare_parameter('clear_selection_on_lost', True)
        self.declare_parameter('selected_lost_clear_frames', 0)
        self.declare_parameter('use_appearance_match', True)
        self.declare_parameter('appearance_weight', 0.35)
        self.declare_parameter('appearance_match_thresh', 0.45)
        self.declare_parameter('appearance_update_rate', 0.15)
        self.declare_parameter('pending_selection_timeout_sec', 5.0)

        self.detection_topic = self.get_parameter('detection_topic').value
        self.selected_detection_topic = self.get_parameter('selected_detection_topic').value
        self.image_topic = self.get_parameter('image_topic').value
        self.tracks_topic = self.get_parameter('tracks_topic').value
        self.selected_track_topic = self.get_parameter('selected_track_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        self.debug_image_topic = self.get_parameter('debug_image_topic').value
        self.track_high_thresh = float(self.get_parameter('track_high_thresh').value)
        self.track_low_thresh = float(self.get_parameter('track_low_thresh').value)
        self.new_track_thresh = float(self.get_parameter('new_track_thresh').value)
        self.match_thresh = float(self.get_parameter('match_thresh').value)
        self.second_match_thresh = float(self.get_parameter('second_match_thresh').value)
        self.selected_match_thresh = float(self.get_parameter('selected_match_thresh').value)
        self.track_buffer = int(self.get_parameter('track_buffer').value)
        self.max_new_tracks_per_frame = int(
            self.get_parameter('max_new_tracks_per_frame').value
        )
        self.min_box_area = float(self.get_parameter('min_box_area').value)
        self.class_aware = _as_bool(self.get_parameter('class_aware').value)
        self.enable_debug_image = _as_bool(self.get_parameter('enable_debug_image').value)
        self.draw_lost_tracks = _as_bool(self.get_parameter('draw_lost_tracks').value)
        self.selected_only_after_lock = _as_bool(
            self.get_parameter('selected_only_after_lock').value
        )
        self.clear_selection_on_lost = _as_bool(
            self.get_parameter('clear_selection_on_lost').value
        )
        self.selected_lost_clear_frames = int(
            self.get_parameter('selected_lost_clear_frames').value
        )
        self.use_appearance_match = _as_bool(
            self.get_parameter('use_appearance_match').value
        )
        self.appearance_weight = float(self.get_parameter('appearance_weight').value)
        self.appearance_match_thresh = float(
            self.get_parameter('appearance_match_thresh').value
        )
        self.appearance_update_rate = float(
            self.get_parameter('appearance_update_rate').value
        )
        self.pending_selection_timeout = float(
            self.get_parameter('pending_selection_timeout_sec').value
        )

        self._bridge = CvBridge()
        self._latest_image = None
        self._latest_image_header = None
        self._tracks: list[Track] = []
        self._next_track_id = 1
        self._frame_index = 0
        self._selected_track_id: Optional[int] = None
        self._pending_selection: Optional[Detection] = None
        self._pending_selection_time = 0.0
        self._last_detection_header = None
        self._selected_hist = None
        self._selection_lost_final = False

        sensor_qos = QoSProfile(
            depth=2,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(PerceptionTargets, self.detection_topic,
                                 self._on_detections, 10)
        self.create_subscription(PerceptionTargets, self.selected_detection_topic,
                                 self._on_selected_detection, 10)
        if self.enable_debug_image:
            self.create_subscription(Image, self.image_topic, self._on_image, sensor_qos)

        self._tracks_pub = self.create_publisher(PerceptionTargets, self.tracks_topic, 10)
        self._selected_pub = self.create_publisher(
            PerceptionTargets, self.selected_track_topic, 10
        )
        self._status_pub = self.create_publisher(String, self.status_topic, 10)
        self._debug_image_pub = (
            self.create_publisher(Image, self.debug_image_topic, 2)
            if self.enable_debug_image else None
        )

        self.get_logger().info(
            'object_tracker started | '
            f'detections={self.detection_topic} | selected={self.selected_detection_topic} | '
            f'high={self.track_high_thresh:.2f} low={self.track_low_thresh:.2f} '
            f'new={self.new_track_thresh:.2f} buffer={self.track_buffer} '
            f'selected_only={self.selected_only_after_lock} '
            f'clear_lost={self.clear_selection_on_lost} '
            f'appearance={self.use_appearance_match}'
        )

    def _on_image(self, msg: Image) -> None:
        try:
            self._latest_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self._latest_image_header = copy.deepcopy(msg.header)
        except Exception as exc:
            self.get_logger().warn(f'image conversion failed: {exc}')

    def _on_selected_detection(self, msg: PerceptionTargets) -> None:
        detections = self._extract_detections(msg, min_score=0.0)
        if not detections:
            self.get_logger().warn('selected detection message has no valid bbox')
            return
        self._pending_selection = detections[0]
        self._pending_selection_time = time.monotonic()
        self._selected_track_id = None
        self._selected_hist = self._compute_hist(self._pending_selection.box)
        self._assign_pending_selection(create_if_missing=True)

    def _on_detections(self, msg: PerceptionTargets) -> None:
        self._frame_index += 1
        self._last_detection_header = copy.deepcopy(msg.header)

        detections = self._extract_detections(msg, min_score=self.track_low_thresh)
        high_dets = [det for det in detections if det.score >= self.track_high_thresh]
        low_dets = [
            det for det in detections
            if self.track_low_thresh <= det.score < self.track_high_thresh
        ]
        self._selection_lost_final = False

        selected_only = self._selected_only_active()
        if selected_only:
            selected_track = self._get_selected_track()
            self._tracks = [selected_track] if selected_track is not None else []

        for track in self._tracks:
            track.updated_in_frame = False

        if selected_only:
            self._update_selected_track(high_dets + low_dets)
        else:
            active_tracks = [
                track for track in self._tracks
                if track.lost_frames <= self.track_buffer
            ]
            matches, unmatched_track_idx, unmatched_high_idx = _greedy_match(
                active_tracks, high_dets, self.match_thresh, self.class_aware
            )
            for ti, di in matches:
                active_tracks[ti].update(high_dets[di])

            second_tracks = [active_tracks[i] for i in unmatched_track_idx]
            second_matches, second_unmatched_track_idx, _ = _greedy_match(
                second_tracks, low_dets, self.second_match_thresh, self.class_aware
            )
            for local_ti, di in second_matches:
                second_tracks[local_ti].update(low_dets[di])

            for local_ti in second_unmatched_track_idx:
                second_tracks[local_ti].mark_lost()

            created = 0
            for di in unmatched_high_idx:
                if created >= self.max_new_tracks_per_frame:
                    break
                det = high_dets[di]
                if det.score >= self.new_track_thresh:
                    self._tracks.append(Track(self._next_track_id, det))
                    self._next_track_id += 1
                    created += 1

        self._tracks = [
            track for track in self._tracks
            if track.lost_frames <= self.track_buffer
        ]
        self._clear_lost_selection_if_needed()

        self._assign_pending_selection(create_if_missing=False)
        self._drop_expired_pending_selection()
        self._publish_tracks(msg)
        self._publish_status()
        self._publish_debug_image()

    def _update_selected_track(self, detections: list[Detection]) -> None:
        selected_track = self._get_selected_track()
        if selected_track is None:
            return

        candidates = []
        for det in detections:
            if self.class_aware and det.class_name != selected_track.class_name:
                continue

            iou = _bbox_iou(selected_track.predicted_box(), det.box)
            if iou < self.second_match_thresh:
                continue

            appearance = self._appearance_similarity(det.box)
            if self.use_appearance_match and appearance < self.appearance_match_thresh:
                continue

            score = iou
            if self.use_appearance_match:
                weight = max(0.0, min(1.0, self.appearance_weight))
                score = (1.0 - weight) * iou + weight * appearance
            candidates.append((score, iou, appearance, det))

        if not candidates:
            selected_track.mark_lost()
            return

        candidates.sort(reverse=True, key=lambda item: item[0])
        _, iou, appearance, det = candidates[0]
        selected_track.update(det)
        self._update_selected_hist(det.box)
        self.get_logger().debug(
            f'selected match track_id={selected_track.track_id} '
            f'iou={iou:.3f} appearance={appearance:.3f}'
        )

    def _extract_detections(self, msg: PerceptionTargets, min_score: float) -> list[Detection]:
        detections = []
        for target in msg.targets:
            if not target.rois:
                continue
            roi = target.rois[0]
            rect = roi.rect
            score = float(getattr(roi, 'confidence', 0.0))
            width = float(rect.width)
            height = float(rect.height)
            area = width * height
            if score < min_score or width <= 0.0 or height <= 0.0 or area < self.min_box_area:
                continue

            x1 = float(rect.x_offset)
            y1 = float(rect.y_offset)
            x2 = x1 + width
            y2 = y1 + height
            detections.append(Detection(
                box=np.array([x1, y1, x2, y2], dtype=np.float32),
                score=score,
                class_name=target.type or 'unknown',
                target=copy.deepcopy(target),
            ))
        return detections

    def _assign_pending_selection(self, create_if_missing: bool = False) -> None:
        if self._pending_selection is None:
            return
        current_tracks = [track for track in self._tracks if track.lost_frames == 0]

        best_track = None
        best_iou = 0.0
        for track in current_tracks:
            if self.class_aware and track.class_name != self._pending_selection.class_name:
                continue
            iou = _bbox_iou(track.box, self._pending_selection.box)
            if iou > best_iou:
                best_iou = iou
                best_track = track

        if best_track is not None and best_iou >= self.selected_match_thresh:
            self._selected_track_id = best_track.track_id
            self._pending_selection = None
            self.get_logger().info(
                f'locked VLM target to track_id={best_track.track_id} '
                f'class={best_track.class_name} iou={best_iou:.3f}'
            )
            return

        if not create_if_missing:
            return

        new_track = Track(self._next_track_id, self._pending_selection)
        self._next_track_id += 1
        self._tracks.append(new_track)
        self._selected_track_id = new_track.track_id
        self._pending_selection = None
        self.get_logger().info(
            f'created selected track_id={new_track.track_id} from VLM target '
            f'class={new_track.class_name}'
        )

    def _compute_hist(self, box: np.ndarray):
        if self._latest_image is None:
            return None
        h, w = self._latest_image.shape[:2]
        x1, y1, x2, y2 = _clip_box(box, w, h)
        if x2 <= x1 or y2 <= y1:
            return None

        patch = self._latest_image[y1:y2, x1:x2]
        if patch.size == 0:
            return None

        hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
        hist = cv2.calcHist([hsv], [0, 1], None, [30, 32], [0, 180, 0, 256])
        cv2.normalize(hist, hist, alpha=1.0, norm_type=cv2.NORM_L1)
        return hist

    def _appearance_similarity(self, box: np.ndarray) -> float:
        if not self.use_appearance_match:
            return 1.0
        if self._selected_hist is None:
            return 1.0
        if self._latest_image is None:
            return 1.0

        hist = self._compute_hist(box)
        if hist is None:
            return 0.0

        distance = cv2.compareHist(self._selected_hist, hist, cv2.HISTCMP_BHATTACHARYYA)
        return float(max(0.0, min(1.0, 1.0 - distance)))

    def _update_selected_hist(self, box: np.ndarray) -> None:
        if not self.use_appearance_match:
            return

        hist = self._compute_hist(box)
        if hist is None:
            return

        if self._selected_hist is None:
            self._selected_hist = hist
            return

        rate = max(0.0, min(1.0, self.appearance_update_rate))
        self._selected_hist = (1.0 - rate) * self._selected_hist + rate * hist
        cv2.normalize(self._selected_hist, self._selected_hist, alpha=1.0, norm_type=cv2.NORM_L1)

    def _drop_expired_pending_selection(self) -> None:
        if self._pending_selection is None:
            return
        age = time.monotonic() - self._pending_selection_time
        if age > self.pending_selection_timeout:
            self.get_logger().warn(
                f'pending VLM selection expired after {age:.2f}s; no matching track found'
            )
            self._pending_selection = None

    def _clear_lost_selection_if_needed(self) -> None:
        if not self.clear_selection_on_lost or self._selected_track_id is None:
            return

        selected_track = self._get_selected_track()
        clear_frames = self.selected_lost_clear_frames
        if clear_frames <= 0:
            clear_frames = self.track_buffer

        should_clear = selected_track is None
        if selected_track is not None and selected_track.lost_frames >= clear_frames:
            should_clear = True

        if not should_clear:
            return

        old_id = self._selected_track_id
        lost_frames = selected_track.lost_frames if selected_track is not None else clear_frames
        self._selected_track_id = None
        self._selected_hist = None
        self._pending_selection = None
        self._selection_lost_final = True
        self.get_logger().warn(
            f'selected track_id={old_id} cleared after lost_frames={lost_frames}; '
            'waiting for a new VLM selection'
        )

    def _make_targets_msg(self, header, tracks: list[Track]) -> PerceptionTargets:
        out = PerceptionTargets()
        out.header = header
        out.fps = 0
        for track in tracks:
            target = copy.deepcopy(track.target)
            target.track_id = int(track.track_id)
            target.type = track.class_name
            if target.rois:
                target.rois[0].confidence = float(track.score)
                rect = target.rois[0].rect
                x1, y1, x2, y2 = track.box
                rect.x_offset = int(round(x1))
                rect.y_offset = int(round(y1))
                rect.width = int(round(x2 - x1))
                rect.height = int(round(y2 - y1))
            out.targets.append(target)
        return out

    def _publish_tracks(self, src_msg: PerceptionTargets) -> None:
        active = [track for track in self._tracks if track.lost_frames == 0]
        if self._selected_only_active():
            active = [
                track for track in active
                if track.track_id == self._selected_track_id
            ]
        tracks_msg = self._make_targets_msg(src_msg.header, active)
        self._tracks_pub.publish(tracks_msg)

        selected = [
            track for track in active
            if self._selected_track_id is not None and track.track_id == self._selected_track_id
        ]
        if selected:
            self._selected_pub.publish(self._make_targets_msg(src_msg.header, selected))

    def _publish_status(self) -> None:
        selected_track = self._get_selected_track()
        status = {
            'frame_index': self._frame_index,
            'track_count': len([t for t in self._tracks if t.lost_frames == 0]),
            'selected_track_id': self._selected_track_id or 0,
            'selected_active': bool(selected_track and selected_track.lost_frames == 0),
            'selected_lost_frames': selected_track.lost_frames if selected_track else 0,
            'pending_selection': self._pending_selection is not None,
            'appearance_ready': self._selected_hist is not None,
            'selection_lost_final': self._selection_lost_final,
        }
        self._status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))

    def _publish_debug_image(self) -> None:
        if not self.enable_debug_image or self._latest_image is None:
            return

        image = self._latest_image.copy()
        for track in self._tracks:
            if self._selected_only_active() and track.track_id != self._selected_track_id:
                continue
            if track.lost_frames > self.track_buffer:
                continue
            if track.lost_frames > 0 and not self.draw_lost_tracks:
                continue
            selected = self._selected_track_id == track.track_id
            if selected and track.lost_frames == 0:
                color = (0, 255, 0)
            elif track.lost_frames == 0:
                color = (255, 160, 0)
            else:
                color = (80, 80, 255)

            x1, y1, x2, y2 = [int(round(v)) for v in track.box]
            cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
            label = f'id:{track.track_id} {track.class_name} {track.score:.2f}'
            if selected:
                label = 'SELECTED ' + label
            if track.lost_frames:
                label += f' lost:{track.lost_frames}'
            cv2.putText(image, label, (x1, max(22, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        msg = self._bridge.cv2_to_imgmsg(cv2.cvtColor(image, cv2.COLOR_BGR2RGB), encoding='rgb8')
        msg.header = self._latest_image_header or self._last_detection_header
        self._debug_image_pub.publish(msg)

    def _get_selected_track(self) -> Optional[Track]:
        if self._selected_track_id is None:
            return None
        for track in self._tracks:
            if track.track_id == self._selected_track_id:
                return track
        return None

    def _selected_only_active(self) -> bool:
        return self.selected_only_after_lock and self._selected_track_id is not None


def main(args=None):
    rclpy.init(args=args)
    node = ObjectTrackerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

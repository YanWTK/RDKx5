import time

import cv2
import numpy as np
import rclpy
from ai_msgs.msg import PerceptionTargets, Target, Roi, Perf
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, RegionOfInterest
from std_msgs.msg import Bool, String

from .yolo_engine import COCO_CLASSES, YOLOEngine

DEFAULT_MODEL = "/opt/xiaogua/models/yolo_model.bin"
DEFAULT_CLASS_NAMES = "person,cell phone,mouse,remote,book,bottle,cup,bowl,apple,banana,teddy bear,bag_wrapper,box"


def _as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


class YoloDetectorNode(Node):
    """YOLOv8 BPU 检测节点，支持类别过滤，发布 ai_msgs 和标注图像。"""

    def __init__(self):
        super().__init__('yolo_detector')

        # ---- 参数 ----
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('target_classes', DEFAULT_CLASS_NAMES)
        self.declare_parameter('conf_threshold', 0.07)
        self.declare_parameter('nms_threshold', 0.7)
        self.declare_parameter('model_path', DEFAULT_MODEL)
        self.declare_parameter('class_names', DEFAULT_CLASS_NAMES)
        self.declare_parameter('preprocess_mode', 'letterbox')
        self.declare_parameter('max_fps', 15.0)
        self.declare_parameter('task_gated', False)
        self.declare_parameter('fetch_state_topic', '/voice_fetch/state')
        self.declare_parameter('target_confirm_cmd_topic', '/target_confirm/confirm_cmd')
        self.declare_parameter('target_select_cmd_topic', '/memory_target_selector/select_cmd')
        self.declare_parameter('patrol_scan_cmd_topic', '/patrol_scan_cmd')
        self.declare_parameter('patrol_scan_done_topic', '/patrol_scan_done')

        raw_classes = self.get_parameter('target_classes').value.strip()
        if raw_classes:
            self._target_classes = {c.strip() for c in raw_classes.split(',') if c.strip()}
        else:
            self._target_classes = None  # None = 不过滤
        self._target_class_ids = None

        conf = float(self.get_parameter('conf_threshold').value)
        nms = float(self.get_parameter('nms_threshold').value)
        model_path = self.get_parameter('model_path').value
        class_names = str(self.get_parameter('class_names').value)
        preprocess_mode = str(self.get_parameter('preprocess_mode').value)
        self._max_fps = max(float(self.get_parameter('max_fps').value), 0.0)
        self._task_gated = _as_bool(self.get_parameter('task_gated').value)
        self._fetch_active = False
        self._patrol_active = False
        self._last_frame_started = 0.0
        self._last_enabled = not self._task_gated

        # ---- 推理引擎 ----
        self._engine = YOLOEngine(
            model_path,
            conf,
            nms,
            class_names=class_names,
            preprocess_mode=preprocess_mode,
        )
        self._target_class_ids = self._resolve_target_class_ids(self._target_classes)
        self._bridge = CvBridge()

        # ---- 订阅图像 ----
        self._image_topic = self.get_parameter('image_topic').value
        self._image_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._image_sub = None
        self._set_image_subscription(not self._task_gated)
        if self._task_gated:
            self.create_subscription(
                String,
                str(self.get_parameter('fetch_state_topic').value),
                self._on_fetch_state,
                10,
            )
            self.create_subscription(
                String,
                str(self.get_parameter('patrol_scan_cmd_topic').value),
                self._on_patrol_scan_cmd,
                10,
            )
            self.create_subscription(
                String,
                str(self.get_parameter('target_confirm_cmd_topic').value),
                self._on_fetch_trigger,
                10,
            )
            self.create_subscription(
                String,
                str(self.get_parameter('target_select_cmd_topic').value),
                self._on_fetch_trigger,
                10,
            )
            self.create_subscription(
                String,
                str(self.get_parameter('patrol_scan_done_topic').value),
                self._on_patrol_scan_done,
                10,
            )

        # ---- 发布检测结果 ----
        self._det_pub = self.create_publisher(PerceptionTargets, '/yolo_detector/detections', 10)

        # ---- 发布标注图像 ----
        self._img_pub = self.create_publisher(Image, '/yolo_detector/image', 2)
        active_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._active_pub = self.create_publisher(
            Bool, '/yolo_detector/active', active_qos
        )

        # ---- FPS 统计 ----
        self._frame_count = 0
        self._fps_start = time.time()
        self._fps = 0.0

        cls_info = ','.join(sorted(self._target_classes)) if self._target_classes else '全部'
        mode_info = '限定类别解码' if self._target_classes else '全类别解码'
        self.get_logger().info(
            f'yolo_detector 已启动 | image={self._image_topic} | model={model_path} | '
            f'classes={cls_info} | mode={mode_info} | conf={conf} | nms={nms} | '
            f'preprocess={preprocess_mode} | max_fps={self._max_fps:.1f} | '
            f'task_gated={self._task_gated}'
        )
        self._publish_active_state(force=True)

    # ==================== 图像回调 ====================

    def _on_image(self, msg: Image):
        if not self._is_enabled():
            return

        now = time.monotonic()
        if self._max_fps > 0.0:
            min_period = 1.0 / self._max_fps
            if now - self._last_frame_started < min_period:
                return
        self._last_frame_started = now

        # 支持 RGB8、MONO8、16UC1 等编码
        try:
            if msg.encoding in ('rgb8', 'bgr8'):
                cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            elif msg.encoding in ('mono8', '8UC1'):
                mono = self._bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
                cv_image = cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)
            elif msg.encoding in ('16UC1', 'mono16'):
                raw = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                norm = (raw.astype(np.float32) / raw.max() * 255).astype(np.uint8) if raw.max() > 0 else raw.astype(np.uint8)
                cv_image = cv2.cvtColor(norm, cv2.COLOR_GRAY2BGR)
            else:
                cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'图像转换失败: {e}')
            return

        # 推理
        t0 = time.time()
        detections = self._engine.detect(
            cv_image, allowed_class_ids=self._target_class_ids
        )
        infer_ms = (time.time() - t0) * 1000.0

        # === TRACKING HOOK ===
        # 后续在此处接入跟踪器：
        #   detections = self._tracker.update(detections, timestamp=msg.header.stamp)
        # detections 格式: list[dict]，每个 dict 包含 box, score, class_id, class_name
        # 跟踪器只需往 dict 里加 track_id 字段即可
        # =====================

        # 类别过滤
        if self._target_classes is not None:
            detections = [d for d in detections if d['class_name'] in self._target_classes]

        # 发布 PerceptionTargets
        self._publish_detections(msg, detections, infer_ms)

        # 发布标注图像
        if self._img_pub.get_subscription_count() > 0:
            self._publish_annotated(msg, cv_image, detections)

        # FPS
        self._update_fps()

    def _is_enabled(self):
        return not self._task_gated or self._fetch_active or self._patrol_active

    def _set_image_subscription(self, enabled: bool):
        if enabled:
            if self._image_sub is None:
                self._image_sub = self.create_subscription(
                    Image, self._image_topic, self._on_image, self._image_qos
                )
        else:
            if self._image_sub is not None:
                self.destroy_subscription(self._image_sub)
                self._image_sub = None

    def _on_fetch_state(self, msg: String):
        state = msg.data.strip()
        if state in {
            'idle',
            'received_command',
            'turn_to_speaker',
            'understand_command',
            'query_memory',
            'navigate_to_object_point',
        }:
            self._fetch_active = False
        elif state == 'fetch_vision_active' or state in {
            'record_person_pose',
            'confirm_target',
            'select_tracking_target',
            'wait_tracker_lock',
            'auto_aim',
            'auto_aim_done',
        }:
            self._fetch_active = True
        self._publish_active_state()

    def _on_patrol_scan_cmd(self, _msg: String):
        self._patrol_active = True
        self._publish_active_state()

    def _on_fetch_trigger(self, _msg: String):
        self._fetch_active = True
        self._publish_active_state()

    def _on_patrol_scan_done(self, _msg: String):
        self._patrol_active = False
        self._publish_active_state()

    def _publish_active_state(self, force=False):
        enabled = self._is_enabled()
        if not force and enabled == self._last_enabled:
            return
        self._last_enabled = enabled
        self._set_image_subscription(enabled)
        if enabled:
            self._last_frame_started = 0.0
        self._active_pub.publish(Bool(data=enabled))
        self.get_logger().info(
            f'YOLO推理{"开启" if enabled else "暂停"} '
            f'(fetch={self._fetch_active}, patrol={self._patrol_active})'
        )

    # ==================== 发布检测结果 ====================

    def _resolve_target_class_ids(self, target_classes):
        if target_classes is None:
            return None

        class_names = getattr(self, '_engine', None).class_names if hasattr(self, '_engine') else COCO_CLASSES
        name_to_id = {name: idx for idx, name in enumerate(class_names)}
        class_ids = []
        unknown = []
        for class_name in sorted(target_classes):
            if class_name in name_to_id:
                class_ids.append(name_to_id[class_name])
            else:
                unknown.append(class_name)

        if unknown:
            self.get_logger().warn(
                f'未知 YOLO 类别，将忽略: {",".join(unknown)}'
            )
        if not class_ids:
            self.get_logger().warn('target_classes 没有有效类别，本节点不会输出检测结果')
        return class_ids

    def _publish_detections(self, orig_msg, detections, infer_ms):
        perception = PerceptionTargets()
        perception.header = orig_msg.header
        perception.fps = int(self._fps)

        # 推理性能
        perf = Perf()
        perf.type = 'yolov8'
        perf.time_ms_duration = infer_ms
        perception.perfs = [perf]

        for det in detections:
            target = Target()
            target.type = det['class_name']
            target.track_id = 0  # 跟踪器接入后填充

            roi = Roi()
            roi.type = 'body'
            roi.confidence = det['score']
            x1, y1, x2, y2 = det['box']
            roi.rect = RegionOfInterest()
            roi.rect.x_offset = x1
            roi.rect.y_offset = y1
            roi.rect.width = x2 - x1
            roi.rect.height = y2 - y1
            target.rois = [roi]

            perception.targets.append(target)

        self._det_pub.publish(perception)

    # ==================== 发布标注图像 ====================

    def _publish_annotated(self, orig_msg, cv_image, detections):
        annotated = cv_image.copy()
        for det in detections:
            x1, y1, x2, y2 = det['box']
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 180, 0), 2)
            label = f"{det['class_name']} {det['score']:.2f}"
            cv2.putText(annotated, label, (x1, max(20, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 180, 0), 2)

        annotated_rgb = cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB)
        img_msg = self._bridge.cv2_to_imgmsg(annotated_rgb, encoding='rgb8')
        img_msg.header = orig_msg.header
        self._img_pub.publish(img_msg)

    # ==================== FPS ====================

    def _update_fps(self):
        self._frame_count += 1
        elapsed = time.time() - self._fps_start
        if elapsed >= 1.0:
            self._fps = self._frame_count / elapsed
            self._frame_count = 0
            self._fps_start = time.time()


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

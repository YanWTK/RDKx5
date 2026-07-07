import cv2
import numpy as np
from hobot_dnn import pyeasy_dnn

from .nms import nms_per_class

INPUT_SIZE = 640
REG_MAX = 16
STRIDES = [8, 16, 32]

COCO_CLASSES = [
    "person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa",
    "pottedplant", "bed", "diningtable", "toilet", "tvmonitor", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush",
]


def _parse_class_names(class_names):
    if not class_names:
        return None
    if isinstance(class_names, str):
        parsed = [name.strip() for name in class_names.split(",") if name.strip()]
    else:
        parsed = [str(name).strip() for name in class_names if str(name).strip()]
    return parsed or None


def _sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


def _softmax(x, axis=-1):
    x = x - np.max(x, axis=axis, keepdims=True)
    e = np.exp(x)
    return e / np.sum(e, axis=axis, keepdims=True)


def _bgr_to_nv12(image):
    h, w = image.shape[:2]
    area = h * w
    yuv420p = cv2.cvtColor(image, cv2.COLOR_BGR2YUV_I420).reshape(area * 3 // 2)
    y = yuv420p[:area]
    uv_planar = yuv420p[area:].reshape(2, area // 4)
    uv_packed = uv_planar.T.reshape(area // 2)
    return np.concatenate([y, uv_packed]).astype(np.uint8)


def _letterbox(image, size=INPUT_SIZE, color=114):
    orig_h, orig_w = image.shape[:2]
    ratio = min(size / orig_w, size / orig_h)
    new_w = int(round(orig_w * ratio))
    new_h = int(round(orig_h * ratio))
    resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((size, size, 3), color, dtype=np.uint8)
    dw = (size - new_w) // 2
    dh = (size - new_h) // 2
    canvas[dh:dh + new_h, dw:dw + new_w] = resized
    return canvas, ratio, dw, dh


def _decode_scale(cls_logits, box_raw, stride, conf_thresh, allowed_class_ids=None):
    """DFL 解码单个尺度的检测结果。"""
    h, w, _ = cls_logits.shape
    scores_all = _sigmoid(cls_logits)
    if allowed_class_ids is None:
        scores = scores_all.max(axis=-1)
        class_ids = scores_all.argmax(axis=-1)
    else:
        allowed = np.asarray(allowed_class_ids, dtype=np.int32)
        allowed = allowed[(allowed >= 0) & (allowed < scores_all.shape[-1])]
        if allowed.size == 0:
            return []
        class_scores = scores_all[..., allowed]
        best_allowed = class_scores.argmax(axis=-1)
        scores = class_scores.max(axis=-1)
        class_ids = allowed[best_allowed]
    ys, xs = np.where(scores >= conf_thresh)
    if len(xs) == 0:
        return []

    d = box_raw[ys, xs].reshape(-1, 4, REG_MAX)
    prob = _softmax(d, axis=-1)
    bins = np.arange(REG_MAX, dtype=np.float32)
    dist = (prob * bins).sum(axis=-1) * stride

    cx = (xs.astype(np.float32) + 0.5) * stride
    cy = (ys.astype(np.float32) + 0.5) * stride
    x1 = cx - dist[:, 0]
    y1 = cy - dist[:, 1]
    x2 = cx + dist[:, 2]
    y2 = cy + dist[:, 3]
    boxes = np.stack([x1, y1, x2, y2], axis=1)

    return [(box, float(score), int(cls_id))
            for box, score, cls_id in zip(boxes, scores[ys, xs], class_ids[ys, xs])]


class YOLOEngine:
    """YOLOv8 BPU 推理引擎，封装模型加载、前处理、推理、后处理。"""

    def __init__(
        self,
        model_path,
        conf_threshold=0.25,
        nms_threshold=0.7,
        class_names=None,
        preprocess_mode="resize",
    ):
        self._conf = conf_threshold
        self._nms = nms_threshold
        self._model = pyeasy_dnn.load(model_path)[0]
        self._custom_class_names = _parse_class_names(class_names)
        self._class_names = self._custom_class_names or COCO_CLASSES
        self._preprocess_mode = str(preprocess_mode or "resize").strip().lower()
        if self._preprocess_mode not in {"resize", "letterbox"}:
            self._preprocess_mode = "resize"

    @property
    def class_names(self):
        return self._class_names

    def detect(self, image_bgr, allowed_class_ids=None):
        """输入 BGR 图像，返回检测结果列表。

        每个结果: {"box": [x1,y1,x2,y2], "score": float, "class_id": int, "class_name": str}
        """
        orig_h, orig_w = image_bgr.shape[:2]
        if self._preprocess_mode == "letterbox":
            input_image, ratio, dw, dh = _letterbox(image_bgr)
            scale_info = ("letterbox", ratio, dw, dh)
        else:
            input_image = cv2.resize(
                image_bgr, (INPUT_SIZE, INPUT_SIZE), interpolation=cv2.INTER_AREA
            )
            scale_info = ("resize", orig_w / INPUT_SIZE, orig_h / INPUT_SIZE)
        nv12 = _bgr_to_nv12(input_image)
        outputs = self._model.forward(nv12)
        return self._postprocess(outputs, orig_h, orig_w, allowed_class_ids, scale_info)

    def _postprocess(self, outputs, orig_h, orig_w, allowed_class_ids=None, scale_info=None):
        named_outputs = self._map_named_outputs(outputs)
        if named_outputs:
            return self._postprocess_named_outputs(
                named_outputs, orig_h, orig_w, allowed_class_ids, scale_info
            )
        return self._postprocess_ordered_outputs(
            outputs, orig_h, orig_w, allowed_class_ids, scale_info
        )

    def _postprocess_ordered_outputs(
        self, outputs, orig_h, orig_w, allowed_class_ids=None, scale_info=None
    ):
        all_dets = []
        for scale_idx, stride in enumerate(STRIDES):
            cls_out = np.array(outputs[scale_idx * 2].buffer, dtype=np.float32).squeeze()
            box_tensor = outputs[scale_idx * 2 + 1]
            box_deq = self._tensor_array(box_tensor)
            all_dets.extend(
                _decode_scale(cls_out, box_deq, stride, self._conf, allowed_class_ids)
            )
        return self._format_detections(all_dets, orig_h, orig_w, scale_info)

    def _postprocess_named_outputs(
        self, mapped, orig_h, orig_w, allowed_class_ids=None, scale_info=None
    ):
        all_dets = []
        for suffix, stride in (("80", 8), ("40", 16), ("20", 32)):
            cls_chw = mapped.get(f"cls{suffix}")
            box_chw = mapped.get(f"box{suffix}")
            if cls_chw is None or box_chw is None:
                continue
            cls_hwc = np.transpose(cls_chw, (1, 2, 0))
            box_hwc = np.transpose(box_chw, (1, 2, 0))
            all_dets.extend(
                _decode_scale(cls_hwc, box_hwc, stride, self._conf, allowed_class_ids)
            )
        return self._format_detections(all_dets, orig_h, orig_w, scale_info)

    def _format_detections(self, all_dets, orig_h, orig_w, scale_info=None):
        if not all_dets:
            return []

        boxes = np.array([d[0] for d in all_dets], dtype=np.float32)
        scores = np.array([d[1] for d in all_dets], dtype=np.float32)
        class_ids = np.array([d[2] for d in all_dets], dtype=np.int32)

        kept = nms_per_class(boxes, scores, class_ids, self._nms)

        if scale_info is None:
            scale_info = ("resize", orig_w / INPUT_SIZE, orig_h / INPUT_SIZE)
        dets = []
        for i in kept:
            x1, y1, x2, y2 = boxes[i]
            if scale_info[0] == "letterbox":
                _, ratio, dw, dh = scale_info
                x1 = (x1 - dw) / ratio
                x2 = (x2 - dw) / ratio
                y1 = (y1 - dh) / ratio
                y2 = (y2 - dh) / ratio
            else:
                _, sx, sy = scale_info
                x1 *= sx
                x2 *= sx
                y1 *= sy
                y2 *= sy
            cid = int(class_ids[i])
            class_name = (
                self._class_names[cid]
                if cid < len(self._class_names)
                else f"class_{cid}"
            )
            dets.append({
                "box": [
                    int(np.clip(round(x1), 0, orig_w - 1)),
                    int(np.clip(round(y1), 0, orig_h - 1)),
                    int(np.clip(round(x2), 0, orig_w - 1)),
                    int(np.clip(round(y2), 0, orig_h - 1)),
                ],
                "score": float(scores[i]),
                "class_id": cid,
                "class_name": class_name,
            })
        return dets

    def _tensor_array(self, tensor):
        arr = np.array(tensor.buffer, dtype=np.float32).squeeze()
        scale_data = getattr(getattr(tensor, "properties", None), "scale_data", None)
        if scale_data is not None and len(scale_data):
            scale = scale_data.astype(np.float32)
            if arr.ndim == 3 and arr.shape[-1] == scale.size:
                arr = arr * scale.reshape(1, 1, -1)
            elif arr.ndim == 3 and arr.shape[0] == scale.size:
                arr = arr * scale.reshape(-1, 1, 1)
        return arr

    def _map_named_outputs(self, outputs):
        mapped = {}
        for out in outputs:
            name = getattr(out, "name", "")
            arr = self._tensor_array(out)
            if arr.ndim != 3:
                continue
            if "cv2.0" in name:
                mapped["box80"] = arr
            elif "cv2.1" in name:
                mapped["box40"] = arr
            elif "cv2.2" in name:
                mapped["box20"] = arr
            elif "cv3.0" in name:
                mapped["cls80"] = arr
            elif "cv3.1" in name:
                mapped["cls40"] = arr
            elif "cv3.2" in name:
                mapped["cls20"] = arr
        if {"box80", "box40", "box20", "cls80", "cls40", "cls20"} <= set(mapped):
            if self._custom_class_names is None:
                cls_count = int(mapped["cls80"].shape[0])
                if cls_count != len(COCO_CLASSES):
                    self._class_names = [f"class_{i}" for i in range(cls_count)]
            return mapped
        return {}

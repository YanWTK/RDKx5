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


def _decode_scale(cls_logits, box_raw, stride, conf_thresh):
    """DFL 解码单个尺度的检测结果。"""
    h, w, _ = cls_logits.shape
    scores_all = _sigmoid(cls_logits)
    scores = scores_all.max(axis=-1)
    class_ids = scores_all.argmax(axis=-1)
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

    def __init__(self, model_path, conf_threshold=0.25, nms_threshold=0.7):
        self._conf = conf_threshold
        self._nms = nms_threshold
        self._model = pyeasy_dnn.load(model_path)[0]

    def detect(self, image_bgr):
        """输入 BGR 图像，返回检测结果列表。

        每个结果: {"box": [x1,y1,x2,y2], "score": float, "class_id": int, "class_name": str}
        """
        orig_h, orig_w = image_bgr.shape[:2]
        resized = cv2.resize(image_bgr, (INPUT_SIZE, INPUT_SIZE), interpolation=cv2.INTER_AREA)
        nv12 = _bgr_to_nv12(resized)
        outputs = self._model.forward(nv12)
        return self._postprocess(outputs, orig_h, orig_w)

    def _postprocess(self, outputs, orig_h, orig_w):
        all_dets = []
        for scale_idx, stride in enumerate(STRIDES):
            cls_out = np.array(outputs[scale_idx * 2].buffer, dtype=np.float32).squeeze()
            box_tensor = outputs[scale_idx * 2 + 1]
            box_raw = np.array(box_tensor.buffer, dtype=np.float32).squeeze()
            scale = box_tensor.properties.scale_data.astype(np.float32).reshape(1, 1, -1)
            box_deq = box_raw * scale
            all_dets.extend(_decode_scale(cls_out, box_deq, stride, self._conf))

        if not all_dets:
            return []

        boxes = np.array([d[0] for d in all_dets], dtype=np.float32)
        scores = np.array([d[1] for d in all_dets], dtype=np.float32)
        class_ids = np.array([d[2] for d in all_dets], dtype=np.int32)

        kept = nms_per_class(boxes, scores, class_ids, self._nms)

        sx, sy = orig_w / INPUT_SIZE, orig_h / INPUT_SIZE
        dets = []
        for i in kept:
            x1, y1, x2, y2 = boxes[i]
            cid = int(class_ids[i])
            dets.append({
                "box": [
                    int(np.clip(round(x1 * sx), 0, orig_w - 1)),
                    int(np.clip(round(y1 * sy), 0, orig_h - 1)),
                    int(np.clip(round(x2 * sx), 0, orig_w - 1)),
                    int(np.clip(round(y2 * sy), 0, orig_h - 1)),
                ],
                "score": float(scores[i]),
                "class_id": cid,
                "class_name": COCO_CLASSES[cid] if cid < len(COCO_CLASSES) else f"class_{cid}",
            })
        return dets

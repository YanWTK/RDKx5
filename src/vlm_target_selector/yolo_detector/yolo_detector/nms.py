import numpy as np


def nms(boxes, scores, iou_thresh):
    """标准贪心 NMS，返回保留的索引。"""
    order = scores.argsort()[::-1]
    keep = []
    x1, y1, x2, y2 = boxes.T
    areas = np.maximum(0, x2 - x1) * np.maximum(0, y2 - y1)
    while order.size:
        i = order[0]
        keep.append(i)
        if order.size == 1:
            break
        rest = order[1:]
        xx1 = np.maximum(x1[i], x1[rest])
        yy1 = np.maximum(y1[i], y1[rest])
        xx2 = np.minimum(x2[i], x2[rest])
        yy2 = np.minimum(y2[i], y2[rest])
        inter = np.maximum(0, xx2 - xx1) * np.maximum(0, yy2 - yy1)
        iou = inter / (areas[i] + areas[rest] - inter + 1e-6)
        order = rest[iou <= iou_thresh]
    return keep


def nms_per_class(boxes, scores, class_ids, iou_thresh):
    """逐类别 NMS，返回按分数降序排列的全局索引。"""
    keep_all = []
    for cls_id in np.unique(class_ids):
        idx = np.where(class_ids == cls_id)[0]
        for k in nms(boxes[idx], scores[idx], iou_thresh):
            keep_all.append(idx[k])
    return sorted(keep_all, key=lambda i: float(scores[i]), reverse=True)

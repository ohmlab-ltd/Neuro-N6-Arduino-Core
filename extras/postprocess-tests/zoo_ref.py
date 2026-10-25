# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT
"""Reference decoders for the model-zoo post-processors.

Detections are (x_center, y_center, width, height, conf, class), all
NN-normalised.
"""
import math

import numpy as np

MAX_DET = 100      # ZOO_PP_MAX_DET


def deq(q, scale, zp):
    return (q.astype(np.float32) - np.float32(zp)) * np.float32(scale)


def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


def iou(a, b):
    ax0, ax1 = a[0] - a[2] / 2, a[0] + a[2] / 2
    ay0, ay1 = a[1] - a[3] / 2, a[1] + a[3] / 2
    bx0, bx1 = b[0] - b[2] / 2, b[0] + b[2] / 2
    by0, by1 = b[1] - b[3] / 2, b[1] + b[3] / 2
    iw = min(ax1, bx1) - max(ax0, bx0)
    ih = min(ay1, by1) - max(ay0, by0)
    if iw <= 0 or ih <= 0:
        return 0.0
    inter = iw * ih
    uni = a[2] * a[3] + b[2] * b[3] - inter
    return inter / uni if uni > 0 else 0.0


def nms(dets, thr, max_keep=MAX_DET):
    """Greedy per-class NMS, best first, as zoo_nms does."""
    out = []
    for d in sorted(dets, key=lambda d: -d[4]):
        if len(out) >= max_keep:
            break
        if any(k[5] == d[5] and iou(k, d) > thr for k in out):
            continue
        out.append(d)
    return out


def st_nms_per_class(dets, iou_thr, limit, nb_classes):
    """ST's NMS: per class, best first, keeping at most limit of each class."""
    kept = list(dets)
    dead = [False] * len(kept)
    for k in range(nb_classes):
        idx = [i for i in range(len(kept)) if kept[i][5] == k and not dead[i]]
        idx.sort(key=lambda i: -kept[i][4])
        alive = []
        for i in idx:
            if any(iou(kept[j], kept[i]) > iou_thr for j in alive):
                dead[i] = True
                continue
            alive.append(i)
        for i in alive[limit:]:
            dead[i] = True
    return [kept[i] for i in range(len(kept)) if not dead[i]]


def yolo_grid_decode(cell, nc, nb_anchors, anchors, g, conf_thr, scale, zp, single_sigmoid):
    """One grid of a YOLOv2 or ST YOLOX head, laid out [row][col][anchor][5+nc].

    As in ST's int8 path, the raw objectness is compared with the quantised
    threshold."""
    dets = []
    stride = 5 + nc
    if single_sigmoid:
        computed = -math.log(1.0 / conf_thr - 1.0)
        thr_s8 = np.int8(np.int32(np.int8(np.int32(computed / scale + 0.5))) + np.int32(zp))
    for row in range(g):
        for col in range(g):
            for a in range(nb_anchors):
                o = ((row * g + col) * nb_anchors + a) * stride
                q = cell[o:o + stride]
                if single_sigmoid:
                    if q[4] < thr_s8:
                        continue
                    conf = float(sigmoid(deq(q[4], scale, zp)))
                    cls = 0
                else:
                    v = deq(q, scale, zp)
                    prob = float(sigmoid(v[4]))
                    e = np.exp(v[5:])
                    p = e / e.sum()
                    cls = int(np.argmax(v[5:]))
                    conf = float(p[cls] * prob)
                    if conf < conf_thr:
                        continue
                v = deq(q, scale, zp)
                x = (col + float(sigmoid(v[0]))) / g
                y = (row + float(sigmoid(v[1]))) / g
                w = anchors[2 * a + 0] * float(np.exp(v[2])) / g
                h = anchors[2 * a + 1] * float(np.exp(v[3])) / g
                dets.append((x, y, w, h, conf, cls))
    return dets


def decode_yolox(levels, nc, conf_thr, iou_thr):
    """levels: list of (chw array [5+nc, g, g], scale, zp), largest grid first."""
    dets = []
    for arr, scale, zp in levels:
        g = arr.shape[1]
        cell = np.transpose(arr, (1, 2, 0)).reshape(-1)   # [row][col][ch]
        anchors = [0.5 * g, 0.5 * g]
        dets += yolo_grid_decode(cell, nc, 1, anchors, g, conf_thr, scale, zp, nc == 1)
    dets = st_nms_per_class(dets, iou_thr, MAX_DET, nc)
    dets = [d for d in dets if d[4] >= conf_thr]
    return nms(dets, iou_thr)


ST_YOLOV2_ANCHORS = [0.076023, 0.258508, 0.163031, 0.413531, 0.234769,
                     0.702585, 0.427054, 0.715892, 0.748154, 0.857092]


def decode_pixelkit(arr, scale, zp, dim, conf_thr, iou_thr):
    """PixelKit's YOLOX head, [rows][5 + nc]: a row per cell of the stride
    8, 16 and 32 grids in that order, each dx, dy, dw, dh in grid units, then
    the objectness and the class scores, both already sigmoided."""
    v = arr.astype(np.float32) if arr.dtype == np.float32 else deq(arr, scale, zp)
    dets, r = [], 0
    for stride in (8, 16, 32):
        g = dim // stride
        for gy in range(g):
            for gx in range(g):
                row = v[r]
                r += 1
                c = int(np.argmax(row[5:]))
                score = float(row[4]) * float(row[5 + c])
                if not score >= conf_thr:
                    continue
                ew = min(max(float(row[2]), -16.0), 16.0)
                eh = min(max(float(row[3]), -16.0), 16.0)
                dets.append(((float(row[0]) + gx) * stride / dim,
                             (float(row[1]) + gy) * stride / dim,
                             math.exp(ew) * stride / dim,
                             math.exp(eh) * stride / dim, score, c))
    return nms(dets, iou_thr)


def decode_yolov2(arr, scale, zp, conf_thr, iou_thr):
    """arr: [anchors*(5+nc), g, g] as the N6 leaves it."""
    g = arr.shape[1]
    ch = arr.shape[0]
    nc = ch // 5 - 5
    cell = np.transpose(arr, (1, 2, 0)).reshape(-1)
    dets = yolo_grid_decode(cell, nc, 5, ST_YOLOV2_ANCHORS, g, conf_thr, scale, zp, nc == 1)
    dets = st_nms_per_class(dets, iou_thr, MAX_DET, nc)
    dets = [d for d in dets if d[4] != 0]
    return nms(dets, iou_thr)


def decode_yolov8(arr, scale, zp, conf_thr, iou_thr, input_wh=None):
    """arr: [4 + nc][boxes], quantised, or float with scale 1 and zp 0."""
    nc = arr.shape[0] - 4
    quantised = arr.dtype != np.float32
    v = deq(arr, scale, zp) if quantised else arr
    # ST thresholds the quantised score, then the dequantised one after NMS.
    thr_q = np.int8(np.int32(np.int8(np.int32(conf_thr / scale + 0.5))) + np.int32(zp)) if quantised else None
    dets = []
    for i in range(arr.shape[1]):
        cls = int(np.argmax(v[4:, i]))
        conf = float(v[4 + cls, i])
        if quantised and arr[4 + cls, i] < thr_q:
            continue
        if conf < conf_thr:
            continue
        dets.append((float(v[0, i]), float(v[1, i]), float(v[2, i]), float(v[3, i]), conf, cls))
    dets = st_nms_per_class(dets, iou_thr, MAX_DET, nc)
    dets = [d for d in dets if d[4] >= conf_thr]
    if input_wh is not None and any(max(d[0], d[1], d[2], d[3]) > 1.5 for d in dets):
        w, h = input_wh
        dets = [(d[0] / w, d[1] / h, d[2] / w, d[3] / h, d[4], d[5]) for d in dets]
    return nms(dets, iou_thr)


SSD_SPECS = [(19, 16, 60, 105), (10, 32, 105, 150), (5, 64, 150, 195),
             (3, 100, 195, 240), (2, 150, 240, 285), (1, 300, 285, 330)]


def ssd_priors(image_size=300.0):
    """generate_ssd_priors() from ST's postprocess.py, in numpy."""
    priors = []
    for fm, shrink, smin, smax in SSD_SPECS:
        scale = image_size / shrink
        for j in range(fm):
            for i in range(fm):
                cx = (i + 0.5) / scale
                cy = (j + 0.5) / scale
                s = smin / image_size
                big = math.sqrt(smax * smin) / image_size
                priors.append([cx, cy, s, s])
                priors.append([cx, cy, big, big])
                for ratio in (2, 3):
                    r = math.sqrt(ratio)
                    priors.append([cx, cy, s * r, s / r])
                    priors.append([cx, cy, s / r, s * r])
    return np.clip(np.array(priors, dtype=np.float32), 0.0, 1.0)


def decode_ssd(scores, boxes, conf_thr, iou_thr, image_size=300.0):
    """scores: [priors][1 + nc] raw, boxes: [priors][4] offsets, both float."""
    pri = ssd_priors(image_size)
    e = np.exp(scores - scores.max(axis=1, keepdims=True))
    p = e / e.sum(axis=1, keepdims=True)
    dets = []
    for i in range(scores.shape[0]):
        cls = int(np.argmax(p[i, 1:])) + 1
        if p[i, cls] < conf_thr:
            continue
        cx = boxes[i, 0] * 0.1 * pri[i, 2] + pri[i, 0]
        cy = boxes[i, 1] * 0.1 * pri[i, 3] + pri[i, 1]
        w = math.exp(boxes[i, 2] * 0.2) * pri[i, 2]
        h = math.exp(boxes[i, 3] * 0.2) * pri[i, 3]
        dets.append((float(cx), float(cy), float(w), float(h), float(p[i, cls]), cls - 1))
    return nms(dets, iou_thr)


def blazeface_anchors(size):
    out = []
    for grid, per in ((size // 8, 2), (size // 16, 6)):
        for y in range(grid):
            for x in range(grid):
                for _ in range(per):
                    out.append(((x + 0.5) / grid, (y + 0.5) / grid))
    return np.array(out, dtype=np.float32)


def decode_blazeface(levels, size, conf_thr, iou_thr):
    """levels: [(boxes [n][cols] float, scores [n] float)], stride 8 first."""
    anchors = blazeface_anchors(size)
    dets = []
    base = 0
    for bx, sc in levels:
        computed = -math.log(1.0 / conf_thr - 1.0)
        for i in range(bx.shape[0]):
            if sc[i] < computed:
                continue
            a = anchors[base + i]
            dets.append((float(bx[i, 0] / size + a[0]), float(bx[i, 1] / size + a[1]),
                         float(bx[i, 2] / size), float(bx[i, 3] / size),
                         float(sigmoid(sc[i])), 0))
        base += bx.shape[0]
    dets = st_nms_per_class(dets, iou_thr, MAX_DET, 1)
    dets = [d for d in dets if d[4] >= conf_thr]
    return nms(dets, iou_thr)


def decode_movenet(hm, scale, zp, conf_thr):
    """hm: [h][w][17] quantised heatmaps. Returns (keypoints, box or None)."""
    h, w, k = hm.shape
    kps = []
    for i in range(k):
        flat = hm[:, :, i].reshape(-1)
        idx = int(np.argmax(flat))          # first maximum wins, as ST's helper does
        x = ((idx % h) + 0.5) / h
        y = ((idx // h) + 0.5) / w
        kps.append((x, y, float(deq(flat[idx], scale, zp))))
    good = [p for p in kps if p[2] >= conf_thr]
    box = None
    if len(good) >= 3:
        x0 = min(p[0] for p in good)
        x1 = max(p[0] for p in good)
        y0 = min(p[1] for p in good)
        y1 = max(p[1] for p in good)
        px = 0.1 * (x1 - x0) + 0.02
        py = 0.1 * (y1 - y0) + 0.02
        x0, x1 = max(0.0, x0 - px), min(1.0, x1 + px)
        y0, y1 = max(0.0, y0 - py), min(1.0, y1 + py)
        conf = sum(p[2] for p in good) / len(good)
        box = ((x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0, conf, 0)
    return kps, box


def decode_seg(arr, chw=True):
    """arr: [classes][h][w] or [h][w][classes]. Returns (label map, regions)."""
    if chw:
        ncls, h, w = arr.shape
        lab = np.argmax(arr, axis=0).astype(np.uint8)
    else:
        h, w, ncls = arr.shape
        lab = np.argmax(arr, axis=2).astype(np.uint8)
    npix = h * w
    min_count = npix // 256 + 1
    regions = []
    for c in range(1, ncls):
        ys, xs = np.nonzero(lab == c)
        if xs.size < min_count:
            continue
        x0, x1 = int(xs.min()), int(xs.max()) + 1
        y0, y1 = int(ys.min()), int(ys.max()) + 1
        regions.append(((x0 + x1) / 2 / w, (y0 + y1) / 2 / h, (x1 - x0) / w,
                        (y1 - y0) / h, xs.size / npix, c))
    regions.sort(key=lambda r: -r[4])
    return lab, regions[:MAX_DET]


def decode_classifier(v, conf_thr, top_k, probs):
    """v: dequantised class scores."""
    if not probs:
        e = np.exp(v - v.max())
        p = e / e.sum()
    else:
        p = v
    order = np.argsort(-p, kind="stable")[:top_k]
    out = []
    for i in order:
        if p[i] < conf_thr:
            break
        out.append((int(i), float(p[i])))
    return out

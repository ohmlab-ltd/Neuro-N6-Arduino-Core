# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT
"""Host tests for the model-zoo post-processors.

Runs test_zoo.exe over each case and checks its output against zoo_ref.py.

    python run_tests.py [name ...]
"""
import math
import os
import subprocess
import sys
import tempfile

import numpy as np

import zoo_ref as R

HERE = os.path.dirname(os.path.abspath(__file__))
# build.bat uses the same directory for test_zoo.exe.
WORK = os.environ.get("N6_PP_WORK",
                      os.path.join(tempfile.gettempdir(), "n6-postprocess-tests"))
EXE = os.path.join(WORK, "test_zoo.exe")
CASES = os.path.join(WORK, "cases")
ZOO = os.environ.get("N6_ZOO_DIR", os.path.join(HERE, "models"))

os.makedirs(CASES, exist_ok=True)
rng = np.random.default_rng(7)

PASS, FAIL = [], []


def check(name, cond, msg=""):
    (PASS if cond else FAIL).append(name)
    print(("  ok   " if cond else "  FAIL ") + name + (("  " + msg) if (msg and not cond) else ""))


def quantise(v, scale, zp, dtype=np.int8):
    lo, hi = (-128, 127) if dtype == np.int8 else (0, 255)
    return np.clip(np.rint(v / scale) + zp, lo, hi).astype(dtype)


def q8(v, scale, zp):
    """One value quantised to int8."""
    return np.int8(np.clip(round(v / scale) + zp, -128, 127))


def write_manifest(name, tensors, pp, conf, iou=0.5, top_k=5, input_wh=None, camera=None,
                   slot=0):
    """tensors: list of (name, array, scale, zp, chpos). Returns the path."""
    path = os.path.join(CASES, name + ".txt")
    lines = ["slot %d" % slot] if slot else []
    if input_wh:
        lines.append("input %d %d" % input_wh)
    if camera:
        lines.append("camera %d %d %d %d %d %d" % camera)
    for tname, arr, scale, zp, chpos in tensors:
        bin_path = os.path.join(CASES, "%s_%s.bin" % (name, tname))
        arr.tofile(bin_path)
        kind = {np.dtype("float32"): "float", np.dtype("int8"): "int8",
                np.dtype("uint8"): "uint8"}[arr.dtype]
        lines.append("tensor %s %s %.9g %d %d %d %s %s" % (
            tname, kind, scale, zp, chpos, arr.ndim,
            " ".join(str(d) for d in arr.shape), bin_path))
    lines.append("pp %s %.6f %.6f %d" % (pp, conf, iou, top_k))
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
    return path


def run(path):
    out = subprocess.run([EXE, path], capture_output=True, text=True)
    if out.returncode != 0:
        raise RuntimeError("test_zoo failed: %s%s" % (out.stdout, out.stderr))
    res = {"det": [], "meta": b"", "top": None}
    for line in out.stdout.splitlines():
        p = line.split()
        if p[0] == "status":
            res["status"] = int(p[1])
        elif p[0] == "lcd":
            res["lcd"] = p[1]
            res["drawn"] = int(p[3])
            res["commits"] = int(p[5])
        elif p[0] == "nb":
            res["nb"] = int(p[1])
        elif p[0] == "det":
            res["det"].append(tuple(float(x) for x in p[1:5]) + (float(p[5]), int(p[6])))
        elif p[0] == "top":
            res["top"] = (int(p[1]), float(p[2]))
        elif p[0] == "meta":
            res["meta"] = bytes.fromhex(p[2]) if len(p) > 2 else b""
    return res


def tlvs(meta):
    out, i = [], 0
    while i + 3 <= len(meta):
        tag = meta[i]
        ln = meta[i + 1] | (meta[i + 2] << 8)
        out.append((tag, meta[i + 3:i + 3 + ln]))
        i += 3 + ln
    return out


def same_dets(got, want, tol=2e-3):
    if len(got) != len(want):
        return False, "%d boxes, expected %d" % (len(got), len(want))
    for i, (g, w) in enumerate(zip(got, want)):
        if g[5] != w[5]:
            return False, "box %d: class %d, expected %d" % (i, g[5], w[5])
        for k in range(5):
            if abs(g[k] - w[k]) > tol:
                return False, "box %d field %d: %.5f, expected %.5f" % (i, k, g[k], w[k])
    return True, ""


def cam_norm(v, crop_off, crop_len, full):
    return (crop_off + v * crop_len) / full


def check_boxes_meta(name, res, dets, camera=(640, 480, 0, 0, 640, 480), labels=None):
    """META_BOXES must hold the same boxes, mapped to camera coordinates with
    the corners clipped to the frame, and each one's name must follow in
    META_ISEG_CLASSSTR."""
    recs = tlvs(res["meta"])
    ms = [v for t, v in recs if t == 1]
    check(name + ": inference time record", len(ms) == 1 and int.from_bytes(ms[0], "little") == 12)
    boxes = [v for t, v in recs if t == 2]
    if not dets:
        check(name + ": box record", len(boxes) == 1 and boxes[0][0] == 0)
        return
    if len(boxes) != 1:
        check(name + ": box record", False, "%d box records" % len(boxes))
        return
    b = boxes[0]
    n = b[0]
    ok = n == len(dets)
    msg = "%d boxes in the record, expected %d" % (n, len(dets))
    fw, fh, cx, cy, cw, ch = camera
    for i, d in enumerate(dets[:n]):
        off = 1 + i * 10
        got = [int.from_bytes(b[off + 2 * k:off + 2 * k + 2], "little") / 65535.0 for k in range(4)]
        l, r, t, bo = (min(max(v, 0.0), 1.0) for v in (
            cam_norm(d[0] - d[2] / 2, cx, cw, fw), cam_norm(d[0] + d[2] / 2, cx, cw, fw),
            cam_norm(d[1] - d[3] / 2, cy, ch, fh), cam_norm(d[1] + d[3] / 2, cy, ch, fh)))
        want = [(l + r) / 2, (t + bo) / 2, r - l, bo - t]
        if any(abs(got[k] - want[k]) > 2e-3 for k in range(4)):
            ok, msg = False, "box %d: %s, expected %s" % (i, got, want)
            break
        if b[off + 9] != d[5]:
            ok, msg = False, "box %d: class %d in the record, expected %d" % (i, b[off + 9], d[5])
            break
    check(name + ": boxes in the metadata", ok, msg)
    names = [v for t, v in recs if t == 10]
    if labels is not None and dets:
        got_names = [bytes(v[1:]).decode() for v in names]
        want_names = [labels[d[5]] if d[5] < len(labels) else "class %d" % d[5] for d in dets[:n]]
        check(name + ": class names", got_names == want_names,
              "%s, expected %s" % (got_names[:3], want_names[:3]))


COCO = None  # set by main()


def have(model):
    """True when the zoo file is in N6_ZOO_DIR."""
    if os.path.isfile(os.path.join(ZOO, model)):
        return True
    print("  skip  %s (not in %s)" % (model, ZOO))
    return False


def onnx_run(model, seed=1):
    import onnxruntime as ort
    sess = ort.InferenceSession(os.path.join(ZOO, model), providers=["CPUExecutionProvider"])
    inp = sess.get_inputs()[0]
    shape = [d if isinstance(d, int) else 1 for d in inp.shape]
    r = np.random.default_rng(seed)
    x = r.random(shape, dtype=np.float32)
    if inp.type == "tensor(uint8)":
        x = (x * 255).astype(np.uint8)
    outs = sess.run(None, {inp.name: x})
    return {o.name: v for o, v in zip(sess.get_outputs(), outs)}, shape


# Real models

def case_yolox(conf=0.3):
    if not have("st_yoloxn_d033_w025_192_qdq_int8.onnx"):
        return
    outs, shape = onnx_run("st_yoloxn_d033_w025_192_qdq_int8.onnx")
    vals = sorted(outs.values(), key=lambda v: -v.shape[-1])      # 24, 12, 6
    qp = [(0.3383, 110), (0.16744, 90), (0.18053, 91)]            # scale, zp on the N6
    tensors, levels = [], []
    for i, (v, (sc, zp)) in enumerate(zip(vals, qp)):
        q = quantise(v, sc, zp)
        tensors.append(("out%d" % i, q, sc, zp, 1))
        levels.append((q[0], sc, zp))
    name = "yolox_%g" % conf
    path = write_manifest(name, tensors, "yolox", conf, 0.5, input_wh=(192, 192))
    res = run(path)
    want = R.decode_yolox(levels, 1, conf, 0.5)
    check(name + ": status", res["status"] == 0, str(res["status"]))
    ok, msg = same_dets(res["det"], want)
    check(name + ": boxes (%d)" % len(want), ok, msg)
    check(name + ": overlay in bounds", res["lcd"] == "ok")
    check_boxes_meta(name, res, want, labels=["class 0"])


def case_yolov2(conf=0.3):
    if not have("yolov2t_192_qdq_int8.onnx"):
        return
    outs, shape = onnx_run("yolov2t_192_qdq_int8.onnx")
    v = list(outs.values())[0]
    sc, zp = 0.10411, 24
    q = quantise(v, sc, zp)
    name = "yolov2_%g" % conf
    path = write_manifest(name, [("out", q, sc, zp, 1)], "yolov2", conf, 0.5, input_wh=(192, 192))
    res = run(path)
    want = R.decode_yolov2(q[0], sc, zp, conf, 0.5)
    check(name + ": status", res["status"] == 0, str(res["status"]))
    ok, msg = same_dets(res["det"], want)
    check(name + ": boxes (%d)" % len(want), ok, msg)
    check(name + ": overlay in bounds", res["lcd"] == "ok")
    check_boxes_meta(name, res, want)


def case_ssd(conf=0.3):
    if not have("ssd_mobilenetv2_pt_coco_300_qdq_int8.onnx"):
        return
    outs, shape = onnx_run("ssd_mobilenetv2_pt_coco_300_qdq_int8.onnx")
    scores = [v for v in outs.values() if v.shape[-1] > 4][0]
    boxes = [v for v in outs.values() if v.shape[-1] == 4][0]
    s_sc, s_zp = 0.14933, -55
    b_sc, b_zp = 0.09028, 72
    qs, qb = quantise(scores, s_sc, s_zp), quantise(boxes, b_sc, b_zp)
    name = "ssd_%g" % conf
    path = write_manifest(name, [("scores", qs, s_sc, s_zp, 2), ("boxes", qb, b_sc, b_zp, 2)],
                          "ssd", conf, 0.5, input_wh=(300, 300))
    res = run(path)
    want = R.decode_ssd(R.deq(qs[0], s_sc, s_zp), R.deq(qb[0], b_sc, b_zp), conf, 0.5, 300.0)
    check(name + ": status", res["status"] == 0, str(res["status"]))
    ok, msg = same_dets(res["det"], want)
    check(name + ": boxes (%d)" % len(want), ok, msg)
    check(name + ": overlay in bounds", res["lcd"] == "ok")


def case_deeplab():
    if not have("st_deeplabv3_mnv2_a050_s16_asppv2_256_qdq_int8.onnx"):
        return
    outs, shape = onnx_run("st_deeplabv3_mnv2_a050_s16_asppv2_256_qdq_int8.onnx")
    v = list(outs.values())[0]
    sc, zp = 0.23244, 2
    q = quantise(v, sc, zp)
    name = "deeplab"
    path = write_manifest(name, [("out", q, sc, zp, 1)], "deeplab", 0.0, 0.0, input_wh=(256, 256))
    res = run(path)
    lab, want = R.decode_seg(q[0], chw=True)
    check(name + ": status", res["status"] == 0, str(res["status"]))
    ok, msg = same_dets(res["det"], want)
    check(name + ": regions (%d)" % len(want), ok, msg)
    check(name + ": overlay in bounds", res["lcd"] == "ok")
    recs = tlvs(res["meta"])
    masks = [v for t, v in recs if t == 11]
    check(name + ": one mask a region", len(masks) == len(want), "%d masks" % len(masks))
    if masks and want:
        w = int(masks[0][1])
        h = int(masks[0][2])
        nbytes = int.from_bytes(masks[0][3:5], "little")
        check(name + ": mask size", nbytes == (w * h + 7) // 8 and len(masks[0]) == 5 + nbytes,
              "%dx%d, %d bytes, record %d" % (w, h, nbytes, len(masks[0])))


def case_classifier(conf=0.01, top_k=5):
    if not have("mobilenetv2_a035_128_qdq_int8.onnx"):
        return
    outs, shape = onnx_run("mobilenetv2_a035_128_qdq_int8.onnx")
    v = np.asarray(list(outs.values())[0], dtype=np.float32)
    name = "classifier_float"
    path = write_manifest(name, [("out", v, 1.0, 0, 0)], "classifier", conf, 0.0, top_k,
                          input_wh=(128, 128))
    res = run(path)
    flat = v.reshape(-1)
    probs = bool(flat.min() >= -0.01 and flat.max() <= 1.01)
    want = R.decode_classifier(flat, conf, top_k, probs)
    recs = tlvs(res["meta"])
    rec = [x for t, x in recs if t == 0x54]
    check(name + ": status", res["status"] == 0, str(res["status"]))
    check(name + ": one class record", len(rec) == 1)
    if rec:
        got = parse_image_class(rec[0])
        check(name + ": class count", got["count"] == len(want), "%d, expected %d" % (got["count"], len(want)))
        check(name + ": class total", got["num_classes"] == flat.size)
        okv = all(e[0] == w[0] and abs(e[1] / 1000.0 - w[1]) <= 0.002 for e, w in zip(got["entries"], want))
        check(name + ": class entries", okv, "%s vs %s" % (got["entries"][:3], want[:3]))
    check(name + ": top", res["top"] is not None and want and res["top"][0] == want[0][0])
    check(name + ": overlay in bounds", res["lcd"] == "ok")


def parse_image_class(v):
    out = {"count": v[0], "num_classes": int.from_bytes(v[2:4], "little"),
           "threshold": int.from_bytes(v[4:6], "little"), "entries": [], "names": []}
    i = 6
    for _ in range(out["count"]):
        idx = int.from_bytes(v[i:i + 2], "little")
        score = int.from_bytes(v[i + 2:i + 4], "little")
        nl = v[i + 4]
        out["entries"].append((idx, score))
        out["names"].append(bytes(v[i + 5:i + 5 + nl]).decode())
        i += 5 + nl
    return out


# Synthetic

def plant_yolov8(nb, nc, boxes):
    """[4 + nc][nb] floats with a few boxes planted, the rest noise."""
    a = rng.random((4 + nc, nb), dtype=np.float32) * 0.05
    a[0:4] = rng.random((4, nb), dtype=np.float32)
    for i, (cx, cy, w, h, score, cls) in enumerate(boxes):
        a[0, i], a[1, i], a[2, i], a[3, i] = cx, cy, w, h
        a[4 + cls, i] = score
    return a


def case_yolov8_int8():
    nb, nc = 1344, 80
    boxes = [(0.5, 0.5, 0.2, 0.4, 0.9, 0), (0.52, 0.51, 0.2, 0.4, 0.8, 0),
             (0.2, 0.3, 0.1, 0.1, 0.7, 15)]
    a = plant_yolov8(nb, nc, boxes)
    sc, zp = 1.0 / 255.0, -128
    q = quantise(a, sc, zp)[None, :, :]
    name = "yolov8_int8"
    path = write_manifest(name, [("out", q, sc, zp, 1)], "yolov8", 0.5, 0.5, input_wh=(256, 256))
    res = run(path)
    want = R.decode_yolov8(q[0], sc, zp, 0.5, 0.5, (256, 256))
    check(name + ": status", res["status"] == 0, str(res["status"]))
    ok, msg = same_dets(res["det"], want)
    check(name + ": boxes (%d)" % len(want), ok, msg)
    check(name + ": the overlapping duplicate is dropped", len(want) == 2, str(len(want)))
    check(name + ": overlay in bounds", res["lcd"] == "ok")
    check_boxes_meta(name, res, want, labels=[c for c in COCO])


def case_yolov8_pixels():
    """An ONNX-style export: float output with boxes in input pixels."""
    nb, nc, side = 1344, 80, 256.0
    boxes = [(0.5 * side, 0.5 * side, 0.2 * side, 0.4 * side, 0.9, 3)]
    a = plant_yolov8(nb, nc, boxes)
    name = "yolov8_pixels"
    path = write_manifest(name, [("out", a[None, :, :], 1.0, 0, 1)], "yolov8", 0.5, 0.5,
                          input_wh=(256, 256))
    res = run(path)
    want = R.decode_yolov8(a, 1.0, 0, 0.5, 0.5, (256.0, 256.0))
    check(name + ": status", res["status"] == 0, str(res["status"]))
    ok, msg = same_dets(res["det"], want)
    check(name + ": boxes rescaled to 0..1 (%d)" % len(want), ok, msg)


def case_yolov8_transposed():
    """[boxes][4 + nc] instead of ST's order."""
    nb, nc = 1344, 80
    boxes = [(0.5, 0.5, 0.2, 0.4, 0.9, 7)]
    a = plant_yolov8(nb, nc, boxes)
    sc, zp = 1.0 / 255.0, -128
    q = quantise(a, sc, zp)
    name = "yolov8_transposed"
    path = write_manifest(name, [("out", np.ascontiguousarray(q.T)[None, :, :], sc, zp, 2)],
                          "yolov8", 0.5, 0.5, input_wh=(256, 256))
    res = run(path)
    want = R.decode_yolov8(q, sc, zp, 0.5, 0.5, (256, 256))
    check(name + ": status", res["status"] == 0, str(res["status"]))
    ok, msg = same_dets(res["det"], want)
    check(name + ": boxes (%d)" % len(want), ok, msg)


def case_blazeface():
    size = 128
    n0, n1 = (size // 8) ** 2 * 2, (size // 16) ** 2 * 6
    b0 = (rng.random((n0, 16), dtype=np.float32) - 0.5) * 20.0
    b1 = (rng.random((n1, 16), dtype=np.float32) - 0.5) * 20.0
    s0 = np.full(n0, -8.0, dtype=np.float32)
    s1 = np.full(n1, -8.0, dtype=np.float32)
    s0[100] = 4.0                      # one clear face
    b0[100, :4] = [2.0, -3.0, 40.0, 44.0]
    s1[7] = 2.0
    b1[7, :4] = [0.0, 0.0, 30.0, 30.0]
    name = "blazeface"
    path = write_manifest(name, [("b0", b0[None], 1.0, 0, 2), ("s0", s0[None, :, None], 1.0, 0, 2),
                                 ("s1", s1[None, :, None], 1.0, 0, 2), ("b1", b1[None], 1.0, 0, 2)],
                          "blazeface", 0.5, 0.3, input_wh=(size, size))
    res = run(path)
    want = R.decode_blazeface([(b0, s0), (b1, s1)], size, 0.5, 0.3)
    check(name + ": status", res["status"] == 0, str(res["status"]))
    ok, msg = same_dets(res["det"], want)
    check(name + ": faces (%d)" % len(want), ok, msg)
    check(name + ": overlay in bounds", res["lcd"] == "ok")
    check_boxes_meta(name, res, want, labels=["face"])


def movenet_tensor():
    h = w = 48
    sc, zp = 0.0038002, -128
    hm = np.full((h, w, 17), zp, dtype=np.int8)
    peaks = [(10, 12), (11, 13), (12, 14), (13, 15), (14, 16), (20, 10), (20, 30),
             (25, 8), (25, 32), (30, 6), (30, 34), (32, 14), (32, 26), (40, 13),
             (40, 27), (46, 12), (46, 28)]
    for k, (r, c) in enumerate(peaks):
        hm[r, c, k] = 100 if k < 15 else -120      # the last two stay low
    return hm, sc, zp


def case_movenet():
    hm, sc, zp = movenet_tensor()
    name = "movenet"
    path = write_manifest(name, [("hm", hm[None], sc, zp, 1)], "movenet", 0.2, 0.0,
                          input_wh=(192, 192))
    res = run(path)
    kps, box = R.decode_movenet(hm, sc, zp, 0.2)
    check(name + ": status", res["status"] == 0, str(res["status"]))
    ok, msg = same_dets(res["det"], [box] if box else [])
    check(name + ": person box", ok, msg)
    recs = tlvs(res["meta"])
    kp_rec = [v for t, v in recs if t == 3]
    bi_rec = [v for t, v in recs if t == 4]
    check(name + ": keypoint record", len(kp_rec) == 1 and kp_rec[0][0] == 17,
          str(kp_rec[0][0]) if kp_rec else "none")
    check(name + ": skeleton record", len(bi_rec) == 1 and bi_rec[0][0] == 19,
          str(bi_rec[0][0]) if bi_rec else "none")
    if kp_rec:
        okk = True
        for i, (x, y, p) in enumerate(kps):
            off = 1 + i * 7   # id, colour, x, y, conf
            gx = int.from_bytes(kp_rec[0][off + 2:off + 4], "little") / 65535.0
            gy = int.from_bytes(kp_rec[0][off + 4:off + 6], "little") / 65535.0
            if abs(gx - x) > 2e-3 or abs(gy - y) > 2e-3:
                okk = False
                break
        check(name + ": keypoint positions", okk)
    check(name + ": overlay in bounds", res["lcd"] == "ok")


def case_movenet_chw():
    hm, sc, zp = movenet_tensor()
    chw = np.ascontiguousarray(np.transpose(hm, (2, 0, 1)))
    name = "movenet_chw"
    path = write_manifest(name, [("hm", chw[None], sc, zp, 1)], "movenet", 0.2, 0.0,
                          input_wh=(192, 192))
    res = run(path)
    kps, box = R.decode_movenet(hm, sc, zp, 0.2)
    check(name + ": status", res["status"] == 0, str(res["status"]))
    ok, msg = same_dets(res["det"], [box] if box else [])
    check(name + ": person box from channel-first heatmaps", ok, msg)


def case_seg_hwc():
    h = w = 64
    a = np.full((h, w, 3), -100, dtype=np.int8)
    a[:, :, 0] = 100
    a[10:40, 20:50, 1] = 120
    a[0:3, 0:3, 2] = 120            # too small to report
    sc, zp = 0.1, 0
    name = "seg_hwc"
    path = write_manifest(name, [("out", a[None], sc, zp, 2)], "deeplab", 0.0, 0.0,
                          input_wh=(64, 64))
    res = run(path)
    lab, want = R.decode_seg(a, chw=False)
    check(name + ": status", res["status"] == 0, str(res["status"]))
    ok, msg = same_dets(res["det"], want)
    check(name + ": regions (%d)" % len(want), ok, msg)
    check(name + ": the tiny class is left out", len(want) == 1, str(len(want)))


def case_classifier_logits():
    n = 1000
    v = (rng.random(n, dtype=np.float32) - 0.5) * 4.0
    v[42] = 8.0
    v[7] = 6.0
    sc, zp = 0.08, -10
    q = quantise(v, sc, zp)
    name = "classifier_logits_int8"
    path = write_manifest(name, [("out", q[None], sc, zp, 0)], "classifier", 0.05, 0.0, 3,
                          input_wh=(128, 128))
    res = run(path)
    want = R.decode_classifier(R.deq(q, sc, zp), 0.05, 3, probs=False)
    rec = [x for t, x in tlvs(res["meta"]) if t == 0x54]
    check(name + ": status", res["status"] == 0, str(res["status"]))
    got = parse_image_class(rec[0]) if rec else {"count": -1, "entries": []}
    check(name + ": entries", got["count"] == len(want) and
          all(e[0] == w[0] and abs(e[1] / 1000.0 - w[1]) <= 0.002 for e, w in zip(got["entries"], want)),
          "%s vs %s" % (got["entries"], want))
    check(name + ": softmax applied to raw scores", want and abs(want[0][1] - 0.75) < 0.3,
          str(want[:1]))
    check(name + ": names come from the label list", got.get("names", [""])[0] == COCO[want[0][0]]
          if want and want[0][0] < 80 else True)


def case_classifier_empty():
    v = np.full(10, 0.1, dtype=np.float32)
    name = "classifier_empty"
    path = write_manifest(name, [("out", v[None], 1.0, 0, 0)], "classifier", 0.5, 0.0, 5,
                          input_wh=(128, 128))
    res = run(path)
    rec = [x for t, x in tlvs(res["meta"]) if t == 0x54]
    got = parse_image_class(rec[0]) if rec else None
    check(name + ": a record even with nothing above the threshold", got is not None and got["count"] == 0)
    check(name + ": threshold reported", got is not None and got["threshold"] == 500)
    check(name + ": top says nothing", res["top"] == (-1, 0.0), str(res["top"]))


def case_crop_mapping():
    nb, nc = 1344, 80
    a = plant_yolov8(nb, nc, [(0.5, 0.5, 0.25, 0.25, 0.9, 0)])
    sc, zp = 1.0 / 255.0, -128
    q = quantise(a, sc, zp)[None]
    camera = (640, 480, 80, 0, 480, 480)     # ASPECT_CROP_CENTER on VGA
    name = "crop_mapping"
    path = write_manifest(name, [("out", q, sc, zp, 1)], "yolov8", 0.5, 0.5,
                          input_wh=(256, 256), camera=camera)
    res = run(path)
    want = R.decode_yolov8(q[0], sc, zp, 0.5, 0.5, (256, 256))
    check(name + ": status", res["status"] == 0, str(res["status"]))
    check_boxes_meta(name, res, want, camera=camera, labels=[c for c in COCO])


def case_letterbox_mapping():
    """ASPECT_LETTERBOX on VGA: the window starts above the frame, so a box in
    the top bar is clipped to the frame."""
    nb, nc = 1344, 80
    a = plant_yolov8(nb, nc, [(0.5, 0.15, 0.25, 0.2, 0.9, 0), (0.3, 0.5, 0.2, 0.3, 0.8, 2)])
    sc, zp = 1.0 / 255.0, -128
    q = quantise(a, sc, zp)[None]
    camera = (640, 480, 0, -80, 640, 640)
    name = "letterbox_mapping"
    path = write_manifest(name, [("out", q, sc, zp, 1)], "yolov8", 0.5, 0.5,
                          input_wh=(256, 256), camera=camera)
    res = run(path)
    want = R.decode_yolov8(q[0], sc, zp, 0.5, 0.5, (256, 256))
    check(name + ": status", res["status"] == 0, str(res["status"]))
    check(name + ": two boxes", len(want) == 2, "%d boxes" % len(want))
    check(name + ": overlay in bounds", res["lcd"] == "ok")
    check_boxes_meta(name, res, want, camera=camera, labels=[c for c in COCO])


def case_other_slot():
    """On slots 1 to 3 a model sends no names or masks: they would index slot
    0's box list."""
    nb, nc = 1344, 80
    a = plant_yolov8(nb, nc, [(0.5, 0.5, 0.2, 0.4, 0.9, 0)])
    sc, zp = 1.0 / 255.0, -128
    q = quantise(a, sc, zp)[None]
    res = run(write_manifest("slot1_detect", [("out", q, sc, zp, 1)], "yolov8", 0.5, 0.5,
                             input_wh=(256, 256), slot=1))
    tags = [t for t, v in tlvs(res["meta"])]
    check("slot 1 detector: decodes", res["status"] == 0 and res["nb"] == 1, str(res["nb"]))
    check("slot 1 detector: no boxes, names or inference time",
          not any(t in (1, 2, 10) for t in tags), str(tags))

    h = w = 64
    s = np.full((h, w, 3), -100, dtype=np.int8)
    s[:, :, 0] = 100
    s[10:40, 20:50, 1] = 120
    res = run(write_manifest("slot1_seg", [("out", s[None], 0.1, 0, 2)], "deeplab", 0.0, 0.0,
                             input_wh=(64, 64), slot=1))
    tags = [t for t, v in tlvs(res["meta"])]
    check("slot 1 segmentation: one region", res["status"] == 0 and res["nb"] == 1, str(res["nb"]))
    check("slot 1 segmentation: no masks, names or boxes",
          not any(t in (1, 2, 10, 11) for t in tags), str(tags))


def case_errors():
    a = rng.random((1, 6, 24, 24), dtype=np.float32)
    path = write_manifest("err_yolox_one_map", [("out", a, 1.0, 0, 1)], "yolox", 0.5, 0.5,
                          input_wh=(192, 192))
    res = run(path)
    check("errors: YOLOX with one map", res["status"] == -10 and res["nb"] == 0, str(res["status"]))

    u = (rng.random((1, 84, 1344)) * 255).astype(np.uint8)
    path = write_manifest("err_yolov8_uint8", [("out", u, 1.0 / 255, 0, 1)], "yolov8", 0.5, 0.5,
                          input_wh=(256, 256))
    res = run(path)
    check("errors: uint8 YOLOv8 output", res["status"] == -11, str(res["status"]))

    b = rng.random((1, 3000, 4), dtype=np.float32)
    path = write_manifest("err_ssd_no_scores", [("boxes", b, 1.0, 0, 2)], "ssd", 0.5, 0.5,
                          input_wh=(300, 300))
    res = run(path)
    check("errors: SSD without scores", res["status"] == -10, str(res["status"]))

    small = rng.random((1, 2, 600, 600), dtype=np.float32)
    path = write_manifest("err_seg_too_big", [("out", small, 1.0, 0, 1)], "deeplab", 0.0, 0.0,
                          input_wh=(600, 600))
    res = run(path)
    check("errors: segmentation larger than the label map", res["status"] == -12, str(res["status"]))


def case_yolox_multiclass():
    """With several classes ST's decoder scores a softmax over the classes
    times the objectness."""
    nc, grids = 5, (24, 12, 6)
    sc, zp = 0.25, 10
    tensors, levels = [], []
    for i, g in enumerate(grids):
        a = np.full((5 + nc, g, g), q8(-6.0, sc, zp), dtype=np.int8)
        a[0:4] = q8(0.0, sc, zp)
        if i == 0:
            a[4, 6, 8] = q8(4.0, sc, zp)       # objectness
            a[5 + 2, 6, 8] = q8(5.0, sc, zp)   # class 2 wins here
            a[5 + 1, 6, 8] = q8(2.0, sc, zp)
            a[4, 10, 3] = q8(3.0, sc, zp)
            a[5 + 4, 10, 3] = q8(4.0, sc, zp)
        if i == 2:
            a[4, 1, 1] = q8(2.0, sc, zp)
            a[5 + 0, 1, 1] = q8(6.0, sc, zp)
        tensors.append(("out%d" % i, a[None], sc, zp, 1))
        levels.append((a, sc, zp))
    name = "yolox_multiclass"
    res = run(write_manifest(name, tensors, "yolox", 0.3, 0.5, input_wh=(192, 192)))
    want = R.decode_yolox(levels, nc, 0.3, 0.5)
    check(name + ": status", res["status"] == 0, str(res["status"]))
    check(name + ": three classes found", len(want) == 3, "%d boxes" % len(want))
    ok, msg = same_dets(res["det"], want)
    check(name + ": boxes (%d)" % len(want), ok, msg)
    check(name + ": classes", sorted(d[5] for d in want) == [0, 2, 4],
          str(sorted(d[5] for d in want)))
    check_boxes_meta(name, res, want)
    # Five classes, so the COCO names do not apply and the numbers stand in.
    names = [bytes(v[1:]).decode() for t, v in tlvs(res["meta"]) if t == 10]
    check(name + ": names fall back to the class number",
          names == ["class %d" % d[5] for d in want], str(names))


# Output scale and zero point of a 320 px PixelKit export in the N6 build.
PK_SCALE, PK_ZP = 0.0212278161197901, -28


def pixelkit_rows(dim):
    return sum((dim // s) ** 2 for s in (8, 16, 32))


def plant_pixelkit(dim, nc, cells):
    """[rows][5 + nc] with the given cells planted, each
    (stride, gx, gy, dx, dy, width_px, height_px, obj, cls, cls_score)."""
    rows = pixelkit_rows(dim)
    a = np.zeros((rows, 5 + nc), dtype=np.float32)
    a[:, 0:2] = rng.random((rows, 2), dtype=np.float32)
    a[:, 2:4] = rng.random((rows, 2), dtype=np.float32) * 2.0 - 1.0
    a[:, 4] = rng.random(rows, dtype=np.float32) * 0.02
    a[:, 5:] = rng.random((rows, nc), dtype=np.float32) * 0.05
    offset = {8: 0, 16: (dim // 8) ** 2, 32: (dim // 8) ** 2 + (dim // 16) ** 2}
    for stride, gx, gy, dx, dy, wpx, hpx, obj, cls, score in cells:
        r = offset[stride] + gy * (dim // stride) + gx
        a[r, 0:5] = (dx, dy, math.log(wpx / stride), math.log(hpx / stride), obj)
        a[r, 5 + cls] = score
    return a


def case_pixelkit_int8():
    dim, nc = 320, 2
    cells = [(8, 12, 10, 0.5, 0.5, 40, 60, 0.95, 0, 0.9),     # a box at (100, 84)
             (16, 6, 5, 0.25, 0.25, 40, 60, 0.9, 0, 0.8),     # the same box again, weaker
             (32, 3, 7, 0.5, 0.5, 64, 96, 0.8, 1, 0.85),      # the other class
             (8, 30, 30, 0.5, 0.5, 30, 30, 0.5, 1, 0.5)]      # below the threshold
    q = quantise(plant_pixelkit(dim, nc, cells), PK_SCALE, PK_ZP)
    name = "pixelkit_int8"
    path = write_manifest(name, [("out", q[None], PK_SCALE, PK_ZP, 1)], "pixelkit", 0.5, 0.45,
                          top_k=nc, input_wh=(dim, dim))
    res = run(path)
    want = R.decode_pixelkit(q, PK_SCALE, PK_ZP, dim, 0.5, 0.45)
    check(name + ": status", res["status"] == 0, str(res["status"]))
    check(name + ": two boxes survive", len(want) == 2, "%d boxes" % len(want))
    ok, msg = same_dets(res["det"], want)
    check(name + ": boxes (%d)" % len(want), ok, msg)
    check(name + ": overlay in bounds", res["lcd"] == "ok")
    check_boxes_meta(name, res, want, labels=COCO[:nc])


def case_pixelkit_float():
    """A float output at 256 px with three classes and no names given."""
    dim, nc = 256, 3
    cells = [(8, 4, 20, 0.3, 0.7, 24, 24, 0.9, 2, 0.95),
             (16, 10, 3, 0.5, 0.5, 80, 50, 0.85, 1, 0.9)]
    a = plant_pixelkit(dim, nc, cells)
    name = "pixelkit_float"
    res = run(write_manifest(name, [("out", a[None], 1.0, 0, 1)], "pixelkit", 0.4, 0.45,
                             top_k=0, input_wh=(dim, dim)))
    want = R.decode_pixelkit(a, 1.0, 0, dim, 0.4, 0.45)
    check(name + ": status", res["status"] == 0, str(res["status"]))
    ok, msg = same_dets(res["det"], want)
    check(name + ": boxes (%d)" % len(want), ok, msg)
    names = [bytes(v[1:]).decode() for t, v in tlvs(res["meta"]) if t == 10]
    check(name + ": names fall back to the class number",
          names == ["class %d" % d[5] for d in want], str(names))


def case_pixelkit_errors():
    dim, nc = 320, 2
    q = quantise(plant_pixelkit(dim, nc, []), PK_SCALE, PK_ZP)[None]
    res = run(write_manifest("err_pk_names", [("out", q, PK_SCALE, PK_ZP, 1)], "pixelkit", 0.5,
                             0.45, top_k=3, input_wh=(dim, dim)))
    check("pixelkit errors: names from another export", res["status"] == -10 and res["nb"] == 0,
          str(res["status"]))
    res = run(write_manifest("err_pk_size", [("out", q, PK_SCALE, PK_ZP, 1)], "pixelkit", 0.5,
                             0.45, top_k=nc, input_wh=(256, 256)))
    check("pixelkit errors: rows for another input size", res["status"] == -10, str(res["status"]))
    res = run(write_manifest("err_pk_square", [("out", q, PK_SCALE, PK_ZP, 1)], "pixelkit", 0.5,
                             0.45, top_k=nc, input_wh=(320, 240)))
    check("pixelkit errors: input not square", res["status"] == -10, str(res["status"]))
    u = (q.astype(np.int16) + 128).astype(np.uint8)
    res = run(write_manifest("err_pk_uint8", [("out", u, PK_SCALE, 100, 1)], "pixelkit", 0.5,
                             0.45, top_k=nc, input_wh=(dim, dim)))
    check("pixelkit errors: uint8 output", res["status"] == -11, str(res["status"]))


def case_pixelkit_model():
    """The PixelKit export named by N6_PIXELKIT_ONNX, its <name>_int8.onnx,
    decoded at a low and an ordinary threshold."""
    path = os.environ.get("N6_PIXELKIT_ONNX")
    if not path or not os.path.isfile(path):
        print("  skip  set N6_PIXELKIT_ONNX to a PixelKit export's int8 ONNX")
        return
    import onnx
    import onnxruntime as ort
    g = onnx.load(path).graph
    init = {t.name: onnx.numpy_helper.to_array(t) for t in g.initializer}
    last = next(n for n in g.node if g.output[0].name in n.output)
    if last.op_type != "DequantizeLinear":
        check("pixelkit model: the output is dequantised int8", False, last.op_type)
        return
    sc, zp = float(init[last.input[1]]), int(init[last.input[2]])
    sess = ort.InferenceSession(path, providers=["CPUExecutionProvider"])
    inp = sess.get_inputs()[0]
    dim = int(inp.shape[-1])
    x = np.random.default_rng(3).random((1, 3, dim, dim), dtype=np.float32)
    out = sess.run(None, {inp.name: x})[0][0]
    # One cell is raised to a 48 px box of class 0 centred at (44, 60).
    out[7 * (dim // 8) + 5, 0:6] = (0.5, 0.5, math.log(6.0), math.log(6.0), 0.95, 0.95)
    q = quantise(out, sc, zp)
    for conf in (0.05, 0.3):
        name = "pixelkit_model_%g" % conf
        res = run(write_manifest(name, [("out", q[None], sc, zp, 1)], "pixelkit", conf, 0.45,
                                 top_k=q.shape[1] - 5, input_wh=(dim, dim)))
        want = R.decode_pixelkit(q, sc, zp, dim, conf, 0.45)
        check(name + ": status", res["status"] == 0, str(res["status"]))
        ok, msg = same_dets(res["det"], want)
        check(name + ": boxes (%d)" % len(want), ok, msg)
        hit = [d for d in res["det"] if d[5] == 0 and abs(d[0] - 44.0 / dim) < 2.0 / dim
               and abs(d[1] - 60.0 / dim) < 2.0 / dim and abs(d[2] - 48.0 / dim) < 3.0 / dim]
        check(name + ": the planted box is found where it was put", len(hit) == 1, str(res["det"][:3]))


def case_yolov2_multiclass():
    nc, g, anchors = 3, 13, 5
    sc, zp = 0.2, -5
    a = np.full((anchors * (5 + nc), g, g), q8(-6.0, sc, zp), dtype=np.int8)
    for ch in range(anchors):
        a[ch * (5 + nc):ch * (5 + nc) + 4] = q8(0.0, sc, zp)
    a[2 * (5 + nc) + 4, 4, 5] = q8(4.0, sc, zp)
    a[2 * (5 + nc) + 5 + 1, 4, 5] = q8(5.0, sc, zp)
    name = "yolov2_multiclass"
    res = run(write_manifest(name, [("out", a[None], sc, zp, 1)], "yolov2", 0.3, 0.5,
                             input_wh=(416, 416)))
    want = R.decode_yolov2(a, sc, zp, 0.3, 0.5)
    check(name + ": status", res["status"] == 0, str(res["status"]))
    check(name + ": one box, class 1", len(want) == 1 and want[0][5] == 1, str(want))
    ok, msg = same_dets(res["det"], want)
    check(name + ": boxes (%d)" % len(want), ok, msg)


def case_blazeface_int8():
    size = 128
    n0, n1 = (size // 8) ** 2 * 2, (size // 16) ** 2 * 6
    b_sc, b_zp = 0.35, -3
    s_sc, s_zp = 0.12, 20
    b0 = np.full((n0, 16), q8(0.0, b_sc, b_zp), dtype=np.int8)
    b1 = np.full((n1, 16), q8(0.0, b_sc, b_zp), dtype=np.int8)
    s0 = np.full(n0, q8(-8.0, s_sc, s_zp), dtype=np.int8)
    s1 = np.full(n1, q8(-8.0, s_sc, s_zp), dtype=np.int8)
    s0[42] = q8(4.0, s_sc, s_zp)
    b0[42, :4] = [q8(2.0, b_sc, b_zp), q8(-3.0, b_sc, b_zp),
                  q8(40.0, b_sc, b_zp), q8(40.0, b_sc, b_zp)]
    name = "blazeface_int8"
    res = run(write_manifest(name, [("b0", b0[None], b_sc, b_zp, 2), ("s0", s0[None, :, None], s_sc, s_zp, 2),
                                    ("s1", s1[None, :, None], s_sc, s_zp, 2), ("b1", b1[None], b_sc, b_zp, 2)],
                             "blazeface", 0.5, 0.3, input_wh=(size, size)))
    want = R.decode_blazeface([(R.deq(b0, b_sc, b_zp), R.deq(s0, s_sc, s_zp)),
                               (R.deq(b1, b_sc, b_zp), R.deq(s1, s_sc, s_zp))], size, 0.5, 0.3)
    check(name + ": status", res["status"] == 0, str(res["status"]))
    check(name + ": one face", len(want) == 1, "%d faces" % len(want))
    ok, msg = same_dets(res["det"], want, tol=6e-3)
    check(name + ": boxes (%d)" % len(want), ok, msg)


def case_classifier_uint8():
    """A model whose scores are already probabilities, quantised unsigned."""
    n = 40
    sc, zp = 1.0 / 255.0, 0
    v = np.full(n, 2, dtype=np.uint8)
    v[9] = 200
    v[31] = 120
    name = "classifier_uint8"
    res = run(write_manifest(name, [("out", v[None], sc, zp, 0)], "classifier", 0.3, 0.0, 4,
                             input_wh=(128, 128)))
    want = R.decode_classifier(R.deq(v, sc, zp), 0.3, 4, probs=True)
    rec = [x for t, x in tlvs(res["meta"]) if t == 0x54]
    got = parse_image_class(rec[0]) if rec else {"count": -1, "entries": []}
    check(name + ": status", res["status"] == 0, str(res["status"]))
    check(name + ": probabilities used as they are", got["count"] == len(want) and
          all(e[0] == w[0] and abs(e[1] / 1000.0 - w[1]) <= 0.002 for e, w in zip(got["entries"], want)),
          "%s vs %s" % (got["entries"], want))
    check(name + ": no softmax over probabilities",
          bool(want) and abs(want[0][1] - 200 / 255.0) < 0.01, str(want[:1]))


def case_seg_many_classes():
    """Twenty-one classes, the Pascal VOC count, channel-first."""
    ncls, h, w = 21, 128, 128
    sc, zp = 0.15, -8
    a = np.full((ncls, h, w), q8(-4.0, sc, zp), dtype=np.int8)
    a[0] = q8(1.0, sc, zp)
    a[15, 20:90, 30:100] = q8(6.0, sc, zp)     # person
    a[7, 100:120, 10:60] = q8(6.0, sc, zp)     # car
    name = "seg_many_classes"
    res = run(write_manifest(name, [("out", a[None], sc, zp, 1)], "deeplab", 0.0, 0.0,
                             input_wh=(128, 128)))
    lab, want = R.decode_seg(a, chw=True)
    check(name + ": status", res["status"] == 0, str(res["status"]))
    check(name + ": two regions, largest first", [d[5] for d in want] == [15, 7],
          str([d[5] for d in want]))
    ok, msg = same_dets(res["det"], want)
    check(name + ": regions (%d)" % len(want), ok, msg)
    recs = tlvs(res["meta"])
    names = [bytes(v[1:]).decode() for t, v in recs if t == 10]
    check(name + ": names fall back to the class number",
          names == ["class 15", "class 7"], str(names))


def case_seg_many_classes_hwc():
    """Twenty-one classes, channel-last: more than 16 values a pixel."""
    ncls, h, w = 21, 96, 80
    sc, zp = 0.15, -8
    a = np.full((h, w, ncls), q8(-4.0, sc, zp), dtype=np.int8)
    a[:, :, 0] = q8(1.0, sc, zp)
    a[10:60, 5:45, 15] = q8(6.0, sc, zp)
    a[70:90, 50:78, 20] = q8(6.0, sc, zp)
    name = "seg_many_classes_hwc"
    res = run(write_manifest(name, [("out", a[None], sc, zp, 2)], "deeplab", 0.0, 0.0,
                             input_wh=(w, h)))
    lab, want = R.decode_seg(a, chw=False)
    check(name + ": status", res["status"] == 0, str(res["status"]))
    check(name + ": two regions, largest first", [d[5] for d in want] == [15, 20],
          str([d[5] for d in want]))
    ok, msg = same_dets(res["det"], want)
    check(name + ": regions (%d)" % len(want), ok, msg)


CASES_ALL = {
    "yolox": lambda: (case_yolox(0.3), case_yolox(0.05)),
    "yolov2": lambda: (case_yolov2(0.3), case_yolov2(0.05)),
    "ssd": lambda: case_ssd(0.3),
    "deeplab": case_deeplab,
    "classifier": case_classifier,
    "yolov8": lambda: (case_yolov8_int8(), case_yolov8_pixels(), case_yolov8_transposed()),
    "blazeface": case_blazeface,
    "movenet": lambda: (case_movenet(), case_movenet_chw()),
    "seg_hwc": case_seg_hwc,
    "classifier_logits": case_classifier_logits,
    "classifier_empty": case_classifier_empty,
    "crop": case_crop_mapping,
    "letterbox": case_letterbox_mapping,
    "other_slot": case_other_slot,
    "errors": case_errors,
    "yolox_multiclass": case_yolox_multiclass,
    "pixelkit": lambda: (case_pixelkit_int8(), case_pixelkit_float(), case_pixelkit_errors(),
                         case_pixelkit_model()),
    "yolov2_multiclass": case_yolov2_multiclass,
    "blazeface_int8": case_blazeface_int8,
    "classifier_uint8": case_classifier_uint8,
    "seg_many_classes": case_seg_many_classes,
    "seg_many_classes_hwc": case_seg_many_classes_hwc,
}


def add_planted():
    import planted
    CASES_ALL["yolox_planted"] = planted.case_yolox_planted
    CASES_ALL["yolov2_planted"] = planted.case_yolov2_planted
    CASES_ALL["ssd_planted"] = planted.case_ssd_planted
    CASES_ALL["deeplab_planted"] = planted.case_deeplab_planted
    CASES_ALL["classifier_planted"] = planted.case_classifier_planted


def main():
    global COCO
    COCO = [line.strip() for line in open(os.path.join(WORK, "coco.txt"))]
    add_planted()
    wanted = sys.argv[1:] or list(CASES_ALL)
    for name in wanted:
        print(name + ":")
        CASES_ALL[name]()
    print("\n%d passed, %d failed" % (len(PASS), len(FAIL)))
    for f in FAIL:
        print("  FAILED: " + f)
    return 1 if FAIL else 0


if __name__ == "__main__":
    # Run through the imported module, not this __main__ copy: planted.py's
    # "import run_tests" must be the same module object.
    import run_tests

    sys.exit(run_tests.main())

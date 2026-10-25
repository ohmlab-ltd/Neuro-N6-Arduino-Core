# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT
"""Cases that plant detections into the real models' outputs."""
import numpy as np

import run_tests as T
import zoo_ref as R


def q(v, scale, zp):
    return np.int8(np.clip(round(v / scale) + zp, -128, 127))


def case_yolox_planted():
    if not T.have("st_yoloxn_d033_w025_192_qdq_int8.onnx"):
        return
    outs, _ = T.onnx_run("st_yoloxn_d033_w025_192_qdq_int8.onnx")
    vals = sorted(outs.values(), key=lambda v: -v.shape[-1])
    qp = [(0.3383, 110), (0.16744, 90), (0.18053, 91)]
    arrs = [T.quantise(v, sc, zp)[0] for v, (sc, zp) in zip(vals, qp)]

    # Two people in the fine grid, one overlapped by a weaker box, and one in
    # the coarse grid. Channels are x, y, w, h, objectness, class.
    plant = [(0, 6, 8, 4.0, 0.0, 0.0), (0, 6, 9, 2.0, 0.0, 0.0),
             (0, 18, 14, 3.0, 0.5, -0.5), (2, 2, 3, 3.0, 0.0, 0.0)]
    for lvl, row, col, obj, dx, dw in plant:
        sc, zp = qp[lvl]
        a = arrs[lvl]
        a[0, row, col] = q(dx, sc, zp)
        a[1, row, col] = q(dx, sc, zp)
        a[2, row, col] = q(dw, sc, zp)
        a[3, row, col] = q(dw, sc, zp)
        a[4, row, col] = q(obj, sc, zp)

    tensors = [("out%d" % i, a[None], qp[i][0], qp[i][1], 1) for i, a in enumerate(arrs)]
    levels = [(a, qp[i][0], qp[i][1]) for i, a in enumerate(arrs)]
    name = "yolox_planted"
    res = T.run(T.write_manifest(name, tensors, "yolox", 0.5, 0.5, input_wh=(192, 192)))
    want = R.decode_yolox(levels, 1, 0.5, 0.5)
    T.check(name + ": status", res["status"] == 0, str(res["status"]))
    T.check(name + ": planted boxes are found", len(want) >= 3, "%d boxes" % len(want))
    ok, msg = T.same_dets(res["det"], want)
    T.check(name + ": boxes (%d)" % len(want), ok, msg)
    T.check_boxes_meta(name, res, want, labels=["class 0"])
    # Centre of cell (6, 8) of the 24 grid, half the picture wide.
    b = sorted(want, key=lambda d: -d[4])[0]
    T.check(name + ": box lands where it was planted",
            abs(b[0] - (8 + 0.5) / 24) < 0.02 and abs(b[1] - (6 + 0.5) / 24) < 0.02 and
            abs(b[2] - 0.5) < 0.02, str(b))


def case_yolov2_planted():
    if not T.have("yolov2t_192_qdq_int8.onnx"):
        return
    outs, _ = T.onnx_run("yolov2t_192_qdq_int8.onnx")
    v = list(outs.values())[0]
    sc, zp = 0.10411, 24
    a = T.quantise(v, sc, zp)[0]        # [30, 6, 6] = 5 anchors x (5 + 1 class)
    for row, col, anch, obj in ((2, 3, 1, 3.0), (4, 1, 3, 2.5)):
        base = anch * 6
        a[base + 0, row, col] = q(0.0, sc, zp)
        a[base + 1, row, col] = q(0.0, sc, zp)
        a[base + 2, row, col] = q(1.0, sc, zp)
        a[base + 3, row, col] = q(1.0, sc, zp)
        a[base + 4, row, col] = q(obj, sc, zp)
    name = "yolov2_planted"
    res = T.run(T.write_manifest(name, [("out", a[None], sc, zp, 1)], "yolov2", 0.5, 0.5,
                                 input_wh=(192, 192)))
    want = R.decode_yolov2(a, sc, zp, 0.5, 0.5)
    T.check(name + ": status", res["status"] == 0, str(res["status"]))
    T.check(name + ": planted boxes are found", len(want) == 2, "%d boxes" % len(want))
    ok, msg = T.same_dets(res["det"], want)
    T.check(name + ": boxes (%d)" % len(want), ok, msg)
    if want:
        b = sorted(want, key=lambda d: -d[4])[0]
        # anchor 1 is 0.163031 x 0.413531 grid cells, times e^1, over a 6 grid
        T.check(name + ": anchor size applied",
                abs(b[2] - 0.163031 * np.e / 6) < 0.01 and abs(b[3] - 0.413531 * np.e / 6) < 0.01,
                str(b))


def case_ssd_planted():
    if not T.have("ssd_mobilenetv2_pt_coco_300_qdq_int8.onnx"):
        return
    outs, _ = T.onnx_run("ssd_mobilenetv2_pt_coco_300_qdq_int8.onnx")
    scores = [v for v in outs.values() if v.shape[-1] > 4][0]
    boxes = [v for v in outs.values() if v.shape[-1] == 4][0]
    s_sc, s_zp = 0.14933, -55
    b_sc, b_zp = 0.09028, 72
    qs = T.quantise(scores, s_sc, s_zp)[0]
    qb = T.quantise(boxes, b_sc, b_zp)[0]
    for prior, cls, logit in ((1000, 1, 6.0), (1001, 1, 5.0), (2500, 17, 4.0)):
        qs[prior, :] = q(-4.0, s_sc, s_zp)
        qs[prior, cls] = q(logit, s_sc, s_zp)
        qb[prior, :] = q(0.0, b_sc, b_zp)
    name = "ssd_planted"
    res = T.run(T.write_manifest(name, [("scores", qs[None], s_sc, s_zp, 2),
                                        ("boxes", qb[None], b_sc, b_zp, 2)],
                                 "ssd", 0.5, 0.5, input_wh=(300, 300)))
    want = R.decode_ssd(R.deq(qs, s_sc, s_zp), R.deq(qb, b_sc, b_zp), 0.5, 0.5, 300.0)
    T.check(name + ": status", res["status"] == 0, str(res["status"]))
    T.check(name + ": planted boxes are found", len(want) >= 2, "%d boxes" % len(want))
    ok, msg = T.same_dets(res["det"], want)
    T.check(name + ": boxes (%d)" % len(want), ok, msg)
    T.check_boxes_meta(name, res, want, labels=T.COCO)
    pri = R.ssd_priors(300.0)
    b = [d for d in want if d[5] == 0]
    T.check(name + ": box sits on its prior",
            bool(b) and abs(b[0][0] - pri[1000][0]) < 1e-3 and abs(b[0][2] - pri[1000][2]) < 1e-3,
            str(b[:1]) + " prior " + str(pri[1000]))


def case_deeplab_planted():
    if not T.have("st_deeplabv3_mnv2_a050_s16_asppv2_256_qdq_int8.onnx"):
        return
    outs, _ = T.onnx_run("st_deeplabv3_mnv2_a050_s16_asppv2_256_qdq_int8.onnx")
    v = list(outs.values())[0]
    sc, zp = 0.23244, 2
    a = T.quantise(v, sc, zp)[0]        # [2, 256, 256]
    a[0, 40:200, 60:180] = q(-8.0, sc, zp)
    a[1, 40:200, 60:180] = q(8.0, sc, zp)
    name = "deeplab_planted"
    res = T.run(T.write_manifest(name, [("out", a[None], sc, zp, 1)], "deeplab", 0.0, 0.0,
                                 input_wh=(256, 256)))
    lab, want = R.decode_seg(a, chw=True)
    T.check(name + ": status", res["status"] == 0, str(res["status"]))
    T.check(name + ": the planted region is reported", len(want) == 1, "%d regions" % len(want))
    ok, msg = T.same_dets(res["det"], want)
    T.check(name + ": regions (%d)" % len(want), ok, msg)
    if want:
        d = want[0]
        T.check(name + ": region covers the planted rectangle",
                abs(d[2] - 120 / 256) < 0.02 and abs(d[3] - 160 / 256) < 0.02 and
                abs(d[4] - (120 * 160) / (256 * 256)) < 0.02, str(d))
    recs = T.tlvs(res["meta"])
    masks = [x for t, x in recs if t == 11]
    T.check(name + ": mask published", len(masks) == 1)
    if masks:
        m = masks[0]
        w, h = m[1], m[2]
        bits = m[5:]
        set_bits = sum(bin(byte).count("1") for byte in bits)
        T.check(name + ": mask is solid", set_bits > 0.9 * w * h,
                "%d of %d bits" % (set_bits, w * h))


def case_classifier_planted():
    """The real classifier, with one class raised so a name is published."""
    if not T.have("mobilenetv2_a035_128_qdq_int8.onnx"):
        return
    outs, _ = T.onnx_run("mobilenetv2_a035_128_qdq_int8.onnx")
    v = np.asarray(list(outs.values())[0], dtype=np.float32).reshape(-1).copy()
    v[:] = v / max(v.sum(), 1e-6)
    v[17] = 0.62
    v[3] = 0.21
    name = "classifier_planted"
    res = T.run(T.write_manifest(name, [("out", v[None], 1.0, 0, 0)], "classifier", 0.1, 0.0, 5,
                                 input_wh=(128, 128)))
    want = R.decode_classifier(v, 0.1, 5, probs=True)
    rec = [x for t, x in T.tlvs(res["meta"]) if t == 0x54]
    got = T.parse_image_class(rec[0]) if rec else {"count": -1, "entries": [], "names": []}
    T.check(name + ": every class above the threshold is reported",
            got["count"] == len(want), "%d, expected %d" % (got["count"], len(want)))
    T.check(name + ": entries match", all(e[0] == w[0] and abs(e[1] / 1000.0 - w[1]) <= 0.002
                                          for e, w in zip(got["entries"], want)),
            "%s vs %s" % (got["entries"], want))
    T.check(name + ": names are the label list's",
            bool(want) and got["names"][:1] == [T.COCO[want[0][0]]], str(got["names"][:1]))
    T.check(name + ": top is the class that was raised",
            bool(want) and res["top"][0] == want[0][0], str(res["top"]))

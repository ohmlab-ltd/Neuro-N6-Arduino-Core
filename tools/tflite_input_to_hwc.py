# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT
"""Move a TFLite model's input past a leading NCHW to NHWC transpose, so the
DCMIPP can DMA interleaved RGB888 straight into it.

    python tools/tflite_input_to_hwc.py centerface.tflite centerface-hwc.tflite 133 --drop-ops

The tensor index is the output of the leading TRANSPOSE, the first tensor
whose shape reads [1,H,W,3]. --drop-ops removes the two leading operators.
"""

import sys

USAGE = ("usage: tflite_input_to_hwc.py <in.tflite> <out.tflite> "
         "<tensor index> [--drop-ops]")

if len(sys.argv) < 4 or "-h" in sys.argv or "--help" in sys.argv:
    # Before the tflite import, so the usage still prints without that package.
    sys.exit(USAGE + "\n\n" + __doc__.strip())

from tflite.Model import Model                     # noqa: E402

SRC, DST, NEW = sys.argv[1], sys.argv[2], int(sys.argv[3])
DROP = "--drop-ops" in sys.argv

raw = open(SRC, "rb").read()
buf = bytearray(raw)

def wr_i32(pos, val):
    buf[pos:pos + 4] = int(val).to_bytes(4, "little", signed=True)

m = Model.GetRootAsModel(raw, 0)
sg = m.Subgraphs(0)

# subgraph.inputs[0], vtable offset 6
vec = sg._tab.Vector(sg._tab.Offset(6))
print(f"subgraph.inputs[0] @{vec}: {int.from_bytes(buf[vec:vec+4],'little',signed=True)} -> {NEW}")
wr_i32(vec, NEW)

# Empties the SignatureDef vector, Model vtable offset 18. ST Edge AI reads it
# in preference to subgraph.inputs, and its tensor_index of 0 is a default
# that flatbuffers omits, so there is no byte to patch.
if m.SignatureDefsLength() > 0:
    off = m._tab.Offset(18)
    svec = m._tab.Vector(off)
    print(f"signature_defs: {m.SignatureDefsLength()} -> 0 (length prefix @{svec-4})")
    wr_i32(svec - 4, 0)

# Drops the two leading operators.
if DROP:
    ovec = sg._tab.Vector(sg._tab.Offset(10))     # operators, vtable offset 10
    n = sg.OperatorsLength()
    # Each offset is relative to its own slot.
    addrs = []
    for i in range(n):
        slot = ovec + i * 4
        rel = int.from_bytes(buf[slot:slot + 4], "little", signed=False)
        addrs.append(slot + rel)
    keep = addrs[2:]
    for i, a in enumerate(keep):
        slot = ovec + i * 4
        buf[slot:slot + 4] = (a - slot).to_bytes(4, "little", signed=False)
    wr_i32(ovec - 4, len(keep))                   # vector length prefix
    print(f"operators: {n} -> {len(keep)} (dropped the leading QUANTIZE + TRANSPOSE)")

open(DST, "wb").write(bytes(buf))
assert len(buf) == len(raw), "file size changed"

# Re-parses the output.
m2 = Model.GetRootAsModel(open(DST, "rb").read(), 0)
s2 = m2.Subgraphs(0)
ins = [s2.Inputs(i) for i in range(s2.InputsLength())]
t = s2.Tensors(ins[0])
q = t.Quantization()
print("\nverified:")
print(f"  subgraph inputs   : {ins}")
print(f"  signature_defs    : {m2.SignatureDefsLength()}")
print(f"  tensor {ins[0]} shape={[t.Shape(i) for i in range(t.ShapeLength())]} "
      f"type={t.Type()} scale={q.Scale(0)} zp={q.ZeroPoint(0)}")
print(f"  operators         : {s2.OperatorsLength()}")
op0 = s2.Operators(0)
print(f"  op0 in={[op0.Inputs(i) for i in range(op0.InputsLength())]} "
      f"out={[op0.Outputs(i) for i in range(op0.OutputsLength())]}")

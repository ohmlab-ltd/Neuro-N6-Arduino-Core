# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT
"""Draw one case's overlay to a PNG, to look at what the board would draw.

    python overlay_png.py <case> [out.png]

<case> is a case name from run_tests.py. The overlay is 800x480 ARGB4444 on
layer 1 of the panel; this composes it over a grey stand-in for the camera
picture, as the panel does.
"""
import os
import subprocess
import sys

import numpy as np
from PIL import Image

import run_tests as T

W, H = 800, 480


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    name = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(T.WORK, name + ".png")

    T.COCO = [line.strip() for line in open(os.path.join(T.WORK, "coco.txt"))]
    T.add_planted()
    T.OVERLAY = os.path.join(T.WORK, name + ".ovl")
    # Running the case writes its manifest.
    T.CASES_ALL[name]()
    manifest = os.path.join(T.CASES, name + ".txt")
    if not os.path.isfile(manifest):
        # A case may build several manifests; take the newest.
        newest = max((f for f in os.listdir(T.CASES) if f.endswith(".txt")),
                     key=lambda f: os.path.getmtime(os.path.join(T.CASES, f)))
        manifest = os.path.join(T.CASES, newest)
    subprocess.run([T.EXE, manifest, T.OVERLAY], capture_output=True, text=True, check=True)

    raw = np.fromfile(T.OVERLAY, dtype="<u2").reshape(H, W)
    a = ((raw >> 12) & 0xF) / 15.0
    r = ((raw >> 8) & 0xF) * 17
    g = ((raw >> 4) & 0xF) * 17
    b = (raw & 0xF) * 17
    base = np.full((H, W, 3), 70, dtype=np.float32)          # the camera picture
    base[:, :80] = 20                                        # letterbox, VGA on WVGA
    base[:, 720:] = 20
    rgb = base * (1 - a[..., None]) + np.stack([r, g, b], axis=-1) * a[..., None]
    Image.fromarray(rgb.astype(np.uint8)).save(out)
    print("wrote " + out)
    return 0


if __name__ == "__main__":
    sys.exit(main())

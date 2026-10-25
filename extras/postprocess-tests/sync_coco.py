# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT
"""Copies PostProcess's COCO-80 names into the host test driver and coco.txt."""
import os
import re
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "..", "..", "libraries", "PostProcess", "src",
                   "classes_table_default.c")

body = open(SRC, encoding="utf-8").read()
names = re.findall(r'"([^"]*)"', body.split("{", 1)[1])
assert len(names) == 80, len(names)

WORK = os.environ.get("N6_PP_WORK",
                      os.path.join(tempfile.gettempdir(), "n6-postprocess-tests"))
os.makedirs(WORK, exist_ok=True)
with open(os.path.join(WORK, "coco.txt"), "w", encoding="utf-8", newline="\n") as f:
    f.write("\n".join(names) + "\n")

drv = os.path.join(HERE, "test_zoo.c")
text = open(drv, encoding="utf-8").read()
lines = []
for i in range(0, 80, 4):
    lines.append("  " + ", ".join('"%s"' % n for n in names[i:i + 4]) + ("," if i + 4 < 80 else ""))
table = "const char *classes_table[80] = {\n" + "\n".join(lines) + "\n};"
text = re.sub(r"const char \*classes_table\[80\] = \{.*?\n\};", table, text, count=1, flags=re.S)
open(drv, "w", encoding="utf-8", newline="\n").write(text)
print("synced %d names" % len(names))

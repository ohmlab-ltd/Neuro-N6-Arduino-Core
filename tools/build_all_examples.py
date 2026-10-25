#!/usr/bin/env python3
# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT
"""Compile every example and record a fingerprint of each binary.

    python tools/build_all_examples.py --out baseline.json
    python tools/build_all_examples.py --out after.json
    python tools/build_all_examples.py --compare baseline.json after.json
"""
import argparse
import concurrent.futures
import glob
import hashlib
import json
import os
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FQBN = "Ohmlab:stm32n6:neuron6"


def _find_cli():
    """ARDUINO_CLI, else the IDE's bundled copy, else arduino-cli on PATH."""
    env = os.environ.get("ARDUINO_CLI")
    if env:
        return env
    bundled = os.path.join(os.environ.get("LOCALAPPDATA", ""), "Programs", "Arduino IDE",
                           "resources", "app", "lib", "backend", "resources", "arduino-cli.exe")
    return bundled if os.path.isfile(bundled) else "arduino-cli"


CLI = _find_cli()


def _scan(root):
    out = []
    if not os.path.isdir(root):
        return out
    for dirpath, _dirnames, filenames in os.walk(root):
        for fn in filenames:
            if fn.endswith(".ino") and fn[:-4] == os.path.basename(dirpath):
                out.append(os.path.relpath(dirpath, ROOT).replace("\\", "/"))
    return out


def find_examples(include_bench=True):
    """Library examples, plus the bench sketches under extras/bench."""
    out = []
    for lib in sorted(os.listdir(os.path.join(ROOT, "libraries"))):
        out += _scan(os.path.join(ROOT, "libraries", lib, "examples"))
    if include_bench:
        out += _scan(os.path.join(ROOT, "extras", "bench"))
    return sorted(out)


def build_one(sketch, workdir):
    # Short name: the sketch path can pass the Windows 260-character limit.
    name = "%s_%s" % (os.path.basename(sketch)[:24],
                      hashlib.sha1(sketch.encode()).hexdigest()[:8])
    outdir = os.path.join(workdir, name)
    t0 = time.time()
    proc = subprocess.run(
        [CLI, "compile", "--fqbn", FQBN, "--output-dir", outdir, os.path.join(ROOT, sketch)],
        capture_output=True, text=True, cwd=ROOT,
    )
    rec = {"sketch": sketch, "ok": proc.returncode == 0, "seconds": round(time.time() - t0, 1)}
    if proc.returncode != 0:
        tail = (proc.stderr or proc.stdout or "").strip().split("\n")
        rec["error"] = "\n".join(tail[-6:])[:900]
        return rec
    def read_when_ready(paths, tries=30):
        """Read the first readable file. The signing step can hold it open."""
        last = None
        for _ in range(tries):
            for q in paths:
                try:
                    return open(q, "rb").read(), q, None
                except OSError as exc:
                    last = exc
            time.sleep(1.0)
        return None, None, last

    # *_Trusted.bin is not hashed: ST's signing tool pads it with random bytes.
    # On Windows the files can appear after the compiler returns.
    cands = elfs = []
    for _attempt in range(30):
        cands = (sorted(glob.glob(os.path.join(outdir, "*.ino.bin")))
                 or [q for q in sorted(glob.glob(os.path.join(outdir, "*.bin")))
                     if not q.endswith("_Trusted.bin")])
        elfs = sorted(glob.glob(os.path.join(outdir, "*.elf")))
        if cands:
            break
        time.sleep(1.0)

    data, chosen, why = read_when_ready(cands) if cands else (None, None, None)
    if data is not None:
        rec["bin"] = os.path.basename(chosen)
        rec["size"] = len(data)
        rec["sha256"] = hashlib.sha256(data).hexdigest()
        edata, _e, _w = read_when_ready(elfs) if elfs else (None, None, None)
        if edata is not None:
            rec["elf_sha256"] = hashlib.sha256(edata).hexdigest()
    else:
        rec["ok"] = False
        if not cands:
            rec["error"] = "compiled, but no binary appeared in %s" % outdir
        else:
            rec["error"] = "compiled, but %s stayed unreadable: %s" % (
                os.path.basename(cands[0]), why)
    return rec


def uses_model(sketch):
    ino = os.path.join(ROOT, sketch, os.path.basename(sketch) + ".ino")
    with open(ino, "rb") as fh:
        return b"#pragma neuron6" in fh.read()


def cmd_build(args):
    sketches = find_examples()
    if args.filter:
        sketches = [s for s in sketches if args.filter.lower() in s.lower()]
    if args.no_models:
        sketches = [s for s in sketches if not uses_model(s)]
    workdir = args.workdir or os.path.join(
        os.environ.get("TEMP", "/tmp"), "n6-buildall-%d" % os.getpid())
    os.makedirs(workdir, exist_ok=True)
    print("building %d examples into %s with %d workers" % (len(sketches), workdir, args.jobs))
    results = []
    t0 = time.time()
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as pool:
        futs = {pool.submit(build_one, s, workdir): s for s in sketches}
        for i, fut in enumerate(concurrent.futures.as_completed(futs), 1):
            r = fut.result()
            results.append(r)
            flag = "ok  " if r["ok"] else "FAIL"
            print("  [%3d/%3d] %s %-58s %s" % (
                i, len(sketches), flag, r["sketch"],
                r.get("sha256", "")[:16] if r["ok"] else r.get("error", "").split("\n")[-1][:70]))
            sys.stdout.flush()
    results.sort(key=lambda r: r["sketch"])
    ok = sum(1 for r in results if r["ok"])
    report = {"fqbn": FQBN, "total": len(results), "ok": ok,
              "failed": len(results) - ok,
              "elapsed_s": round(time.time() - t0, 1), "results": results}
    with open(args.out, "w") as fh:
        json.dump(report, fh, indent=1)
    print("\n%d/%d compiled, %d failed, %.0fs -> %s" % (
        ok, len(results), len(results) - ok, report["elapsed_s"], args.out))
    return 0 if ok == len(results) else 1


def cmd_compare(args):
    a = json.load(open(args.compare[0]))
    b = json.load(open(args.compare[1]))
    am = {r["sketch"]: r for r in a["results"]}
    bm = {r["sketch"]: r for r in b["results"]}
    gone = sorted(set(am) - set(bm))
    new = sorted(set(bm) - set(am))
    broke, fixed, changed, same = [], [], [], 0
    for s in sorted(set(am) & set(bm)):
        x, y = am[s], bm[s]
        if x["ok"] and not y["ok"]:
            broke.append((s, y.get("error", "").split("\n")[-1][:90]))
        elif not x["ok"] and y["ok"]:
            fixed.append(s)
        elif x["ok"] and y["ok"]:
            if (x.get("sha256"), x.get("elf_sha256")) != (y.get("sha256"), y.get("elf_sha256")):
                changed.append((s, x.get("size"), y.get("size")))
            else:
                same += 1
    print("baseline %s: %d ok / %d" % (args.compare[0], a["ok"], a["total"]))
    print("after    %s: %d ok / %d" % (args.compare[1], b["ok"], b["total"]))
    print("\nidentical binaries : %d" % same)
    print("changed binaries   : %d" % len(changed))
    print("newly broken       : %d" % len(broke))
    print("newly fixed        : %d" % len(fixed))
    print("removed sketches   : %d" % len(gone))
    print("added sketches     : %d" % len(new))
    for s, e in broke:
        print("  BROKE   %s\n          %s" % (s, e))
    for s, sa, sb in changed:
        print("  CHANGED %-58s %s -> %s bytes" % (s, sa, sb))
    for s in fixed:
        print("  FIXED   %s" % s)
    for s in gone:
        print("  REMOVED %s" % s)
    for s in new:
        print("  ADDED   %s" % s)
    return 1 if broke else 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="build-report.json")
    ap.add_argument("--jobs", type=int, default=1,
                    help="keep at 1: the platform shares one core.a cache, "
                         "so parallel builds drop model symbols")
    ap.add_argument("--filter")
    ap.add_argument("--no-models", action="store_true",
                    help="skip sketches that embed a neural network (they need ST Edge AI Core)")
    ap.add_argument("--workdir")
    ap.add_argument("--compare", nargs=2, metavar=("BASELINE", "AFTER"))
    args = ap.parse_args()
    return cmd_compare(args) if args.compare else cmd_build(args)


if __name__ == "__main__":
    sys.exit(main())

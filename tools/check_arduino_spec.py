#!/usr/bin/env python3
# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT
"""Check this platform and its bundled libraries against the Arduino specs.

    python tools/check_arduino_spec.py
"""
import io
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

REQUIRED = ["name", "version", "author", "maintainer", "sentence",
            "paragraph", "category", "url", "architectures"]
OPTIONAL = ["depends", "dot_a_linkage", "includes", "precompiled", "ldflags"]
CATEGORIES = {"Display", "Communication", "Signal Input/Output", "Sensors",
              "Device Control", "Timing", "Data Storage", "Data Processing",
              "Other", "Uncategorized"}
SEMVER = re.compile(r"^\d+\.\d+(\.\d+)?$")
LIBNAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9 ._-]*$")
FOLDER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,62}$")
BOARDID = re.compile(r"^[A-Za-z0-9_-]+$")

errors = []
warnings = []


def err(where, msg):
    errors.append("%s: %s" % (where, msg))


def warn(where, msg):
    warnings.append("%s: %s" % (where, msg))


def read_props(path):
    d = {}
    for line in io.open(path, encoding="utf-8", errors="replace"):
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        k, v = line.split("=", 1)
        d[k.strip()] = v.strip()
    return d


def check_library(libdir):
    lib = os.path.basename(libdir)
    where = "libraries/%s" % lib
    prop = os.path.join(libdir, "library.properties")
    if not os.path.isfile(prop):
        err(where, "no library.properties")
        return
    p = read_props(prop)

    for f in REQUIRED:
        if f not in p:
            err(where, "library.properties missing required field '%s'" % f)
        elif not p[f]:
            err(where, "library.properties field '%s' is empty" % f)

    if "name" in p:
        if not LIBNAME.match(p["name"]):
            err(where, "name '%s' uses characters the spec disallows" % p["name"])
        if p["name"].lower().startswith("arduino") and lib != "ArduinoBLE":
            warn(where, "name starts with 'Arduino', reserved for official libraries")
    if "version" in p and not SEMVER.match(p["version"]):
        err(where, "version '%s' is not semver" % p["version"])
    if "category" in p and p["category"] not in CATEGORIES:
        err(where, "category '%s' is not one of the allowed values" % p["category"])
    if "maintainer" in p and "<" not in p["maintainer"]:
        warn(where, "maintainer has no contact address")

    for k in p:
        if k not in REQUIRED and k not in OPTIONAL:
            warn(where, "unknown library.properties field '%s'" % k)

    if not FOLDER.match(lib):
        err(where, "folder name is not spec-legal")

    if not os.path.isdir(os.path.join(libdir, "src")):
        has_src = any(f.endswith((".c", ".cpp", ".h")) for f in os.listdir(libdir))
        if has_src:
            warn(where, "legacy 1.0 layout: sources at the library root, no src/")

    exdir = os.path.join(libdir, "examples")
    if os.path.isdir(exdir):
        check_sketches(exdir, where)
    # os.path.isdir is case-insensitive on Windows, so the entries are listed.
    for entry in os.listdir(libdir):
        if entry.lower() == "examples" and entry != "examples":
            err(where, "examples folder must be lowercase, found '%s'" % entry)

    kw = os.path.join(libdir, "keywords.txt")
    if os.path.isfile(kw):
        for n, line in enumerate(io.open(kw, encoding="utf-8", errors="replace"), 1):
            if not line.strip() or line.startswith("#"):
                continue
            if "\t" not in line:
                err(where, "keywords.txt:%d fields must be separated by a TAB" % n)


def check_sketches(root, where):
    for dirpath, _dirnames, filenames in os.walk(root):
        inos = [f for f in filenames if f.endswith(".ino")]
        if not inos:
            continue
        folder = os.path.basename(dirpath)
        if not any(f[:-4] == folder for f in inos):
            err(where, "sketch folder '%s' has no .ino matching its name (%s)"
                % (folder, ", ".join(inos)))
        if not FOLDER.match(folder):
            err(where, "sketch folder '%s' is not spec-legal" % folder)


def check_platform():
    for f in ("boards.txt", "platform.txt"):
        if not os.path.isfile(os.path.join(ROOT, f)):
            err("platform", "missing %s" % f)
    p = read_props(os.path.join(ROOT, "platform.txt"))
    if "name" not in p:
        err("platform.txt", "missing name")
    if "version" not in p:
        err("platform.txt", "missing version")
    elif not SEMVER.match(p["version"]):
        err("platform.txt", "version '%s' is not semver" % p["version"])
    else:
        vh = os.path.join(ROOT, "cores", "NeuroN6", "NeuroN6_Version.h")
        txt = open(vh, encoding="utf-8").read() if os.path.isfile(vh) else ""
        parts = [re.search(r"#define NEURON6_CORE_VERSION_%s\s+(\d+)" % k, txt) for k in ("MAJOR", "MINOR", "PATCH")]
        s = re.search(r'#define NEURON6_CORE_VERSION_STRING\s+"([^"]*)"', txt)
        got = ".".join(m.group(1) for m in parts) if all(parts) else None
        if got != p["version"].split("-")[0] or (s is None or s.group(1) != p["version"]):
            err("NeuroN6_Version.h", "does not match platform.txt version %s" % p["version"])

    b = read_props(os.path.join(ROOT, "boards.txt"))
    ids = set()
    for k in b:
        if k.startswith("menu."):
            continue
        ids.add(k.split(".", 1)[0])
    for i in sorted(ids):
        if not BOARDID.match(i):
            err("boards.txt", "board id '%s' uses disallowed characters" % i)
    for i in sorted(ids):
        for need in ("build.core", "build.board"):
            if "%s.%s" % (i, need) not in b:
                err("boards.txt", "board '%s' has no %s" % (i, need))

    arch = os.path.basename(ROOT)
    if not re.match(r"^[A-Za-z0-9_.-]+$", arch):
        err("platform", "architecture folder '%s' has disallowed characters" % arch)
    return arch


def check_repo():
    for dirpath, dirnames, filenames in os.walk(ROOT):
        dirnames[:] = [d for d in dirnames if d not in (".git", "__pycache__")]
        for f in filenames:
            full = os.path.join(dirpath, f)
            rel = os.path.relpath(full, ROOT).replace("\\", "/")
            if f == ".development":
                err(rel, "a .development file must never be committed")
            if f.endswith(".exe"):
                err(rel, ".exe files are rejected by the Library Manager registry")
            if os.path.islink(full):
                err(rel, "symlinks are rejected by the Library Manager registry")


def main():
    arch = check_platform()
    check_repo()
    libs = os.path.join(ROOT, "libraries")
    nlib = 0
    for lib in sorted(os.listdir(libs)):
        d = os.path.join(libs, lib)
        if os.path.isdir(d):
            check_library(d)
            nlib += 1

    for lib in sorted(os.listdir(libs)):
        prop = os.path.join(libs, lib, "library.properties")
        if os.path.isfile(prop):
            p = read_props(prop)
            if p.get("architectures") == "*":
                warn("libraries/%s" % lib,
                     "architectures='*'; a bundled library should pin '%s'" % arch)

    print("checked platform '%s' and %d bundled libraries" % (arch, nlib))
    for w in warnings:
        print("  WARN  %s" % w)
    for e in errors:
        print("  ERROR %s" % e)
    print("\n%d errors, %d warnings" % (len(errors), len(warnings)))
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())

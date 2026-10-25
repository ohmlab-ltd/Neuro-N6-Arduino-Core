#!/usr/bin/env python3
# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT
"""Build the Boards Manager package: the platform archive and its index.

    python tools/packaging/build_package.py --out dist
        dist/stm32n6-<version>.zip        platform archive, one root folder
        dist/package_ohmlab_index.json    index pointing at the GitHub release

    python tools/packaging/build_package.py --out dist --base-url http://127.0.0.1:8765/ --tools my_tools.json
        index pointing at a local server
"""
import argparse
import hashlib
import json
import os
import subprocess
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
PACKAGER = "Ohmlab"
ARCH = "stm32n6"
RELEASE_URL = "https://github.com/ohmlab-ltd/Neuro-N6-Arduino-Core/releases/download/v{version}/"
TOOLS_URL = "https://github.com/ohmlab-ltd/Neuro-N6-Arduino-Core/releases/download/tools-20260901/"
HELP_URL = "https://github.com/ohmlab-ltd/Neuro-N6-Arduino-Core"
FIXED_DATE = (2026, 1, 1, 0, 0, 0)
NUL = chr(0)


def platform_version():
    for line in open(os.path.join(ROOT, "platform.txt")):
        if line.startswith("version="):
            return line.split("=", 1)[1].strip()
    raise SystemExit("platform.txt has no version= line")


def package_files():
    """Tracked files, except those marked export-ignore."""
    listing = subprocess.run(["git", "ls-files", "-z", "--cached"],
                             cwd=ROOT, capture_output=True, check=True).stdout
    names = [n for n in listing.decode("utf-8").split(NUL) if n]
    attrs = subprocess.run(["git", "check-attr", "-z", "--stdin", "export-ignore"],
                           cwd=ROOT, input=NUL.join(names).encode("utf-8"),
                           capture_output=True, check=True).stdout.decode("utf-8").split(NUL)
    ignored = {attrs[i] for i in range(0, len(attrs) - 2, 3) if attrs[i + 2] == "set"}
    return [n for n in names if n not in ignored and os.path.isfile(os.path.join(ROOT, n))]


def build_archive(out_dir, version):
    name = "%s-%s.zip" % (ARCH, version)
    path = os.path.join(out_dir, name)
    prefix = "%s-%s/" % (ARCH, version)
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as zf:
        for rel in sorted(package_files()):
            info = zipfile.ZipInfo(prefix + rel, date_time=FIXED_DATE)
            info.compress_type = zipfile.ZIP_DEFLATED
            info.create_system = 3          # Unix, so extractors honour the mode bits
            info.external_attr = (0o755 if rel.endswith(".sh") else 0o644) << 16
            with open(os.path.join(ROOT, rel), "rb") as fh:
                zf.writestr(info, fh.read())
    return path


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def apply_rehosted(tools, rehosted_path, rehost_base):
    """Point python3 systems at the repacked archives listed in rehosted.json."""
    if not os.path.isfile(rehosted_path):
        return tools
    with open(rehosted_path) as fh:
        rehosted = json.load(fh)
    for t in tools:
        if t["name"] != "python3":
            continue
        for s in t["systems"]:
            r = rehosted.get(s["archiveFileName"])
            if r:
                s.update({"url": rehost_base + r["file"], "archiveFileName": r["file"],
                          "size": r["size"], "checksum": r["checksum"]})
    return tools


def build_index(archive, version, base_url, tools):
    platform = {
        "name": "Neuro N6",
        "architecture": ARCH,
        "version": version,
        "category": "Contributed",
        "help": {"online": HELP_URL},
        "url": base_url + os.path.basename(archive),
        "archiveFileName": os.path.basename(archive),
        "checksum": "SHA-256:" + sha256(archive),
        "size": str(os.path.getsize(archive)),
        "boards": [{"name": "Neuro N6"}],
        "toolsDependencies": [{"packager": PACKAGER, "name": t["name"], "version": t["version"]} for t in tools],
    }
    return {"packages": [{
        "name": PACKAGER,
        "maintainer": "Ohm Lab",
        "websiteURL": "https://neuron6.com",
        "email": "team@ohmlab.co.uk",
        "help": {"online": HELP_URL},
        "platforms": [platform],
        "tools": tools,
    }]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(ROOT, "dist"))
    ap.add_argument("--base-url", default=None, help="where the archive will be served; default: the GitHub release")
    ap.add_argument("--tools", default=os.path.join(HERE, "tools.json"))
    ap.add_argument("--rehosted", default=os.path.join(HERE, "rehosted.json"),
                    help="repacked Python archives from repack_python.sh; ignored when absent")
    ap.add_argument("--rehost-base", default=TOOLS_URL,
                    help="where the repacked Python archives are served; default: the tools release")
    args = ap.parse_args()
    version = platform_version()
    os.makedirs(args.out, exist_ok=True)
    archive = build_archive(args.out, version)
    base_url = args.base_url or RELEASE_URL.format(version=version)
    with open(args.tools) as fh:
        tools = json.load(fh)
    tools = apply_rehosted(tools, args.rehosted, args.rehost_base)
    index = build_index(archive, version, base_url, tools)
    index_path = os.path.join(args.out, "package_%s_index.json" % PACKAGER.lower())
    with open(index_path, "w", newline="\n") as fh:
        json.dump(index, fh, indent=2)
        fh.write("\n")
    print("archive  %s  %d bytes" % (archive, os.path.getsize(archive)))
    print("index    %s" % index_path)
    print("platform %s %s, url %s" % (ARCH, version, index["packages"][0]["platforms"][0]["url"]))


if __name__ == "__main__":
    main()

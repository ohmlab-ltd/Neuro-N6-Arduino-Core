#!/usr/bin/env python3
# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT
"""Refresh tools.json from the upstream release manifests.

    python tools/packaging/update_tools.py
"""
import json
import os
import re
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
HEADERS = {"User-Agent": "neuron6-packaging"}

GCC_REPO = "xpack-dev-tools/arm-none-eabi-gcc-xpack"
GCC_TAG = "v13.3.1-1.1"
GCC_VERSION = "13.3.1-1.1"
GCC_HOSTS = [
    ("x86_64-mingw32", "win32-x64.zip"),
    ("i686-mingw32", "win32-x64.zip"),
    ("x86_64-apple-darwin", "darwin-x64.tar.gz"),
    ("arm64-apple-darwin", "darwin-arm64.tar.gz"),
    ("x86_64-pc-linux-gnu", "linux-x64.tar.gz"),
    ("aarch64-linux-gnu", "linux-arm64.tar.gz"),
    ("arm-linux-gnueabihf", "linux-arm.tar.gz"),
]

PY_REPO = "astral-sh/python-build-standalone"
PY_TAG = "20260901"
PY_SERIES = "3.12"
PY_HOSTS = [
    ("x86_64-mingw32", "x86_64-pc-windows-msvc"),
    ("i686-mingw32", "x86_64-pc-windows-msvc"),
    ("x86_64-apple-darwin", "x86_64-apple-darwin"),
    ("arm64-apple-darwin", "aarch64-apple-darwin"),
    ("x86_64-pc-linux-gnu", "x86_64-unknown-linux-gnu"),
    ("aarch64-linux-gnu", "aarch64-unknown-linux-gnu"),
    ("arm-linux-gnueabihf", "armv7-unknown-linux-gnueabihf"),
]


def get(url):
    req = urllib.request.Request(url, headers=HEADERS)
    return urllib.request.urlopen(req, timeout=60).read()


def release(repo, tag):
    data = json.loads(get("https://api.github.com/repos/%s/releases/tags/%s" % (repo, tag)))
    return {a["name"]: a for a in data["assets"]}


def system(host, asset, sha256):
    return {"host": host, "url": asset["browser_download_url"], "archiveFileName": asset["name"],
            "checksum": "SHA-256:" + sha256, "size": str(asset["size"])}


def gcc_tool():
    assets = release(GCC_REPO, GCC_TAG)
    systems = []
    for host, suffix in GCC_HOSTS:
        name = "xpack-arm-none-eabi-gcc-%s-%s" % (GCC_VERSION, suffix)
        sha = get(assets[name + ".sha"]["browser_download_url"]).decode().split()[0]
        systems.append(system(host, assets[name], sha))
    return {"name": "xpack-arm-none-eabi-gcc", "version": GCC_VERSION, "systems": systems}


def python_tool():
    assets = release(PY_REPO, PY_TAG)
    series = PY_SERIES.replace(".", "[.]")
    pat = re.compile("^cpython-(" + series + "[.][0-9]+)[+]" + PY_TAG + "-(.+)-install_only_stripped[.]tar[.]gz$")
    by_triple, version = {}, None
    for name, a in assets.items():
        m = pat.match(name)
        if m:
            by_triple[m.group(2)] = a
            version = m.group(1)
    sums = None
    systems = []
    for host, triple in PY_HOSTS:
        a = by_triple[triple]
        digest = a.get("digest")
        if digest:
            sha = digest.split(":", 1)[1]
        else:
            if sums is None:
                text = get(assets["SHA256SUMS"]["browser_download_url"]).decode()
                sums = dict(reversed(l.split()) for l in text.splitlines() if l.strip())
            sha = sums[a["name"]]
        systems.append(system(host, a, sha))
    return {"name": "python3", "version": version, "systems": systems}


def main():
    tools = [gcc_tool(), python_tool()]
    path = os.path.join(HERE, "tools.json")
    with open(path, "w", newline="\n") as fh:
        json.dump(tools, fh, indent=2)
        fh.write("\n")
    for t in tools:
        print("%s %s: %d systems" % (t["name"], t["version"], len(t["systems"])))
    print("wrote", path)


if __name__ == "__main__":
    main()

#!/usr/bin/env bash
# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT
# Repacks the python-build-standalone archives with explicit directory
# entries, which arduino-cli needs to extract their symlinks.
#
# Run on Linux. Reads the upstream URLs and checksums from tools.json, writes
# python3-<version>-<triple>.tar.gz into the output directory and records size
# and SHA-256 in rehosted.json.
#
#   tools/packaging/repack_python.sh                # all archives into dist/tools
#   tools/packaging/repack_python.sh --only x86_64-pc-linux-gnu --out dist
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
OUT="$HERE/../../dist/tools"
ONLY=""
while [ $# -gt 0 ]; do
  case "$1" in
    --only) ONLY="$2"; shift 2 ;;
    --out) OUT="$2"; shift 2 ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
done
mkdir -p "$OUT"
OUT="$(cd "$OUT" && pwd)"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

python3 - "$HERE/tools.json" "$ONLY" > "$WORK/list" <<'PY'
import json, sys
tools = json.load(open(sys.argv[1])); only = sys.argv[2]
for t in tools:
    if t["name"] != "python3":
        continue
    seen = set()
    for s in t["systems"]:
        if only and s["host"] != only:
            continue
        if s["archiveFileName"] in seen:
            continue
        seen.add(s["archiveFileName"])
        print(t["version"], s["archiveFileName"], s["url"], s["checksum"].split(":", 1)[1])
PY

: > "$WORK/results"
while read -r version upstream url sha; do
  triple="${upstream#cpython-*-}"
  triple="${triple%-install_only_stripped.tar.gz}"
  repacked="python3-${version}-${triple}.tar.gz"
  echo "== $upstream"
  curl -fsSL -o "$WORK/up.tar.gz" "$url"
  echo "$sha  $WORK/up.tar.gz" | sha256sum -c --quiet
  rm -rf "$WORK/x"; mkdir -p "$WORK/x"
  tar -xzf "$WORK/up.tar.gz" -C "$WORK/x"
  tar --sort=name --owner=0 --group=0 --numeric-owner -C "$WORK/x" -czf "$OUT/$repacked" python
  size=$(stat -c %s "$OUT/$repacked")
  newsha=$(sha256sum "$OUT/$repacked" | cut -d' ' -f1)
  echo "   -> $repacked  $size bytes"
  printf '%s\t%s\t%s\t%s\n' "$upstream" "$repacked" "$size" "$newsha" >> "$WORK/results"
done < "$WORK/list"

python3 - "$HERE/rehosted.json" "$WORK/results" "$ONLY" <<'PY'
import json, os, sys
path, results, only = sys.argv[1], sys.argv[2], sys.argv[3]
data = json.load(open(path)) if os.path.isfile(path) else {}
for line in open(results):
    upstream, repacked, size, sha = line.rstrip("\n").split("\t")
    data[upstream] = {"file": repacked, "size": size, "checksum": "SHA-256:" + sha}
json.dump(data, open(path, "w"), indent=2, sort_keys=True)
open(path, "a").write("\n")
print("wrote %s (%d archives%s)" % (path, len(data), ", partial" if only else ""))
PY

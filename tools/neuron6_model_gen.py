#!/usr/bin/env python3
# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT
"""Sketch-model pipeline for the Neuro N6, run as a build hook.

Runs ST Edge AI Core on each model a sketch names with a pragma, writes
include shims for the generated code to {build.path}/neuron6_model/ and
writes {build.path}/neuron6_models.json for the upload.

Pragma, one line per model, up to 8 models:
    #pragma neuron6 model="<file>" [name=<c_identifier>] [address=0x9XXXXXXX]

  model    .tflite or .onnx path: absolute, or relative to the sketch folder,
           a models/ subfolder of the sketch, or <platform>/models/.
  name     C name of the network. Default: the sanitized file stem.
  address  weights base in external flash. Default: the mempool's weights
           pool base, 0x90300000, for the first model. Later models pack
           after the previous blob, 64 KB aligned.

Usage:
    neuron6_model_gen.py <sketch_dir> <build_path> <platform_path> [verbosity]
    neuron6_model_gen.py --prune-radio <build_path>
"""

import glob
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import time
import zlib

MAX_MODELS = 8                       # matches cores/NeuroN6/neuron6_user_model_[0-7].c
WEIGHTS_ALIGN = 0x10000              # 64 KB
SECTOR = 0x1000                      # XSPI NOR sector size

# IW610 firmware, a manifest entry in a wireless sketch. MAYA_FW_ADDR must
# match IW610_FW_BLOB_ADDR in iw610_cpu12_se.c.
MAYA_FW_ADDR = 0x91E00000
MAYA_FW_BIN_REL = os.path.join("libraries", "MayaW476", "firmware",
                               "sduartspi_iw610.bin.se")
MAYA_FW_LEN_SRC_REL = os.path.join(
    "libraries", "MayaW476", "src", "wifi_nxp", "wifi_bt_firmware",
    "iw610x", "iw610_cpu12_se.c")
# Sketch includes that link the wireless stack. Case is ignored, as the
# include itself ignores it on Windows and macOS.
MAYA_FW_INCLUDE_RE = re.compile(
    r'#\s*include\s*[<"](?:MayaW476|MayaBLE|ArduinoBLE'
    r'|WiFi|WiFiClient|WiFiServer|WiFiUdp)\.h[>"]', re.IGNORECASE)
# The libraries those headers belong to.
WIRELESS_LIBS = ("MayaW476", "MayaBLE", "ArduinoBLE")

# ST Neural-ART compiler options, the same for every model.
NEURAL_ART_OPTIONS = ("--native-float --cache-maintenance --Ocache-opt "
                      "--enable-virtual-mem-pools --Os --optimization 3 "
                      "--Omax-ca-pipe 4 --Oauto-sched --all-buffers-info")

# The DCMIPP writes interleaved RGB888, uint8 and channel-last, straight into
# the model's input buffer. The model's input quantisation must be asymmetric:
# scale 1/255, zero-point -128 as int8.
GENERATE_INTERFACE_OPTIONS = "--input-data-type uint8 --inputs-ch-position chlast"

MODEL_EXTS = (".tflite", ".onnx")


sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from neuron6_style import WIDTH, group, human, item, out as log


VERBOSITY = "minimal"   # "minimal" or "verbose"


def fail(msg):
    log()
    log("=" * WIDTH)
    log("  NEURO N6 MODEL PIPELINE - ERROR")
    log("=" * WIDTH)
    for line in msg.splitlines():
        log("  " + line)
    log("=" * WIDTH)
    sys.exit(1)


# Pragma parsing

def strip_comments(src):
    """Remove // and /* */ comments, keeping line numbers."""
    out = []
    i, n = 0, len(src)
    state = None  # None | 'line' | 'block' | '"' | "'"
    while i < n:
        c = src[i]
        nxt = src[i + 1] if i + 1 < n else ""
        if state is None:
            if c == "/" and nxt == "/":
                state = "line"
                i += 2
                continue
            if c == "/" and nxt == "*":
                state = "block"
                i += 2
                continue
            if c in "\"'":
                state = c
            out.append(c)
        elif state == "line":
            if c == "\n":
                state = None
                out.append(c)
        elif state == "block":
            if c == "*" and nxt == "/":
                state = None
                i += 2
                continue
            if c == "\n":
                out.append(c)
        else:  # inside a string literal
            out.append(c)
            if c == "\\" and nxt:
                out.append(nxt)
                i += 2
                continue
            if c == state:
                state = None
        i += 1
    return "".join(out)


PRAGMA_RE = re.compile(r"^\s*#\s*pragma\s+neuron6\b(.*)$", re.IGNORECASE)
# key=value, with the value bare or double-quoted.
KV_RE = re.compile(r'(\w+)\s*=\s*(?:"([^"]*)"|([^\s"]+))')
IDENT_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


def parse_pragmas(sketch_dir):
    """Return the models the sketch's pragmas name."""
    models = []
    exts = (".ino", ".pde", ".h", ".hpp", ".c", ".cpp")
    try:
        entries = sorted(os.listdir(sketch_dir))
    except OSError as e:
        fail("Cannot read sketch folder %s\n%s" % (sketch_dir, e))
    for fname in entries:
        if not fname.lower().endswith(exts):
            continue
        path = os.path.join(sketch_dir, fname)
        try:
            with open(path, "r", encoding="utf-8", errors="replace") as f:
                text = strip_comments(f.read())
        except OSError:
            continue
        for lineno, line in enumerate(text.splitlines(), 1):
            m = PRAGMA_RE.match(line)
            if not m:
                continue
            args = m.group(1).strip()
            where = "%s:%d" % (fname, lineno)
            kv = {}
            for k, vq, vb in KV_RE.findall(args):
                kv[k.lower()] = vq if vq != "" else vb
            if "model" not in kv or not kv["model"]:
                fail("Malformed pragma at %s\n"
                     "  %s\n"
                     "Expected: #pragma neuron6 model=\"file.tflite\" "
                     "[name=my_net] [address=0x90300000]" % (where, line.strip()))
            entry = {"model": kv["model"], "where": where}
            if "name" in kv:
                if not IDENT_RE.match(kv["name"]):
                    fail("Invalid model name '%s' at %s\n"
                         "The name must be a valid C identifier "
                         "(letters, digits, underscore; not starting with a digit)."
                         % (kv["name"], where))
                entry["name"] = kv["name"]
            if "address" in kv:
                try:
                    entry["address"] = int(kv["address"], 0)
                except ValueError:
                    fail("Invalid address '%s' at %s (expected hex, e.g. 0x90300000)"
                         % (kv["address"], where))
            unknown = set(kv) - {"model", "name", "address"}
            if unknown:
                fail("Unknown pragma option(s) %s at %s\n"
                     "Supported: model=, name=, address=" % (", ".join(sorted(unknown)), where))
            models.append(entry)
    return models


def detect_maya_fw(sketch_dir):
    """True if a top-level sketch source includes a wireless header."""
    exts = (".ino", ".pde", ".h", ".hpp", ".c", ".cpp")
    try:
        entries = sorted(os.listdir(sketch_dir))
    except OSError:
        return False
    for fname in entries:
        if not fname.lower().endswith(exts):
            continue
        try:
            with open(os.path.join(sketch_dir, fname), "r",
                      encoding="utf-8", errors="replace") as f:
                text = strip_comments(f.read())
        except OSError:
            continue
        if MAYA_FW_INCLUDE_RE.search(text):
            return True
    return False


def maya_fw_manifest_entry(platform_path):
    """Manifest entry for the IW610 firmware, shaped like a model entry."""
    bin_path = os.path.join(platform_path, MAYA_FW_BIN_REL)
    if not os.path.isfile(bin_path):
        fail("This sketch uses the wireless stack, but the IW610 firmware\n"
             "blob is missing from the core:\n"
             "  %s\n"
             "Reinstall the core, or restore sduartspi_iw610.bin.se\n"
             "(995,992 bytes)." % bin_path)
    with open(bin_path, "rb") as f:
        data = f.read()
    src_path = os.path.join(platform_path, MAYA_FW_LEN_SRC_REL)
    try:
        with open(src_path, "r", encoding="utf-8", errors="replace") as f:
            src = f.read()
    except OSError:
        src = ""
    m = re.search(r"IW610_FW_BLOB_LEN\s*\(\s*(\d+)\s*[uU]?\s*\)", src)
    if m and int(m.group(1)) != len(data):
        fail("IW610 firmware blob / library length mismatch:\n"
             "  %s = %d bytes\n"
             "  IW610_FW_BLOB_LEN (iw610_cpu12_se.c) = %s bytes\n"
             "Update the blob and the library constant TOGETHER - the\n"
             "runtime reads exactly IW610_FW_BLOB_LEN bytes from flash."
             % (bin_path, len(data), m.group(1)))
    crc = zlib.crc32(data) & 0xFFFFFFFF
    m = re.search(r"IW610_FW_BLOB_CRC\s*\(\s*0[xX]([0-9A-Fa-f]+)\s*[uU]?\s*\)",
                  src)
    if m and int(m.group(1), 16) != crc:
        fail("IW610 firmware blob / library CRC32 mismatch:\n"
             "  %s = 0x%08X\n"
             "  IW610_FW_BLOB_CRC (iw610_cpu12_se.c) = 0x%s\n"
             "Update the blob and the library constant TOGETHER - the\n"
             "runtime checks the image in flash against IW610_FW_BLOB_CRC."
             % (bin_path, crc, m.group(1).upper()))
    return {
        "name": "maya_iw610_fw",
        "weights_bin": bin_path,
        "address": "0x%08X" % MAYA_FW_ADDR,
        "size": len(data),
        "sha256": hashlib.sha256(data).hexdigest(),
        "crc32": "0x%08X" % crc,
    }


def detected_libraries(build_path):
    """Names of the libraries the build's library detection added, from
    includes.cache, or None when it cannot be read."""
    try:
        with open(os.path.join(build_path, "includes.cache"), "r",
                  encoding="utf-8") as f:
            entries = json.load(f)
    except (OSError, ValueError):
        return None
    if not isinstance(entries, list):
        return None
    names = set()
    for e in entries:
        if not isinstance(e, dict):
            continue
        # arduino-cli 1.x writes added_include_path, older builders Includepath.
        for key in ("added_include_path", "Includepath"):
            p = e.get(key)
            if isinstance(p, str) and p:
                parts = os.path.normpath(p).split(os.sep)
                if "libraries" in parts:
                    i = len(parts) - 1 - parts[::-1].index("libraries")
                    if i + 1 < len(parts):
                        names.add(parts[i + 1])
    return names


def prune_radio_firmware(build_path):
    """Drops the IW610 firmware entry when no wireless library was linked."""
    manifest_path = os.path.join(build_path, "neuron6_models.json")
    try:
        with open(manifest_path, "r", encoding="utf-8") as f:
            manifest = json.load(f)
    except (OSError, ValueError):
        return
    entries = manifest.get("models", [])
    if not any(e.get("name") == "maya_iw610_fw" for e in entries):
        return
    libs = detected_libraries(build_path)
    if libs is None or libs & set(WIRELESS_LIBS):
        return
    kept = [e for e in entries if e.get("name") != "maya_iw610_fw"]
    if kept:
        manifest["models"] = kept
        with open(manifest_path, "w", encoding="utf-8") as f:
            json.dump(manifest, f, indent=1)
    else:
        os.remove(manifest_path)
    group("MAYA FIRMWARE")
    item("upload", "not needed: the build links no wireless library")


def print_maya_fw_banner(entry):
    group("MAYA FIRMWARE")
    item("blob", os.path.basename(entry["weights_bin"]))
    item("flash", "%s bytes (%s) at %s" %
         ("{:,}".format(entry["size"]), human(entry["size"]),
          entry["address"]))
    item("upload", "flashed with the sketch (Tools > Model weights policy)")


def sanitize_name(stem):
    name = re.sub(r"[^A-Za-z0-9_]", "_", stem)
    if not name or name[0].isdigit():
        name = "m_" + name
    return name


def resolve_model_file(spec, sketch_dir, platform_path):
    cands = []
    if os.path.isabs(spec):
        cands.append(spec)
    else:
        cands.append(os.path.join(sketch_dir, spec))
        cands.append(os.path.join(sketch_dir, "models", spec))
        cands.append(os.path.join(platform_path, "models", spec))
    for c in cands:
        if os.path.isfile(c):
            return os.path.abspath(c)
    return None


# ST Edge AI Core discovery

def read_ll_aton_version(header_path):
    try:
        with open(header_path, "r", encoding="utf-8", errors="replace") as f:
            text = f.read()
        vals = {}
        for k in ("MAJOR", "MINOR", "MICRO", "DEV"):
            m = re.search(r"LL_ATON_VERSION_%s\s+\((\d+)\)" % k, text)
            if not m:
                return None
            vals[k] = int(m.group(1))
        return (vals["MAJOR"], vals["MINOR"], vals["MICRO"], vals["DEV"])
    except OSError:
        return None


def install_version_key(exe):
    """Sort key for <base>/<version>/Utilities/<os>/stedgeai: the version's
    numbers compare as numbers, so 10.0 sorts after 9.0 and 10.0-beta before
    10.0."""
    ver = os.path.basename(os.path.dirname(os.path.dirname(os.path.dirname(exe))))
    parts = [(2, int(x), "") if x.isdigit() else (0, 0, x) for x in re.split(r"[.\-_]", ver)]
    return parts + [(1, 0, "")]


def find_stedgeai(platform_path):
    """Locate stedgeai, preferring an install whose ll_aton version matches
    the core's. Returns (path, matched)."""
    core_ver = read_ll_aton_version(os.path.join(
        platform_path, "cores", "NeuroN6", "AI_Runtime", "Npu", "ll_aton",
        "ll_aton_version.h"))

    cands = []
    env = os.environ.get("NEURON6_STEDGEAI")
    if env:
        if not os.path.isfile(env):
            fail("NEURON6_STEDGEAI is set but does not exist:\n  %s" % env)
        cands.append(env)
    if sys.platform.startswith("win"):
        pats = [r"C:\ST\STEdgeAI\*\Utilities\windows\stedgeai.exe",
                r"C:\Program Files\STMicroelectronics\STEdgeAI\*\Utilities\windows\stedgeai.exe"]
    elif sys.platform == "darwin":
        pats = ["/Applications/STEdgeAI/*/Utilities/mac/stedgeai",
                os.path.expanduser("~/STEdgeAI/*/Utilities/mac/stedgeai")]
    else:
        pats = ["/opt/ST/STEdgeAI/*/Utilities/linux/stedgeai",
                os.path.expanduser("~/STEdgeAI/*/Utilities/linux/stedgeai")]
    found = []
    for p in pats:
        found.extend(glob.glob(p))
    cands.extend(sorted(found, key=install_version_key, reverse=True))  # newest first, any folder
    which = shutil.which("stedgeai")
    if which:
        cands.append(which)

    seen = set()
    matching, fallback = None, None
    for c in cands:
        c = os.path.abspath(c)
        if c in seen or not os.path.isfile(c):
            continue
        seen.add(c)
        if fallback is None:
            fallback = c
        if core_ver is not None:
            # <install>/Utilities/<os>/stedgeai -> <install>/Middlewares/ST/AI/Npu/ll_aton
            install = os.path.dirname(os.path.dirname(os.path.dirname(c)))
            hdr = os.path.join(install, "Middlewares", "ST", "AI", "Npu",
                               "ll_aton", "ll_aton_version.h")
            if read_ll_aton_version(hdr) == core_ver:
                matching = c
                break
    if matching:
        return matching, True
    if fallback:
        return fallback, False
    fail("ST Edge AI Core (stedgeai) was not found.\n"
         "This board compiles sketch neural models with ST Edge AI Core.\n"
         "Install it from st.com (search 'ST Edge AI Core', v4.x), or set the\n"
         "NEURON6_STEDGEAI environment variable to the stedgeai executable.\n"
         "Expected location (Windows): C:\\ST\\STEdgeAI\\4.0\\Utilities\\windows\\stedgeai.exe")


def stedgeai_version(exe):
    try:
        out = subprocess.run([exe, "--version"], capture_output=True, text=True,
                             timeout=120).stdout
        m = re.search(r"ST Edge AI Core\s+(v[\w.\-]+)", out)
        return m.group(1) if m else out.strip().splitlines()[0]
    except Exception as e:
        fail("Could not run stedgeai (--version):\n  %s\n  %s" % (exe, e))


# Mempool handling

# The board's NPU memory map, applied to stm32n6.mpool from the ST Edge AI
# install, which gives each pool its access properties. The pools are written
# in this order. props_of names the ST pool whose "prop" a pool takes.
BOARD_ONCHIP_SRAM_KB = "1792"
BOARD_POOLS = [
    {"fname": "AXIFLEXMEM"},
    {"fname": "AXISRAM1", "offset": "0x34080000"},
    {"fname": "AXISRAM2", "size": ("0", "KBYTES")},
    {"fname": "AXISRAM3", "name": "npuRAM", "size": ("1792", "KBYTES")},
    {"fname": "xSPI2", "name": "PSRAM", "props_of": "xSPI1",
     "prop": {"constants_preferred": "false"}, "size": ("14", "MBYTES")},
    {"fname": "xSPI1", "name": "OCTAFLASH", "props_of": "xSPI2",
     "prop": {"byteWidth": 2, "freqRatio": 5.0},
     "offset": "0x90300000", "size": ("26112", "KBYTES")},
]


def board_mempool(exe):
    """Path and data of the board's mempool, made from the ST Edge AI install
    that exe belongs to. Written once per result to the model cache."""
    install = os.path.dirname(os.path.dirname(os.path.dirname(exe)))
    st_path = os.path.join(install, "scripts", "N6_scripts", "my_mpools", "stm32n6.mpool")
    if not os.path.isfile(st_path):
        fail("ST Edge AI's memory-pool template is missing:\n  %s\n"
             "Reinstall ST Edge AI Core 4.x." % st_path)
    with open(st_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    try:
        st_pools = {p["fname"]: p for p in data["memory"]["mempools"]}
        pools = []
        for b in BOARD_POOLS:
            p = json.loads(json.dumps(st_pools[b["fname"]]))
            if "props_of" in b:
                p["prop"] = json.loads(json.dumps(st_pools[b["props_of"]]["prop"]))
            p["prop"].update(b.get("prop", {}))
            if "name" in b:
                p["name"] = b["name"]
            if "offset" in b:
                p["offset"]["value"] = b["offset"]
            if "size" in b:
                p["size"]["value"], p["size"]["magnitude"] = b["size"]
            pools.append(p)
        data["memory"]["mempools"] = pools
        for prm in data["params"]["param"]:
            if prm["paramname"] == "max_onchip_sram_size":
                prm["value"], prm["magnitude"] = BOARD_ONCHIP_SRAM_KB, "KBYTES"
    except (KeyError, TypeError) as e:
        fail("Unexpected layout in %s (%s).\n"
             "This core expects ST Edge AI Core 4.x." % (st_path, e))
    text = json.dumps(data, indent=1)
    out_dir = os.path.join(cache_root(), "mpool")
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, "neuron6-%s.mpool"
                       % hashlib.sha256(text.encode()).hexdigest()[:16])
    if not os.path.isfile(out):
        tmp = "%s.%d.tmp" % (out, os.getpid())
        with open(tmp, "w", encoding="utf-8") as f:
            f.write(text)
        os.replace(tmp, out)
    return out, data


def load_mempool(exe):
    mpool_path, data = board_mempool(exe)
    pools = data["memory"]["mempools"]
    weights = None
    for p in pools:
        if p.get("fname") == "xSPI1":
            weights = p
            break
    if weights is None:
        fail("No xSPI1 (weights flash) pool in the board mempool")
    base = int(weights["offset"]["value"], 0)
    mag = weights["size"]["magnitude"].upper()
    mult = {"BYTES": 1, "KBYTES": 1024, "MBYTES": 1024 * 1024}.get(mag)
    if mult is None:
        fail("Unsupported size magnitude '%s' in mempool xSPI1 pool" % mag)
    size = int(weights["size"]["value"], 0) * mult
    return mpool_path, data, base, size


def npu_psram_pool(mpool_data):
    """(base, size) of xSPI2, the NPU's PSRAM working pool, or None."""
    for p in mpool_data["memory"]["mempools"]:
        if p.get("fname") == "xSPI2":
            mult = {"BYTES": 1, "KBYTES": 1024, "MBYTES": 1024 * 1024}.get(
                p["size"]["magnitude"].upper())
            if mult is None:
                fail("Unsupported size magnitude '%s' in mempool xSPI2 pool"
                     % p["size"]["magnitude"])
            return int(p["offset"]["value"], 0), int(p["size"]["value"], 0) * mult
    return None


_UNIT_BYTES = {"B": 1, "KB": 1024, "MB": 1024 * 1024}


def psram_bytes_used(c_path):
    """Bytes of NPU working memory a generated network places in PSRAM."""
    with open(c_path, "r", encoding="utf-8", errors="replace") as f:
        text = f.read()
    m = re.search(r"index=(\d+) file postfix=xSPI2\b[^*]*?offset=(0x[0-9a-fA-F]+)", text)
    if not m:
        return 0
    idx, base = m.group(1), int(m.group(2), 16)
    used = 0
    s = re.search(r"global pool %s is ([0-9.]+) ?(B|KB|MB)\b" % idx, text)
    if s:
        used = int(float(s.group(1)) * _UNIT_BYTES[s.group(2)] + 0.5)
    if used == 0:
        return 0
    for off in re.findall(r"0x%xUL \+ (\d+)\)" % base, text, flags=re.IGNORECASE):
        used = max(used, int(off))
    return used


def write_effective_mpool(mpool_data, base, size, address, out_path):
    """Write a copy of the mempool with the xSPI1 pool rebased to address."""
    data = json.loads(json.dumps(mpool_data))  # deep copy
    for p in data["memory"]["mempools"]:
        if p.get("fname") == "xSPI1":
            p["offset"]["value"] = "0x%08X" % address
            p["size"]["value"] = str(size - (address - base))
            p["size"]["magnitude"] = "BYTES"
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=1)


# Cache and generation

def cache_root():
    if sys.platform.startswith("win"):
        base = os.environ.get("LOCALAPPDATA") or os.path.expanduser("~")
        return os.path.join(base, "NeuroN6", "modelcache")
    return os.path.join(os.path.expanduser("~"), ".neuron6", "modelcache")


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def prune_cache(root, keep=24):
    try:
        entries = [os.path.join(root, d) for d in os.listdir(root)]
        entries = [d for d in entries if os.path.isdir(d)]
        entries.sort(key=os.path.getmtime, reverse=True)
        for d in entries[keep:]:
            shutil.rmtree(d, ignore_errors=True)
    except OSError:
        pass


def extract_epoch_report(report_path):
    """Return (counts, text) of the report's 'Epochs details' section."""
    counts = {"total": None, "sw": None, "hybrid": None, "hw": None}
    section = []
    try:
        with open(report_path, "r", encoding="utf-8", errors="replace") as f:
            lines = f.read().splitlines()
    except OSError:
        return counts, ""
    in_sec = False
    for ln in lines:
        if "Epochs details" in ln:
            in_sec = True
        if in_sec:
            if "Requested memory size" in ln:
                break
            section.append(ln.rstrip())
            m = re.search(r"Total number of epochs\s+(\d+)", ln)
            if m:
                counts["total"] = int(m.group(1))
            m = re.search(r"pure software \(SW\) epochs\s+(\d+)", ln)
            if m:
                counts["sw"] = int(m.group(1))
            m = re.search(r"hybrid epochs.*?(\d+)\s*$", ln)
            if m and "hybrid" in ln:
                counts["hybrid"] = int(m.group(1))
            m = re.search(r"pure hardware \(HW or EC\) epochs\s+(\d+)", ln)
            if m:
                counts["hw"] = int(m.group(1))
    while section and not section[-1].strip():
        section.pop()
    return counts, "\n".join(section)


def run_generation(exe, model_file, name, address, mpool_info, work_dir):
    """Run stedgeai generate into work_dir/out. Returns a dict of paths."""
    mpool_path, mpool_data, base, size = mpool_info
    os.makedirs(work_dir, exist_ok=True)
    eff_mpool = os.path.join(work_dir, "NeuroN6_MemPool.mpool")
    if address != base:
        write_effective_mpool(mpool_data, base, size, address, eff_mpool)
    else:
        shutil.copyfile(mpool_path, eff_mpool)

    profile = {
        "Globals": {},
        "Profiles": {
            "neuron6": {
                "memory_pool": "./NeuroN6_MemPool.mpool",
                "options": NEURAL_ART_OPTIONS,
            }
        },
    }
    profile_path = os.path.join(work_dir, "neuron6_neural_art.json")
    with open(profile_path, "w", encoding="utf-8") as f:
        json.dump(profile, f, indent=1)

    out_dir = os.path.join(work_dir, "out")
    ws_dir = os.path.join(work_dir, "ws")
    os.makedirs(out_dir, exist_ok=True)
    cmd = [exe, "generate",
           "-m", model_file,
           "--target", "stm32n6",
           "--st-neural-art", "neuron6@" + profile_path.replace("\\", "/"),
           "-n", name,
           "--enable-epoch-controller",
           "--output", out_dir,
           "--workspace", ws_dir]
    cmd += GENERATE_INTERFACE_OPTIONS.split()
    log("  Compiling model with ST Edge AI (first build for this model/config")
    log("  takes ~1-3 min; later builds reuse the cached result)...")
    logfile = os.path.join(work_dir, "stedgeai.log")
    t0 = time.time()
    try:
        with open(logfile, "w", encoding="utf-8", errors="replace") as lf:
            proc = subprocess.run(cmd, stdout=lf, stderr=subprocess.STDOUT,
                                  cwd=work_dir, timeout=1800)
    except subprocess.TimeoutExpired:
        fail("stedgeai generate timed out (30 min) for model '%s'.\n"
             "Log: %s" % (name, logfile))
    except OSError as e:
        fail("Could not run stedgeai:\n  %s\n  %s" % (exe, e))
    if proc.returncode != 0:
        tail = ""
        try:
            with open(logfile, "r", encoding="utf-8", errors="replace",
                      newline="\n") as lf:
                tail_lines = [l.rstrip() for l in lf.readlines()]
            # Progress bars use \r: only the text after the last one is kept.
            tail_lines = [l.split("\r")[-1] for l in tail_lines if l.strip()]
            tail = "\n".join(tail_lines[-25:])
        except OSError:
            pass
        fail("ST Edge AI failed to convert model '%s' (exit %d).\n"
             "Model : %s\n"
             "Log   : %s\n"
             "--- last output ---\n%s" % (name, proc.returncode, model_file,
                                          logfile, tail))
    log("  Model compiled in %.1f s" % (time.time() - t0))

    c_file = os.path.join(out_dir, name + ".c")
    ecblobs = os.path.join(out_dir, name + "_ecblobs.h")
    report = os.path.join(out_dir, name + "_generate_report.txt")
    raws = sorted(glob.glob(os.path.join(out_dir, name + "_atonbuf.*.raw")))
    for p, what in ((c_file, "network C file"), (ecblobs, "epoch-controller blobs"),
                    (report, "generate report")):
        if not os.path.isfile(p):
            fail("stedgeai finished but the expected %s is missing:\n  %s\n"
                 "Log: %s" % (what, p, logfile))
    if len(raws) != 1 or not raws[0].endswith("_atonbuf.xSPI1.raw"):
        fail("Expected exactly one weights blob (%s_atonbuf.xSPI1.raw), got:\n  %s\n"
             "All model constants must fit the external-flash weights pool.\n"
             "Log: %s" % (name, "\n  ".join(raws) or "(none)", logfile))
    return {"c": c_file, "ecblobs": ecblobs, "report": report, "raw": raws[0],
            "out_dir": out_dir, "ws_dir": ws_dir, "log": logfile}


def ensure_cached(exe, exe_ver, model_file, name, address, mpool_info):
    """Return (cache_dir, meta), generating on a cache miss."""
    mpool_path = mpool_info[0]
    model_sha = sha256_file(model_file)
    key_src = "|".join([model_sha, sha256_file(mpool_path), NEURAL_ART_OPTIONS,
                        GENERATE_INTERFACE_OPTIONS,
                        name, "0x%08X" % address, exe_ver, "v1"])
    key = hashlib.sha256(key_src.encode()).hexdigest()[:16]
    root = cache_root()
    cdir = os.path.join(root, "%s-%s" % (name, key))
    meta_path = os.path.join(cdir, "meta.json")
    if os.path.isfile(meta_path):
        try:
            with open(meta_path, "r", encoding="utf-8") as f:
                meta = json.load(f)
            needed = [os.path.join(cdir, meta["files"][k])
                      for k in ("c", "ecblobs", "raw", "report")]
            if all(os.path.isfile(p) for p in needed):
                os.utime(cdir, None)  # freshen for LRU pruning
                return cdir, meta
        except (OSError, KeyError, ValueError):
            pass
        shutil.rmtree(cdir, ignore_errors=True)

    os.makedirs(root, exist_ok=True)
    # Kept when the generation fails: the error names its log.
    work = cdir + ".build-%d" % os.getpid()
    shutil.rmtree(work, ignore_errors=True)
    art = run_generation(exe, model_file, name, address, mpool_info, work)
    counts, section = extract_epoch_report(art["report"])
    raw_size = os.path.getsize(art["raw"])
    stage = os.path.join(work, "cache")
    os.makedirs(stage, exist_ok=True)
    files = {}
    for k in ("c", "ecblobs", "raw", "report"):
        files[k] = os.path.basename(art[k])
        shutil.copyfile(art[k], os.path.join(stage, files[k]))
    with open(art["raw"], "rb") as f:
        crc = zlib.crc32(f.read()) & 0xFFFFFFFF
    meta = {
        "name": name,
        "model_file": model_file,
        "model_sha256": model_sha,
        "address": "0x%08X" % address,
        "options": NEURAL_ART_OPTIONS,
        "stedgeai": exe_ver,
        "weights_size": raw_size,
        "weights_sha256": sha256_file(art["raw"]),
        "weights_crc32": "0x%08X" % crc,
        "epochs": counts,
        "epoch_report": section,
        "files": files,
        "created": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    with open(os.path.join(stage, "meta.json"), "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=1)
    shutil.rmtree(cdir, ignore_errors=True)
    os.replace(stage, cdir)
    shutil.rmtree(work, ignore_errors=True)
    prune_cache(root)
    return cdir, meta


# Build-tree outputs

def clean_slots(inc_dir):
    if os.path.isdir(inc_dir):
        for f in glob.glob(os.path.join(inc_dir, "neuron6_generated_model_*.inc")):
            try:
                os.remove(f)
            except OSError:
                pass


def _core_cache_entry_is_ours(path):
    """True for a core cache entry built from this platform."""
    # arduino-cli names an entry after the FQBN, or by an MD5 when that name
    # is too long. Only a hashed entry's core.a is searched for this core.
    name = os.path.basename(path).lower()
    if name.startswith("ohmlab_stm32n6_"):
        return True
    if not re.fullmatch(r"[0-9a-f]{32}", name):
        return False
    try:
        with open(os.path.join(path, "core.a"), "rb") as f:
            tail = b""
            while True:
                chunk = f.read(1 << 20)
                if not chunk:
                    return False
                if b"NeuroN6_Setup" in tail + chunk:
                    return True
                tail = chunk[-16:]
    except OSError:
        return False


def purge_shared_core_cache():
    """Delete this platform's entries in arduino-cli's shared core cache. A
    core.a built for one sketch holds that sketch's model symbols."""
    roots = []
    env = os.environ.get("ARDUINO_BUILD_CACHE_PATH")
    if env:
        roots.append(env)
    if sys.platform.startswith("win"):
        base = os.environ.get("LOCALAPPDATA")
        if base:
            roots.append(os.path.join(base, "arduino"))
    elif sys.platform == "darwin":
        roots.append(os.path.expanduser("~/Library/Caches/arduino"))
    else:
        roots.append(os.path.join(
            os.environ.get("XDG_CACHE_HOME", os.path.expanduser("~/.cache")),
            "arduino"))
    purged = 0
    for root in roots:
        cores = os.path.join(root, "cores")
        if not os.path.isdir(cores):
            continue
        for d in os.listdir(cores):
            p = os.path.join(cores, d)
            if os.path.isdir(p) and _core_cache_entry_is_ours(p):
                shutil.rmtree(p, ignore_errors=True)
                purged += 1
    return purged


def print_model_banner(idx, total, meta, model_file, psram=0, psram_base=None):
    c = meta["epochs"]
    tag = "MODEL %s" % meta["name"]
    if total > 1:
        tag += "  (%d/%d)" % (idx + 1, total)
    group(tag)
    item("source", os.path.basename(model_file))
    item("weights", "%s bytes (%s) at %s" %
         ("{:,}".format(meta["weights_size"]), human(meta["weights_size"]),
          meta["address"]))
    if psram and psram_base is not None:
        item("psram", "%s bytes (%s) of NPU working memory at 0x%08X;"
             " camera and event buffers stay below it" %
             ("{:,}".format(psram), human(psram), psram_base))
    if c.get("total") is not None:
        counts = ["?" if c.get(k) is None else c[k] for k in ("hw", "hybrid", "sw")]
        item("epochs", "%d total = %s NPU + %s hybrid + %s CPU" %
             tuple([c["total"]] + counts))
    if VERBOSITY == "verbose" and meta.get("epoch_report"):
        log()
        for ln in meta["epoch_report"].splitlines():
            log("  " + ln)


def main():
    global VERBOSITY
    if len(sys.argv) == 3 and sys.argv[1] == "--prune-radio":
        prune_radio_firmware(os.path.abspath(sys.argv[2]))
        return
    if len(sys.argv) in (2, 3) and sys.argv[1] == "--print-mpool":
        if len(sys.argv) == 3:
            exe = os.path.abspath(sys.argv[2])
        else:
            exe = find_stedgeai(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))[0]
        print(json.dumps(board_mempool(exe)[1], indent=1))
        return
    if len(sys.argv) < 4:
        fail("usage: neuron6_model_gen.py <sketch_dir> <build_path> <platform_path> [verbosity]\n"
             "       neuron6_model_gen.py --prune-radio <build_path>\n"
             "       neuron6_model_gen.py --print-mpool [stedgeai]")
    sketch_dir, build_path, platform_path = (os.path.abspath(a) for a in sys.argv[1:4])
    if len(sys.argv) > 4 and sys.argv[4] == "verbose":
        VERBOSITY = "verbose"

    inc_dir = os.path.join(build_path, "neuron6_model")
    manifest_path = os.path.join(build_path, "neuron6_models.json")

    purge_shared_core_cache()

    models = parse_pragmas(sketch_dir)
    maya_fw = detect_maya_fw(sketch_dir)

    if not models:
        # No pragma: stale outputs are removed.
        clean_slots(inc_dir)
        if maya_fw:
            # The manifest carries only the IW610 firmware.
            entry = maya_fw_manifest_entry(platform_path)
            manifest = {
                "version": 1,
                "generated": time.strftime("%Y-%m-%d %H:%M:%S"),
                "models": [entry],
            }
            with open(manifest_path, "w", encoding="utf-8") as f:
                json.dump(manifest, f, indent=1)
            print_maya_fw_banner(entry)
            return
        if os.path.isfile(manifest_path):
            try:
                os.remove(manifest_path)
            except OSError:
                pass
        return

    if len(models) > MAX_MODELS:
        fail("Too many '#pragma neuron6' models: %d (maximum %d per sketch)."
             % (len(models), MAX_MODELS))

    seen_names = {}
    for entry in models:
        f = resolve_model_file(entry["model"], sketch_dir, platform_path)
        if f is None:
            fail("Model file not found: %s   (from %s)\n"
                 "Searched:\n"
                 "  - the sketch folder: %s\n"
                 "  - %s\n"
                 "  - %s\n"
                 "Drop the .tflite/.onnx next to the sketch, or use an absolute path."
                 % (entry["model"], entry["where"], sketch_dir,
                    os.path.join(sketch_dir, "models"),
                    os.path.join(platform_path, "models")))
        if not f.lower().endswith(MODEL_EXTS):
            fail("Unsupported model type: %s   (from %s)\n"
                 "Supported: %s" % (f, entry["where"], ", ".join(MODEL_EXTS)))
        entry["file"] = f
        if "name" not in entry:
            entry["name"] = sanitize_name(os.path.splitext(os.path.basename(f))[0])
        if entry["name"] in seen_names:
            fail("Two models share the name '%s' (%s and %s).\n"
                 "Give one of them a distinct name=<...> in its pragma."
                 % (entry["name"], seen_names[entry["name"]], entry["where"]))
        seen_names[entry["name"]] = entry["where"]

    exe, ver_match = find_stedgeai(platform_path)
    exe_ver = stedgeai_version(exe)
    mpool_info = load_mempool(exe)
    _, _, pool_base, pool_size = mpool_info
    if VERBOSITY == "verbose":
        group("ST EDGE AI")
        item("version", exe_ver)
        item("tool", exe)
    if not ver_match:
        log("  WARNING       could not confirm this ST Edge AI install matches")
        log("                the core's ATON runtime (atonn 1.1.3-275 = v4.x).")
        log("                If the build fails with an ll_aton #error, install")
        log("                ST Edge AI Core v4.x or set NEURON6_STEDGEAI.")

    # Models generate in order: a packed address follows the previous blob.
    results = []
    next_addr = pool_base
    for i, entry in enumerate(models):
        addr = entry.get("address", next_addr)
        if addr % SECTOR:
            fail("Weights address 0x%08X for '%s' is not 4 KB aligned."
                 % (addr, entry["name"]))
        if addr < pool_base or addr >= pool_base + pool_size:
            fail("Weights address 0x%08X for '%s' is outside the weights pool\n"
                 "(0x%08X .. 0x%08X)." % (addr, entry["name"], pool_base,
                                          pool_base + pool_size - 1))
        cdir, meta = ensure_cached(exe, exe_ver, entry["file"], entry["name"],
                                   addr, mpool_info)
        end = addr + meta["weights_size"]
        if end > pool_base + pool_size:
            fail("Weights of '%s' (%d bytes @ 0x%08X) overflow the weights pool\n"
                 "(ends 0x%08X, pool ends 0x%08X)."
                 % (entry["name"], meta["weights_size"], addr, end,
                    pool_base + pool_size))
        results.append((entry, addr, cdir, meta))
        next_addr = (end + WEIGHTS_ALIGN - 1) // WEIGHTS_ALIGN * WEIGHTS_ALIGN

    # An explicit address can overlap a packed one.
    spans = sorted(((a, a + m["weights_size"], e["name"]) for e, a, _, m in results))
    for (s1, e1, n1), (s2, e2, n2) in zip(spans, spans[1:]):
        if e1 > s2:
            fail("Weights of '%s' (0x%08X..0x%08X) overlap '%s' (starts 0x%08X).\n"
                 "Adjust the address=<...> pragma options." % (n1, s1, e1, n2, s2))

    maya_entry = maya_fw_manifest_entry(platform_path) if maya_fw else None
    if maya_entry is not None:
        m_start = MAYA_FW_ADDR
        m_end = MAYA_FW_ADDR + maya_entry["size"]
        for e, a, _, m in results:
            if a < m_end and a + m["weights_size"] > m_start:
                fail("Weights of '%s' (0x%08X..0x%08X) overlap the MAYA IW610\n"
                     "firmware slot (0x%08X..0x%08X). Move the model with an\n"
                     "address=<...> pragma option."
                     % (e["name"], a, a + m["weights_size"], m_start, m_end))

    # The models share one PSRAM pool for NPU working memory, so the largest
    # use counts. Slot 0 reports the range to the core.
    psram_pool = npu_psram_pool(mpool_info[1])
    psram_used = [psram_bytes_used(os.path.join(c, m["files"]["c"]))
                  for _, _, c, m in results]
    psram_max = max(psram_used) if psram_used else 0

    # One include file per slot, read by cores/NeuroN6/neuron6_user_model_N.c.
    os.makedirs(inc_dir, exist_ok=True)
    clean_slots(inc_dir)
    for i, (entry, addr, cdir, meta) in enumerate(results):
        c_path = os.path.join(cdir, meta["files"]["c"]).replace("\\", "/")
        inc = os.path.join(inc_dir, "neuron6_generated_model_%d.inc" % i)
        with open(inc, "w", encoding="utf-8") as f:
            f.write("/* Auto-generated by neuron6_model_gen.py - do not edit.\n"
                    " * Network '%s' from %s\n */\n"
                    "#include \"%s\"\n"
                    % (meta["name"], entry["file"].replace("\\", "/"), c_path))
            if i == 0 and psram_max > 0 and psram_pool is not None:
                f.write("\n#include <stdint.h>\n"
                        "/* A model in this sketch keeps NPU working memory in PSRAM.\n"
                        " * Overrides the core's weak default, so the core's PSRAM\n"
                        " * allocator keeps camera and event buffers below it. */\n"
                        "void neuron6_npu_psram_region(uint32_t *out_base, uint32_t *out_bytes);\n"
                        "void neuron6_npu_psram_region(uint32_t *out_base, uint32_t *out_bytes)\n"
                        "{\n"
                        "  if (out_base)  *out_base  = 0x%08XUL;\n"
                        "  if (out_bytes) *out_bytes = %dUL;\n"
                        "}\n" % (psram_pool[0], psram_max))

    manifest = {
        "version": 1,
        "generated": time.strftime("%Y-%m-%d %H:%M:%S"),
        "stedgeai": exe_ver,
        "models": [
            {
                "name": m["name"],
                "model_file": e["file"],
                "weights_bin": os.path.join(c, m["files"]["raw"]),
                "address": "0x%08X" % a,
                "size": m["weights_size"],
                "sha256": m["weights_sha256"],
                "crc32": m["weights_crc32"],
                "epochs": m["epochs"],
            }
            for e, a, c, m in results
        ] + ([maya_entry] if maya_entry is not None else []),
    }
    with open(manifest_path, "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=1)

    for i, (entry, addr, cdir, meta) in enumerate(results):
        print_model_banner(i, len(results), meta, entry["file"],
                           psram_used[i], psram_pool[0] if psram_pool else None)
    if maya_entry is not None:
        print_maya_fw_banner(maya_entry)
    if VERBOSITY == "verbose":
        log("  Upload flashes the weights automatically before the app")
        log("  (Tools > Model weights controls when they re-upload).")


if __name__ == "__main__":
    main()

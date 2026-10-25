# Contributing

## Reporting a problem

Open an issue with:

- the board revision, camera and carrier;
- the core version and the bootloader version the upload prints;
- the Arduino IDE version and the operating system;
- a minimal sketch and the full build or serial output.

Report a vulnerability as [SECURITY.md](SECURITY.md) describes.

## Pull requests

```bash
python tools/check_ascii.py
python tools/build_all_examples.py --out after.json
```

- Without ST Edge AI Core, add `--no-models`. The SD card sketches need SdFat:
  `arduino-cli lib install SdFat`.
- For a change to the core, build `main` first with `--out baseline.json`,
  then run `python tools/build_all_examples.py --compare baseline.json after.json`.
  A change that alters no behaviour gives identical binaries, unless it moves
  a line whose number an assert records.
- State the board revision you tested on.
- A contribution is accepted under the licence of the files it changes.

## Style

- ASCII source, two-space indent, LF endings, K&R braces: see `.editorconfig`.
- A comment states one fact in one line. History goes in `CHANGELOG.md`.
- Libraries follow the
  [Arduino library specification](https://arduino.github.io/arduino-cli/latest/library-specification/).
- Vendored code is recorded in `THIRD-PARTY-NOTICES.md` with its upstream,
  version, licence and local changes, and each local change is marked in the
  file.

## Building from a clone

```bash
git clone https://github.com/ohmlab-ltd/Neuro-N6-Arduino-Core <Sketchbook>/hardware/Ohmlab/stm32n6
```

Create `platform.local.txt` beside `platform.txt`. An empty `compiler.path`
uses the `arm-none-eabi-gcc` on `PATH`, GCC 11 or newer:

```
compiler.path=
neuron6.python=python3
```

On Windows:

```
compiler.path=
neuron6.python.windows=cmd /c call "{runtime.platform.path}/tools/run_python.cmd"
```

- The uploader uses `pyserial` from that Python, or `tools/pyserial`.
- On Linux, run `sudo ./post_install.sh` once to install the udev rule.

## Continuous integration

Each push to `main` and each pull request builds the Boards Manager package,
installs it on a hosted runner and compiles the sketches that need no model
tooling. Build the full set locally with `python tools/build_all_examples.py`.

## Releasing

1. Set `version=` in `platform.txt`, the same numbers in
   `cores/NeuroN6/NeuroN6_Version.h` and the version line of `README.md`, and
   add a `CHANGELOG.md` entry. `tools/check_arduino_spec.py` checks that the
   first two match.
2. Run `python tools/packaging/build_package.py`. It writes
   `dist/stm32n6-<version>.zip` and `dist/package_ohmlab_index.json`.
3. Commit the index to `main` as `package_ohmlab_index.json`.
4. Create the GitHub release `v<version>` and attach the zip and the
   bootloader image the README names. The tag runs *Actions > Release
   package*, which builds and lints the same pair; its index lint needs every
   URL in the index to resolve.

- `tools/packaging/tools.json` pins the compiler and Python the index names;
  `update_tools.py` refreshes their checksums.
- The Python archives are served from the release named by `TOOLS_URL` in
  `build_package.py`. `repack_python.sh` builds them into `dist/tools` and
  records them in `rehosted.json`.

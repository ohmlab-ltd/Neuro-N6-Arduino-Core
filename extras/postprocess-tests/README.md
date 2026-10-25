# Post-processor host tests

Compiles the model zoo post-processors in `libraries/PostProcess` for a PC,
runs them on tensors, and compares each result with a decode written in
Python.

## Requirements

- MSVC build tools. `build.bat` finds `vcvars64.bat`, or set `VC` to it.
- Python with `numpy`.
- `onnxruntime` and the ST model zoo files, for the cases that use models.
  Without them those cases report `skip`.

The zoo files are named in `run_tests.py`. `models/CATALOG.md` lists them,
except `mobilenetv2_a035_128_qdq_int8.onnx`, an ONNX form of its MobileNet.
They are not in this repository.

## Running

```
build.bat
set N6_ZOO_DIR=C:\where\the\zoo\files\are
python sync_coco.py
python run_tests.py
```

`sync_coco.py` copies PostProcess's COCO-80 names into the test driver.
`run_tests.py` takes case names to run a subset.

Generated files go to `%TEMP%\n6-postprocess-tests`, or to `N6_PP_WORK`.

## Cases

| Kind | Input |
|---|---|
| Models | Each ONNX model run through onnxruntime, its outputs quantised as the N6 build quantises them |
| Planted detections, `planted.py` | The same outputs with a few cells raised |
| Synthetic | Boxes in input pixels, a transposed output, channel-first heatmaps, a classifier of raw scores, no result above the threshold, a cropped and a letterboxed camera mapping, and outputs that do not fit the family |

Each case compares the boxes, the metadata bytes and the overlay bounds with
`zoo_ref.py`.

`python overlay_png.py <case>` writes the 800x480 overlay for one case. It
needs `pillow`.

`shim/` holds host versions of the board headers the sources include.

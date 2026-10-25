# FaceID

A CenterFace detector and a MobileFaceNet embedder as post-processors, with
an embedding bank. Examples: `FaceRecognition`, `GlobalShutterFaceDetection`.

Licence: MIT. The models the examples embed are not:

- `centerface-hwc.tflite`, Star-Clouds' CenterFace, under MIT
  (`models/LICENSE-MIT-CenterFace.txt`);
- `face4-int8.onnx`, foamliu's MobileFaceNet v1.0, under Apache-2.0
  (`models/face4-int8.onnx.license`). It was trained on InsightFace's MS1M
  data, which is for non-commercial research only.

`models/README.md` has the details.

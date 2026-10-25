/*
  Custom sensor

  A skeleton ImageSensor class for a camera the core does not support. As it
  stands init() reports a failure and nothing streams.

  Hardware: Neuro N6 with a camera wired to the camera connector.

  This example code is in the public domain.
*/

/* ImageSensor and its types come with <Arduino.h>. */
class MyStubSensor : public ImageSensor {
public:
  MyStubSensor() {
    // Data-lane order: 1 usual, 2 reversed.
    registerAsActive(1u);
  }

  // Writes the frame size. Returns 0, or a negative value on failure.
  int init(CameraResolutionN6 res, CameraFpsN6 fps, CameraMirrorFlipN6 flip,
           uint16_t &out_w, uint16_t &out_h) override {
    (void)res; (void)fps; (void)flip;
    out_w = 640; out_h = 480;
    // The camera's reset, chip check and settings go here. -1 reports that
    // there is no camera.
    return -1;
  }

  // Expected chip ID.
  uint32_t chipId() override { return 0x0000u; }

  // Must match what the camera sends.
  void getFormat(SensorFormat &out) override {
    out.pixel_format = N6_PIXFMT_YUV422_YUYV;   // or N6_PIXFMT_RGB565, or N6_PIXFMT_RAW8
    out.color_space  = N6_COLORSPACE_BT601;
  }

  void        streamOn()  override { /* start streaming */ }
  void        streamOff() override { /* stop streaming */ }
  const char *name()      override { return "MyStubSensor"; }
};

MyStubSensor camera;

void setup() {
  // No model.
  Vision.begin();
}

void loop() {
}

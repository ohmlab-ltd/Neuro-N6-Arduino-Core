/*
  Global shutter barcode reader

  Reads EAN-13, UPC-A and EAN-8 barcodes across the global-shutter camera's
  1120x720 frame and streams each one's digits, box and crop to Neuro Studio
  over USB. The bars must be upright, and the code between 20% and 80% of
  the frame's height. Every two seconds the sketch reports how many frames it
  read and how many held a code.

  Hardware: Neuro N6 with a VD66GY camera on the Neuro Vision carrier.

  This example code is in the public domain.
*/

#include <VD66GY_Arduino.h>
#include <PostProcess.h>
#include <barcode1d.h>
#include <n6_imgproc.h>

// The decoder reads the whole CAMERA_720P frame.
#define BC_W        1120
#define BC_H        720
#define VISION_KIND 0x02   // barcode, for Neuro Studio

// The capture buffer must be 32-byte aligned.
static uint8_t g_snapshot[BC_W * BC_H * 3]
  __attribute__((aligned(32), section(".ext_psram")));
static uint8_t g_gray[BC_W * BC_H] __attribute__((section(".ext_psram")));

static uint8_t g_payload[VND_VISION_CROP_DIM * VND_VISION_CROP_DIM * 2 + 64]
  __attribute__((section(".ext_psram")));

// VD66GY camera(VD66GY_CARRIER_THERMAL); on the Thermal carrier.
VD66GY camera;

// The last code printed, and when. A code is printed again after REPRINT_MS.
static char     g_last[16] = {0};
static uint32_t g_last_print_ms = 0;
static const uint32_t REPRINT_MS = 3000;

static uint32_t g_frames = 0, g_decodes = 0, g_last_beat_ms = 0;

static const char *sym_name(barcode1d_sym_t s) {
  switch (s) {
    case BARCODE1D_EAN13: return "EAN-13";
    case BARCODE1D_UPCA:  return "UPC-A";
    case BARCODE1D_EAN8:  return "EAN-8";
    default:              return "?";
  }
}

void setup() {
  Serial.begin(115200);

  // No model runs. cfg.nn sets the snapshot: the whole frame, unscaled.
  VisionConfig cfg;
  cfg.camera      = CAMERA_720P;             // 1120x720, 60 frames per second
  cfg.isp_enabled = VD66_isp_tuning_auto;    // automatic white balance
  cfg.transport   = TRANSPORT_USB;
  cfg.nn.width    = BC_W;
  cfg.nn.height   = BC_H;
  cfg.nn.aspect   = ASPECT_STRETCH;
  Vision.begin(cfg);

  Serial.println("[BARCODE] ready: EAN-13, UPC-A, EAN-8");
}

void loop() {
  vnd_meta_vision_t vis;
  vnd_meta_vision_begin(&vis, VND_META_SLOT_BARCODE, META_BARCODE_FRAME,
                        g_payload, sizeof(g_payload));

  if (NN_capture_snapshot(g_snapshot, sizeof(g_snapshot))) {
    g_frames++;
    n6img_rgb888_to_gray(g_snapshot, g_gray, BC_W, BC_H);

    barcode1d_result_t res;
    if (barcode1d_scan_gray(g_gray, BC_W, BC_H, &res)) {
      g_decodes++;

      // The box spans the bars, in a band a quarter of the frame high around
      // the row that read them.
      int bw = res.x_end - res.x_start + 1;
      if (bw < 1) bw = 1;
      const float cx = ((float)res.x_start + (float)bw * 0.5f) / (float)BC_W;
      const float cy = (float)res.y / (float)BC_H;

      // Extra: u8 symbology, u16 length, then the digits.
      const int tlen = (int)strnlen(res.text, sizeof(res.text));
      uint8_t extra[3 + sizeof(res.text)];
      extra[0] = (uint8_t)res.sym;
      vnd_meta_put_u16_le(&extra[1], (uint16_t)tlen);
      memcpy(&extra[3], res.text, (size_t)tlen);

      char label[VND_VISION_LABEL_MAX + 1];
      snprintf(label, sizeof(label), "%s %s", sym_name(res.sym), res.text);

      (void)vnd_meta_vision_add(&vis, cx, cy, (float)bw / (float)BC_W, 0.25f,
                                label, extra, (uint16_t)(3 + tlen),
                                g_snapshot, BC_W, BC_H);

      const uint32_t now = millis();
      if (strcmp(res.text, g_last) != 0 || (now - g_last_print_ms) >= REPRINT_MS) {
        strcpy(g_last, res.text);
        g_last_print_ms = now;
        char line[48];
        snprintf(line, sizeof(line), "[BARCODE] %s: %s", sym_name(res.sym), res.text);
        Serial.println(line);
        vnd_meta_publish_debug_text(line);
      }
    }
  }

  // An empty report clears the last box from the video.
  vnd_meta_vision_finish(&vis);

  const uint32_t now = millis();
  if (now - g_last_beat_ms >= 2000) {
    g_last_beat_ms = now;
    vnd_meta_vision_announce(VISION_KIND);
    char beat[64];
    snprintf(beat, sizeof(beat), "[BARCODE] %lu frames, %lu with a code",
             (unsigned long)g_frames, (unsigned long)g_decodes);
    Serial.println(beat);
    vnd_meta_publish_debug_text(beat);
    g_frames = 0;
    g_decodes = 0;
  }
}

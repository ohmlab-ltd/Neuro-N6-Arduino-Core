/*
  Barcode reader

  Reads EAN-13, UPC-A and EAN-8 barcodes held level across the middle of the
  camera frame and streams each one's digits, box and crop to Neuro Studio
  over USB.

  Hardware: Neuro N6 with an OV5640 camera.

  This example code is in the public domain.
*/

#include <OV5640_Arduino.h>
#include <PostProcess.h>
#include <barcode1d.h>

// Decoder input size, the camera's full frame.
#define BC_W        1920
#define BC_H        1080
#define VISION_KIND 0x02   // barcode, for Neuro Studio

// The hardware that fills g_snapshot needs it aligned to 32 bytes.
static uint8_t g_snapshot[BC_W * BC_H * 3]
  __attribute__((aligned(32), section(".ext_psram")));
static uint8_t g_gray[BC_W * BC_H]         __attribute__((section(".ext_psram")));

static uint8_t g_payload[VND_VISION_CROP_DIM * VND_VISION_CROP_DIM * 2 + 64]
  __attribute__((section(".ext_psram")));

OV5640 camera;

static char     g_last[24] = {0};
static uint32_t g_last_ms = 0;
static const uint32_t REANNOUNCE_MS = 3000;

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
  VisionConfig cfg;
  cfg.camera    = CAMERA_1080P;
  cfg.fps       = FPS_30;
  cfg.transport = TRANSPORT_USB;
  cfg.nn.width  = BC_W;
  cfg.nn.height = BC_H;
  cfg.nn.aspect = ASPECT_CROP_CENTER;
  cfg.af        = AF_CONTINUOUS;
  Vision.begin(cfg);
  Serial.println("[BARCODE] ready -- EAN-13 / UPC-A / EAN-8 (continuous AF)");
  vnd_meta_publish_debug_text("[BARCODE] scanner ready");
}

void loop() {
  vnd_meta_vision_t vis;
  vnd_meta_vision_begin(&vis, VND_META_SLOT_BARCODE, META_BARCODE_FRAME,
                        g_payload, sizeof(g_payload));

  if (NN_capture_snapshot(g_snapshot, sizeof(g_snapshot))) {
    g_frames++;

    const uint8_t *s = g_snapshot;
    for (int i = 0; i < BC_W * BC_H; i++) {
      g_gray[i] = (uint8_t)((s[0] * 77u + s[1] * 151u + s[2] * 28u) >> 8);
      s += 3;
    }

    barcode1d_result_t res;
    if (barcode1d_scan_gray(g_gray, BC_W, BC_H, &res)) {
      g_decodes++;

      // The box is the barcode's width, in a band around the row that read it.
      int bx = res.x_start, bw = res.x_end - res.x_start + 1;
      int band = BC_H / 4, by = res.y - band / 2, bh = band;
      if (bw < 1) bw = 1;

      const float ncx = (bx + bw * 0.5f) / (float)BC_W;
      const float ncy = (by + bh * 0.5f) / (float)BC_H;
      const float nw  = (float)bw / (float)BC_W;
      const float nh  = (float)bh / (float)BC_H;

      // extra: u8 symbology, u16 length, then the digits.
      const int tlen = (int)strnlen(res.text, sizeof(res.text));
      uint8_t extra[3 + sizeof(res.text)];
      extra[0] = (uint8_t)res.sym;
      vnd_meta_put_u16_le(&extra[1], (uint16_t)tlen);
      memcpy(&extra[3], res.text, (size_t)tlen);

      char label[VND_VISION_LABEL_MAX + 1];
      snprintf(label, sizeof(label), "%s %s", sym_name(res.sym), res.text);

      (void)vnd_meta_vision_add(&vis, ncx, ncy, nw, nh, label,
                                extra, (uint16_t)(3 + tlen),
                                g_snapshot, BC_W, BC_H);

      const uint32_t now = millis();
      if (strcmp(res.text, g_last) != 0 || (now - g_last_ms) >= REANNOUNCE_MS) {
        strncpy(g_last, res.text, sizeof(g_last) - 1);
        g_last_ms = now;
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
    vnd_meta_vision_announce(VISION_KIND);
    char beat[64];
    snprintf(beat, sizeof(beat), "[BARCODE] frames=%lu decodes=%lu",
             (unsigned long)g_frames, (unsigned long)g_decodes);
    Serial.println(beat);
    vnd_meta_publish_debug_text(beat);
    g_last_beat_ms = now;
  }
  delay(10);
}

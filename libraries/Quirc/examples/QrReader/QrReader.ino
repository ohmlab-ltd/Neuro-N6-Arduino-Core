/*
  QR reader

  Reads QR codes from the camera on the CPU and sends each to Neuro Studio
  with its text, a box and a thumbnail.

  Hardware: Neuro N6 with an OV5640 camera.

  This example code is in the public domain.
*/

#include <OV5640_Arduino.h>
#include <PostProcess.h>
#include <quirc.h>

#define QR_DIM       256   // frame side handed to the decoder, pixels
#define QR_MAX_SHOW  512   // longest text kept from one code
#define MAX_RESULTS  4     // codes reported per frame
#define VISION_KIND  0x01  // QR, for vnd_meta_vision_announce()

// The capture buffer must be 32-byte aligned.
static uint8_t g_snapshot[QR_DIM * QR_DIM * 3]
  __attribute__((aligned(32), section(".ext_psram")));

static uint8_t g_qr_payload[MAX_RESULTS *
    (VND_VISION_CROP_DIM * VND_VISION_CROP_DIM * 2 + QR_MAX_SHOW + 32) + 8]
  __attribute__((section(".ext_psram")));

OV5640 camera;

static struct quirc *g_qr = nullptr;

// About 13 KB together.
static struct quirc_code s_code __attribute__((section(".ext_psram")));
static struct quirc_data s_data __attribute__((section(".ext_psram")));

// The last code printed, and when.
static char     g_last_payload[QR_MAX_SHOW + 1] = {0};
static uint32_t g_last_seen_ms = 0;
static const uint32_t REANNOUNCE_MS = 3000;

static uint32_t g_frames = 0, g_decodes = 0, g_last_beat_ms = 0;

void setup() {
  Serial.begin(115200);

  VisionConfig cfg;
  cfg.camera    = CAMERA_VGA;
  cfg.fps       = FPS_30;
  cfg.transport = TRANSPORT_USB;
  cfg.nn.width  = QR_DIM;
  cfg.nn.height = QR_DIM;
  cfg.nn.aspect = ASPECT_CROP_CENTER;
  cfg.af        = AF_CONTINUOUS;
  Vision.begin(cfg);

  g_qr = quirc_new();
  if (g_qr == nullptr || quirc_resize(g_qr, QR_DIM, QR_DIM) < 0) {
    Serial.println("[QR] FATAL: could not allocate quirc buffers -- lower QR_DIM.");
    if (g_qr) { quirc_destroy(g_qr); g_qr = nullptr; }
  } else {
    Serial.print("[QR] ready -- quirc "); Serial.println(quirc_version());
    vnd_meta_publish_debug_text("[QR] scanner ready");
  }
}

void loop() {
  if (g_qr == nullptr) { delay(500); return; }

  int kept = 0;
  vnd_meta_vision_t vis;
  vnd_meta_vision_begin(&vis, VND_META_SLOT_QR, META_QR_FRAME,
                        g_qr_payload, sizeof(g_qr_payload));

  if (NN_capture_snapshot(g_snapshot, sizeof(g_snapshot))) {
    g_frames++;

    // RGB888 to grey.
    int w = 0, h = 0;
    uint8_t *img = quirc_begin(g_qr, &w, &h);
    const uint8_t *src = g_snapshot;
    for (int i = 0, n = w * h; i < n; i++) {
      img[i] = (uint8_t)((src[0] * 77u + src[1] * 151u + src[2] * 28u) >> 8);
      src += 3;
    }
    quirc_end(g_qr);

    const int count = quirc_count(g_qr);
    for (int i = 0; i < count && kept < MAX_RESULTS; i++) {
      quirc_extract(g_qr, i, &s_code);
      if (quirc_decode(&s_code, &s_data) != QUIRC_SUCCESS) continue;
      g_decodes++;

      // Bounding box of the four corners.
      int minx = s_code.corners[0].x, maxx = minx, miny = s_code.corners[0].y, maxy = miny;
      for (int c = 1; c < 4; c++) {
        const int px = s_code.corners[c].x, py = s_code.corners[c].y;
        if (px < minx) minx = px; if (px > maxx) maxx = px;
        if (py < miny) miny = py; if (py > maxy) maxy = py;
      }
      const int bw = maxx - minx + 1, bh = maxy - miny + 1;

      // Normalised to the frame, 0 to 1.
      const float ncx = (minx + bw * 0.5f) / (float)QR_DIM;
      const float ncy = (miny + bh * 0.5f) / (float)QR_DIM;
      const float nw  = (float)bw / (float)QR_DIM;
      const float nh  = (float)bh / (float)QR_DIM;

      // Extra: u8 version, u8 ECC level, u16 length, text.
      int tlen = s_data.payload_len; if (tlen > QR_MAX_SHOW) tlen = QR_MAX_SHOW;
      uint8_t extra[4 + QR_MAX_SHOW];
      extra[0] = (uint8_t)s_data.version;
      extra[1] = (uint8_t)(s_data.ecc_level & 3);
      vnd_meta_put_u16_le(&extra[2], (uint16_t)tlen);
      memcpy(&extra[4], s_data.payload, (size_t)tlen);

      char label[VND_VISION_LABEL_MAX + 1];
      int ll = tlen; if (ll > VND_VISION_LABEL_MAX) ll = VND_VISION_LABEL_MAX;
      memcpy(label, s_data.payload, (size_t)ll);
      label[ll] = '\0';

      (void)vnd_meta_vision_add(&vis, ncx, ncy, nw, nh, label,
                                extra, (uint16_t)(4 + tlen),
                                g_snapshot, QR_DIM, QR_DIM);
      kept++;

      // Print a code when it changes, or again after REANNOUNCE_MS.
      char payload[QR_MAX_SHOW + 1];
      memcpy(payload, s_data.payload, (size_t)tlen); payload[tlen] = '\0';
      const uint32_t now = millis();
      if (strcmp(payload, g_last_payload) != 0 || (now - g_last_seen_ms) >= REANNOUNCE_MS) {
        strncpy(g_last_payload, payload, sizeof(g_last_payload) - 1);
        g_last_seen_ms = now;
        char line[QR_MAX_SHOW + 48];
        snprintf(line, sizeof(line), "[QR] v%d ecc=%c: %s",
                 s_data.version, "MLHQ"[s_data.ecc_level & 3], payload);
        Serial.println(line);
        vnd_meta_publish_debug_text(line);
      }
    }
  }

  // An empty report clears the previous result.
  vnd_meta_vision_finish(&vis);

  const uint32_t now = millis();
  if (now - g_last_beat_ms >= 2000) {
    vnd_meta_vision_announce(VISION_KIND);
    char beat[80];
    snprintf(beat, sizeof(beat), "[QR] scanning... frames=%lu decodes=%lu",
             (unsigned long)g_frames, (unsigned long)g_decodes);
    Serial.println(beat);
    g_last_beat_ms = now;
  }
  delay(10);
}

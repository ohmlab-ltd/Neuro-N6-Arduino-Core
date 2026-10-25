/*
  Global shutter QR reader

  Reads QR codes from the global-shutter camera on the CPU and sends each to
  Neuro Studio with its text, a box and a thumbnail. A code on a moving
  object keeps its square shape. Every two seconds the sketch reports how
  many frames it read and how many held a code.

  Hardware: Neuro N6 with a VD66GY camera on the Neuro Vision carrier.

  This example code is in the public domain.
*/

#include <VD66GY_Arduino.h>
#include <PostProcess.h>
#include <quirc.h>
#include <n6_imgproc.h>

#define QR_DIM       256   // side of the square handed to the decoder, pixels
#define QR_MAX_TEXT  512   // longest text kept from one code
#define MAX_RESULTS  4     // codes reported per frame
#define VISION_KIND  0x01  // QR, for Neuro Studio

// The capture buffer must be 32-byte aligned.
static uint8_t g_snapshot[QR_DIM * QR_DIM * 3]
  __attribute__((aligned(32), section(".ext_psram")));

static uint8_t g_payload[MAX_RESULTS *
    (VND_VISION_CROP_DIM * VND_VISION_CROP_DIM * 2 + QR_MAX_TEXT + 32) + 8]
  __attribute__((section(".ext_psram")));

// VD66GY camera(VD66GY_CARRIER_THERMAL); on the Thermal carrier.
VD66GY camera;

static struct quirc *g_qr = nullptr;
static struct quirc_code s_code __attribute__((section(".ext_psram")));
static struct quirc_data s_data __attribute__((section(".ext_psram")));

// The last text printed, and when. A code is printed again after REPRINT_MS.
static char     g_last_text[QR_MAX_TEXT + 1] = {0};
static uint32_t g_last_print_ms = 0;
static const uint32_t REPRINT_MS = 3000;

static uint32_t g_frames = 0, g_frames_with_code = 0, g_last_beat_ms = 0;

void setup() {
  Serial.begin(115200);

  // No model runs. cfg.nn sets the snapshot: a centred square, QR_DIM wide.
  VisionConfig cfg;
  cfg.camera      = CAMERA_VGA;              // 88 frames per second
  cfg.isp_enabled = VD66_isp_tuning_auto;    // automatic white balance
  cfg.transport   = TRANSPORT_USB;
  cfg.nn.width    = QR_DIM;
  cfg.nn.height   = QR_DIM;
  cfg.nn.aspect   = ASPECT_CROP_CENTER;
  Vision.begin(cfg);

  g_qr = quirc_new();
  if (g_qr == nullptr || quirc_resize(g_qr, QR_DIM, QR_DIM) < 0) {
    Serial.println("[QR] not enough memory for the decoder");
    if (g_qr) { quirc_destroy(g_qr); g_qr = nullptr; }
    return;
  }
  Serial.print("[QR] ready, quirc ");
  Serial.println(quirc_version());
}

void loop() {
  if (g_qr == nullptr) { delay(500); return; }

  vnd_meta_vision_t vis;
  vnd_meta_vision_begin(&vis, VND_META_SLOT_QR, META_QR_FRAME,
                        g_payload, sizeof(g_payload));

  if (NN_capture_snapshot(g_snapshot, sizeof(g_snapshot))) {
    g_frames++;

    int w = 0, h = 0;
    uint8_t *img = quirc_begin(g_qr, &w, &h);
    n6img_rgb888_to_gray(g_snapshot, img, w, h);
    quirc_end(g_qr);

    int kept = 0;
    const int count = quirc_count(g_qr);
    for (int i = 0; i < count && kept < MAX_RESULTS; i++) {
      quirc_extract(g_qr, i, &s_code);
      if (quirc_decode(&s_code, &s_data) != QUIRC_SUCCESS) continue;

      // Box around the four corners, as fractions of the square.
      int minx = s_code.corners[0].x, maxx = minx;
      int miny = s_code.corners[0].y, maxy = miny;
      for (int c = 1; c < 4; c++) {
        if (s_code.corners[c].x < minx) minx = s_code.corners[c].x;
        if (s_code.corners[c].x > maxx) maxx = s_code.corners[c].x;
        if (s_code.corners[c].y < miny) miny = s_code.corners[c].y;
        if (s_code.corners[c].y > maxy) maxy = s_code.corners[c].y;
      }
      const float bw = (float)(maxx - minx + 1);
      const float bh = (float)(maxy - miny + 1);
      const float cx = ((float)minx + bw * 0.5f) / (float)QR_DIM;
      const float cy = ((float)miny + bh * 0.5f) / (float)QR_DIM;

      // Extra: u8 version, u8 ECC level, u16 length, then the text.
      int tlen = s_data.payload_len;
      if (tlen > QR_MAX_TEXT) tlen = QR_MAX_TEXT;
      uint8_t extra[4 + QR_MAX_TEXT];
      extra[0] = (uint8_t)s_data.version;
      extra[1] = (uint8_t)(s_data.ecc_level & 3);
      vnd_meta_put_u16_le(&extra[2], (uint16_t)tlen);
      memcpy(&extra[4], s_data.payload, (size_t)tlen);

      char text[QR_MAX_TEXT + 1];
      memcpy(text, s_data.payload, (size_t)tlen);
      text[tlen] = '\0';

      char label[VND_VISION_LABEL_MAX + 1];
      strncpy(label, text, sizeof(label) - 1);
      label[sizeof(label) - 1] = '\0';

      (void)vnd_meta_vision_add(&vis, cx, cy, bw / (float)QR_DIM, bh / (float)QR_DIM,
                                label, extra, (uint16_t)(4 + tlen),
                                g_snapshot, QR_DIM, QR_DIM);
      kept++;

      const uint32_t now = millis();
      if (strcmp(text, g_last_text) != 0 || (now - g_last_print_ms) >= REPRINT_MS) {
        strcpy(g_last_text, text);
        g_last_print_ms = now;
        char line[QR_MAX_TEXT + 32];
        snprintf(line, sizeof(line), "[QR] v%d ecc=%c: %s",
                 s_data.version, "MLHQ"[s_data.ecc_level & 3], text);
        Serial.println(line);
        vnd_meta_publish_debug_text(line);
      }
    }
    if (kept > 0) g_frames_with_code++;
  }

  // An empty report clears the last box from the video.
  vnd_meta_vision_finish(&vis);

  const uint32_t now = millis();
  if (now - g_last_beat_ms >= 2000) {
    g_last_beat_ms = now;
    vnd_meta_vision_announce(VISION_KIND);
    char beat[64];
    snprintf(beat, sizeof(beat), "[QR] %lu frames, %lu with a code",
             (unsigned long)g_frames, (unsigned long)g_frames_with_code);
    Serial.println(beat);
    vnd_meta_publish_debug_text(beat);
    g_frames = 0;
    g_frames_with_code = 0;
  }
}

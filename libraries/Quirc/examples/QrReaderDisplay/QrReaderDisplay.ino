/*
  QR reader display

  Reads QR codes from the camera on the CPU and draws the decoded text on the
  display.

  Hardware: Neuro N6 with an OV5640 camera on the TFT carrier.

  This example code is in the public domain.
*/

#include <OV5640_Arduino.h>
#include <PostProcess.h>
#include <ohmlab_lcd_draw.h>
#include <quirc.h>

#define QR_DIM       256        // frame side handed to the decoder, pixels
#define QR_MAX_SHOW  512        // longest text kept from one code
#define PANEL_W      800
#define PANEL_H      480

OV5640 camera(CARRIER_TFT);

static struct quirc *g_qr = nullptr;

static struct quirc_code s_code;
static struct quirc_data s_data;

// The text shown, and when it was decoded.
static char     g_shown[QR_MAX_SHOW + 1] = {0};
static uint32_t g_shown_ms = 0;
static const uint32_t HOLD_MS = 4000;

void setup() {
  Serial.begin(115200);

  VisionConfig cfg;
  cfg.camera    = CAMERA_WVGA;
  cfg.fps       = FPS_30;
  cfg.transport = TRANSPORT_LCD;
  cfg.af        = AF_CONTINUOUS;
  Vision.begin(cfg);

  g_qr = quirc_new();
  if (g_qr == nullptr || quirc_resize(g_qr, QR_DIM, QR_DIM) < 0) {
    Serial.println("[QR] FATAL: could not allocate quirc buffers -- lower QR_DIM.");
    if (g_qr) { quirc_destroy(g_qr); g_qr = nullptr; }
  } else {
    Serial.print("[QR] ready -- quirc "); Serial.println(quirc_version());
  }
}

static void draw_overlay(const char *text) {
  uint16_t *buf = OVL_GetBackBuffer();
  if (buf == nullptr) return;
  ohm_lcd_bind_argb4444(buf, PANEL_W, PANEL_H, PANEL_W);
  ohm_lcd_clear_transparent();
  ohm_lcd_set_font(&Font16);
  ohm_lcd_text(16, 12, text);
  OVL_Commit();
}

void loop() {
  if (g_qr == nullptr) { delay(500); return; }

  // NULL on a carrier with no display.
  const uint16_t *frame = LCD_GetCameraBuffer();
  if (frame == nullptr) {
    static bool warned = false;
    if (!warned) { Serial.println("[QR] no LCD camera buffer -- need CARRIER_TFT. Idling."); warned = true; }
    delay(200);
    return;
  }

  const uint32_t cw = g_camera_full_w;
  const uint32_t ch = g_camera_full_h;

  // The largest centred square, scaled to QR_DIM, RGB565 to grey.
  const uint32_t side = (cw < ch) ? cw : ch;
  const uint32_t ox = (cw - side) / 2u;
  const uint32_t oy = (ch - side) / 2u;

  int w = 0, h = 0;
  uint8_t *img = quirc_begin(g_qr, &w, &h);
  for (int y = 0; y < h; y++) {
    const uint32_t sy = oy + (uint32_t)y * side / (uint32_t)h;
    const uint16_t *row = frame + (size_t)sy * cw;
    uint8_t *dst = img + (size_t)y * w;
    for (int x = 0; x < w; x++) {
      const uint32_t sx = ox + (uint32_t)x * side / (uint32_t)w;
      const uint16_t px = row[sx];
      const uint32_t r = ((px >> 11) & 0x1Fu) << 3;
      const uint32_t g = ((px >> 5)  & 0x3Fu) << 2;
      const uint32_t b = (px & 0x1Fu) << 3;
      dst[x] = (uint8_t)((r * 77u + g * 151u + b * 28u) >> 8);
    }
  }

  quirc_end(g_qr);

  bool decoded = false;
  int count = quirc_count(g_qr);
  for (int i = 0; i < count && !decoded; i++) {
    quirc_extract(g_qr, i, &s_code);
    if (quirc_decode(&s_code, &s_data) == QUIRC_SUCCESS) {
      int len = s_data.payload_len;
      if (len > QR_MAX_SHOW) len = QR_MAX_SHOW;
      memcpy(g_shown, s_data.payload, len);
      g_shown[len] = '\0';
      g_shown_ms = millis();
      decoded = true;
      Serial.print("[QR] "); Serial.println(g_shown);
    }
  }

  // The text stays on screen for HOLD_MS.
  if (g_shown[0] && (millis() - g_shown_ms) < HOLD_MS) {
    draw_overlay(g_shown);
  } else {
    g_shown[0] = '\0';
    draw_overlay("Point a QR code at the camera...");
  }

  delay(10);
}

/*
  Face recognition

  Finds faces with one network and identifies them with a second. A known
  face gets a green box, an unknown one a red box and a REMEMBER button that
  stores it until the next reset.

  Hardware: Neuro N6 on the TFT carrier with its touch display and an OV5640
  camera.

  This example code is in the public domain. The models it embeds are not:
  CenterFace is licensed under MIT and MobileFaceNet under Apache-2.0, and
  MobileFaceNet was trained on data licensed for non-commercial research only.
  See models/README.md before using this sketch in a product.
*/

#include <OV5640_Arduino.h>
#include <PostProcess.h>
#include <ohmlab_lcd_draw.h>
#include <Wire.h>
#include <GT911.h>
#include <n6_face.h>
#include <n6_face_bank.h>

#pragma neuron6 model="centerface-hwc.tflite"  name=face_det
#pragma neuron6 model="face4-int8.onnx"    name=face_emb

static const int   BANK_SLOTS      = 5;      // faces that can be stored
static const float MATCH_THRESHOLD = 0.45f;  // cosine similarity for a match
static const float DETECT_CONF     = 0.50f;  // detector score needed
static const int   MAX_SHOWN       = 4;      // faces identified per frame

// There is no liveness check: a photograph of a face matches that face.

OV5640 camera(CARRIER_TFT);
GT911  touch;
NEURON6_DECLARE_MODEL(face_det);
NEURON6_DECLARE_MODEL(face_emb);

// Stored faces, held in RAM only.
N6_FACE_BANK_STORAGE(bank_mem, BANK_SLOTS);
static n6_face_bank_t bank;

static const int PANEL_W = 800;
static const int PANEL_H = 480;

struct FaceResult {
  int   slot;      // bank slot, or -1 when unknown
  float sim;       // best match score
  bool  embedded;  // the embedder ran on it
};
static FaceResult results[N6_FACE_MAX];

static bool g_touch_ok = false;

// The REMEMBER button of a face: under its box, clamped to the panel.
static void button_rect(const n6_face_t *f, int *bx, int *by, int *bw, int *bh)
{
  const int x = n6_face_cam_to_lcd_x(f->cx - f->w * 0.5f);
  const int y = n6_face_cam_to_lcd_y(f->cy + f->h * 0.5f);
  int w = n6_face_cam_to_lcd_w(f->w);

  if (w < 130) w = 130;
  if (w > 260) w = 260;

  int bxx = x + n6_face_cam_to_lcd_w(f->w) / 2 - w / 2;
  if (bxx < 0) bxx = 0;
  if (bxx + w > PANEL_W) bxx = PANEL_W - w;

  const int h = 44;
  int byy = y + 6;
  if (byy + h > PANEL_H) byy = PANEL_H - h;

  *bx = bxx; *by = byy; *bw = w; *bh = h;
}

void setup() {
  Serial.begin(115200);

  VisionConfig cfg;
  cfg.camera    = CAMERA_WVGA;
  cfg.fps       = FPS_30;
  cfg.transport = TRANSPORT_LCD;
  cfg.af        = AF_CONTINUOUS;

  // The detector sees a centred square: faces at the far left and right of
  // the display are out of its view.
  cfg.nn.width  = 128;
  cfg.nn.height = 128;
  cfg.nn.aspect = ASPECT_CROP_CENTER;

  Vision.begin(cfg);                   // no model: loop() runs both

  n6_face_detector_set_threshold(DETECT_CONF);
  n6_face_bank_init(&bank, bank_mem, BANK_SLOTS);

  Wire.begin();
  g_touch_ok = touch.begin();

  Serial.println();
  Serial.println("=== face recognition : detect -> align -> embed -> match ===");
  Serial.print("bank slots ");        Serial.print(BANK_SLOTS);
  Serial.print(", match threshold ");  Serial.println(MATCH_THRESHOLD, 2);
  if (!g_touch_ok)
    Serial.println("touch: NOT FOUND - faces will be boxed but cannot be taught");
}

void loop() {
  if (g_touch_ok) touch.read();

  Vision.runWith(&NN_Instance_face_det, n6_face_detector(), OVL_RELOAD_CLEAR_NONE);

  int n = n6_face_count();
  if (n > MAX_SHOWN) n = MAX_SHOWN;

  for (int i = 0; i < n; i++) {
    const n6_face_t *f = n6_face_get(i);
    results[i].slot = -1;
    results[i].sim  = 0.0f;

    // The embedder's crop is aligned on the eyes. Head tilt is not corrected.
    const bool ran = Vision.runRoi(n6_face_align_roi(f), &NN_Instance_face_emb,
                                   n6_face_embedder(), OVL_RELOAD_CLEAR_NONE);
    results[i].embedded = ran && n6_face_embedding_valid();
    if (!results[i].embedded) continue;

    results[i].slot = n6_face_bank_match(&bank, n6_face_embedding(),
                                         MATCH_THRESHOLD, &results[i].sim);

    // Checked here: n6_face_embedding() holds only the most recent face.
    if (results[i].slot < 0 && g_touch_ok) {
      int bx, by, bw, bh;
      button_rect(f, &bx, &by, &bw, &bh);
      if (touch.clicked(bx, by, bw, bh)) {
        const int slot = n6_face_bank_add(&bank, n6_face_embedding());
        if (slot >= 0) {
          results[i].slot = slot;
          results[i].sim  = 1.0f;
          Serial.print("remembered face in slot "); Serial.println(slot);
        } else {
          Serial.println("bank full - press CLEAR to start over");
        }
      }
    }
  }

  uint16_t *ovl = OVL_GetBackBuffer();
  if (ovl == nullptr) return;

  ohm_lcd_bind_argb4444(ovl, PANEL_W, PANEL_H, PANEL_W);
  ohm_lcd_clear_transparent();
  ohm_lcd_set_font(&Font16);

  for (int i = 0; i < n; i++) {
    const n6_face_t *f = n6_face_get(i);
    const int x = n6_face_cam_to_lcd_x(f->cx - f->w * 0.5f);
    const int y = n6_face_cam_to_lcd_y(f->cy - f->h * 0.5f);
    const int w = n6_face_cam_to_lcd_w(f->w);
    const int h = n6_face_cam_to_lcd_h(f->h);

    char label[40];

    if (results[i].slot >= 0) {
      ohm_lcd_set_color(40, 230, 40);                     // green
      ohm_lcd_rect_thick(x, y, w, h, 3);
      snprintf(label, sizeof(label), "RECOGNISED #%d  %.2f",
               results[i].slot, results[i].sim);
      ohm_lcd_text(x, (y > 20) ? y - 20 : y + h + 4, label);
    } else {
      ohm_lcd_set_color(255, 40, 40);                     // red
      ohm_lcd_rect_thick(x, y, w, h, 3);
      if (results[i].embedded)
        snprintf(label, sizeof(label), "UNKNOWN  best %.2f", results[i].sim);
      else
        snprintf(label, sizeof(label), "UNKNOWN");
      ohm_lcd_text(x, (y > 20) ? y - 20 : y + h + 4, label);

      if (g_touch_ok && !n6_face_bank_full(&bank)) {
        int bx, by, bw, bh;
        button_rect(f, &bx, &by, &bw, &bh);
        const bool held = touch.pressed(bx, by, bw, bh);

        if (held) {
          ohm_lcd_set_color(255, 150, 0);
          ohm_lcd_fillrect(bx, by, bw, bh);
          ohm_lcd_set_color(0, 0, 0);
        } else {
          ohm_lcd_set_color(255, 200, 60);
          ohm_lcd_rect_thick(bx, by, bw, bh, 2);
        }
        ohm_lcd_text(bx + (bw - 8 * 11) / 2, by + 14, "REMEMBER");
      }
    }
  }

  ohm_lcd_set_color(255, 255, 255);
  char msg[64];
  // peak is the detector's best score this frame, above DETECT_CONF or not.
  snprintf(msg, sizeof(msg), "faces %d   peak %.2f   remembered %u/%d",
           n, (double)n6_face_last_peak(),
           (unsigned)n6_face_bank_used(&bank), BANK_SLOTS);
  ohm_lcd_text(12, 8, msg);

  if (g_touch_ok && n6_face_bank_used(&bank) > 0) {
    const int cx = PANEL_W - 130, cy = 6, cw = 118, ch = 38;
    if (touch.clicked(cx, cy, cw, ch)) {
      n6_face_bank_clear(&bank);
      Serial.println("bank cleared");
    }
    ohm_lcd_set_color(200, 200, 200);
    ohm_lcd_rect_thick(cx, cy, cw, ch, 2);
    ohm_lcd_text(cx + 26, cy + 11, "CLEAR");
  }

  OVL_Commit();

  static uint32_t last = 0;
  const uint32_t now = millis();
  if (now - last >= 1000u) {
    last = now;
    Serial.print("[face] detected="); Serial.print(n6_face_count());
    Serial.print(" peak=");           Serial.print(n6_face_last_peak(), 2);
    if (n > 0) {
      const n6_face_t *f = n6_face_get(0);
      Serial.print(" score=");  Serial.print(f->score, 2);
      Serial.print(" at ");     Serial.print(f->cx, 2);
      Serial.print(",");        Serial.print(f->cy, 2);
      Serial.print(" best=");   Serial.print(results[0].sim, 2);
      Serial.print(results[0].slot >= 0 ? " KNOWN" : " unknown");
    }
    Serial.print(" bank="); Serial.println(n6_face_bank_used(&bank));
  }
}

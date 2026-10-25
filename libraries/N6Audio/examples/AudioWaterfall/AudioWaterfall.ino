/*
  Audio waterfall

  Draws a scrolling spectrogram of the on-board microphone: frequency up the
  side, time across the bottom and colour for level.

  Hardware: Neuro N6 with the TFT carrier and its 800x480 display.

  This example code is in the public domain.
*/

#include <OV5640_Arduino.h>
#include <PostProcess.h>
#include <ohmlab_lcd_draw.h>
#include <string.h>
#include <Microphone.h>
#include <n6_fft.h>
#include <n6_spectrum.h>

static const int   NBANDS   = 64;
static const float F_LO     = 50.0f;      // bottom row, Hz
static const float F_HI     = 16000.0f;   // top row, Hz
static const float DB_FLOOR = -70.0f;     // quietest level shown, dB

// The camera object brings up the display carrier. The image is not used.
OV5640     camera(CARRIER_TFT);
Microphone mic;

static const int PANEL_W = 800;
static const int PANEL_H = 480;

static const int WF_X0   = 44;
static const int WF_COLS = 186;
static const int COL_W   = 4;
static const int WF_Y0   = 60;
static const int WF_Y1   = 440;
static const int WF_H    = WF_Y1 - WF_Y0;

static uint8_t        wf[WF_COLS][NBANDS];          // level history
static int            wf_head = 0;                  // newest column

static int16_t        chunk[N6_FFT_HOP];
static n6_frame_t     frame;
static float          mag[N6_FFT_BINS];
static n6_spectrum_t  spec;
static bool           micOK = false;

// y of the lower edge of band b. Low frequencies are at the bottom.
static inline int band_edge(int b)
{
  return WF_Y1 - (int)((float)b * (float)WF_H / (float)NBANDS + 0.5f);
}

// v from 0 to 1: black, blue, purple, red, orange, yellow, white.
static void heat(float v, uint8_t *r, uint8_t *g, uint8_t *b)
{
  static const uint8_t PAL[7][3] = {
    {   0,   0,   0 }, {  20,   0,  80 }, { 120,   0, 140 },
    { 220,  30,  40 }, { 255, 140,   0 }, { 255, 240,  60 },
    { 255, 255, 255 },
  };

  if (v < 0.0f) v = 0.0f;
  if (v > 1.0f) v = 1.0f;

  const float f = v * 6.0f;                 // 6 steps between 7 colours
  int i = (int)f;
  if (i > 5) i = 5;
  const float t = f - (float)i;

  *r = (uint8_t)(PAL[i][0] + (PAL[i + 1][0] - PAL[i][0]) * t);
  *g = (uint8_t)(PAL[i][1] + (PAL[i + 1][1] - PAL[i][1]) * t);
  *b = (uint8_t)(PAL[i][2] + (PAL[i + 1][2] - PAL[i][2]) * t);
}

void setup() {
  Serial.begin(115200);

  VisionConfig cfg;
  cfg.camera    = CAMERA_WVGA;
  cfg.fps       = FPS_30;
  cfg.transport = TRANSPORT_LCD;
  Vision.begin(cfg);

  micOK = mic.begin();
  if (micOK) {
    // mic.read() needs the stream to Neuro Studio off.
    mic.streamToNS(false);
    n6_fft_init();
    n6_frame_init(&frame);
    n6_spectrum_init(&spec, NBANDS, (float)mic.sampleRate(), F_LO, F_HI);
    spec.db_floor = DB_FLOOR;

    // 1.0 turns the smoothing over time off.
    spec.attack  = 1.0f;
    spec.release = 1.0f;
  }
  memset(wf, 0, sizeof(wf));

  Serial.println();
  Serial.println("=== audio waterfall ===");
  if (micOK) {
    Serial.print("mic ");       Serial.print(mic.sampleRate());
    Serial.print(" Hz, ");      Serial.print((float)mic.sampleRate() / N6_FFT_SIZE, 1);
    Serial.print(" Hz/bin, ");  Serial.print(NBANDS);
    Serial.print(" bands, ");   Serial.print(WF_COLS);
    Serial.print(" cols = ");
    Serial.print((float)WF_COLS * N6_FFT_HOP / mic.sampleRate(), 1);
    Serial.println(" s on screen");
  } else {
    Serial.println("microphone did not start (Tools > Debug console: On prints [MICDBG] details)");
  }
}

void loop() {
  if (!micOK) { delay(200); return; }

  if (mic.available() < N6_FFT_HOP) { delay(2); return; }
  // Each drawing waits for the panel, which is slower than the hops arrive.
  // Older hops only fill the window, so the picture keeps up with the sound.
  while (mic.available() >= 2u * N6_FFT_HOP) {
    (void)n6_frame_push(&frame, chunk, mic.read(chunk, N6_FFT_HOP));
  }
  const uint32_t got = mic.read(chunk, N6_FFT_HOP);
  if (got == 0) return;
  if (!n6_frame_push(&frame, chunk, got)) return;   // window still filling

  n6_fft_magnitude(n6_frame_data(&frame), mag);
  n6_spectrum_update(&spec, mag);

  wf_head = (wf_head + 1) % WF_COLS;
  for (int b = 0; b < NBANDS; b++) {
    float v = spec.bars[b];
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    wf[wf_head][b] = (uint8_t)(v * 255.0f);
  }

  uint16_t *ovl = OVL_GetBackBuffer();
  if (ovl == nullptr) return;

  ohm_lcd_bind_argb4444(ovl, PANEL_W, PANEL_H, PANEL_W);
  ohm_lcd_clear_transparent();
  ohm_lcd_set_font(&Font16);

  // Oldest column on the left: one past the newest, wrapping.
  for (int c = 0; c < WF_COLS; c++) {
    const int src = (wf_head + 1 + c) % WF_COLS;
    const int x   = WF_X0 + c * COL_W;

    for (int b = 0; b < NBANDS; b++) {
      const int y0 = band_edge(b + 1);
      const int y1 = band_edge(b);
      const int h  = y1 - y0;
      if (h <= 0) continue;

      uint8_t r, g, bl;
      heat((float)wf[src][b] * (1.0f / 255.0f), &r, &g, &bl);
      ohm_lcd_set_color(r, g, bl);
      ohm_lcd_fillrect(x, y0, COL_W, h);
    }
  }

  ohm_lcd_set_color(190, 190, 190);
  const float ticks[] = { 100.0f, 1000.0f, 10000.0f };
  const char *names[] = { "100", " 1k", "10k" };
  for (int t = 0; t < 3; t++) {
    for (int b = 0; b < NBANDS - 1; b++) {
      if (spec.centre_hz[b] <= ticks[t] && spec.centre_hz[b + 1] > ticks[t]) {
        const int y = band_edge(b);
        ohm_lcd_text(4, y - 8, names[t]);
        ohm_lcd_hline(WF_X0 - 4, y, 4);
        break;
      }
    }
  }

  char msg[80];
  ohm_lcd_set_color(255, 255, 255);
  ohm_lcd_text(16, 8, "AUDIO WATERFALL");

  const int loud = n6_spectrum_loudest(&spec);
  if (loud >= 0) snprintf(msg, sizeof(msg), "peak %5.0f Hz", (double)spec.centre_hz[loud]);
  else           snprintf(msg, sizeof(msg), "quiet");
  ohm_lcd_set_color(120, 220, 255);
  ohm_lcd_text(200, 8, msg);

  snprintf(msg, sizeof(msg), "%.1f s on screen   newest ->",
           (double)WF_COLS * N6_FFT_HOP / mic.sampleRate());
  ohm_lcd_set_color(150, 150, 150);
  ohm_lcd_text(430, 8, msg);

  ohm_lcd_set_color(120, 120, 120);
  ohm_lcd_hline(WF_X0, WF_Y1 + 2, WF_COLS * COL_W);
  ohm_lcd_text(WF_X0, WF_Y1 + 8, "older");
  ohm_lcd_text(WF_X0 + WF_COLS * COL_W - 66, WF_Y1 + 8, "newer");

  OVL_Commit();

  static uint32_t last = 0;
  const uint32_t now = millis();
  if (now - last >= 1000u) {
    Serial.print("level=");           Serial.print(mic.peak());
    if (loud >= 0) { Serial.print(" peak="); Serial.print(spec.centre_hz[loud], 0); Serial.print("Hz"); }
    Serial.println();
    last = now;
  }
}

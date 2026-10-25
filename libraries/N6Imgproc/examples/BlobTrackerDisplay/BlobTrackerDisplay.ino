/*
  Blob tracker display

  Follows the largest patch of a chosen colour in the camera picture, with no
  neural network, and draws a box and crosshair over it on the TFT carrier's
  display.

  Hardware: Neuro N6 with an OV5640 camera on the TFT carrier, the only
  carrier with the 800x480 display.

  This example code is in the public domain.
*/

#include <OV5640_Arduino.h>
#include <PostProcess.h>
#include <ohmlab_lcd_draw.h>

#define PANEL_W      800
#define PANEL_H      480

#define SCAN_STEP    2          // every second pixel, across and down

// The colour to look for. GATE_HUE 0 follows the largest strongly coloured
// patch of any colour.
#define GATE_HUE     1
#define TARGET_HUE   0     // degrees: 0 red, 60 yellow, 120 green, 240 blue
#define HUE_TOL      22    // degrees either side of that hue
#define SAT_MIN      55    // how colourful a pixel has to be, 0 to 255
#define VAL_MIN      50    // how bright a pixel has to be

// Matches are counted into a grid and the busiest connected patch is boxed.
#define GRID_W       40
#define GRID_H       30
#define MIN_CELL_HITS 2    // matches a cell needs to count
#define MIN_BLOB_PX  18    // matches the whole patch needs
#define MISS_HOLD    4     // empty frames the last box is kept for

OV5640 camera(CARRIER_TFT);

// Globals, to stay off the small stack that loop() runs on.
static uint16_t g_grid[GRID_W * GRID_H];
static uint16_t g_queue[GRID_W * GRID_H];
static uint8_t  g_visited[GRID_W * GRID_H];

// The smoothed box, in camera pixels.
static int  t_minx, t_miny, t_maxx, t_maxy, t_cx, t_cy;
static bool t_have = false;
static int  t_miss = 0;
static uint32_t g_last_beat_ms = 0;

static inline int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Hue in degrees.
static int rgb_hue(int r, int g, int b, int mx, int chroma) {
  int h;
  if (mx == r)      h = (60 * (g - b)) / chroma;
  else if (mx == g) h = 120 + (60 * (b - r)) / chroma;
  else              h = 240 + (60 * (r - g)) / chroma;
  if (h < 0) h += 360;
  return h;
}

static inline int cam_to_panel_x(int cx) {
  return g_lcd_camera_x + (int)((int32_t)cx * (int32_t)g_lcd_camera_w / (int32_t)g_camera_full_w);
}
static inline int cam_to_panel_y(int cy) {
  return g_lcd_camera_y + (int)((int32_t)cy * (int32_t)g_lcd_camera_h / (int32_t)g_camera_full_h);
}

void setup() {
  Serial.begin(115200);
  VisionConfig cfg;
  cfg.camera    = CAMERA_WVGA;    // 800x480, the size of the display
  cfg.fps       = FPS_30;
  cfg.transport = TRANSPORT_LCD;
  Vision.begin(cfg);
  Serial.print("[BLOB] LCD tracker ready -- gate=");
  Serial.println(GATE_HUE ? "hue" : "vivid");
}

static void draw_overlay(bool have, int px0, int py0, int pw, int ph,
                         int pcx, int pcy, const char *hud) {
  uint16_t *buf = OVL_GetBackBuffer();
  if (buf == nullptr) return;
  ohm_lcd_bind_argb4444(buf, PANEL_W, PANEL_H, PANEL_W);
  ohm_lcd_clear_transparent();
  ohm_lcd_set_font(&Font16);
  if (have) {
    ohm_lcd_rect_white((uint32_t)px0, (uint32_t)py0, (uint32_t)pw, (uint32_t)ph);
    const int cxp = clampi(pcx, 0, PANEL_W - 1);
    const int cyp = clampi(pcy, 0, PANEL_H - 1);
    ohm_lcd_hline_white((uint32_t)clampi(cxp - 10, 0, PANEL_W - 1), (uint32_t)cyp, 21);
    ohm_lcd_vline_white((uint32_t)cxp, (uint32_t)clampi(cyp - 10, 0, PANEL_H - 1), 21);
  }
  ohm_lcd_text(12, 10, hud);
  OVL_Commit();
}

void loop() {
  // The picture the display is showing. NULL on a carrier without a display,
  // or before the display has started.
  const uint16_t *frame = LCD_GetCameraBuffer();
  if (frame == nullptr) {
    static bool warned = false;
    if (!warned) { Serial.println("[BLOB] no LCD camera buffer -- need CARRIER_TFT. Idling."); warned = true; }
    delay(200);
    return;
  }

  const int cw = (int)g_camera_full_w;
  const int ch = (int)g_camera_full_h;

  memset(g_grid, 0, sizeof(g_grid));
  int max_chroma = 0, hue_at_max = -1;
  for (int y = 0; y < ch; y += SCAN_STEP) {
    const uint16_t *row = frame + (size_t)y * cw;
    const int gy = y * GRID_H / ch;
    for (int x = 0; x < cw; x += SCAN_STEP) {
      const uint16_t p = row[x];
      const int r = (int)(((p >> 11) & 0x1Fu) << 3);
      const int g = (int)(((p >> 5)  & 0x3Fu) << 2);
      const int b = (int)((p & 0x1Fu) << 3);
      const int mx = r > g ? (r > b ? r : b) : (g > b ? g : b);
      const int mn = r < g ? (r < b ? r : b) : (g < b ? g : b);
      const int chroma = mx - mn;
      if (chroma > max_chroma) { max_chroma = chroma; hue_at_max = chroma > 0 ? rgb_hue(r, g, b, mx, chroma) : -1; }
      if (mx < VAL_MIN || chroma < SAT_MIN) continue;
#if GATE_HUE
      const int h = rgb_hue(r, g, b, mx, chroma);
      int dh = h - TARGET_HUE; if (dh < 0) dh = -dh; if (dh > 180) dh = 360 - dh;
      if (dh > HUE_TOL) continue;
#endif
      const int gx = x * GRID_W / cw;
      g_grid[gy * GRID_W + gx]++;
    }
  }

  int peak = 0, peak_i = -1;
  for (int i = 0; i < GRID_W * GRID_H; i++) {
    if (g_grid[i] > peak) { peak = g_grid[i]; peak_i = i; }
  }

  bool found = false;
  int minx = 0, miny = 0, maxx = 0, maxy = 0;
  unsigned long sumc = 0, sumcx = 0, sumcy = 0;
  if (peak_i >= 0 && peak >= MIN_CELL_HITS) {
    const int thr = (peak / 3 > MIN_CELL_HITS) ? peak / 3 : MIN_CELL_HITS;  // matches a cell needs to join the patch
    memset(g_visited, 0, sizeof(g_visited));
    int head = 0, tail = 0;
    g_queue[tail++] = (uint16_t)peak_i;
    g_visited[peak_i] = 1;
    int cminx = GRID_W, cminy = GRID_H, cmaxx = -1, cmaxy = -1;
    while (head < tail) {
      const int ci = g_queue[head++];
      const int cx = ci % GRID_W, cy = ci / GRID_W;
      const unsigned long c = g_grid[ci];
      sumc += c;
      // The centre of the patch, weighted by matches, in camera pixels.
      sumcx += c * (unsigned long)(((2 * cx + 1) * cw) / (2 * GRID_W));
      sumcy += c * (unsigned long)(((2 * cy + 1) * ch) / (2 * GRID_H));
      if (cx < cminx) cminx = cx; if (cx > cmaxx) cmaxx = cx;
      if (cy < cminy) cminy = cy; if (cy > cmaxy) cmaxy = cy;
      const int nb[4] = { ci - 1, ci + 1, ci - GRID_W, ci + GRID_W };
      const bool ok[4] = { cx > 0, cx < GRID_W - 1, cy > 0, cy < GRID_H - 1 };
      for (int k = 0; k < 4; k++) {
        if (!ok[k]) continue;
        const int ni = nb[k];
        if (!g_visited[ni] && g_grid[ni] >= (uint16_t)thr) {
          g_visited[ni] = 1;
          g_queue[tail++] = (uint16_t)ni;
        }
      }
    }
    if (sumc >= MIN_BLOB_PX) {
      found = true;
      minx = cminx * cw / GRID_W;
      maxx = (cmaxx + 1) * cw / GRID_W - 1;
      miny = cminy * ch / GRID_H;
      maxy = (cmaxy + 1) * ch / GRID_H - 1;
    }
  }

  bool show;
  if (found) {
    const int cx = (int)(sumcx / sumc);
    const int cy = (int)(sumcy / sumc);
    if (t_have) {
      t_minx = (t_minx + minx) / 2; t_miny = (t_miny + miny) / 2;
      t_maxx = (t_maxx + maxx) / 2; t_maxy = (t_maxy + maxy) / 2;
      t_cx = (t_cx + cx) / 2;       t_cy = (t_cy + cy) / 2;
    } else {
      t_minx = minx; t_miny = miny; t_maxx = maxx; t_maxy = maxy; t_cx = cx; t_cy = cy;
    }
    t_have = true; t_miss = 0; show = true;
  } else {
    if (t_have && ++t_miss <= MISS_HOLD) { show = true; }
    else { t_have = false; show = false; }
  }

  char hud[64];
  if (show) {
    const int px0 = clampi(cam_to_panel_x(t_minx), 0, PANEL_W - 1);
    const int py0 = clampi(cam_to_panel_y(t_miny), 0, PANEL_H - 1);
    const int px1 = clampi(cam_to_panel_x(t_maxx), 0, PANEL_W - 1);
    const int py1 = clampi(cam_to_panel_y(t_maxy), 0, PANEL_H - 1);
    snprintf(hud, sizeof(hud), "blob px=%lu @(%d,%d)", sumc, t_cx, t_cy);
    draw_overlay(true, px0, py0, (px1 > px0 ? px1 - px0 : 1), (py1 > py0 ? py1 - py0 : 1),
                 cam_to_panel_x(t_cx), cam_to_panel_y(t_cy), hud);
  } else {
    snprintf(hud, sizeof(hud), "no blob  maxChroma=%d hue=%d (need>=%d)", max_chroma, hue_at_max, SAT_MIN);
    draw_overlay(false, 0, 0, 0, 0, 0, 0, hud);
  }

  const uint32_t now = millis();
  if (now - g_last_beat_ms >= 1000) { Serial.print("[BLOB] "); Serial.println(hud); g_last_beat_ms = now; }
  delay(10);
}

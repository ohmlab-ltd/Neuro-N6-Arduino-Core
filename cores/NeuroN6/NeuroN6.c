/**
  ******************************************************************************
  * @file           : NeuroN6.c
  * @brief          : NeuroN6 board support: camera, USB, LCD and NPU drivers
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025-2026 Ohm Lab Ltd.
  * SPDX-License-Identifier: MIT AND LicenseRef-ST-SLA0044
  *
  * Portions derived from STMicroelectronics example code,
  * licensed under the ST SLA0044 license.
  *
  ******************************************************************************
  */

#include "main.h"
#include "NeuroN6_app.h"
#include "../jpeg/jpegencapi.h"
#include "stm32n6xx_ll_venc.h"
#include "stm32n6xx_hal_dcmipp.h"
#include "tusb_config.h"
#include "tusb.h"
#include "ll_aton_rt_user_api.h"
#include "NeuroN6_FrameQueue.h"
#include "NeuroN6_Tune.h"
#include "n6_host_contract.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

/* Weak stubs. The PostProcess library supplies the strong ones. */
__attribute__((weak))
bool vnd_meta_get(const uint8_t **out_meta, uint32_t *out_len)
{
  *out_meta = NULL; *out_len = 0; return false;
}

__attribute__((weak))
bool vnd_meta_get_pingpong(uint32_t idx, const uint8_t **out_meta, uint32_t *out_len)
{
  (void)idx; *out_meta = NULL; *out_len = 0; return false;
}

__attribute__((weak))
void vnd_meta_init(uint8_t *meta_buf, uint32_t max_bytes)
{
  (void)meta_buf;
  (void)max_bytes;
}

__attribute__((weak))
void vnd_meta_invalidate(void) {}

__attribute__((weak))
uint32_t vnd_meta_pub_seq(void) { return 0u; }

__attribute__((weak))
uint32_t vnd_meta_thermal_pub_seq(void) { return 0u; }

/* slot is a vnd_meta_slot_t, declared here as uint32_t. */
__attribute__((weak))
bool vnd_meta_get_slot(uint32_t slot, const uint8_t **ptr, uint32_t *len)
{
  (void)slot; if (ptr) *ptr = NULL; if (len) *len = 0u; return false;
}

__attribute__((weak))
bool vnd_meta_publish_tlv(uint32_t slot, uint8_t type, const void *value, uint16_t len)
{
  (void)slot; (void)type; (void)value; (void)len; return false;
}

__attribute__((weak))
bool vnd_meta_publish_debug_text(const char *text) { (void)text; return false; }

/* Console line state: bytes since the last line ending, and the first of them. */
#if !defined(NEURON6_DEBUG_CONSOLE) || (NEURON6_DEBUG_CONSOLE != 0)
static uint8_t s_con_len = 0u;
static uint8_t s_con_key = 0u;
#endif

/* link_idle is int, not bool: basetype.h, included above, retypes bool. */
__attribute__((weak))
void vnd_meta_debug_mark_replay(void) {}
__attribute__((weak))
void vnd_meta_debug_replay_step(int link_idle) { (void)link_idle; }

/* Mirrors VND_META_SLOT_THERMAL in vnd_meta.h. */
#define N6_META_SLOT_THERMAL  7u

/* libraries/Microphone overrides these two. */
__attribute__((weak))
bool n6_audio_drain(const uint8_t **out_tlv, uint32_t *out_len)
{
  (void)out_tlv; (void)out_len; return false;
}

__attribute__((weak))
void n6_audio_debug_dump(void) {}

/* Called on the capture task for every encoded frame. Must not block. */
__attribute__((weak))
void n6_frame_tap(const uint8_t *jpeg, uint32_t jpeg_len,
                  const uint8_t *meta, uint32_t meta_len)
{
  (void)jpeg; (void)jpeg_len; (void)meta; (void)meta_len;
}

#define ALIGN_UP(x,a) (((x)+((a)-1U)) & ~((a)-1U))

#define DCACHE_ALIGN 32U

#define VND_MAGIC N6_FRAME_MAGIC

/* Crash record in the TAMP backup registers, reported on the next boot. */
#define N6_CRASH_MAGIC      0x4E36DEADu   /* BKP24R: record valid */
#define N6_CRASH_TAG_HAL    7u            /* Error_Handler. CPU faults use 1 to 6. */
#define N6_CRASH_TAG_CONFIGASSERT 9u      /* FreeRTOS configASSERT. Tag 8 is WDG-EWI. */

#define FORCEDFU 0xF4CDF4
/* BKP2R from the FSBL: a reset in the RTC second the app started in enters
 * DFU while it is set. */
#define N6_BOOT_PRESS_KEY 0x00FE4ADEu

typedef n6_frame_hdr_t vnd_frame_hdr_t;

static volatile uint8_t g_run_led_enabled = 1u;

static void n6_run_led_apply(void)
{
  if (g_run_led_enabled) {
    HAL_GPIO_WritePin(RED_LED_GPIO_Port,   RED_LED_Pin,   GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BLUE_LED_GPIO_Port,  BLUE_LED_Pin,  GPIO_PIN_RESET);
  } else {
    HAL_GPIO_WritePin(RED_LED_GPIO_Port,   RED_LED_Pin,   GPIO_PIN_SET);
    HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BLUE_LED_GPIO_Port,  BLUE_LED_Pin,  GPIO_PIN_SET);
  }
}

void NeuroN6_SetRunLedEnabled(bool enabled)
{
  g_run_led_enabled = enabled ? 1u : 0u;
  n6_run_led_apply();
}

bool NeuroN6_GetRunLedEnabled(void)
{
  return g_run_led_enabled != 0u;
}

static volatile bool     vnd_tx_active = false;
static vnd_frame_hdr_t   vnd_hdr;
static uint32_t          vnd_off = 0;
static uint32_t          vnd_total = 0;
/* Bytes still owed, as zeros, to the USB reader of a frame cut off when a
 * network host took the stream. */
static uint32_t          vnd_usb_owed = 0;

static const uint8_t*    vnd_meta = NULL;
static uint32_t          vnd_meta_len = 0;

static const uint8_t*    vnd_payload = NULL;
static uint32_t          vnd_payload_len = 0;

static uint32_t          vnd_frame_id = 0;

/* The attached network host, or NULL. Written only by n6_usb_task. */
static const n6_net_link_t *volatile s_net = NULL;

/* The SoftAP announce. It carries a passphrase, so it goes to USB only. */
static uint8_t s_softap_announce_buf[3U + 1U + 32U + 1U + 64U];

volatile int32_t cameraFrameReceived = 0;
volatile int32_t pipe1ready = 1;
/* Set by HAL_DCMIPP_PIPE_FrameEventCallback, read by the PIPE1 snapshot. */
volatile int32_t pipe1FrameReceived = 0;

static bool dcmipp_pipe1_csi_configured = false;
static bool dcmipp_pipe1_color_conv_configured = false;
static bool dcmipp_pipe1_pipe_configured = false;
static bool dcmipp_pipe2_configured = false;

DCMIPP_HandleTypeDef hdcmipp;
I2C_HandleTypeDef hi2c1;
LTDC_HandleTypeDef hltdc;
PCD_HandleTypeDef hpcd_USB_OTG_HS1;
RTC_HandleTypeDef hrtc;

uint32_t ts[2] = { 0 };  /* inference start and end ticks, read by PostProcess */

/* PSRAM buffer pool, in MPU Region 1, non-cacheable. */
extern uint8_t __ext_psram_pool_start[];   /* first free PSRAM byte */
extern uint8_t __ext_psram_pool_end[];     /* one past the PSRAM region end */

static uint8_t *g_psram_brk = 0;            /* bump pointer */

/* NPU working memory in PSRAM. neuron6_model_gen.py overrides this default. */
void neuron6_npu_psram_region(uint32_t *out_base, uint32_t *out_bytes);
__attribute__((weak))
void neuron6_npu_psram_region(uint32_t *out_base, uint32_t *out_bytes)
{
  if (out_base)  *out_base  = 0u;
  if (out_bytes) *out_bytes = 0u;
}

/* One past the last byte the pool may hand out. */
static uint8_t *psram_pool_limit(void)
{
  uint32_t npu_base = 0u, npu_bytes = 0u;
  neuron6_npu_psram_region(&npu_base, &npu_bytes);
  uint8_t *limit = __ext_psram_pool_end;
  if (npu_bytes != 0u && (uintptr_t)npu_base < (uintptr_t)limit) {
    limit = (uint8_t *)(uintptr_t)npu_base;
  }
  return limit;
}

/* Returns a 32-byte-aligned block, or 0 when the pool is exhausted. */
static void *psram_pool_alloc(uint32_t bytes)
{
  if (g_psram_brk == 0) g_psram_brk = __ext_psram_pool_start;
  uint8_t *p   = (uint8_t *)ALIGN_UP((uintptr_t)g_psram_brk, 32u);
  uint8_t *end = p + ALIGN_UP(bytes, 32u);
  if (end > psram_pool_limit()) return 0;
  g_psram_brk = end;
  return p;
}

/* For buffers handed to a DMA master: the malloc heap is cacheable. */
void *n6_psram_alloc(uint32_t bytes) { return psram_pool_alloc(bytes); }

/* Non-cacheable AXISRAM pool, MPU Region 3, carved down from the MSP stack reserve. */
volatile uint32_t g_axisram_heap_ceiling = 0u;   /* read by _sbrk, 0 for the default ceiling */
static uint8_t   *g_axisram_pool_base = 0;        /* lowest carved byte, 0 when nothing is carved */

/* Sets one MPU region non-cacheable. */
static void mpu_set_nc_region(uint32_t region, uint32_t base, uint32_t limit_incl)
{
  MPU_Region_InitTypeDef rc = {0};
  uint32_t pm = __get_PRIMASK(); __disable_irq();
  HAL_MPU_Disable();
  rc.Enable           = MPU_REGION_ENABLE;
  rc.Number           = region;
  rc.BaseAddress      = base;
  rc.LimitAddress     = limit_incl;
  rc.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;
  rc.AccessPermission = MPU_REGION_ALL_RW;
  rc.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
  rc.AttributesIndex  = MPU_ATTRIBUTES_NUMBER0;   /* non-cacheable, set in MPU_Config */
  HAL_MPU_ConfigRegion(&rc);
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
  __set_PRIMASK(pm);
}

extern void *_sbrk(ptrdiff_t incr);   /* _sbrk(0) is the current break */

/* Returns a 32-byte-aligned block, or 0 when it would reach the heap. */
static void *axisram_carve_alloc(uint32_t bytes)
{
  extern uint8_t  _estack;           /* top of RAM */
  extern uint32_t _Min_Stack_Size;   /* MSP stack reserve */
  if (bytes == 0u) return 0;
  const uint32_t pool_top = (uint32_t)&_estack - (uint32_t)&_Min_Stack_Size;
  const uint32_t cur_base = g_axisram_pool_base ? (uint32_t)g_axisram_pool_base : pool_top;
  uint32_t new_base = (cur_base - bytes) & ~31u;
  uint32_t heap_now = (uint32_t)_sbrk(0);
  if (new_base < heap_now) return 0;
  g_axisram_heap_ceiling = new_base;
  mpu_set_nc_region(MPU_REGION_NUMBER3, new_base, pool_top - 1u);
  SCB_CleanInvalidateDCache_by_Addr((void *)new_base, (int32_t)(cur_base - new_base));
  g_axisram_pool_base = (uint8_t *)new_base;
  return (void *)new_base;
}

/* USB frame size, set by DCMIPP_USB_Init and Vision.begin. */
uint32_t usb_frame_width = 800;
uint32_t usb_frame_height = 480;

/* The camera image's window on the panel, LTDC Layer 0. */
int16_t  g_lcd_camera_x = 0;
int16_t  g_lcd_camera_y = 0;
uint16_t g_lcd_camera_w = 800;
uint16_t g_lcd_camera_h = 480;

/* The camera pixels the NN input covers. Set by DCMIPP_NN_InitWithCrop and
 * DCMIPP_NN_InitLetterbox. */
int16_t  g_nn_crop_x = 0;
int16_t  g_nn_crop_y = 0;
uint16_t g_nn_crop_w = 800;
uint16_t g_nn_crop_h = 480;

/* NN input size. Zero until the NN pipe is configured. */
uint16_t g_nn_out_w     = 0;
uint16_t g_nn_out_h     = 0;
uint8_t  g_nn_pipe_ready = 0u;

/* Letterbox: where PIPE2's picture sits in the NN input. s_nn_pic_w is 0 when
 * the picture fills the input. */
static uint16_t s_nn_pic_x = 0u;
static uint16_t s_nn_pic_y = 0u;
static uint16_t s_nn_pic_w = 0u;
static uint16_t s_nn_pic_h = 0u;
#define N6_LETTERBOX_GREY  114u

/* Set by LCD_feed_start. */
static uint8_t g_lcd_feed_active = 0u;

/* 16-bit copy of usb_frame_width and usb_frame_height. */
uint16_t g_camera_full_w = 800;
uint16_t g_camera_full_h = 480;

/* JPEG output, g_mjpeg_max bytes each. Frame N is encoded into buf[N&1]. */
static uint8_t *mjpeg_buf0 = 0;
static uint8_t *mjpeg_buf1 = 0;
static uint32_t g_mjpeg_max = 0;

#define LCD_W 800
#define LCD_H 480
#define LCD_SIZE    (LCD_W * LCD_H * 2)
#define OVL_BYTES   (800 * 480 * 2u)   /* ARGB4444, 2 bytes per pixel */

/* LCD frame buffer. PIPE0 writes it, LTDC scans it out. */
uint8_t  *pipe2buf     = 0;
/* Toggled by console key 'o'. */
volatile uint8_t g_lcd_overlay_on = 1u;
/* Toggled by console key 'i'. 0 skips the NPU compute only. */
volatile uint8_t g_nn_inference_on = 1u;
static uint16_t *overlay_buf0 = 0;
static uint16_t *overlay_buf1 = 0;

static uint16_t *g_ovl_front = 0;
static uint16_t *g_ovl_back  = 0;

static void dbg_print(const char *msg);
static void dbg_detail(const char *msg);

/* Line-buffer JPEG path, DCMIPP to VENC, with venc_rdy driven by the DCMIPP LINE
 * event. imageEncode.c externs the band and the producer counter. */
volatile uint8_t  g_venc_lb_hw = 0u;   /* 1: hardware handshake, VENC 0x7C4 bit29 */
/* 1 while the sensor streams. The capture task arms the pipe only then. */
volatile uint8_t  g_n6_sensor_streaming = 1u;
/* Gates PIPE1 and the VENC, 0 for TRANSPORT_LCD. Set before DCMIPP_USB_Init. */
volatile uint8_t  g_n6_usb_enabled = 1u;
/* DCMIPP config held for a sensor that defers streaming. */
n6_deferred_cfg_t g_n6_deferred_cfg = {0};
uint8_t  *g_venc_band       = NULL;   /* band base */
uint32_t  g_venc_band_size  = 0u;
uint32_t  g_venc_band_pitch = 0u;     /* bytes per row */
uint32_t  g_venc_band_depth = 0u;     /* MB-lines per half: 2 or 4 */
/* Byte ceiling for a depth-4 band. */
#define N6_VENC_BAND_MAX  491520u
volatile uint32_t g_dcmipp_wr_mblines = 0u;   /* MB-lines the DCMIPP has written this frame */
/* ms between the last two PIPE1 frame ends, 0 until two are seen. */
static volatile uint32_t s_pipe1_frame_ms    = 0u;
static volatile uint32_t s_pipe1_last_end_ms = 0u;
uint32_t  g_venc_total_mblines = 0u;          /* MB-rows of the coded height, which is floored to 8 */
volatile uint32_t g_dbg_lb_cb_calls = 0u;     /* cbFunc calls */
volatile int32_t  g_dbg_lb_init_ret = 0;      /* last line-buffer JpegEncInit return, 0 is OK */
volatile int32_t  g_dbg_lb_start_ret = 0;     /* last LINEBUF Start return, HAL_OK is 0 */
volatile int32_t  g_dbg_lb_enc_rv    = 0;     /* last LINEBUF JpegEncEncode return, 1 is FRAME_READY */
volatile uint32_t g_dbg_lb_rd_max    = 0u;    /* max VENC rdPtr, 0x7C4[17:9], seen mid-encode */
volatile uint32_t g_dbg_lb_wr_max    = 0u;    /* max g_dcmipp_wr_mblines seen alongside rdPtr */
volatile uint32_t g_dbg_lb_gap_max   = 0u;    /* max producer lead over rdPtr, in MB-rows */
volatile uint8_t  g_venc_lb_source_arm = 0u;  /* 1: EWLWaitHwRdy starts the DCMIPP */
volatile uint8_t  g_lb_dcmipp_continuous = 0u; /* 0: the next capture restarts the DCMIPP */
volatile uint8_t  g_lb_venc_reset_pending = 0u; /* the next start runs the ES0620 VENC reset */
/* VENC 0x7C4 after a LINEBUF encode: [27]LowLatencyEn [28]LoopBack [29]HwIntf
 * [26:18]depth [17:9]rdPtr [8:0]wrPtr */
volatile uint32_t g_dbg_lb_instant   = 0u;

/* Called once, before the DCMIPP pipe inits. Returns -1 when the JPEG or LCD
 * buffers do not fit. Without room for the band no stream runs. Blocks a
 * sketch took with n6_psram_alloc() before this stay its own. */
int n6_alloc_camera_buffers(uint32_t cam_w, uint32_t cam_h, int lcd_enabled,
                            n6_mem_loc_t band_loc, n6_mem_loc_t framebuf_loc)
{
  g_axisram_pool_base = 0;
  const int band_needed = (g_n6_usb_enabled != 0u);

  {
    uint32_t npu_base = 0u, npu_bytes = 0u;
    neuron6_npu_psram_region(&npu_base, &npu_bytes);
    if (npu_bytes != 0u) {
      const uint8_t *limit = psram_pool_limit();
      const uint32_t free_kb = (limit > __ext_psram_pool_start)
          ? (uint32_t)((limit - __ext_psram_pool_start) / 1024) : 0u;
      char nb[160];
      snprintf(nb, sizeof(nb),
        "[NPU] a model keeps %lu KB of working memory in PSRAM at %08lX; "
        "camera and event buffers get the %lu KB below it\r\n",
        (unsigned long)(npu_bytes / 1024u), (unsigned long)npu_base, (unsigned long)free_kb);
      dbg_print(nb);
    }
  }

  uint32_t jpeg_sz = (cam_w * cam_h) / 4u;
  if (jpeg_sz < (512u * 1024u)) jpeg_sz = 512u * 1024u;
  mjpeg_buf0  = (uint8_t *)psram_pool_alloc(jpeg_sz);
  g_mjpeg_max = (mjpeg_buf0 != 0) ? jpeg_sz : 0u;
  mjpeg_buf1  = (uint8_t *)psram_pool_alloc(jpeg_sz);   /* may be NULL: single-buffered */
  {
    char jb[96];
    snprintf(jb, sizeof(jb), "[JPGBUF] mjpeg0=%08lX mjpeg1=%08lX gmax=%lu (0x70=PSRAM)\r\n",
      (unsigned long)(uint32_t)mjpeg_buf0, (unsigned long)(uint32_t)mjpeg_buf1, (unsigned long)g_mjpeg_max);
    dbg_detail(jb);
  }

  {
    uint32_t mbW   = (cam_w + 15u)/16u;
    /* Band depth: 32-line chunks the DCMIPP buffers ahead of the VENC reader. An
     * odd mbW and tall RAW8 need 4. */
    uint32_t pitch = ((cam_w * 2u) + 15u) & ~15u;
    uint32_t depth = ((pitch * (4u * 16u * 2u)) <= N6_VENC_BAND_MAX) ? 4u : 2u;
    if ((mbW & 1u) != 0u ||
        (g_n6_sensor_format.csi_data_type == DCMIPP_DT_RAW8 && cam_h > 1024u)) {
      depth = 4u;
    }
    /* WVGA and VGA use depth 2. */
    if (cam_h == 480u && (cam_w == 800u || cam_w == 640u)) {
      depth = 2u;
    }
    uint32_t bsize = pitch * (depth * 16u * 2u);
    g_venc_band_pitch    = pitch;
    g_venc_band_depth    = depth;
    g_venc_total_mblines = ((cam_h & ~7u) + 15u)/16u;   /* the encoder floors the height to 8 */
    g_venc_band      = NULL;
    g_venc_band_size = 0u;
    if (band_needed) {
      if (band_loc != N6_MEM_PSRAM) {
        g_venc_band = (uint8_t *)axisram_carve_alloc(bsize);
      }
      if (g_venc_band == NULL && band_loc != N6_MEM_AXISRAM) {
        g_venc_band = (uint8_t *)psram_pool_alloc(bsize);
      }
      g_venc_band_size = g_venc_band ? bsize : 0u;
    }
    {
      char lbi[128];
      snprintf(lbi, sizeof(lbi), "[LBINIT] band=%lu addr=0x%08lX depth=%lu pitch=%lu (%s)\r\n",
        (unsigned long)g_venc_band_size, (unsigned long)(uint32_t)g_venc_band,
        (unsigned long)depth, (unsigned long)pitch,
        g_venc_band ? (((uint32_t)g_venc_band >= 0x70000000u) ? "PSRAM" : "AXISRAM") : "none");
      dbg_detail(lbi);
    }
  }

  if (lcd_enabled) {
    pipe2buf = 0;
    if (framebuf_loc != N6_MEM_PSRAM) {
      pipe2buf = (uint8_t *)axisram_carve_alloc(LCD_SIZE);
    }
    if (pipe2buf == 0 && framebuf_loc != N6_MEM_AXISRAM) {
      pipe2buf = (uint8_t *)psram_pool_alloc(LCD_SIZE);
    }
    overlay_buf0 = (uint16_t *)psram_pool_alloc(OVL_BYTES);
    overlay_buf1 = (uint16_t *)psram_pool_alloc(OVL_BYTES);
    g_ovl_front  = overlay_buf0;
    g_ovl_back   = overlay_buf1;
    {
      char pb[96];
      snprintf(pb, sizeof(pb), "[LCDBUF] pipe2buf=%08lX ovl=%08lX (%s)\r\n",
        (unsigned long)(uint32_t)pipe2buf,
        (unsigned long)(uint32_t)overlay_buf0,
        pipe2buf ? (((uint32_t)pipe2buf >= 0x70000000u) ? "PSRAM" : "AXISRAM") : "none");
      dbg_detail(pb);
    }
  } else {
    pipe2buf = 0;
    overlay_buf0 = 0;
    overlay_buf1 = 0;
    g_ovl_front = 0;
    g_ovl_back  = 0;
  }

  const int lcd_ok = !lcd_enabled || (pipe2buf && overlay_buf0 && overlay_buf1);
  if (!mjpeg_buf0 || !lcd_ok) {
    return -1;
  }
  return 0;
}

static inline uint16_t ARGB4444(uint8_t a4, uint8_t r4, uint8_t g4, uint8_t b4)
{
   return (uint16_t)(((a4 & 0xF) << 12) | ((r4 & 0xF) << 8) | ((g4 & 0xF) << 4) | (b4 & 0xF));
}

/* Debug counters, printed in the [CAMDBG] line. */
static volatile uint32_t g_dbg_pipe1_frames  = 0;  /* PIPE1 frame-complete callbacks */
static volatile uint32_t g_dbg_encode_ok     = 0;  /* frames encoded and posted */
static volatile uint32_t g_dbg_encode_fail   = 0;  /* encodes that returned no usable frame */
static volatile uint32_t g_dbg_tx_started    = 0;  /* frames that took the transmitter */
static volatile uint32_t g_dbg_pipe1_err     = 0;  /* DCMIPP PIPE1 error IRQs */
static volatile uint32_t g_dbg_dcmipp_err    = 0;  /* general DCMIPP error IRQs */
static volatile uint32_t g_dbg_csi_line_err  = 0;  /* CSI D-PHY line-error IRQs */
static volatile uint32_t g_dbg_last_err_code = 0;  /* last hdcmipp.ErrorCode */
static volatile uint32_t g_dbg_last_jpeg_sz  = 0;  /* bytes in the last JPEG */
static volatile uint32_t g_dbg_last_enc_ms   = 0;  /* ms taken by the last encode */
volatile int32_t g_dbg_ff_init_ret = 0;  /* full-frame JpegEncInit/SetPictureSize return, 0 is OK */
/* VENC probes, armed by console key 'e'. */
volatile int32_t  g_dbg_ff_jeinit   = 0;  /* JpegEncInit() return */
volatile int32_t  g_dbg_ff_jesps    = 0;  /* JpegEncSetPictureSize() return */
volatile uint32_t g_dbg_swreg_lo[64];     /* VENC swreg[0..63] */
volatile uint32_t g_dbg_swreg_hi[16];     /* VENC swreg[290..305] */
volatile uint8_t  g_dbg_venc_arm    = 0;  /* the next encode captures the swregs */
volatile uint8_t  g_dbg_venc_print  = 0;  /* a capture is ready to print */
volatile uint32_t g_dbg_cmsr2_sticky = 0; /* OR of DCMIPP->CMSR2 as read in the error IRQs */

/* The producer encodes only while g_enc_seq - g_tx_seq < 2. Each has one writer. */
static volatile uint32_t g_enc_seq = 0;
static volatile uint32_t g_tx_seq  = 0;

/* Releases a frame's buffer. A consumer calls it once per frame. */
void n6_video_tx_complete(void) { g_tx_seq++; }

static volatile uint32_t g_dbg_pipe1_line_events = 0;

static inline bool usb_ready_to_tx(void);
static inline void vnd_tx_reset(void);
static void reboot_to_bootloader(void);
static inline bool vnd_tx_start_frame(const uint8_t* meta, uint32_t meta_len,
    const uint8_t* payload, uint32_t payload_len, uint32_t flags);
static inline void vnd_tx_pump(void);
static void n6_host_command(const uint8_t *cmd_buf, uint32_t count, bool from_usb);
static void n6_net_detach(void);

/* imageEncode.c. encoder_jpeg_init returns 1 on success, 0 on failure. */
uint8_t encoder_jpeg_init(uint32_t type, int32_t width, int32_t height, void *pOut, uint32_t szOut);
void encoder_jpeg_set_output(void *pOut);
void encoder_jpeg_end(void);
void encoder_jpeg_caps(uint32_t out[8]);
int  encoder_jpeg_band_probe(uint32_t width, uint32_t wrap_lines, uint32_t *band_out, void **addr_out);
void     encoder_jpeg_linebuf_ensure_init(int32_t width, int32_t height, uint32_t szOut);
void     encoder_jpeg_linebuf_begin_frame(void);
uint32_t encode_jpeg_frame_linebuf(void);
void     encoder_jpeg_venc_errata_reset(void);   /* ES0620 VENC reset */

static int  dbg_say(const char *msg);
void NeuroN6_Notice(const char *msg);   /* NeuroN6.h */

/* CDC connect edges, counted by the USB task. A one-off message records the
 * connection it last went to, N6_SAID_NEVER before the first. */
#define N6_SAID_NEVER 0xFFFFFFFFu
static volatile uint32_t s_cdc_connects = 0u;
static volatile uint8_t  s_cdc_up = 0u;
static void dbg_say_to_host(uint32_t *said_on, const char *msg);

static void CSI_Init(void);

static void NeuralNetwork_init(void);
static void Run_Inference(NN_Instance_TypeDef *instance);
static void NPUCache_config(void);
static void NPURam_enable(void);
static void NPU_NVIC_Config(void);

static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
void MX_USB1_OTG_HS_PCD_Init(void);
static void SystemIsolation_Config(void);
static void Security_Config(void);
static void IAC_Config(void);

/* USB vendor functions */

/* Mounted and not suspended. */
static inline bool usb_ready_to_tx(void)
{
  return tud_ready();
}

static inline void vnd_tx_reset(void)
{
  vnd_tx_active = false;
  vnd_off = 0;
  vnd_total = 0;
  vnd_meta = NULL;
  vnd_meta_len = 0;
  vnd_payload = NULL;
  vnd_payload_len = 0;
}

/* Called when USB mounts, or a network host attaches or leaves. */
static void n6_host_session_start(void)
{
  vnd_tx_reset();
  vnd_frame_id = 0;
  /* Each dropped JPEG gives its ping-pong slot back. */
  n6_frame_msg_t stale;
  while (g_n6_video_usb_queue != NULL &&
         xQueueReceive(g_n6_video_usb_queue, &stale, 0) == pdTRUE) {
    if (stale.jpeg_len > 0u) n6_video_tx_complete();
  }
  vnd_meta_invalidate();
  vnd_meta_debug_mark_replay();
}

/* Both mount callbacks run in n6_usb_task, or before it exists. A new reader is
 * owed nothing. */
void tud_mount_cb(void)
{
  vnd_usb_owed = 0u;
  if (s_net == NULL) n6_host_session_start();
}

void tud_umount_cb(void)
{
  vnd_usb_owed = 0u;
  if (s_net == NULL) vnd_tx_reset();
}

static void reboot_to_bootloader(void)
{
  /* The FSBL tests BKP2R for FORCEDFU. BKP1R is the double-tap timestamp. An
   * upload cancels an update the sketch staged and has not applied: BKP20R
   * holds the OTA trigger and BKP16R the asset trigger (n6_ota_layout.h). */
  HAL_PWR_EnableBkUpAccess();
  TAMP->BKP20R = 0;
  TAMP->BKP16R = 0;
  TAMP->BKP2R = FORCEDFU;
  TAMP->BKP1R = 0;
  __DSB();
  __ISB();
  NVIC_SystemReset();
}

void n6_boot_press_clear(void)
{
  PWR->DBPCR |= PWR_DBPCR_DBP;
  if (TAMP->BKP2R == N6_BOOT_PRESS_KEY) TAMP->BKP2R = 0u;
  __DSB();
}

int n6_boot_dfu_pending(void)
{
  return (TAMP->BKP2R == FORCEDFU) ? 1 : 0;
}

/* VisionConfig.isp_enabled, an IspTuningMode. */
volatile uint8_t g_isp_tuning_mode = 0u;

void *n6_dcmipp_handle(void) { return &hdcmipp; }

void n6_dcmipp_prepare_start(uint32_t pipe)
{
  if (pipe == DCMIPP_PIPE0) {
    CLEAR_BIT(DCMIPP->P0FCTCR, DCMIPP_P0FCTCR_CPTMODE);
    __HAL_DCMIPP_CLEAR_FLAG(&hdcmipp, DCMIPP_FLAG_PIPE0_FRAME | DCMIPP_FLAG_PIPE0_VSYNC |
                                      DCMIPP_FLAG_PIPE0_LINE | DCMIPP_FLAG_PIPE0_LIMIT |
                                      DCMIPP_FLAG_PIPE0_OVR);
  } else if (pipe == DCMIPP_PIPE1) {
    CLEAR_BIT(DCMIPP->P1FCTCR, DCMIPP_P1FCTCR_CPTMODE);
    __HAL_DCMIPP_CLEAR_FLAG(&hdcmipp, DCMIPP_FLAG_PIPE1_FRAME | DCMIPP_FLAG_PIPE1_VSYNC |
                                      DCMIPP_FLAG_PIPE1_LINE | DCMIPP_FLAG_PIPE1_OVR);
  } else {
    CLEAR_BIT(DCMIPP->P2FCTCR, DCMIPP_P2FCTCR_CPTMODE);
    __HAL_DCMIPP_CLEAR_FLAG(&hdcmipp, DCMIPP_FLAG_PIPE2_FRAME | DCMIPP_FLAG_PIPE2_VSYNC |
                                      DCMIPP_FLAG_PIPE2_LINE | DCMIPP_FLAG_PIPE2_OVR);
  }
}

static void n6_isp_apply_isp0(const uint8_t *p, uint32_t len);

/* Asset transfer. The hooks are weak: the N6Assets library supplies them. */
__attribute__((weak)) bool n6_asset_stage(const void *d, uint32_t l, uint32_t v)
{ (void)d; (void)l; (void)v; return false; }
__attribute__((weak)) void n6_asset_apply(void) { }
__attribute__((weak)) bool n6_asset_record_stage(uint8_t t, const void *d, uint32_t l)
{ (void)t; (void)d; (void)l; return false; }

#define N6_ASSET_RX_MAX 4076u
static uint8_t  g_ast_buf[N6_ASSET_RX_MAX];
static uint32_t g_ast_len = 0u, g_ast_crc = 0u, g_ast_ver = 0u, g_ast_got = 0u;
static bool     g_ast_active = false;
static uint8_t  g_ast_record = 0u;   /* 0 for the asset, else the record type */

/* Evidence subscription */
static volatile uint32_t g_evidence_mask     = 0u;
static volatile uint32_t g_evidence_expires  = 0u;   /* HAL tick, ms */

bool n6_host_evidence_wanted(uint32_t bit)
{
  if ((g_evidence_mask & bit) == 0u)          return false;
  if (s_net == NULL && !usb_ready_to_tx())    return false;
  /* Wrap-safe: the difference stays negative until the deadline passes. */
  return (int32_t)(HAL_GetTick() - g_evidence_expires) < 0;
}

void tud_vendor_rx_cb(uint8_t idx, const uint8_t *buffer, uint32_t bufsize)
{
  (void)idx;

  uint8_t cmd_buf[N6_HOST_CMD_MAX];
  uint32_t count = 0;

  if (buffer != NULL) {
    count = (bufsize < sizeof(cmd_buf)) ? bufsize : sizeof(cmd_buf);
    memcpy(cmd_buf, buffer, count);
    if (count != 0) n6_host_command(cmd_buf, count, true);
    return;
  }

  /* The FIFO holds one host write: TinyUSB re-arms the OUT endpoint after
   * this callback. A write is one command; the rest is dropped, since bytes
   * left behind would stop the re-arm once a packet's worth builds up. */
  count = tud_vendor_n_read(0, cmd_buf, sizeof(cmd_buf));
  tud_vendor_n_read_flush(0);
  if (count != 0) n6_host_command(cmd_buf, count, true);
}

/* Applies one host command. Runs in n6_usb_task. */
static void n6_host_command(const uint8_t *cmd_buf, uint32_t count, bool from_usb)
{
  if (count >= 4u) {
    uint32_t magic = (uint32_t)cmd_buf[0] | ((uint32_t)cmd_buf[1] << 8) |
                     ((uint32_t)cmd_buf[2] << 16) | ((uint32_t)cmd_buf[3] << 24);
    if (magic == N6_CMD_ISP0 && count >= N6_CMD_ISP0_MIN_BYTES) {
      n6_isp_apply_isp0(cmd_buf, count);
      return;
    }
    if (magic == N6_CMD_CAM0 && count >= sizeof(n6_cmd_cam0_t)) {
      const uint8_t  flags       = cmd_buf[4];
      const uint32_t exposure_us = (uint32_t)cmd_buf[5]        |
                                   ((uint32_t)cmd_buf[6] << 8)  |
                                   ((uint32_t)cmd_buf[7] << 16) |
                                   ((uint32_t)cmd_buf[8] << 24);
      const uint16_t gain_x256   = (uint16_t)((uint16_t)cmd_buf[9] |
                                              ((uint16_t)cmd_buf[10] << 8));
      const uint8_t  apply_mask  = cmd_buf[11];
      /* Non-blocking: this task feeds the watchdog. The host re-sends CAM0. */
      if (n6_i2c_trylock()) {
        (void)n6_sensor_apply_cam0(flags, exposure_us, gain_x256, apply_mask);
        n6_i2c_unlock();
      }
      return;
    }
    /* TUN0: u32 magic, u8 id, u8 type, i32 value, milli-units for a float. */
    if (magic == N6_CMD_TUN0 && count >= sizeof(n6_cmd_tun0_t)) {
      const uint8_t id = cmd_buf[4];
      const int32_t value = (int32_t)((uint32_t)cmd_buf[6]         |
                                      ((uint32_t)cmd_buf[7]  << 8)  |
                                      ((uint32_t)cmd_buf[8]  << 16) |
                                      ((uint32_t)cmd_buf[9]  << 24));
      (void)n6_tune_apply(id, value);
      return;
    }

    if (magic == N6_CMD_SUB0 && count >= sizeof(n6_cmd_sub0_t)) {
      const uint32_t mask = (uint32_t)cmd_buf[4]         |
                            ((uint32_t)cmd_buf[5] << 8)  |
                            ((uint32_t)cmd_buf[6] << 16) |
                            ((uint32_t)cmd_buf[7] << 24);
      g_evidence_mask    = mask;
      g_evidence_expires = HAL_GetTick() + N6_HOST_SUB_TTL_MS;
      return;
    }

    if (!from_usb) return;

    if (magic == N6_CMD_AST0 && count >= sizeof(n6_cmd_ast0_t)) {
      n6_cmd_ast0_t c0;
      memcpy(&c0, cmd_buf, sizeof(c0));
      g_ast_active = (c0.total_len > 0u && c0.total_len <= N6_ASSET_RX_MAX);
      g_ast_len = c0.total_len; g_ast_crc = c0.crc32; g_ast_ver = c0.version;
      g_ast_got = 0u;
      g_ast_record = 0u;
      return;
    }
    /* A zero-length record transfer removes the record. */
    if (magic == N6_CMD_ASR0 && count >= sizeof(n6_cmd_asr0_t)) {
      n6_cmd_asr0_t r0;
      memcpy(&r0, cmd_buf, sizeof(r0));
      g_ast_active = (r0.type != 0u && r0.total_len <= N6_ASSET_RX_MAX);
      g_ast_len = r0.total_len; g_ast_crc = r0.crc32; g_ast_ver = 0u;
      g_ast_got = 0u;
      g_ast_record = r0.type;
      return;
    }
    if (magic == N6_CMD_AST1 && count >= sizeof(n6_cmd_ast1_t)) {
      n6_cmd_ast1_t c1;
      memcpy(&c1, cmd_buf, sizeof(c1));
      const uint32_t avail = count - sizeof(n6_cmd_ast1_t);
      const uint32_t n = (c1.len < avail) ? c1.len : avail;
      if (g_ast_active && n > 0u &&
          (uint32_t)c1.offset + n <= g_ast_len) {
        memcpy(&g_ast_buf[c1.offset], cmd_buf + sizeof(n6_cmd_ast1_t), n);
        g_ast_got += n;
      }
      return;
    }
    if (magic == N6_CMD_AST2 && count >= sizeof(n6_cmd_ast2_t)) {
      if (g_ast_active && g_ast_got >= g_ast_len &&
          n6_host_crc32(0u, g_ast_buf, g_ast_len) == g_ast_crc &&
          (g_ast_record != 0u ? n6_asset_record_stage(g_ast_record, g_ast_buf, g_ast_len)
                              : n6_asset_stage(g_ast_buf, g_ast_len, g_ast_ver))) {
        g_ast_active = false;
        n6_asset_apply();
      }
      g_ast_active = false;
      return;
    }
  }

  if (from_usb && count >= 10 && memcmp(cmd_buf, "REBOOT_DFU", 10) == 0) {
    reboot_to_bootloader();
  }
}

static inline bool vnd_tx_start_frame(const uint8_t* meta, uint32_t meta_len,
                                      const uint8_t* payload, uint32_t payload_len,
                                      uint32_t flags)
{
  /* Called from the USB task and the sketch task: claim and fill are one critical
   * section. Returns true when this call takes the transmitter. */
  if (vnd_tx_active) return false;
  taskENTER_CRITICAL();
  if (vnd_tx_active) { taskEXIT_CRITICAL(); return false; }
  if (payload_len > 0U && !payload) { taskEXIT_CRITICAL(); return false; }
  if (meta_len == 0U && payload_len == 0U) { taskEXIT_CRITICAL(); return false; }

  if (meta_len == 0) meta = NULL;

  vnd_hdr.magic       = VND_MAGIC;
  vnd_hdr.frame_id    = vnd_frame_id++;
  vnd_hdr.meta_len    = meta_len;
  vnd_hdr.payload_len = payload_len;
  vnd_hdr.flags       = flags;

  vnd_meta        = meta;
  vnd_meta_len    = meta_len;
  vnd_payload     = payload;
  vnd_payload_len = payload_len;

  vnd_off   = 0;
  vnd_total = sizeof(vnd_hdr) + vnd_meta_len + vnd_payload_len;
  vnd_tx_active = true;
  taskEXIT_CRITICAL();
  return true;
}

/* The unsent run at vnd_off: the rest of the header, the metadata or the payload. */
static inline uint32_t vnd_tx_chunk(const uint8_t **src)
{
  if (vnd_off < sizeof(vnd_hdr)) {
    *src = ((const uint8_t*)&vnd_hdr) + vnd_off;
    return sizeof(vnd_hdr) - vnd_off;
  }
  if (vnd_off < (sizeof(vnd_hdr) + vnd_meta_len)) {
    *src = vnd_meta + (vnd_off - sizeof(vnd_hdr));
    return (sizeof(vnd_hdr) + vnd_meta_len) - vnd_off;
  }
  const uint32_t poff = vnd_off - (sizeof(vnd_hdr) + vnd_meta_len);
  *src = vnd_payload + poff;
  return vnd_payload_len - poff;
}

/* Header and metadata go out as one write when they fit N6_NET_HEAD_MAX. A host
 * that takes nothing for N6_NET_STALL_MS is dropped. */
#define N6_NET_HEAD_MAX  1024u
#define N6_NET_STALL_MS  5000u

static uint32_t s_net_progress_ms = 0u;   /* last byte taken, or last idle pass */

static inline void vnd_tx_pump_net(const n6_net_link_t *net)
{
  static uint8_t s_head[N6_NET_HEAD_MAX];
  const uint32_t head_len = sizeof(vnd_hdr) + vnd_meta_len;

  while (vnd_off < vnd_total) {
    const uint8_t *src = NULL;
    uint32_t n;
    if (vnd_off < head_len && head_len <= sizeof(s_head)) {
      if (vnd_off == 0u) {
        memcpy(s_head, &vnd_hdr, sizeof(vnd_hdr));
        if (vnd_meta_len != 0u) memcpy(&s_head[sizeof(vnd_hdr)], vnd_meta, vnd_meta_len);
      }
      src = &s_head[vnd_off];
      n = head_len - vnd_off;
    } else {
      n = vnd_tx_chunk(&src);
    }

    const int32_t w = net->send(src, n);
    if (w < 0) { n6_net_detach(); return; }
    if (w == 0) {
      if ((HAL_GetTick() - s_net_progress_ms) > N6_NET_STALL_MS) {
        dbg_print("[NET] the host stopped reading; connection closed.\r\n");
        n6_net_detach();
      }
      return;   /* connection full */
    }
    vnd_off += ((uint32_t)w < n) ? (uint32_t)w : n;
    s_net_progress_ms = HAL_GetTick();
  }
  vnd_tx_active = false;
}

static inline void vnd_tx_pump(void)
{
  if (!vnd_tx_active) return;

  const n6_net_link_t *const net = s_net;
  if (net != NULL) {
    /* A host that attached part way through the SoftAP announce never gets it. */
    if (vnd_meta == s_softap_announce_buf) { vnd_tx_reset(); return; }
    vnd_tx_pump_net(net);
    return;
  }

  /* Unmounted: TinyUSB's reset cleared the FIFO, so the frame is dropped.
   * Suspended: a frame not started is dropped, and one part way out waits for
   * the resume, as part of it may be in the FIFO. */
  if (!tud_mounted()) { vnd_tx_reset(); return; }
  if (!usb_ready_to_tx()) {
    if (vnd_off == 0u) vnd_tx_reset();
    return;
  }

  uint32_t avail = tud_vendor_n_write_available(0);
  if (!avail) return;

  bool wrote_any = false;

  /* The owed zeros go first, so this frame's header lands where the reader
   * looks for one. */
  while (vnd_usb_owed != 0u && avail != 0u) {
    static const uint8_t zeros[64];
    const uint32_t n = (vnd_usb_owed < sizeof(zeros)) ? vnd_usb_owed : (uint32_t)sizeof(zeros);
    const uint32_t w = tud_vendor_n_write(0, zeros, (n < avail) ? n : avail);
    if (w == 0u) break;
    vnd_usb_owed -= w;
    wrote_any = true;
    avail = tud_vendor_n_write_available(0);
  }
  if (vnd_usb_owed != 0u) {
    if (wrote_any) tud_vendor_n_write_flush(0);
    return;
  }

  while (avail)
  {
    uint32_t remaining = vnd_total - vnd_off;
    if (remaining == 0)
    {
      vnd_tx_active = false;
      tud_vendor_n_write_flush(0);
      return;
    }

    const uint8_t* src = NULL;
    const uint32_t chunk_rem = vnd_tx_chunk(&src);

    uint32_t n = (chunk_rem < avail) ? chunk_rem : avail;
    uint32_t w = tud_vendor_n_write(0, src, n);
    if (w == 0) break;

    wrote_any = true;
    vnd_off += w;
    avail -= w;

    avail = tud_vendor_n_write_available(0);
  }

  if (wrote_any) {
    tud_vendor_n_write_flush(0);
  }
}

/* Camera and DCMIPP */

static void CSI_Init(void)
{
  DCMIPP_CSI_ConfTypeDef pCSI_Config = {0};

  hdcmipp.Instance = DCMIPP;
  if (HAL_DCMIPP_Init(&hdcmipp) != HAL_OK)
  {
    Error_Handler();
  }

  pCSI_Config.PHYBitrate = DCMIPP_CSI_PHY_BT_500;
  pCSI_Config.DataLaneMapping = _neuron6_csi_lane_mapping;
  pCSI_Config.NumberOfLanes = DCMIPP_CSI_TWO_DATA_LANES;
  if (HAL_DCMIPP_CSI_SetConfig(&hdcmipp, &pCSI_Config) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_DCMIPP_CSI_SetVCConfig(&hdcmipp, 0U, DCMIPP_CSI_DT_BPP8);
}

/* Call before the pipes start capturing. 0 keeps BT_500. */
void n6_csi_set_phy_bitrate(uint32_t mbps_per_lane)
{
  /* Rounds up: the D-PHY band must sit at or above the real rate. */
  uint32_t phy_bt;
  if      (mbps_per_lane == 0u)    phy_bt = DCMIPP_CSI_PHY_BT_500;
  else if (mbps_per_lane <= 400u)  phy_bt = DCMIPP_CSI_PHY_BT_400;
  else if (mbps_per_lane <= 500u)  phy_bt = DCMIPP_CSI_PHY_BT_500;
  else if (mbps_per_lane <= 800u)  phy_bt = DCMIPP_CSI_PHY_BT_800;
  else if (mbps_per_lane <= 850u)  phy_bt = DCMIPP_CSI_PHY_BT_850;
  else if (mbps_per_lane <= 900u)  phy_bt = DCMIPP_CSI_PHY_BT_900;
  else if (mbps_per_lane <= 1000u) phy_bt = DCMIPP_CSI_PHY_BT_1000;
  else if (mbps_per_lane <= 1200u) phy_bt = DCMIPP_CSI_PHY_BT_1200;
  else if (mbps_per_lane <= 1350u) phy_bt = DCMIPP_CSI_PHY_BT_1350;
  else                             phy_bt = DCMIPP_CSI_PHY_BT_1500;

  {
    char d[88];
    snprintf(d, sizeof(d), "[CSI] PHY set for %lu Mbps/lane (BT bucket=%lu)\r\n",
             (unsigned long)mbps_per_lane, (unsigned long)phy_bt);
    dbg_detail(d);
  }

  DCMIPP_CSI_ConfTypeDef pCSI_Config = {0};
  pCSI_Config.PHYBitrate      = phy_bt;
  pCSI_Config.DataLaneMapping = _neuron6_csi_lane_mapping;
  pCSI_Config.NumberOfLanes   = DCMIPP_CSI_TWO_DATA_LANES;
  if (HAL_DCMIPP_CSI_SetConfig(&hdcmipp, &pCSI_Config) != HAL_OK) {
    dbg_print("[CSI] PHY bitrate re-config failed; keeping boot default.\r\n");
    return;
  }
  HAL_DCMIPP_CSI_SetVCConfig(&hdcmipp, 0U, DCMIPP_CSI_DT_BPP8);
}

/* Camera diagnostics for the debug console. Both dumps run from n6_usb_task. */
#if !defined(NEURON6_DEBUG_CONSOLE) || (NEURON6_DEBUG_CONSOLE != 0)
static uint8_t n6_ov5640_rd(uint16_t reg)
{
  uint8_t v = 0;
  uint8_t a[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFFu) };
  n6_i2c_lock();
  if (HAL_I2C_Master_Transmit(&hi2c1, 0x78, a, 2, 100) != HAL_OK) { v = 0xEEu; goto out; }
  if (HAL_I2C_Master_Receive (&hi2c1, 0x78, &v, 1, 100) != HAL_OK) { v = 0xEEu; }
out:
  n6_i2c_unlock();
  return v;
}

static void n6_dump_ov5640_regs(void)
{
  /* Only an OV5640 answers with chip ID 0x5640. */
  if (n6_ov5640_rd(0x300A) != 0x56u || n6_ov5640_rd(0x300B) != 0x40u) return;
  char d[200];
  snprintf(d, sizeof(d),
    "[OVREG] id=%02X%02X stream(3008)=%02X mipictl(300E)=%02X | PLL 3034=%02X 3035=%02X 3036=%02X 3037=%02X 3108=%02X\r\n",
    n6_ov5640_rd(0x300A), n6_ov5640_rd(0x300B), n6_ov5640_rd(0x3008), n6_ov5640_rd(0x300E),
    n6_ov5640_rd(0x3034), n6_ov5640_rd(0x3035), n6_ov5640_rd(0x3036), n6_ov5640_rd(0x3037), n6_ov5640_rd(0x3108));
  dbg_print(d);
  snprintf(d, sizeof(d),
    "[OVREG] MIPI 4800=%02X 4814=%02X 4837=%02X | fmt 4300=%02X 501F=%02X | sub 3814=%02X 3815=%02X flip 3820=%02X 3821=%02X\r\n",
    n6_ov5640_rd(0x4800), n6_ov5640_rd(0x4814), n6_ov5640_rd(0x4837),
    n6_ov5640_rd(0x4300), n6_ov5640_rd(0x501F),
    n6_ov5640_rd(0x3814), n6_ov5640_rd(0x3815), n6_ov5640_rd(0x3820), n6_ov5640_rd(0x3821));
  dbg_print(d);
  snprintf(d, sizeof(d),
    "[OVREG] out=%02X%02Xx%02X%02X HTS=%02X%02X VTS=%02X%02X isp(5001)=%02X\r\n",
    n6_ov5640_rd(0x3808), n6_ov5640_rd(0x3809), n6_ov5640_rd(0x380A), n6_ov5640_rd(0x380B),
    n6_ov5640_rd(0x380C), n6_ov5640_rd(0x380D), n6_ov5640_rd(0x380E), n6_ov5640_rd(0x380F),
    n6_ov5640_rd(0x5001));
  dbg_print(d);
}

static void n6_dump_csi_regs(const char *tag)
{
  char d[176];
  snprintf(d, sizeof(d),
    "[CSIRAW%s] CR=%08lX PCR=%08lX SR0=%08lX SR1=%08lX ERR1=%08lX ERR2=%08lX PRCR=%08lX\r\n",
    tag,
    (unsigned long)CSI->CR,   (unsigned long)CSI->PCR,
    (unsigned long)CSI->SR0,  (unsigned long)CSI->SR1,
    (unsigned long)CSI->ERR1, (unsigned long)CSI->ERR2,
    (unsigned long)CSI->PRCR);
  dbg_print(d);
  uint32_t xspi2_ker   = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_XSPI2);
  uint32_t xspi2_presc = (XSPI2->DCR2 & XSPI_DCR2_PRESCALER_Msk) >> XSPI_DCR2_PRESCALER_Pos;
  uint32_t xspi2_if    = xspi2_ker / (xspi2_presc + 1u);
  snprintf(d, sizeof(d),
    "[CLK] PLL1=%lu CSI=%lu DCMIPP=%lu | PLL4=%lu XSPI2ker=%lu presc=%lu PSRAMif=%lu Hz (~%lu MB/s DDRx8)\r\n",
    (unsigned long)HAL_RCCEx_GetPLL1CLKFreq(),
    (unsigned long)HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_CSI),
    (unsigned long)HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_DCMIPP),
    (unsigned long)HAL_RCCEx_GetPLL4CLKFreq(),
    (unsigned long)xspi2_ker, (unsigned long)xspi2_presc, (unsigned long)xspi2_if,
    (unsigned long)(xspi2_if / 1000000u * 2u));
  dbg_print(d);
  uint32_t xspi1_ker   = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_XSPI1);
  uint32_t xspi1_presc = (XSPI1->DCR2 & XSPI_DCR2_PRESCALER_Msk) >> XSPI_DCR2_PRESCALER_Pos;
  uint32_t xspi1_if    = xspi1_ker / (xspi1_presc + 1u);
  snprintf(d, sizeof(d),
    "[XSPI] XSPI1(flash/NN)ker=%lu presc=%lu if=%lu | XSPI2(PSRAM)if=%lu Hz | XSPIM_CR=%08lX MUXEN=%lu MODE=%lu (1 shared port)\r\n",
    (unsigned long)xspi1_ker, (unsigned long)xspi1_presc, (unsigned long)xspi1_if,
    (unsigned long)xspi2_if,
    (unsigned long)XSPIM->CR,
    (unsigned long)((XSPIM->CR & XSPIM_CR_MUXEN_Msk) ? 1u : 0u),
    (unsigned long)((XSPIM->CR & XSPIM_CR_MODE_Msk) ? 1u : 0u));
  dbg_print(d);
  /* Latched flags. A working link shows clkACT=1 and SYNC=1 on each lane. */
  uint32_t s1 = CSI->SR1;
  snprintf(d, sizeof(d),
    "[CSIDEC] clkACT=%lu clkSTOP=%lu | L0 HS=%lu SYNC=%lu STOP=%lu | L1 HS=%lu SYNC=%lu STOP=%lu\r\n",
    (unsigned long)!!(s1 & CSI_SR1_ACTCLF),  (unsigned long)!!(s1 & CSI_SR1_STOPCLF),
    (unsigned long)!!(s1 & CSI_SR1_ACTDL0F), (unsigned long)!!(s1 & CSI_SR1_SYNCDL0F), (unsigned long)!!(s1 & CSI_SR1_STOPDL0F),
    (unsigned long)!!(s1 & CSI_SR1_ACTDL1F), (unsigned long)!!(s1 & CSI_SR1_SYNCDL1F), (unsigned long)!!(s1 & CSI_SR1_STOPDL1F));
  dbg_print(d);
  snprintf(d, sizeof(d),
    "[CSIDEC] errL0 sot=%lu sotsync=%lu ctrl=%lu | errL1 sot=%lu sotsync=%lu ctrl=%lu\r\n",
    (unsigned long)!!(s1 & CSI_SR1_ESOTDL0F), (unsigned long)!!(s1 & CSI_SR1_ESOTSYNCDL0F), (unsigned long)!!(s1 & CSI_SR1_ECTRLDL0F),
    (unsigned long)!!(s1 & CSI_SR1_ESOTDL1F), (unsigned long)!!(s1 & CSI_SR1_ESOTSYNCDL1F), (unsigned long)!!(s1 & CSI_SR1_ECTRLDL1F));
  dbg_print(d);
}
#endif /* NEURON6_DEBUG_CONSOLE */

/* Sensor pixel format, set by n6_apply_sensor_format(). */
n6_sensor_format_t g_n6_sensor_format = {
  DCMIPP_DT_YUV422_8,                      /* csi_data_type     */
  DCMIPP_PIXEL_PACKER_FORMAT_YUV422_2,     /* packer_format_usb */
  JPEGENC_YUV422_INTERLEAVED_YUYV,         /* jpeg_enc_format   */
  2u,                                      /* bytes_per_pixel   */
  1u,                                      /* yuv_swap          */
  1u,                                      /* needs_yuv_to_rgb  */
};

/* Demosaicer Bayer order for RAW8. Mono sensors use RGGB. */
static uint32_t g_n6_raw_bayer_order = DCMIPP_RAWBAYER_RGGB;

void n6_apply_sensor_format(int pixel_format, int color_space, int bayer_order)
{
  /* The enums are in a C++ header, so their values appear as integers here.
   * bayer_order: 0 RGGB, 1 GRBG, 2 GBRG, 3 BGGR. */
  if (pixel_format == 1 /* N6_PIXFMT_RGB565 */) {
    /* PIPE1 converts RGB565 to YUV422: the VENC cannot encode RGB at width >= 1920. */
    g_n6_sensor_format.csi_data_type     = DCMIPP_DT_RGB565;
    g_n6_sensor_format.packer_format_usb = DCMIPP_PIXEL_PACKER_FORMAT_YUV422_1;    /* YUYV */
    g_n6_sensor_format.jpeg_enc_format   = JPEGENC_YUV422_INTERLEAVED_YUYV;
    g_n6_sensor_format.bytes_per_pixel   = 2u;
    g_n6_sensor_format.yuv_swap          = 0u;
    g_n6_sensor_format.needs_yuv_to_rgb  = 0u;
  } else if (pixel_format == 2 /* N6_PIXFMT_RAW8 */) {
    /* After the demosaicer, P1DMCR, the path matches RGB565. */
    g_n6_sensor_format.csi_data_type     = DCMIPP_DT_RAW8;
    g_n6_sensor_format.packer_format_usb = DCMIPP_PIXEL_PACKER_FORMAT_YUV422_1;
    g_n6_sensor_format.jpeg_enc_format   = JPEGENC_YUV422_INTERLEAVED_YUYV;
    g_n6_sensor_format.bytes_per_pixel   = 2u;   /* of the YUV422 output */
    g_n6_sensor_format.yuv_swap          = 0u;
    g_n6_sensor_format.needs_yuv_to_rgb  = 0u;
    g_n6_raw_bayer_order = (bayer_order == 1) ? DCMIPP_RAWBAYER_GRBG :
                           (bayer_order == 2) ? DCMIPP_RAWBAYER_GBRG :
                           (bayer_order == 3) ? DCMIPP_RAWBAYER_BGGR :
                                                DCMIPP_RAWBAYER_RGGB;
  } else {
    if (pixel_format != 0 /* != N6_PIXFMT_YUV422_YUYV */) {
      dbg_print("[CAM] unknown sensor pixel format; YUV422 assumed.\r\n");
    }
    g_n6_sensor_format.csi_data_type     = DCMIPP_DT_YUV422_8;
    g_n6_sensor_format.packer_format_usb = DCMIPP_PIXEL_PACKER_FORMAT_YUV422_2;
    g_n6_sensor_format.jpeg_enc_format   = JPEGENC_YUV422_INTERLEAVED_YUYV;
    g_n6_sensor_format.bytes_per_pixel   = 2u;
    g_n6_sensor_format.yuv_swap          = 1u;
    g_n6_sensor_format.needs_yuv_to_rgb  = (color_space != 0 /* != N6_COLORSPACE_NONE */) ? 1u : 0u;
  }
}

/* Writes P1DMCR directly, which works in any DCMIPP state. */
void n6_dcmipp_set_bayer_order(int bayer_order)
{
  g_n6_raw_bayer_order = (bayer_order == 1) ? DCMIPP_RAWBAYER_GRBG :
                         (bayer_order == 2) ? DCMIPP_RAWBAYER_GBRG :
                         (bayer_order == 3) ? DCMIPP_RAWBAYER_BGGR :
                                              DCMIPP_RAWBAYER_RGGB;
  MODIFY_REG(DCMIPP->P1DMCR, DCMIPP_P1DMCR_TYPE, g_n6_raw_bayer_order);
  SET_BIT(DCMIPP->P1DMCR, DCMIPP_P1DMCR_ENABLE);
}

/* Configures PIPE1, the JPEG pipe. */
void DCMIPP_USB_Init(uint32_t width, uint32_t height)
{
  DCMIPP_CSI_PIPE_ConfTypeDef pCSI_PipeConfig = {0};
  DCMIPP_PipeConfTypeDef pPipeConfig = {0};

  usb_frame_width = width;
  usb_frame_height = height;
  /* The NN crop starts as the full frame. */
  g_camera_full_w = (uint16_t)width;
  g_camera_full_h = (uint16_t)height;
  g_nn_crop_x = 0;
  g_nn_crop_y = 0;
  g_nn_crop_w = (uint16_t)width;
  g_nn_crop_h = (uint16_t)height;

  pCSI_PipeConfig.DataTypeMode = DCMIPP_DTMODE_DTIDA;
  pCSI_PipeConfig.DataTypeIDA = g_n6_sensor_format.csi_data_type;
  pCSI_PipeConfig.DataTypeIDB = g_n6_sensor_format.csi_data_type;
  HAL_StatusTypeDef rc = HAL_DCMIPP_CSI_PIPE_SetConfig(&hdcmipp, DCMIPP_PIPE1, &pCSI_PipeConfig);
  if (rc != HAL_OK) Error_Handler();

  /* P1FSCR written directly: the HAL skips it unless State is INIT or READY. */
  MODIFY_REG(DCMIPP->P1FSCR,
    DCMIPP_P1FSCR_DTMODE | DCMIPP_P1FSCR_DTIDA | DCMIPP_P1FSCR_DTIDB,
    DCMIPP_DTMODE_DTIDA | (g_n6_sensor_format.csi_data_type << DCMIPP_P1FSCR_DTIDA_Pos) |
    (g_n6_sensor_format.csi_data_type << DCMIPP_P1FSCR_DTIDB_Pos));
  /* CMCR.INSEL selects the CSI input. Written directly for the same reason. */
  SET_BIT(DCMIPP->CMCR, DCMIPP_CMCR_INSEL);

  pPipeConfig.FrameRate = DCMIPP_FRAME_RATE_1_OVER_4;   /* the LINEBUF path forces FRATE_ALL */
  pPipeConfig.PixelPipePitch = width * g_n6_sensor_format.bytes_per_pixel;
  pPipeConfig.PixelPackerFormat = g_n6_sensor_format.packer_format_usb;
  rc = HAL_DCMIPP_PIPE_SetConfig(&hdcmipp, DCMIPP_PIPE1, &pPipeConfig);
  if (rc != HAL_OK) Error_Handler();

  /* A PIPE1 crop on RAW8 corrupts the line-buffer band. */
  if (g_n6_sensor_format.csi_data_type == DCMIPP_DT_RAW8) {
    (void)HAL_DCMIPP_PIPE_DisableCrop(&hdcmipp, DCMIPP_PIPE1);
  }

  if (g_n6_sensor_format.csi_data_type == DCMIPP_DT_RAW8) {
    DCMIPP_RawBayer2RGBConfTypeDef demo = {0};
    demo.RawBayerType  = g_n6_raw_bayer_order;
    demo.PeakStrength  = DCMIPP_RAWBAYER_ALGO_STRENGTH_4;
    demo.EdgeStrength  = DCMIPP_RAWBAYER_ALGO_STRENGTH_16;
    demo.VLineStrength = DCMIPP_RAWBAYER_ALGO_STRENGTH_8;
    demo.HLineStrength = DCMIPP_RAWBAYER_ALGO_STRENGTH_8;
    /* Return ignored: the direct write below applies the config. */
    (void)HAL_DCMIPP_PIPE_SetISPRawBayer2RGBConfig(&hdcmipp, DCMIPP_PIPE1, &demo);
    (void)HAL_DCMIPP_PIPE_EnableISPRawBayer2RGB(&hdcmipp, DCMIPP_PIPE1);
    /* P1DMCR written directly, as with P1FSCR above. */
    MODIFY_REG(DCMIPP->P1DMCR, DCMIPP_P1DMCR_TYPE, g_n6_raw_bayer_order);
    SET_BIT(DCMIPP->P1DMCR, DCMIPP_P1DMCR_ENABLE);
  }

  /* VD66GY ISP colour for D65, from vd66gy_mai1_isp_param_conf.h. White-balance
   * gains are scaled by 128, matrix coefficients by 256. */
  if (g_n6_sensor_format.csi_data_type == DCMIPP_DT_RAW8) {
    DCMIPP_BlackLevelConfTypeDef blc = {
      .RedCompBlackLevel = 16u, .GreenCompBlackLevel = 16u, .BlueCompBlackLevel = 16u,
    };
    (void)HAL_DCMIPP_PIPE_SetISPBlackLevelCalibrationConfig(&hdcmipp, DCMIPP_PIPE1, &blc);
    (void)HAL_DCMIPP_PIPE_EnableISPBlackLevelCalibration(&hdcmipp, DCMIPP_PIPE1);

    DCMIPP_ExposureConfTypeDef wb = {
      .ShiftRed   = 0u, .MultiplierRed   = 219u,
      .ShiftGreen = 0u, .MultiplierGreen = 128u,
      .ShiftBlue  = 0u, .MultiplierBlue  = 193u,
    };
    (void)HAL_DCMIPP_PIPE_SetISPExposureConfig(&hdcmipp, DCMIPP_PIPE1, &wb);
    (void)HAL_DCMIPP_PIPE_EnableISPExposure(&hdcmipp, DCMIPP_PIPE1);

    /* Rounded to the nearest: each row sums to 256. */
    DCMIPP_ColorConversionConfTypeDef ccm = {
      .RR = 362, .RG =   29, .RB = -135,
      .GR = -88, .GG =  474, .GB = -130,
      .BR = -23, .BG = -102, .BB =  381,
    };
    (void)HAL_DCMIPP_PIPE_SetISPColorConversionConfig(&hdcmipp, DCMIPP_PIPE1, &ccm);
    (void)HAL_DCMIPP_PIPE_EnableISPColorConversion(&hdcmipp, DCMIPP_PIPE1);

    (void)HAL_DCMIPP_PIPE_EnableGammaConversion(&hdcmipp, DCMIPP_PIPE1);
  }

  /* RGB to YUV422, BT.601 full range. */
  if ((g_n6_sensor_format.csi_data_type == DCMIPP_DT_RGB565 ||
       g_n6_sensor_format.csi_data_type == DCMIPP_DT_RAW8) &&
      g_n6_sensor_format.packer_format_usb == DCMIPP_PIXEL_PACKER_FORMAT_YUV422_1) {
    DCMIPP_ColorConversionConfTypeDef rgb_to_yuv = {
      .ClampOutputSamples = DISABLE,
      .OutputSamplesType  = DCMIPP_CLAMP_YUV,
      .RR = 131, .RG = -110, .RB = -21, .RA = 128,   /* Cr */
      .GR =  77, .GG =  150, .GB =  29, .GA =   0,    /* Y */
      .BR = -44, .BG =  -87, .BB = 131, .BA = 128,    /* Cb */
    };
    if (HAL_DCMIPP_PIPE_SetYUVConversionConfig(&hdcmipp, DCMIPP_PIPE1, &rgb_to_yuv) != HAL_OK)
      Error_Handler();
    if (HAL_DCMIPP_PIPE_EnableYUVConversion(&hdcmipp, DCMIPP_PIPE1) != HAL_OK)
      Error_Handler();
  }

  if (g_n6_sensor_format.yuv_swap) {
    (void)HAL_DCMIPP_PIPE_EnableYUVSwap(&hdcmipp, DCMIPP_PIPE1);
  }

  if (g_n6_usb_enabled &&
      !encoder_jpeg_init(g_n6_sensor_format.jpeg_enc_format, width, height, mjpeg_buf0, g_mjpeg_max))
  {
    Error_Handler();
  }
}

/* Configures PIPE2, the NN pipe, to stretch the full camera frame to width x height. */
void DCMIPP_NN_Init(uint32_t width, uint32_t height)
{
  DCMIPP_NN_InitWithCrop(width, height, 0u, 0u, 0u, 0u);
}

static void nn_pipe_program(uint32_t width, uint32_t height,
                            uint32_t pic_w, uint32_t pic_h,
                            uint32_t crop_x, uint32_t crop_y,
                            uint32_t crop_w, uint32_t crop_h);

/* Configures PIPE2 with a crop window in camera pixels. A zero crop_w or crop_h
 * means no crop. */
void DCMIPP_NN_InitWithCrop(uint32_t width, uint32_t height,
                            uint32_t crop_x, uint32_t crop_y,
                            uint32_t crop_w, uint32_t crop_h)
{
  s_nn_pic_w = 0u;
  nn_pipe_program(width, height, width, height, crop_x, crop_y, crop_w, crop_h);
}

/* Scale and size as PixelKit's letterbox(), in the same double precision. The
 * picture's left edge is rounded down to 16 pixels for the DCMIPP. */
void DCMIPP_NN_InitLetterbox(uint32_t width, uint32_t height)
{
  const uint32_t fw = usb_frame_width;
  const uint32_t fh = usb_frame_height;
  if ((fw == 0u) || (fh == 0u) || (width == 0u) || (height == 0u)) return;

  const double s = fmin((double)width / (double)fw, (double)height / (double)fh);
  uint32_t pw = (uint32_t)rint((double)fw * s);
  uint32_t ph = (uint32_t)rint((double)fh * s);
  if (pw > width)  pw = width;
  if (ph > height) ph = height;
  uint32_t px = ((width  - pw) / 2u) & ~15u;   /* a 16-byte aligned start */
  uint32_t py = (height - ph) / 2u;
  while ((((py * width * 3u) & 15u) != 0u) && (py > 0u)) py--;

  s_nn_pic_w = 0u;
  nn_pipe_program(width, height, pw, ph, 0u, 0u, 0u, 0u);
  if (!g_nn_pipe_ready) return;

  s_nn_pic_x = (uint16_t)px;
  s_nn_pic_y = (uint16_t)py;
  s_nn_pic_w = (uint16_t)pw;
  s_nn_pic_h = (uint16_t)ph;

  /* The camera pixels the whole input covers, past the frame on the bar sides. */
  g_nn_crop_w = (uint16_t)(((uint64_t)width  * fw + pw / 2u) / pw);
  g_nn_crop_h = (uint16_t)(((uint64_t)height * fh + ph / 2u) / ph);
  g_nn_crop_x = (int16_t)-(int32_t)(((uint64_t)px * fw + pw / 2u) / pw);
  g_nn_crop_y = (int16_t)-(int32_t)(((uint64_t)py * fh + ph / 2u) / ph);
}

/* Fills a letterboxed input's top and bottom bars with grey and cleans them to
 * memory, before PIPE2 writes the picture between them. The NPU may use the
 * input as scratch, so this runs for every frame. Returns the picture's offset
 * in the input. */
static uint32_t nn_letterbox_prepare(uint8_t *in)
{
  if (s_nn_pic_w == 0u) return 0u;
  const uint32_t pitch = (uint32_t)g_nn_out_w * 3u;
  const uint32_t top   = (uint32_t)s_nn_pic_y * pitch;
  const uint32_t below = ((uint32_t)s_nn_pic_y + s_nn_pic_h) * pitch;
  const uint32_t end   = (uint32_t)g_nn_out_h * pitch;

  memset(in, N6_LETTERBOX_GREY, top);
  memset(in + below, N6_LETTERBOX_GREY, end - below);
  SCB_CleanDCache_by_Addr((uint32_t *)in, (int32_t)top);
  SCB_CleanDCache_by_Addr((uint32_t *)(in + below), (int32_t)(end - below));
  return top + (uint32_t)s_nn_pic_x * 3u;
}

/* Fills a letterboxed input's side bars once PIPE2 has written the frame. The
 * DCMIPP writes each line in whole words, so a line can run into the right bar. */
static void nn_letterbox_finish(uint8_t *in)
{
  if ((s_nn_pic_w == 0u) || (s_nn_pic_w >= g_nn_out_w)) return;
  const uint32_t pitch = (uint32_t)g_nn_out_w * 3u;
  const uint32_t top   = (uint32_t)s_nn_pic_y * pitch;
  const uint32_t below = ((uint32_t)s_nn_pic_y + s_nn_pic_h) * pitch;
  const uint32_t left  = (uint32_t)s_nn_pic_x * 3u;
  const uint32_t right = ((uint32_t)s_nn_pic_x + s_nn_pic_w) * 3u;

  SCB_InvalidateDCache_by_Addr((void *)(in + top), (int32_t)(below - top));
  for (uint32_t r = top; r < below; r += pitch)
  {
    memset(in + r, N6_LETTERBOX_GREY, left);
    memset(in + r + right, N6_LETTERBOX_GREY, pitch - right);
  }
  SCB_CleanDCache_by_Addr((uint32_t *)(in + top), (int32_t)(below - top));
}

/* Programs PIPE2: the crop, then a downsize to pic_w x pic_h, written with the
 * line pitch of a width x height input. */
static void nn_pipe_program(uint32_t width, uint32_t height,
                            uint32_t pic_w, uint32_t pic_h,
                            uint32_t crop_x, uint32_t crop_y,
                            uint32_t crop_w, uint32_t crop_h)
{
  DCMIPP_CSI_PIPE_ConfTypeDef pCSI_PipeConfig = {0};
  DCMIPP_PipeConfTypeDef pPipeConfig = {0};

  /* On RAW8, PIPE2 shares PIPE1's demosaiced RGB, so DCMIPP_USB_Init must run first. */
  if (!dcmipp_pipe1_csi_configured)
  {
    if (g_n6_sensor_format.csi_data_type == DCMIPP_DT_RAW8)
    {
      if (HAL_DCMIPP_PIPE_CSI_EnableShare(&hdcmipp, DCMIPP_PIPE2) != HAL_OK)
      {
        Error_Handler();
      }
    }
    else
    {
      pCSI_PipeConfig.DataTypeMode = DCMIPP_DTMODE_DTIDA;
      pCSI_PipeConfig.DataTypeIDA = g_n6_sensor_format.csi_data_type;
      pCSI_PipeConfig.DataTypeIDB = g_n6_sensor_format.csi_data_type;

      if (HAL_DCMIPP_CSI_PIPE_SetConfig(&hdcmipp, DCMIPP_PIPE2, &pCSI_PipeConfig) != HAL_OK)
      {
        Error_Handler();
      }
    }
    dcmipp_pipe1_csi_configured = true;
  }

  if (!dcmipp_pipe1_pipe_configured)
  {
    pPipeConfig.FrameRate = DCMIPP_FRAME_RATE_ALL;
    pPipeConfig.PixelPipePitch = width * 3;  /* RGB888 */
    pPipeConfig.PixelPackerFormat = DCMIPP_PIXEL_PACKER_FORMAT_RGB888_YUV444_1;

    if (HAL_DCMIPP_PIPE_SetConfig(&hdcmipp, DCMIPP_PIPE2, &pPipeConfig) != HAL_OK)
    {
      Error_Handler();
    }

    /* SWAPRB gives the model RGB order. The HAL skips it unless State is READY. */
    (void)HAL_DCMIPP_PIPE_EnableRedBlueSwap(&hdcmipp, DCMIPP_PIPE2);
    if ((hdcmipp.Instance->P2PPCR & DCMIPP_P2PPCR_SWAPRB) == 0u)
    {
      SET_BIT(hdcmipp.Instance->P2PPCR, DCMIPP_P2PPCR_SWAPRB);
    }

    dcmipp_pipe1_pipe_configured = true;
  }

  /* The pitch is set on every call, as the width can change. Written directly:
   * SetConfig fails once the pipe has started. */
  {
    const uint32_t pitch = width * 3u;   /* RGB888 */
    MODIFY_REG(hdcmipp.Instance->P2PPM0PR, DCMIPP_P2PPM0PR_PITCH,
               pitch << DCMIPP_P2PPM0PR_PITCH_Pos);
  }

  /* YUV to RGB, BT.601, in Q8: 256 is 1.0. */
  if (!dcmipp_pipe1_color_conv_configured)
  {
    if (g_n6_sensor_format.needs_yuv_to_rgb)
    {
      DCMIPP_ColorConversionConfTypeDef yuv_to_rgb =
      {
        .ClampOutputSamples = DISABLE,
        .OutputSamplesType  = DCMIPP_CLAMP_YUV,
        .RR = 409,  .RG = 298, .RB = 0,    .RA = -223,
        .GR = -208, .GG = 298, .GB = -100, .GA = 135,
        .BR = 0,    .BG = 298, .BB = 527,  .BA = -277,
      };

      if (HAL_DCMIPP_PIPE_SetISPColorConversionConfig(&hdcmipp, DCMIPP_PIPE2, &yuv_to_rgb) != HAL_OK)
        Error_Handler();

      if (HAL_DCMIPP_PIPE_EnableISPColorConversion(&hdcmipp, DCMIPP_PIPE2) != HAL_OK)
        Error_Handler();
    }

    dcmipp_pipe1_color_conv_configured = true;
  }

  const bool use_crop = (crop_w != 0u) && (crop_h != 0u);

  const uint32_t in_w = use_crop ? crop_w : usb_frame_width;
  const uint32_t in_h = use_crop ? crop_h : usb_frame_height;

  if (use_crop)
  {
    DCMIPP_CropConfTypeDef CropConf = {
      .HStart   = crop_x,
      .VStart   = crop_y,
      .HSize    = crop_w,
      .VSize    = crop_h,
      .PipeArea = DCMIPP_POSITIVE_AREA,
    };
    if (HAL_DCMIPP_PIPE_SetCropConfig(&hdcmipp, DCMIPP_PIPE2, &CropConf) != HAL_OK)
      Error_Handler();
    if (HAL_DCMIPP_PIPE_EnableCrop(&hdcmipp, DCMIPP_PIPE2) != HAL_OK)
      Error_Handler();
  }
  else
  {
    (void)HAL_DCMIPP_PIPE_DisableCrop(&hdcmipp, DCMIPP_PIPE2);
  }

  g_nn_crop_x = (int16_t)(use_crop ? crop_x : 0u);
  g_nn_crop_y = (int16_t)(use_crop ? crop_y : 0u);
  g_nn_crop_w = (uint16_t)in_w;
  g_nn_crop_h = (uint16_t)in_h;

  /* The downsize ratio (input * 8192) / output must fit 16 bits, so a steeper one is
   * decimated first. PIPE2 applies crop, then decimation, then downsize. */
  uint32_t ds_in_w = in_w;   /* size entering the downsizer */
  uint32_t ds_in_h = in_h;

  uint32_t df = 1u;
  while ((((in_w / df) * 8192u) / pic_w > 65535u) ||
         (((in_h / df) * 8192u) / pic_h > 65535u))
  {
    df <<= 1;
    if (df >= 8u) break;            /* 1/8 is the hardware maximum */
  }

  if (df > 1u)
  {
    const uint32_t hdec = (df >= 8u) ? DCMIPP_HDEC_1_OUT_8 :
                          (df == 4u) ? DCMIPP_HDEC_1_OUT_4 : DCMIPP_HDEC_1_OUT_2;
    const uint32_t vdec = (df >= 8u) ? DCMIPP_VDEC_1_OUT_8 :
                          (df == 4u) ? DCMIPP_VDEC_1_OUT_4 : DCMIPP_VDEC_1_OUT_2;
    DCMIPP_DecimationConfTypeDef DecConf = { .VRatio = vdec, .HRatio = hdec };

    if ((HAL_DCMIPP_PIPE_SetDecimationConfig(&hdcmipp, DCMIPP_PIPE2, &DecConf) == HAL_OK) &&
        (HAL_DCMIPP_PIPE_EnableDecimation(&hdcmipp, DCMIPP_PIPE2) == HAL_OK))
    {
      ds_in_w = in_w / df;
      ds_in_h = in_h / df;
    }
  }
  else
  {
    (void)HAL_DCMIPP_PIPE_DisableDecimation(&hdcmipp, DCMIPP_PIPE2);
  }

  if (((ds_in_w * 8192u) / pic_w > 65535u) || ((ds_in_h * 8192u) / pic_h > 65535u))
  {
    NeuroN6_Notice("[NN] the camera resolution is too high for this NN input, even with 1/8 "
                   "decimation, so the NN pipe is off. Lower cfg.camera or raise cfg.nn.width "
                   "and cfg.nn.height.\r\n");
    g_nn_pipe_ready = 0u;
    return;
  }

  DCMIPP_DownsizeTypeDef DownsizeConf;

  DownsizeConf.HRatio      = (ds_in_w * 8192) / pic_w;
  DownsizeConf.VRatio      = (ds_in_h * 8192) / pic_h;
  DownsizeConf.HSize       = pic_w;
  DownsizeConf.VSize       = pic_h;
  DownsizeConf.HDivFactor  = (pic_w * 1023) / ds_in_w;
  DownsizeConf.VDivFactor  = (pic_h * 1023) / ds_in_h;

  if (HAL_DCMIPP_PIPE_SetDownsizeConfig(&hdcmipp, DCMIPP_PIPE2, &DownsizeConf) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_DCMIPP_PIPE_EnableDownsize(&hdcmipp, DCMIPP_PIPE2) != HAL_OK)
  {
    Error_Handler();
  }

  g_nn_out_w = (uint16_t)width;
  g_nn_out_h = (uint16_t)height;
  g_nn_pipe_ready = 1u;
}

void DCMIPP_LCD_Init(uint32_t width, uint32_t height)
{
  (void)height;   /* PIPE0 needs only the pitch */
  DCMIPP_CSI_PIPE_ConfTypeDef pCSI_PipeConfig = {0};
  DCMIPP_PipeConfTypeDef pPipeConfig = {0};

  /* PIPE0 is the dump pipe and has no ISP. */
  if (!dcmipp_pipe2_configured)
  {
    pCSI_PipeConfig.DataTypeMode = DCMIPP_DTMODE_DTIDA;
    pCSI_PipeConfig.DataTypeIDA = g_n6_sensor_format.csi_data_type;
    pCSI_PipeConfig.DataTypeIDB = g_n6_sensor_format.csi_data_type;

    if (HAL_DCMIPP_CSI_PIPE_SetConfig(&hdcmipp, DCMIPP_PIPE0, &pCSI_PipeConfig) != HAL_OK)
    {
      Error_Handler();
    }

    /* P0FSCR written directly, as with P1FSCR. */
    MODIFY_REG(DCMIPP->P0FSCR,
      DCMIPP_P0FSCR_DTMODE | DCMIPP_P0FSCR_DTIDA | DCMIPP_P0FSCR_DTIDB,
      DCMIPP_DTMODE_DTIDA | (g_n6_sensor_format.csi_data_type << DCMIPP_P0FSCR_DTIDA_Pos) |
      (g_n6_sensor_format.csi_data_type << DCMIPP_P0FSCR_DTIDB_Pos));

    pPipeConfig.FrameRate = DCMIPP_FRAME_RATE_ALL;
    pPipeConfig.PixelPipePitch = width * 2;  /* RGB565 */
    pPipeConfig.PixelPackerFormat = DCMIPP_PIXEL_PACKER_FORMAT_RGB565_1;

    if (HAL_DCMIPP_PIPE_SetConfig(&hdcmipp, DCMIPP_PIPE0, &pPipeConfig) != HAL_OK)
    {
      Error_Handler();
    }

    dcmipp_pipe2_configured = true;
  }
}

/* Called from EWLWaitHwRdy, after EncAsicFrameStart has armed the VENC. */
void n6_linebuf_start_dcmipp(void)
{
  /* The Start needs a 16-byte-aligned destination and INSEL set. */
  SET_BIT(DCMIPP->CMCR, DCMIPP_CMCR_INSEL);
  n6_dcmipp_prepare_start(DCMIPP_PIPE1);
  g_dbg_lb_start_ret = (int32_t)HAL_DCMIPP_CSI_PIPE_Start(&hdcmipp, DCMIPP_PIPE1,
        DCMIPP_VIRTUAL_CHANNEL0, (uint32_t)g_venc_band, DCMIPP_MODE_CONTINUOUS);
}

/* Starts PIPE1 once at boot: the VENC-first start needs a PIPE1 that has already run. */
void n6_linebuf_prime_pipe(void)
{
  if (g_venc_band == NULL) return;
  uint32_t wrap = (g_venc_band_depth >= 4u) ? DCMIPP_WRAP_ADDRESS_128_LINES :
                  (g_venc_band_depth >= 2u) ? DCMIPP_WRAP_ADDRESS_64_LINES : DCMIPP_WRAP_ADDRESS_32_LINES;
  /* Wrapping is enabled before the Start, so the band cannot overflow. */
  (void)HAL_DCMIPP_PIPE_SetLineWrappingConfig(&hdcmipp, DCMIPP_PIPE1, wrap);
  (void)HAL_DCMIPP_PIPE_EnableLineWrapping(&hdcmipp, DCMIPP_PIPE1);
  SET_BIT(DCMIPP->CMCR, DCMIPP_CMCR_INSEL);
  n6_dcmipp_prepare_start(DCMIPP_PIPE1);
  (void)HAL_DCMIPP_CSI_PIPE_Start(&hdcmipp, DCMIPP_PIPE1, DCMIPP_VIRTUAL_CHANNEL0,
        (uint32_t)g_venc_band, DCMIPP_MODE_CONTINUOUS);
}

/* Runs the pipe config held in g_n6_deferred_cfg. Called from bringUp(). */
void n6_vision_deferred_pipes(void)
{
  if (!g_n6_deferred_cfg.valid) return;
  g_n6_deferred_cfg.valid = 0u;

  if (g_n6_deferred_cfg.mbps != 0u) n6_csi_set_phy_bitrate(g_n6_deferred_cfg.mbps);
  DCMIPP_USB_Init(g_n6_deferred_cfg.cam_w, g_n6_deferred_cfg.cam_h);
  if (g_n6_deferred_cfg.letterbox)
  {
    DCMIPP_NN_InitLetterbox(g_n6_deferred_cfg.nn_w, g_n6_deferred_cfg.nn_h);
  }
  else
  {
    DCMIPP_NN_InitWithCrop(g_n6_deferred_cfg.nn_w, g_n6_deferred_cfg.nn_h,
                           g_n6_deferred_cfg.crop_x, g_n6_deferred_cfg.crop_y,
                           g_n6_deferred_cfg.crop_w, g_n6_deferred_cfg.crop_h);
  }
  if (g_n6_deferred_cfg.lcd) DCMIPP_LCD_Init(g_n6_deferred_cfg.cam_w, g_n6_deferred_cfg.cam_h);
}

/* Called from EWLWaitHwRdy on the capture task, before the hardware wait. Empty
 * under FREERTOS_SYNC. */
void n6_linebuf_wait_begin(void)
{
  (void)0;
}

static uint32_t n6_linebuf_capture_one_frame(uint8_t *out_buf)
{
  if (!g_lb_dcmipp_continuous) {
    (void)HAL_DCMIPP_CSI_PIPE_Stop(&hdcmipp, DCMIPP_PIPE1, DCMIPP_VIRTUAL_CHANNEL0);
    /* Before the line event is enabled below. */
    n6_dcmipp_prepare_start(DCMIPP_PIPE1);
    CSI->FCR0 = 0xFFFFFFFF; CSI->FCR1 = 0xFFFFFFFF; __DSB(); __ISB();
    s_pipe1_last_end_ms = 0u;
    s_pipe1_frame_ms = 0u;
    /* ES0620: the DCMIPP must be stopped before the VENC reset. */
    if (g_lb_venc_reset_pending) {
      g_lb_venc_reset_pending = 0u;
      encoder_jpeg_venc_errata_reset();
    }
    /* P1FSCR does not survive the stop. */
    MODIFY_REG(DCMIPP->P1FSCR, DCMIPP_P1FSCR_DTMODE|DCMIPP_P1FSCR_DTIDA|DCMIPP_P1FSCR_DTIDB,
      DCMIPP_DTMODE_DTIDA | (g_n6_sensor_format.csi_data_type<<DCMIPP_P1FSCR_DTIDA_Pos) |
      (g_n6_sensor_format.csi_data_type<<DCMIPP_P1FSCR_DTIDB_Pos));
    uint32_t wrap = (g_venc_band_depth>=4u) ? DCMIPP_WRAP_ADDRESS_128_LINES :
                    (g_venc_band_depth>=2u) ? DCMIPP_WRAP_ADDRESS_64_LINES : DCMIPP_WRAP_ADDRESS_32_LINES;
    (void)HAL_DCMIPP_PIPE_SetLineWrappingConfig(&hdcmipp, DCMIPP_PIPE1, wrap);
    (void)HAL_DCMIPP_PIPE_EnableLineWrapping(&hdcmipp, DCMIPP_PIPE1);
    /* The LINE event drives venc_rdy and must fire every depth*16 lines, the chunk
     * the VENC reads per edge. */
    uint32_t levt = (g_venc_band_depth >= 4u) ? DCMIPP_MULTILINE_64_LINES :
                    (g_venc_band_depth >= 2u) ? DCMIPP_MULTILINE_32_LINES :
                                                DCMIPP_MULTILINE_16_LINES;
    (void)HAL_DCMIPP_PIPE_EnableLineEvent(&hdcmipp, DCMIPP_PIPE1, levt);
    /* At a lower frame rate the VENC times out waiting for the next frame. */
    (void)HAL_DCMIPP_PIPE_SetFrameRate(&hdcmipp, DCMIPP_PIPE1, DCMIPP_FRAME_RATE_ALL);
    /* The VENC must be armed before the DCMIPP starts, so EWLWaitHwRdy starts it. */
    g_venc_lb_source_arm = 1u;
    g_lb_dcmipp_continuous = 1u;
  }
  encoder_jpeg_set_output(out_buf);
  /* The frame-event callback zeroes the producer counter. */
  g_dbg_lb_rd_max = 0u; g_dbg_lb_wr_max = 0u; g_dbg_lb_gap_max = 0u;
  encoder_jpeg_linebuf_begin_frame();
  g_dbg_lb_start_ret = 0;
  uint32_t sz = encode_jpeg_frame_linebuf();   /* blocks */
  static uint32_t s_lb_consec_fail = 0u;
  if (sz == 0u) { if (++s_lb_consec_fail >= 8u) { g_lb_dcmipp_continuous = 0u; g_lb_venc_reset_pending = 1u; s_lb_consec_fail = 0u; } }
  else          { s_lb_consec_fail = 0u; }
  if (g_dbg_lb_start_ret != 0) { g_lb_dcmipp_continuous = 0u; g_lb_venc_reset_pending = 1u; s_lb_consec_fail = 0u; }
  return sz;
}

/* One-shot JPEG without Vision.begin(). Call n6_jpeg_band_setup,
 * encoder_jpeg_linebuf_ensure_init and n6_linebuf_prime_pipe once, then
 * n6_jpeg_capture_one for each capture. */
int n6_jpeg_band_setup(uint32_t cam_w, uint32_t cam_h)
{
  if (g_venc_band != NULL) return 1;

  /* Same sizing as n6_alloc_camera_buffers. */
  uint32_t pitch = ((cam_w * 2u) + 15u) & ~15u;
  uint32_t mbW   = (cam_w + 15u) / 16u;
  uint32_t depth = ((pitch * (4u * 16u * 2u)) <= N6_VENC_BAND_MAX) ? 4u : 2u;
  if ((mbW & 1u) != 0u ||
      (g_n6_sensor_format.csi_data_type == DCMIPP_DT_RAW8 && cam_h > 1024u)) {
    depth = 4u;
  }
  if (cam_h == 480u && (cam_w == 800u || cam_w == 640u)) depth = 2u;

  const uint32_t bsize = pitch * (depth * 16u * 2u);
  /* AXISRAM first: from PSRAM the encoder falls behind the VD66GY's lines. */
  g_venc_band = (uint8_t *)axisram_carve_alloc(bsize);
  if (g_venc_band == NULL) g_venc_band = (uint8_t *)psram_pool_alloc(bsize);
  if (g_venc_band == NULL) return 0;
  g_venc_band_size     = bsize;
  g_venc_band_pitch    = pitch;
  g_venc_band_depth    = depth;
  g_venc_total_mblines = ((cam_h & ~7u) + 15u) / 16u;   /* the encoder floors the height to 8 */
  g_venc_lb_hw         = 1u;
  return 1;
}

uint32_t n6_jpeg_capture_one(uint8_t *out_buf)
{
  if (g_venc_band == NULL || out_buf == NULL) return 0u;
  /* The continuous DCMIPP does not survive between captures, so each one restarts it. */
  g_lb_dcmipp_continuous = 0u;
  return n6_linebuf_capture_one_frame(out_buf);
}

/* Returns a PIPE2 left in ERROR or BUSY to READY. It repeats the HAL's stop for
 * the pipe alone: HAL_DCMIPP_CSI_PIPE_Stop also stops the channel PIPE1 shares. */
static void n6_pipe2_release_held(void)
{
  const HAL_DCMIPP_PipeStateTypeDef st = hdcmipp.PipeState[DCMIPP_PIPE2];
  if ((st != HAL_DCMIPP_PIPE_STATE_ERROR) && (st != HAL_DCMIPP_PIPE_STATE_BUSY)) return;

  CLEAR_BIT(DCMIPP->P2FCTCR, DCMIPP_P2FCTCR_CPTREQ);
  const uint32_t t0 = HAL_GetTick();
  while ((DCMIPP->CMSR1 & DCMIPP_CMSR1_P2CPTACT) != 0u) {
    if ((HAL_GetTick() - t0) > 1000u) return;
  }
  CLEAR_BIT(DCMIPP->P2FSCR, DCMIPP_P2FSCR_PIPEN);
  CLEAR_BIT(DCMIPP->P2PPCR, DCMIPP_P2PPCR_DBM);
  __HAL_DCMIPP_DISABLE_IT(&hdcmipp, DCMIPP_IT_PIPE2_FRAME | DCMIPP_IT_PIPE2_VSYNC |
                                    DCMIPP_IT_PIPE2_LINE | DCMIPP_IT_PIPE2_OVR);
  /* The start enables these interrupts without clearing their flags. */
  __HAL_DCMIPP_CLEAR_FLAG(&hdcmipp, DCMIPP_FLAG_PIPE2_FRAME | DCMIPP_FLAG_PIPE2_VSYNC |
                                    DCMIPP_FLAG_PIPE2_LINE | DCMIPP_FLAG_PIPE2_OVR);
  hdcmipp.PipeState[DCMIPP_PIPE2] = HAL_DCMIPP_PIPE_STATE_READY;
}

/* Returns 0 when the NN pipe's output is larger than the model's input buffer. */
static int nn_input_fits(const LL_Buffer_InfoTypeDef *in)
{
  static uint32_t s_big_said = N6_SAID_NEVER, s_shape_said = N6_SAID_NEVER;
  char msg[220];
  const uint32_t need = (uint32_t)g_nn_out_w * (uint32_t)g_nn_out_h * 3u;
  const uint32_t have = in[0].offset_end - in[0].offset_start;
  uint32_t mw = 0u, mh = 0u;
  if ((in[0].mem_shape != NULL) && (in[0].mem_ndims >= 4) && (in[0].mem_shape[3] == 3u))
  {
    mh = in[0].mem_shape[1];
    mw = in[0].mem_shape[2];
  }
  if (need > have)
  {
    if ((s_cdc_up != 0u) && (s_big_said != s_cdc_connects))
    {
      snprintf(msg, sizeof(msg),
               "[VISION] cfg.nn is %ux%u, more than the model's %lu-byte input takes, so no model "
               "ran. Set cfg.nn.width and cfg.nn.height to the model's input size.\r\n",
               (unsigned)g_nn_out_w, (unsigned)g_nn_out_h, (unsigned long)have);
      dbg_say_to_host(&s_big_said, msg);
    }
    return 0;
  }
  if ((mw != 0u) && ((mw != g_nn_out_w) || (mh != g_nn_out_h)) && (s_cdc_up != 0u) &&
      (s_shape_said != s_cdc_connects))
  {
    snprintf(msg, sizeof(msg),
             "[VISION WARN] cfg.nn is %ux%u but the model takes %lux%lu: set cfg.nn.width and "
             "cfg.nn.height to the model's input size, or its results will be misplaced.\r\n",
             (unsigned)g_nn_out_w, (unsigned)g_nn_out_h, (unsigned long)mw, (unsigned long)mh);
    dbg_say_to_host(&s_shape_said, msg);
  }
  return 1;
}

int NN_feed_start(Feed_Mode_t mode, NN_Instance_TypeDef *instance)
{
  const LL_Buffer_InfoTypeDef *nn_in_info = LL_ATON_Input_Buffers_Info(instance);
  uint8_t *nn_in = (uint8_t *)LL_Buffer_addr_start(&nn_in_info[0]);
  if (!nn_input_fits(nn_in_info)) return 0;

  while (pipe1ready == 0)
  {
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  pipe1ready = 0;
  n6_pipe2_release_held();
  cameraFrameReceived = 0;
  const uint32_t pic = nn_letterbox_prepare(nn_in);
  n6_dcmipp_prepare_start(DCMIPP_PIPE2);
  if (HAL_DCMIPP_CSI_PIPE_Start(&hdcmipp,
        DCMIPP_PIPE2,
        DCMIPP_VIRTUAL_CHANNEL0,
        (uint32_t)(nn_in + pic),
        (mode == FEED_CONTINUOUS) ? DCMIPP_MODE_CONTINUOUS
                                  : DCMIPP_MODE_SNAPSHOT) != HAL_OK)
  {
    Error_Handler();
  }
  return 1;
}

/* Raw pixels without a model: NN_capture_snapshot() on host transports,
 * LCD_GetCameraBuffer() under TRANSPORT_LCD. */

/* Reports a cause once to each host. bit has one bit set, one per cause. */
static int nn_snap_fail(uint8_t bit, const char *msg)
{
  static uint32_t s_said_on[8] = { N6_SAID_NEVER, N6_SAID_NEVER, N6_SAID_NEVER, N6_SAID_NEVER,
                                   N6_SAID_NEVER, N6_SAID_NEVER, N6_SAID_NEVER, N6_SAID_NEVER };
  dbg_say_to_host(&s_said_on[(uint32_t)__builtin_ctz((unsigned)bit) & 7u], msg);
  return 0;
}

int NN_capture_snapshot(void *dst, uint32_t dst_size)
{
  if (dst == NULL) return 0;

  if (!g_nn_pipe_ready || g_nn_out_w == 0u || g_nn_out_h == 0u)
    return nn_snap_fail(0x01u,
      "[NNSNAP] NN pipe not ready: call Vision.begin() first, with a camera "
      "resolution the NN pipe can scale to cfg.nn.\r\n");

  if (!g_n6_sensor_streaming)
    return nn_snap_fail(0x02u,
      "[NNSNAP] the sensor is not streaming yet, so no snapshot was taken.\r\n");

  const uint32_t need = (uint32_t)g_nn_out_w * (uint32_t)g_nn_out_h * 3u;
  if (dst_size < need)
    return nn_snap_fail(0x04u,
      "[NNSNAP] destination buffer too small: need cfg.nn.width * cfg.nn.height "
      "* 3 bytes (RGB888).\r\n");

  if (((uint32_t)dst & 0xFu) != 0u)
    return nn_snap_fail(0x08u,
      "[NNSNAP] the destination buffer is not 16-byte aligned, which the DCMIPP "
      "needs. Declare it with __attribute__((aligned(32))).\r\n");

  /* Two frame periods at the slowest rate is about 145 ms. */
  const uint32_t TIMEOUT_MS = 250u;
  uint32_t t0 = HAL_GetTick();

  while (pipe1ready == 0)
  {
    if ((HAL_GetTick() - t0) > TIMEOUT_MS)             /* the gate is not held here */
      return nn_snap_fail(0x10u,
        "[NNSNAP] timed out waiting for the NN pipe: a model pass or another "
        "snapshot holds it.\r\n");
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  pipe1ready = 0;

  /* Cleared after the release and before the start. */
  n6_pipe2_release_held();
  cameraFrameReceived = 0;
  const uint32_t pic = nn_letterbox_prepare((uint8_t *)dst);
  /* No dirty line of a cacheable dst may be evicted over the frame PIPE2 writes. */
  SCB_CleanInvalidateDCache_by_Addr(dst, (int32_t)need);

  n6_dcmipp_prepare_start(DCMIPP_PIPE2);
  if (HAL_DCMIPP_CSI_PIPE_Start(&hdcmipp,
                                DCMIPP_PIPE2,
                                DCMIPP_VIRTUAL_CHANNEL0,
                                (uint32_t)dst + pic,
                                DCMIPP_MODE_SNAPSHOT) != HAL_OK)
  {
    pipe1ready = 1;
    return nn_snap_fail(0x20u,
      "[NNSNAP] DCMIPP refused to start the NN pipe (HAL_ERROR). PIPE2 is not "
      "in READY state, or the CSI is not in serial mode.\r\n");
  }

  t0 = HAL_GetTick();
  while (cameraFrameReceived == 0)
  {
    if ((HAL_GetTick() - t0) > TIMEOUT_MS)
    {
      /* Stop the capture before releasing the gate. */
      n6_pipe2_release_held();
      pipe1ready = 1;
      return nn_snap_fail(0x40u,
        "[NNSNAP] no frame arrived within 250 ms. Check that the sensor is "
        "streaming.\r\n");
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  cameraFrameReceived = 0;
  nn_letterbox_finish((uint8_t *)dst);
  pipe1ready = 1;

  /* A cacheable dst must be 32-byte aligned, or the invalidate discards the data
   * sharing its end lines. */
  SCB_InvalidateDCache_by_Addr((void *)dst, (int32_t)need);

  return 1;
}

/* Returns the RGB565 frame PIPE0 fills, g_camera_full_w x g_camera_full_h, or NULL
 * when that feed is not running. It is single-buffered and non-cacheable. */
const uint16_t *LCD_GetCameraBuffer(void)
{
  if (!g_lcd_feed_active) return NULL;
  return (const uint16_t *)pipe2buf;
}

/* Neural network */

static void NeuralNetwork_init(void)
{
  LL_ATON_RT_RuntimeInit();
}

/* ATON error state, set by the NPU ISR in ll_aton_runtime.c. */
extern volatile uint32_t g_aton_err_irq_count, g_aton_err_irqs_last, g_aton_err_irqs_last_hi,
                         g_aton_err_ec_irq, g_aton_err_ec_bc, g_aton_err_no_owner,
                         g_aton_err_abort, g_aton_err_burst;

static void Run_Inference(NN_Instance_TypeDef *instance)
{
  LL_ATON_RT_RetValues_t ll_aton_rt_ret = LL_ATON_RT_DONE;
  LL_ATON_RT_Init_Network(instance);
  do
  {
    if (g_aton_err_abort)
      break;
    ll_aton_rt_ret = LL_ATON_RT_RunEpochBlock(instance);

    if (ll_aton_rt_ret == LL_ATON_RT_WFE)
    {
      /* Blocks on a semaphore that the epoch-done ISR and the error ISR both give. */
      LL_ATON_OSAL_WFE();
    }
  } while (ll_aton_rt_ret != LL_ATON_RT_DONE);

  if (g_aton_err_abort)
  {
    char eb[176];
    snprintf(eb, sizeof(eb),
             "[NPU] ATON error IRQ recovered: total=%lu INTREG=0x%08lX%08lX EC_IRQ=0x%08lX "
             "EC_BC=0x%08lX no_owner=%lu - frame dropped, ATON runtime recycled\r\n",
             (unsigned long)g_aton_err_irq_count, (unsigned long)g_aton_err_irqs_last_hi,
             (unsigned long)g_aton_err_irqs_last,
             (unsigned long)g_aton_err_ec_irq, (unsigned long)g_aton_err_ec_bc,
             (unsigned long)g_aton_err_no_owner);
    dbg_print(eb);
    LL_ATON_RT_DeInit_Network(instance);
    LL_ATON_RT_RuntimeDeInit();
    g_aton_err_burst = 0u;   /* the ATON IRQs are disabled here */
    g_aton_err_abort = 0u;
    LL_ATON_RT_RuntimeInit();
    return;
  }

  LL_ATON_RT_Reset_Network(instance);
  LL_ATON_RT_DeInit_Network(instance);
}

static void NPUCache_config(void)
{
  npu_cache_init();
  npu_cache_enable();
}

static void NPURam_enable(void)
{
  __HAL_RCC_NPU_CLK_ENABLE();
  __HAL_RCC_NPU_FORCE_RESET();
  __HAL_RCC_NPU_RELEASE_RESET();

  __HAL_RCC_AXISRAM3_MEM_CLK_ENABLE();
  __HAL_RCC_AXISRAM4_MEM_CLK_ENABLE();
  __HAL_RCC_AXISRAM5_MEM_CLK_ENABLE();
  __HAL_RCC_AXISRAM6_MEM_CLK_ENABLE();
  __HAL_RCC_RAMCFG_CLK_ENABLE();
  RAMCFG_HandleTypeDef hramcfg = {0};
  hramcfg.Instance =  RAMCFG_SRAM3_AXI;
  HAL_RAMCFG_EnableAXISRAM(&hramcfg);
  hramcfg.Instance =  RAMCFG_SRAM4_AXI;
  HAL_RAMCFG_EnableAXISRAM(&hramcfg);
  hramcfg.Instance =  RAMCFG_SRAM5_AXI;
  HAL_RAMCFG_EnableAXISRAM(&hramcfg);
  hramcfg.Instance =  RAMCFG_SRAM6_AXI;
  HAL_RAMCFG_EnableAXISRAM(&hramcfg);
}

static void NPU_NVIC_Config(void)
{
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  HAL_NVIC_SetPriority(NPU0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(NPU0_IRQn);

  HAL_NVIC_SetPriority(NPU1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(NPU1_IRQn);

  HAL_NVIC_SetPriority(NPU2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(NPU2_IRQn);

  HAL_NVIC_SetPriority(NPU3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(NPU3_IRQn);
}

/* Peripheral init */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef default_config = {0};
  MPU_Attributes_InitTypeDef attr_config = {0};
  uint32_t primask_bit = __get_PRIMASK();
  __disable_irq();

  HAL_MPU_Disable();

  attr_config.Attributes = INNER_OUTER(MPU_NOT_CACHEABLE);
  attr_config.Number = MPU_ATTRIBUTES_NUMBER0;

  HAL_MPU_ConfigMemoryAttributes(&attr_config);

  default_config.Enable = MPU_REGION_ENABLE;
  default_config.Number = MPU_REGION_NUMBER0;
  default_config.BaseAddress = __NON_CACHEABLE_SECTION_BEGIN;
  default_config.LimitAddress = __NON_CACHEABLE_SECTION_END;
  default_config.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  default_config.AccessPermission = MPU_REGION_ALL_RW;
  default_config.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  default_config.AttributesIndex = MPU_ATTRIBUTES_NUMBER0;
  HAL_MPU_ConfigRegion(&default_config);

  const uint32_t start = 0x70000000;
  const uint32_t end   = 0x71FFFFFF;

  default_config.Number = MPU_REGION_NUMBER1;
  default_config.BaseAddress      = start;
  default_config.LimitAddress     = end;

  HAL_MPU_ConfigRegion(&default_config);

  /* Region 2: the .axisram_nc section. Skipped when empty, as end - 1 would lie
   * below the base. */
  extern uint8_t __axisram_nc_start[];
  extern uint8_t __axisram_nc_end[];
  if ((uint32_t)__axisram_nc_end > (uint32_t)__axisram_nc_start) {
    default_config.Number       = MPU_REGION_NUMBER2;
    default_config.BaseAddress  = (uint32_t)__axisram_nc_start;
    default_config.LimitAddress = (uint32_t)__axisram_nc_end - 1u;
    HAL_MPU_ConfigRegion(&default_config);
  }

  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

  __set_PRIMASK(primask_bit);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);

  /* I2C1: PE6 SDA, PH9 SCL. */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /* GREEN LED on PD6 */
  GPIO_InitStruct.Pin = GREEN_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GREEN_LED_GPIO_Port, &GPIO_InitStruct);

  /* BLUE LED on PC1 */
  GPIO_InitStruct.Pin = BLUE_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BLUE_LED_GPIO_Port, &GPIO_InitStruct);

  /* RED LED on PE11 */
  GPIO_InitStruct.Pin = RED_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RED_LED_GPIO_Port, &GPIO_InitStruct);
}

/* I2C bus mutex, taken by every hi2c1 transaction. It is recursive, and a no-op
 * before the scheduler starts and in ISRs. */
static SemaphoreHandle_t s_i2c1_mutex = NULL;

void n6_i2c_lock(void)
{
  if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) { return; }
  if (xPortIsInsideInterrupt() != pdFALSE) { return; }

  if (s_i2c1_mutex == NULL) {
    vTaskSuspendAll();
    if (s_i2c1_mutex == NULL) {
      s_i2c1_mutex = xSemaphoreCreateRecursiveMutex();
    }
    (void)xTaskResumeAll();
  }
  if (s_i2c1_mutex != NULL) {
    (void)xSemaphoreTakeRecursive(s_i2c1_mutex, portMAX_DELAY);
  }
}

void n6_i2c_unlock(void)
{
  if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) { return; }
  if (xPortIsInsideInterrupt() != pdFALSE) { return; }
  if (s_i2c1_mutex != NULL) {
    (void)xSemaphoreGiveRecursive(s_i2c1_mutex);
  }
}

/* Holds the bus across several transactions. Claim and release must run on the
 * same task. */
void n6_i2c_claim(void)   { n6_i2c_lock(); }
void n6_i2c_release(void) { n6_i2c_unlock(); }

/* Returns 1 when the lock is taken, and also before the scheduler starts and in
 * ISRs. int, not bool: the VideoEncoder's basetype.h retypes bool before this. */
int n6_i2c_trylock(void)
{
  if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) { return 1; }
  if (xPortIsInsideInterrupt() != pdFALSE) { return 1; }

  if (s_i2c1_mutex == NULL) {
    vTaskSuspendAll();
    if (s_i2c1_mutex == NULL) {
      s_i2c1_mutex = xSemaphoreCreateRecursiveMutex();
    }
    (void)xTaskResumeAll();
  }
  if (s_i2c1_mutex == NULL) { return 1; }   /* matches n6_i2c_lock */
  return (xSemaphoreTakeRecursive(s_i2c1_mutex, 0) == pdTRUE) ? 1 : 0;
}

/* About 5 us at any CPU clock, with or without the scheduler. */
static void n6_i2c_bus_delay(void)
{
  for (volatile uint32_t n = SystemCoreClock / 800000u; n != 0u; n--) { }
}

/* Clocks the last boot sent to free I2C1, 0 when it was idle. */
volatile uint8_t g_n6_i2c1_unstuck_clocks = 0u;

/* Frees a bus a device holds after a reset mid-transfer. While SDA is low, up
 * to nine SCL pulses, SDA following each edge so every pulse ends in a STOP;
 * then a START and a STOP with SCL high. The caller enables the GPIO clocks
 * and gives the pins to the I2C peripheral after. Returns the pulses sent. */
uint32_t n6_i2c_bus_recover(GPIO_TypeDef *sda_port, uint16_t sda_pin,
                            GPIO_TypeDef *scl_port, uint16_t scl_pin)
{
  HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(sda_port, sda_pin, GPIO_PIN_SET);
  GPIO_InitTypeDef g = {0};
  g.Mode  = GPIO_MODE_OUTPUT_OD;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  g.Pin   = scl_pin;
  HAL_GPIO_Init(scl_port, &g);
  g.Pin   = sda_pin;
  HAL_GPIO_Init(sda_port, &g);
  n6_i2c_bus_delay();

  uint32_t clocks = 0u;
  while ((clocks < 9u) && (HAL_GPIO_ReadPin(sda_port, sda_pin) == GPIO_PIN_RESET)) {
    HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_RESET);
    n6_i2c_bus_delay();
    HAL_GPIO_WritePin(sda_port, sda_pin, GPIO_PIN_RESET);
    n6_i2c_bus_delay();
    HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_SET);
    n6_i2c_bus_delay();
    HAL_GPIO_WritePin(sda_port, sda_pin, GPIO_PIN_SET);   /* a STOP once SDA is free */
    n6_i2c_bus_delay();
    clocks++;
  }

  /* START (SDA falls) and STOP (SDA rises), SCL high throughout. */
  HAL_GPIO_WritePin(sda_port, sda_pin, GPIO_PIN_RESET);
  n6_i2c_bus_delay();
  HAL_GPIO_WritePin(sda_port, sda_pin, GPIO_PIN_SET);
  n6_i2c_bus_delay();
  return clocks;
}

static void MX_I2C1_Init(void)
{
  /* PE6 is SDA and PH9 SCL. HAL_I2C_MspInit gives them to I2C1. */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  g_n6_i2c1_unstuck_clocks = (uint8_t)n6_i2c_bus_recover(GPIOE, GPIO_PIN_6, GPIOH, GPIO_PIN_9);

  hi2c1.Instance = I2C1;
  /* About 295 kHz with PCLK1 at 200 MHz. The FLIR Lepton CCI limit is 400 kHz. */
  hi2c1.Init.Timing = 0x10707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* n6_i2c_lock() creates the bus mutex on first use. */
}

  /* LTDC pixel-clock divider from PLL4, read by HAL_LTDC_MspInit. Refresh is
   * PLL4 / (divider x HTOTAL x VTOTAL). */
  volatile uint32_t g_ltdc_clk_divider = 19u;   /* 40 Hz */

  /* Sets the divider for fps, clamped to 30 to 60 Hz. Call before MX_LTDC_Init. */
  void n6_lcd_set_fps(int fps)
  {
    if (fps < 30) fps = 30;
    if (fps > 60) fps = 60;
    const uint32_t ppf   = 976u * 528u;                 /* HTOTAL x VTOTAL, as in MX_LTDC_Init */
    const uint32_t denom = (uint32_t)fps * ppf;
    uint32_t div = (400000000u + denom / 2u) / denom;   /* PLL4 is 400 MHz */
    if (div < 1u)   div = 1u;
    if (div > 256u) div = 256u;                          /* IC divider field range */
    g_ltdc_clk_divider = div;
  }

  static void MX_LTDC_Init(void)
  {
    LTDC_LayerCfgTypeDef pLayerCfg = {0};

    hltdc.Instance = LTDC;

    hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
    hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
    hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
    hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;

    /* HTOTAL is 976, VTOTAL 528. LTDC fields hold value - 1. */
    const uint32_t HSW = 48;
    const uint32_t HBP = 88;
    const uint32_t HFP = 40;

    const uint32_t VSW = 3;
    const uint32_t VBP = 32;
    const uint32_t VFP = 13;

    hltdc.Init.HorizontalSync       = HSW - 1;
    hltdc.Init.VerticalSync         = VSW - 1;

    hltdc.Init.AccumulatedHBP       = (HSW + HBP) - 1;
    hltdc.Init.AccumulatedVBP       = (VSW + VBP) - 1;

    hltdc.Init.AccumulatedActiveW   = (HSW + HBP + 800) - 1;
    hltdc.Init.AccumulatedActiveH   = (VSW + VBP + 480) - 1;

    hltdc.Init.TotalWidth           = (HSW + HBP + 800 + HFP) - 1;
    hltdc.Init.TotalHeigh           = (VSW + VBP + 480 + VFP) - 1;

    hltdc.Init.Backcolor.Red   = 0;
    hltdc.Init.Backcolor.Green = 0;
    hltdc.Init.Backcolor.Blue  = 0;

    if (HAL_LTDC_Init(&hltdc) != HAL_OK) Error_Handler();

    /* Layer 0: the camera image, centred when smaller than the panel. */
    const uint32_t cam_w = (usb_frame_width  > 0u && usb_frame_width  <= LCD_W)
                              ? usb_frame_width  : LCD_W;
    const uint32_t cam_h = (usb_frame_height > 0u && usb_frame_height <= LCD_H)
                              ? usb_frame_height : LCD_H;
    const uint32_t layer_x = (LCD_W - cam_w) / 2u;
    const uint32_t layer_y = (LCD_H - cam_h) / 2u;

    pLayerCfg.WindowX0 = layer_x;
    pLayerCfg.WindowX1 = layer_x + cam_w;
    pLayerCfg.WindowY0 = layer_y;
    pLayerCfg.WindowY1 = layer_y + cam_h;
    pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
    pLayerCfg.FBStartAdress = (uint32_t) pipe2buf;
    pLayerCfg.Alpha  = 255;
    pLayerCfg.Alpha0 = 0;
    pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
    pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
    pLayerCfg.ImageWidth  = cam_w;
    pLayerCfg.ImageHeight = cam_h;

    g_lcd_camera_x = (int16_t)layer_x;
    g_lcd_camera_y = (int16_t)layer_y;
    g_lcd_camera_w = (uint16_t)cam_w;
    g_lcd_camera_h = (uint16_t)cam_h;

    if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK) Error_Handler();
    for (uint32_t i = 0; i < (LCD_W * LCD_H); i++)
    {
      overlay_buf0[i] = ARGB4444(0x0, 0, 0, 0);
      overlay_buf1[i] = ARGB4444(0x0, 0, 0, 0);
    }
    SCB_CleanDCache_by_Addr((uint32_t*)overlay_buf0, ALIGN_UP(OVL_BYTES, DCACHE_ALIGN));
    SCB_CleanDCache_by_Addr((uint32_t*)overlay_buf1, ALIGN_UP(OVL_BYTES, DCACHE_ALIGN));

    g_ovl_front = overlay_buf0;
    g_ovl_back  = overlay_buf1;

    pLayerCfg.FBStartAdress = (uint32_t)g_ovl_front;
    pLayerCfg.WindowX0 = 0;
    pLayerCfg.WindowX1 = 800;
    pLayerCfg.WindowY0 = 0;
    pLayerCfg.WindowY1 = 480;
    pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB4444;

    pLayerCfg.Alpha  = 255;
    pLayerCfg.Alpha0 = 0;
    /* Per-pixel alpha from ARGB4444. */
    pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
    pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
    pLayerCfg.ImageWidth  = 800;
    pLayerCfg.ImageHeight = 480;
    pLayerCfg.Backcolor.Red = 0;
    pLayerCfg.Backcolor.Green = 0;
    pLayerCfg.Backcolor.Blue = 0;

    if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 1) != HAL_OK) Error_Handler();
    if (HAL_LTDC_Reload(&hltdc, LTDC_RELOAD_IMMEDIATE) != HAL_OK) Error_Handler();
  }

/* Call after DCMIPP_LCD_Init() and before LCD_feed_start(). */
void LCD_Init(void)
{
  MX_LTDC_Init();
}

/* Starts PIPE0 capturing into pipe2buf. */
void LCD_feed_start(Feed_Mode_t mode)
{
  n6_dcmipp_prepare_start(DCMIPP_PIPE0);
  if (mode == FEED_CONTINUOUS)
  {
    if (HAL_DCMIPP_CSI_PIPE_Start(&hdcmipp,
          DCMIPP_PIPE0,
          DCMIPP_VIRTUAL_CHANNEL0,
          (uint32_t)pipe2buf,
          DCMIPP_MODE_CONTINUOUS) != HAL_OK)
    {
      Error_Handler();
    }
  }
  else /* FEED_SNAPSHOT */
  {
    if (HAL_DCMIPP_CSI_PIPE_Start(&hdcmipp,
          DCMIPP_PIPE0,
          DCMIPP_VIRTUAL_CHANNEL0,
          (uint32_t)pipe2buf,
          DCMIPP_MODE_SNAPSHOT) != HAL_OK)
    {
      Error_Handler();
    }
  }

  g_lcd_feed_active = 1u;
}

void MX_USB1_OTG_HS_PCD_Init(void)
{
  hpcd_USB_OTG_HS1.Instance = USB1_OTG_HS;
  hpcd_USB_OTG_HS1.Init.dev_endpoints = 9;
  hpcd_USB_OTG_HS1.Init.speed = PCD_SPEED_HIGH;
  hpcd_USB_OTG_HS1.Init.phy_itface = USB_OTG_HS_EMBEDDED_PHY;
  hpcd_USB_OTG_HS1.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_HS1.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_HS1.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_HS1.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_HS1.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_HS1.Init.dma_enable = ENABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_HS1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void SystemIsolation_Config(void)
{
  __HAL_RCC_RIFSC_CLK_ENABLE();
  RIMC_MasterConfig_t RIMC_master = {0};
  RIMC_master.MasterCID = RIF_CID_1;
  RIMC_master.SecPriv = RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV;

  HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_DCMIPP, &RIMC_master);
  HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_LTDC1, &RIMC_master);
  HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_VENC, &RIMC_master);

  HAL_PWR_ConfigAttributes(PWR_ITEM_0,PWR_SEC_NPRIV);

  HAL_GPIO_ConfigPinAttributes(GPIOC,GPIO_PIN_1,GPIO_PIN_SEC|GPIO_PIN_NPRIV);
  HAL_GPIO_ConfigPinAttributes(GPIOD,GPIO_PIN_2,GPIO_PIN_SEC|GPIO_PIN_NPRIV);
  HAL_GPIO_ConfigPinAttributes(GPIOD,GPIO_PIN_6,GPIO_PIN_SEC|GPIO_PIN_NPRIV);
  HAL_GPIO_ConfigPinAttributes(GPIOE,GPIO_PIN_5,GPIO_PIN_SEC|GPIO_PIN_NPRIV);
  HAL_GPIO_ConfigPinAttributes(GPIOE,GPIO_PIN_6,GPIO_PIN_SEC|GPIO_PIN_NPRIV);
}

static void Security_Config(void)
{
  __HAL_RCC_RIFSC_CLK_ENABLE();
  RIMC_MasterConfig_t RIMC_master = {0};
  RIMC_master.MasterCID = RIF_CID_1;
  RIMC_master.SecPriv = RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV;
  HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_NPU, &RIMC_master);
  HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_DMA2D, &RIMC_master);
  HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_DCMIPP, &RIMC_master);
  HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_LTDC1 , &RIMC_master);
  HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_LTDC2 , &RIMC_master);
  /* USB1 OTG is a DMA master under TinyUSB, so it needs the same attributes. */
  HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_OTG1  , &RIMC_master);
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_NPU , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_DMA2D , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_CSI    , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_DCMIPP , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_LTDC   , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_LTDCL1 , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_LTDCL2 , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
  /* A non-secure VENC reads zeros from the secure RISAF regions. */
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_VENC   , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);

  RISAF12->REG[0].CFGR = 0x00000000;
  RISAF12->REG[1].CFGR = 0x00000000;
  RISAF12->REG[0].CIDCFGR = 0x000F000F;
  RISAF12->REG[0].ENDR = 0xFFFFFFFF;
  RISAF12->REG[0].CFGR = 0x00000101;
  RISAF12->REG[1].CIDCFGR = 0x00FF00FF;
  RISAF12->REG[1].ENDR = 0xFFFFFFFF;
  RISAF12->REG[1].CFGR = 0x00000001;

  RISAF2->REG[0].CFGR = 0x00000000;
  RISAF2->REG[1].CFGR = 0x00000000;
  RISAF2->REG[0].CIDCFGR = 0x000F000F;
  RISAF2->REG[0].ENDR = 0xFFFFFFFF;
  RISAF2->REG[0].CFGR = 0x00000101;
  RISAF2->REG[1].CIDCFGR = 0x00FF00FF;
  RISAF2->REG[1].ENDR = 0xFFFFFFFF;
  RISAF2->REG[1].CFGR = 0x00000001;

  RISAF3->REG[0].CFGR = 0x00000000;
  RISAF3->REG[1].CFGR = 0x00000000;
  RISAF3->REG[0].CIDCFGR = 0x000F000F;
  RISAF3->REG[0].ENDR = 0xFFFFFFFF;
  RISAF3->REG[0].CFGR = 0x00000101;
  RISAF3->REG[1].CIDCFGR = 0x00FF00FF;
  RISAF3->REG[1].ENDR = 0xFFFFFFFF;
  RISAF3->REG[1].CFGR = 0x00000001;

  RISAF7->REG[0].CFGR = 0x00000000;
  RISAF7->REG[1].CFGR = 0x00000000;
  RISAF7->REG[0].CIDCFGR = 0x000F000F;
  RISAF7->REG[0].ENDR = 0xFFFFFFFF;
  RISAF7->REG[0].CFGR = 0x00000101;
  RISAF7->REG[1].CIDCFGR = 0x00FF00FF;
  RISAF7->REG[1].ENDR = 0xFFFFFFFF;
  RISAF7->REG[1].CFGR = 0x00000001;
}

static void IAC_Config(void)
{
  __HAL_RCC_IAC_CLK_ENABLE();
  __HAL_RCC_IAC_FORCE_RESET();
  __HAL_RCC_IAC_RELEASE_RESET();
}

/* Debug console */

static void dbg_print(const char *msg)
{
  if (tud_cdc_n_connected(0))
  {
    tud_cdc_n_write(0, (const uint8_t *)msg, strlen(msg));
    tud_cdc_n_write_flush(0);
  }
}

/* Internal detail, printed only when the debug console is built in. */
static void dbg_detail(const char *msg)
{
#if !defined(NEURON6_DEBUG_CONSOLE) || (NEURON6_DEBUG_CONSOLE != 0)
  dbg_print(msg);
#else
  (void)msg;
#endif
}

/* Writes msg whole or not at all. Returns 1 when it went. */
static int dbg_say(const char *msg)
{
  const uint32_t len = (uint32_t)strlen(msg);
  if (!tud_cdc_n_connected(0) || (tud_cdc_n_write_available(0) < len)) return 0;
  tud_cdc_n_write(0, (const uint8_t *)msg, len);
  tud_cdc_n_write_flush(0);
  return 1;
}

/* Says msg once to each host that connects. */
static void dbg_say_to_host(uint32_t *said_on, const char *msg)
{
  const uint32_t conn = s_cdc_connects;
  if ((s_cdc_up != 0u) && (*said_on != conn) && dbg_say(msg)) *said_on = conn;
}

/* Notices are printed and kept for the USB task to replay on each connection.
 * Any task may add one. Each is complete before s_notice_count counts it. */
#define N6_NOTICES     6u
#define N6_NOTICE_LEN  400u
static char s_notices[N6_NOTICES][N6_NOTICE_LEN];
static volatile uint32_t s_notice_count = 0u;

void NeuroN6_Notice(const char *msg)
{
  if (msg == NULL) return;
  dbg_print(msg);
  const bool running = (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED);
  if (running) vTaskSuspendAll();
  const uint32_t n = s_notice_count;
  bool keep = (n < N6_NOTICES);
  for (uint32_t i = 0u; keep && (i < n); i++)
  {
    if (strcmp(s_notices[i], msg) == 0) keep = false;
  }
  if (keep)
  {
    strncpy(s_notices[n], msg, N6_NOTICE_LEN - 1u);
    s_notices[n][N6_NOTICE_LEN - 1u] = '\0';
    if (strlen(msg) >= N6_NOTICE_LEN - 1u)          /* cut short: still end the line */
    {
      s_notices[n][N6_NOTICE_LEN - 3u] = '\r';
      s_notices[n][N6_NOTICE_LEN - 2u] = '\n';
    }
    s_notice_count = n + 1u;
  }
  if (running) (void)xTaskResumeAll();
}

void NeuroN6_DbgPrint(const char *msg)
{
  dbg_print(msg);
}

/* newlib stdio sink, called by _write() in syscalls.c. It never blocks. */
int __io_putchar(int ch)
{
  uint8_t c = (uint8_t)ch;

  if (tud_cdc_n_connected(0))
  {
    tud_cdc_n_write(0, &c, 1u);
    if (c == (uint8_t)'\n') { tud_cdc_n_write_flush(0); }
  }
  return ch;
}

/* Called by _write() after its characters: starts a transfer for the end of a
 * line, which the CDC stack otherwise holds until a newline or a full packet. */
void __io_flush(void)
{
  if (tud_cdc_n_connected(0)) { tud_cdc_n_write_flush(0); }
}

/* Replaces newlib's assertion handler. Prints, then spins forever. */
void __assert_func(const char *file, int line, const char *func, const char *expr)
{
  char msg[192];

  (void)snprintf(msg, sizeof msg,
                 "\r\n[ASSERT] %s:%d in %s(): '%s'\r\n",
                 (file != NULL) ? file : "?", line,
                 (func != NULL) ? func : "?",
                 (expr != NULL) ? expr : "?");
  dbg_print(msg);

  for (;;) { __NOP(); }
}

/* Public API */

/* The reset flags and the Standby flag this boot found. */
static uint32_t s_boot_rsr;
static uint32_t s_boot_sbf;

uint32_t n6_boot_rsr(void) { return s_boot_rsr; }
uint32_t n6_boot_sbf(void) { return s_boot_sbf; }

void NeuroN6_Setup(void)
{
  /* Read before setup() runs: sleepStop() clears SBF. SBF stays set through
   * later resets until CSSF clears it. */
  s_boot_rsr = RCC->RSR;
  RCC->RSR |= RCC_RSR_RMVF;
  s_boot_sbf = PWR->CPUCR & PWR_CPUCR_SBF;
  if (s_boot_sbf != 0u) PWR->CPUCR |= PWR_CPUCR_CSSF;

  SCB_DisableDCache();
  MPU_Config();
  SCB_EnableICache();
  /* Every DMA buffer is in a non-cacheable region or is cleaned explicitly. */
  SCB_EnableDCache();

  HAL_Init();

  /* DWT cycle counter, used by delayMicroseconds(). */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

  Security_Config();

  IAC_Config();

  MX_GPIO_Init();

  n6_run_led_apply();

#ifdef NEURON6_BOOTDIAG
  { extern void n6_bootdiag_mark(int); n6_bootdiag_mark(1); }
#endif

  SystemIsolation_Config();

  MX_USB1_OTG_HS_PCD_Init();

  MX_I2C1_Init();

  HAL_Delay(200);

  /* USB enumerates before the scheduler starts. The vendored port.c and
   * osal_freertos.h carry changes that allow it. */
  tusb_init();

#ifdef NEURON6_BOOTDIAG
  { extern void n6_bootdiag_mark(int); n6_bootdiag_mark(2); }
#endif

  vnd_meta_init(NULL, 0u);
  uint32_t usb_wait_start = HAL_GetTick();
  while (!tud_mounted() && (HAL_GetTick() - usb_wait_start) < 500) {
    tud_task_ext(0, false);   /* tud_task() blocks under OPT_OS_FREERTOS */
    HAL_Delay(10);
  }

#ifdef NEURON6_BOOTDIAG
  { extern void n6_bootdiag_mark(int); n6_bootdiag_mark(3); }
#endif

  if (_neuron6_csi_lane_mapping != NEURON6_CAMERA_NONE) {
    CSI_Init();
  } else {
    NeuroN6_DbgPrint("[NeuroN6] No camera sensor declared - skipping camera/CSI "
                     "init. Declare a sensor object for vision sketches, e.g. "
                     "`OV5640 camera;` (with #include <OV5640_Arduino.h>).\r\n");
  }
}

void NeuroN6_App(void)
{
  /* Called once per loop by main.cpp. Idempotent. */
  NeuroN6_StreamUsbOnlyStart();
}

void NPU_Init(void)
{
  NPURam_enable();
  NPUCache_config();
  NPU_NVIC_Config();
  NeuralNetwork_init();

  /* SYSCFG->NPUNICQOSCR stays at its boot default. */
}

/* Runs the model on its input buffer as it is, with no frame wait. The caller
 * cleans the D-cache over the buffer if the CPU wrote it. */
void NPU_Run_Inference_Direct(NN_Instance_TypeDef *instance)
{
  if (instance == NULL) return;
  if (g_nn_inference_on) { Run_Inference(instance); }
}

int NPU_Run_Inference(NN_Instance_TypeDef *instance)
{
  /* Waits up to 1 s for the frame NN_feed_start armed. Returns int, not bool, for
   * the same reason as n6_i2c_trylock. */
  static uint32_t s_said = N6_SAID_NEVER;
  const uint32_t t0 = HAL_GetTick();
  while (cameraFrameReceived == 0)
  {
    if ((HAL_GetTick() - t0) > 1000u)
    {
      /* Stop the capture before releasing the gate. */
      n6_pipe2_release_held();
      pipe1ready = 1;
      dbg_say_to_host(&s_said, "[VISION] no camera frame in 1 s, so no model ran. Check the "
                               "camera cable; on the TFT carrier declare "
                               "OV5640 camera(CARRIER_TFT).\r\n");
      return 0;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  cameraFrameReceived = 0;
  s_said = N6_SAID_NEVER;   /* a later loss is reported again */
  if ((s_nn_pic_w != 0u) && (s_nn_pic_w < g_nn_out_w))
  {
    const LL_Buffer_InfoTypeDef *in = LL_ATON_Input_Buffers_Info(instance);
    nn_letterbox_finish((uint8_t *)LL_Buffer_addr_start(&in[0]));
  }

  ts[0] = HAL_GetTick();
  if (g_nn_inference_on) { Run_Inference(instance); }
  ts[1] = HAL_GetTick();

  pipe1ready = 1;
  return 1;
}

/* Capture and USB tasks. n6_capture encodes and posts frames, n6_usb sends them. */

QueueHandle_t g_n6_video_usb_queue  = NULL;   /* USB and WiFi, drained by n6_usb_task */
QueueHandle_t g_n6_video_eth_queue  = NULL;   /* Ethernet, drained by EthernetN6 */

/* Set by Vision.begin() from cfg.transport. */
volatile n6_transport_t g_n6_active_transport = TRANSPORT_USB;

/* Weak defaults for the MayaW476 library hooks. */
__attribute__((weak))
int MayaW476_Start(int transport_mode)
{
  (void)transport_mode;
  return MAYAW476_NOT_LINKED;
}

/* 0 means no address. */
__attribute__((weak))
uint32_t MayaW476_GetIPv4_BE(void)
{
  return 0U;
}

__attribute__((weak))
uint16_t MayaW476_GetIPv4_Port(void)
{
  return 0U;
}

__attribute__((weak))
uint32_t MayaW476_SoftApAnnounce(uint8_t *out, uint32_t cap)
{
  (void)out;
  (void)cap;
  return 0U;
}

/* Weak defaults for the EthernetN6 library hooks. */
__attribute__((weak))
int EthernetN6_Start(int transport_mode)
{
  (void)transport_mode;
  return ETHERNETN6_NOT_LINKED;
}

__attribute__((weak))
bool EthernetN6_TxBusy(void)
{
  return false;
}

__attribute__((weak))
uint32_t EthernetN6_GetIPv4_BE(void)
{
  return 0U;
}

__attribute__((weak))
uint16_t EthernetN6_GetIPv4_Port(void)
{
  return 0U;
}

bool n6_usb_tx_busy(void)
{
  return vnd_tx_active;
}

bool n6_eth_tx_busy(void)
{
  return EthernetN6_TxBusy();
}

bool n6_video_tx_busy(void)
{
  return (g_n6_active_transport == TRANSPORT_ETHERNET) ? n6_eth_tx_busy()
                                                       : n6_usb_tx_busy();
}

/* The queue the producer posts to. */
static inline QueueHandle_t n6_video_queue(void)
{
  return (g_n6_active_transport == TRANSPORT_ETHERNET) ? g_n6_video_eth_queue
                                                       : g_n6_video_usb_queue;
}

/* Task handles, NULL until spawned. */
static TaskHandle_t s_n6_capture_handle = NULL;
static TaskHandle_t s_n6_usb_handle     = NULL;

/* Serves USB from boot until the USB task starts. Only one task calls
 * tud_task_ext() at a time. */
static TaskHandle_t  s_n6_boot_pump_handle = NULL;
static volatile bool s_n6_boot_pump_done   = false;

static void n6_usb_boot_pump_task(void *arg)
{
  (void)arg;
  while (s_n6_usb_handle == NULL) {
    tud_task_ext(0, false);
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  s_n6_boot_pump_done = true;
  vTaskDelete(NULL);
}

/* Called by main() before the sketch task is created. */
void NeuroN6_UsbBootPumpStart(void)
{
  if (s_n6_boot_pump_handle != NULL || s_n6_usb_handle != NULL) return;
  (void)xTaskCreate(n6_usb_boot_pump_task, "n6_usbboot",
                    4096u / sizeof(StackType_t),
                    NULL, tskIDLE_PRIORITY + 3u, &s_n6_boot_pump_handle);
}

/* HardwareSerial pumps TinyUSB itself until this returns 1. */
int NeuroN6_UsbPumpTaskStarted(void)
{
  return (s_n6_usb_handle != NULL || s_n6_boot_pump_handle != NULL) ? 1 : 0;
}

#ifdef NEURON6_BOOTDIAG
/* Boot stage LEDs, built with -DNEURON6_BOOTDIAG. The LEDs are active-low.
 *   1 green        NeuroN6_Setup, GPIO up
 *   2 blue         tusb_init() returned
 *   3 green+blue   USB wait done
 *   4 red+green    about to start the scheduler
 *   5 all three    ArduinoLoop task running */
void n6_bootdiag_mark(int stage)
{
  GPIO_PinState r = GPIO_PIN_SET, g = GPIO_PIN_SET, b = GPIO_PIN_SET; /* off */
  if (stage == 1 || stage == 3 || stage == 4 || stage == 5) { g = GPIO_PIN_RESET; }
  if (stage == 2 || stage == 3 || stage == 5) { b = GPIO_PIN_RESET; }
  if (stage == 4 || stage == 5) { r = GPIO_PIN_RESET; }
  HAL_GPIO_WritePin(RED_LED_GPIO_Port,   RED_LED_Pin,   r);
  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, g);
  HAL_GPIO_WritePin(BLUE_LED_GPIO_Port,  BLUE_LED_Pin,  b);
  HAL_Delay(700);
  if (stage != 5) {   /* stage 5 stays lit */
    HAL_GPIO_WritePin(RED_LED_GPIO_Port,   RED_LED_Pin,   GPIO_PIN_SET);
    HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BLUE_LED_GPIO_Port,  BLUE_LED_Pin,  GPIO_PIN_SET);
    HAL_Delay(200);
  }
}

void n6_bootdiag_assert_blink(void)
{
  __disable_irq();
  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BLUE_LED_GPIO_Port,  BLUE_LED_Pin,  GPIO_PIN_SET);
  for (;;) {
    HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
    for (volatile uint32_t i = 0; i < 8000000u; i++) { }
  }
}
#endif /* NEURON6_BOOTDIAG */

/* HardwareSerial.cpp: the Serial RX ring. */
extern void NeuroN6_SerialRxSink(uint8_t ch);
extern uint32_t NeuroN6_SerialRxSpace(void);

/* True when the USB task may take a CDC byte: the Serial ring has room, or, with
 * the debug console built in, it has stayed full for 1 s. */
static uint32_t s_rx_full_since = 0u;
static bool n6_serial_rx_open(void)
{
  if (NeuroN6_SerialRxSpace() > 0u) {
    s_rx_full_since = 0u;
    return true;
  }
#if !defined(NEURON6_DEBUG_CONSOLE) || (NEURON6_DEBUG_CONSOLE != 0)
  uint32_t now = HAL_GetTick();
  if (s_rx_full_since == 0u) {
    s_rx_full_since = now | 1u;   /* 0 means not full */
    return false;
  }
  return (now - s_rx_full_since) >= 1000u;
#else
  return false;
#endif
}

/* Posts an encoded frame to the active transport's queue. Requires
 * 0 < sz <= g_mjpeg_max. */
static void n6_post_encoded_frame(uint8_t *out_buf, uint32_t sz)
{
  g_dbg_encode_ok++;
  g_dbg_last_jpeg_sz = sz;

  n6_frame_msg_t msg = { 0 };
  msg.jpeg_ptr = out_buf;
  msg.jpeg_len = sz;
  /* The metadata snapshot takes the same parity as the JPEG buffer. */
  (void)vnd_meta_get_pingpong(g_enc_seq & 1u, &msg.meta_ptr, &msg.meta_len);

  n6_frame_tap(out_buf, sz, msg.meta_ptr, msg.meta_len);

  const QueueHandle_t q = n6_video_queue();
  if (q != NULL && xQueueSend(q, &msg, 0) == pdTRUE) {
    g_enc_seq++;
  }
}

/* Capture task */

/* The capture task waits five frame periods for a frame end. */
#define N6_ARM_BLANKING_WAIT_MIN_MS         150u    /* floor of that wait */
#define N6_ARM_BLANKING_WAIT_UNMEASURED_MS  1000u   /* the wait while no period is known */

/* Set while the capture task waits for a frame end. The PIPE1 callback then
 * notifies it. */
static volatile uint8_t s_capture_awaits_frame_end = 0u;

/* Set when the last encode returned a frame. */
static bool s_arm_follows_encode = false;

static inline bool n6_pipe1_in_frame(void)
{
  return (g_dcmipp_wr_mblines != 0u) || ((DCMIPP->CMSR1 & DCMIPP_CMSR1_P1CPTACT) != 0u);
}

static void n6_capture_task(void *arg)
{
  (void)arg;

  for (;;) {
    /* One-time line-buffer setup, done in this task so it serialises with the encode. */
    {
      static bool s_lb_setup_done = false;
      if (g_n6_usb_enabled && !s_lb_setup_done) {
        /* Base priority. The line-buffer branch boosts it per frame. */
        vTaskPrioritySet(NULL, tskIDLE_PRIORITY + 3u);
        /* ensure_init reads g_venc_lb_hw, so it is set first. */
        g_venc_lb_hw = 1u;
        g_lb_dcmipp_continuous = 0u;
        encoder_jpeg_linebuf_ensure_init((int32_t)usb_frame_width,
                                         (int32_t)usb_frame_height, g_mjpeg_max);
        if (g_venc_band == NULL) {
          char lbf[140];
          snprintf(lbf, sizeof(lbf),
            "[VISION] the encoder line buffer could not be set up at this resolution, "
            "so there is no video stream (JpegEncInit %ld).\r\n",
            (long)g_dbg_lb_init_ret);
          NeuroN6_Notice(lbf);
        } else {
          char lba[110];
          snprintf(lba, sizeof(lba), "[LBALLOC] band=%luB addr=0x%08lX OK - LINEBUF HW live\r\n",
            (unsigned long)g_venc_band_size, (unsigned long)(uint32_t)g_venc_band);
          dbg_detail(lba);
        }
        s_lb_setup_done = true;
      }
    }

    /* Deferred sensor bring-up. It must run in this task, which does not pump tud_task. */
    {
      static bool s_deferred_bringup_done = false;
      if (!s_deferred_bringup_done) {
        s_deferred_bringup_done = true;
        n6_sensor_bringup();
      }
    }

    if (g_venc_band != NULL && g_n6_sensor_streaming && g_n6_usb_enabled) {
      bool can_encode; uint8_t *out_buf;
      if (mjpeg_buf1 != NULL) {
        can_encode = ((g_enc_seq - g_tx_seq) < 2u);
        out_buf = (g_enc_seq & 1u) ? mjpeg_buf1 : mjpeg_buf0;
      } else {
        const QueueHandle_t q = n6_video_queue();
        can_encode = !n6_video_tx_busy() &&
                     (q == NULL || uxQueueMessagesWaiting(q) == 0u);
        out_buf = mjpeg_buf0;
      }
      if (!can_encode) {
        s_arm_follows_encode = false;
        /* Base priority while the consumer drains. */
        vTaskPrioritySet(NULL, tskIDLE_PRIORITY + 3u);
        vTaskDelay(pdMS_TO_TICKS(1));
        continue;
      }
      /* Arm only between camera frames. An arm that follows an encode straight away
       * is gated on the line counter alone. */
      const bool follow_on = s_arm_follows_encode;
      s_arm_follows_encode = false;
      if (g_lb_dcmipp_continuous &&
          (follow_on ? (g_dcmipp_wr_mblines != 0u) : n6_pipe1_in_frame())) {
        vTaskPrioritySet(NULL, tskIDLE_PRIORITY + 4u);
        (void)ulTaskNotifyTake(pdTRUE, 0);                /* drop a stale wake */
        s_capture_awaits_frame_end = 1u;
        const TickType_t t0 = xTaskGetTickCount();
        const uint32_t period_ms = s_pipe1_frame_ms;
        uint32_t limit_ms = (period_ms != 0u) ? (5u * period_ms)
                                              : N6_ARM_BLANKING_WAIT_UNMEASURED_MS;
        if (limit_ms < N6_ARM_BLANKING_WAIT_MIN_MS) limit_ms = N6_ARM_BLANKING_WAIT_MIN_MS;
        const TickType_t limit = pdMS_TO_TICKS(limit_ms);
        bool frame_ended = false;
        bool timed_out = false;
        while (n6_pipe1_in_frame()) {
          const TickType_t waited = xTaskGetTickCount() - t0;
          if (waited >= limit) { timed_out = true; break; }
          if (ulTaskNotifyTake(pdTRUE, limit - waited) != 0u) frame_ended = true;
        }
        s_capture_awaits_frame_end = 0u;
        if (!g_n6_sensor_streaming) {                   /* stopped while waiting */
          vTaskPrioritySet(NULL, tskIDLE_PRIORITY + 3u);
          continue;
        }
        if (timed_out && !frame_ended) {
          g_lb_dcmipp_continuous = 0u;
          g_lb_venc_reset_pending = 1u;
        }
      }
      /* One priority above the USB task, so the tick cannot slice the re-arm. */
      vTaskPrioritySet(NULL, tskIDLE_PRIORITY + 4u);
      uint32_t _t0 = HAL_GetTick();
      uint32_t sz = n6_linebuf_capture_one_frame(out_buf);
      s_arm_follows_encode = (sz != 0u);
      g_dbg_last_enc_ms = HAL_GetTick() - _t0;
      /* A gap that reached the band's capacity means a torn frame, which is dropped. */
      static uint32_t s_lb_torn = 0u;
      bool torn = (g_dbg_lb_gap_max >= (g_venc_band_depth * 2u));
      if (sz > 0u && sz <= g_mjpeg_max && !torn) {
        vTaskPrioritySet(NULL, tskIDLE_PRIORITY + 4u);
        n6_post_encoded_frame(out_buf, sz);
        s_lb_torn = 0u;
      } else {
        vTaskPrioritySet(NULL, tskIDLE_PRIORITY + 3u);
        if (!torn) g_dbg_encode_fail++;
        else if (++s_lb_torn >= 2u) { g_lb_dcmipp_continuous = 0u; g_lb_venc_reset_pending = 1u; s_lb_torn = 0u; }
      }
      continue;
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

/* USB task */

/* IP announce: a meta-only frame with one META_BOARD_IPV4 TLV, 31 bytes.
 *   20 B  vnd_frame_hdr_t, meta_len 11, payload_len 0
 *   3 B   TLV header: type 0x20, u16 LE len 8
 *   4 B   IPv4 octets, MSB first
 *   4 B   u32 LE port
 * META_BOARD_IPV4 must match the PostProcess TLV registry. */
#define IP_ANNOUNCE_INTERVAL_MS   1000U
#define IP_ANNOUNCE_DEFAULT_PORT  5000U
#define META_BOARD_IPV4           0x20U

static uint8_t  s_ip_announce_buf[11];
static uint32_t s_last_ip_announce_ms = 0U;

static inline uint32_t n6_active_ipv4_be(void)
{
  return (g_n6_active_transport == TRANSPORT_ETHERNET)
           ? EthernetN6_GetIPv4_BE() : MayaW476_GetIPv4_BE();
}
static inline uint16_t n6_active_ipv4_port(void)
{
  return (g_n6_active_transport == TRANSPORT_ETHERNET)
           ? EthernetN6_GetIPv4_Port() : MayaW476_GetIPv4_Port();
}

static inline bool ip_announce_should_emit(void)
{
  if (g_n6_active_transport == TRANSPORT_USB) return false;
  if (vnd_tx_active) return false;
  if (n6_active_ipv4_be() == 0U) return false;
  uint32_t now = HAL_GetTick();
  if ((now - s_last_ip_announce_ms) < IP_ANNOUNCE_INTERVAL_MS) return false;
  s_last_ip_announce_ms = now;
  return true;
}

/* Sends a meta-only frame with one TLV. The value is already at &buf[3]. */
static inline bool n6_meta_announce(uint8_t *buf, uint8_t tag, uint16_t len)
{
  buf[0] = tag;
  buf[1] = (uint8_t)(len & 0xFFu);
  buf[2] = (uint8_t)((len >> 8) & 0xFFu);
  return vnd_tx_start_frame(buf, 3U + (uint32_t)len, NULL, 0U, 0U);
}

/* buf holds the value after three bytes of headroom and must outlive the transfer.
 * Returns false when the transmitter is busy. */
bool NeuroN6_MetaAnnounce(uint8_t *buf, uint8_t tag, uint16_t len)
{
  if (buf == NULL) return false;
  return n6_meta_announce(buf, tag, len);
}

/* Writes the 8-byte META_BOARD_IPV4 value at v. */
static void ip_announce_value(uint8_t *v)
{
  const uint32_t ip = n6_active_ipv4_be();
  uint16_t port = n6_active_ipv4_port();
  if (port == 0U) port = IP_ANNOUNCE_DEFAULT_PORT;

  memcpy(v, &ip, 4U);

  v[4] = (uint8_t)(port & 0xFFU);
  v[5] = (uint8_t)((port >> 8) & 0xFFU);
  v[6] = 0U;
  v[7] = 0U;
}

static inline void ip_announce_emit(void)
{
  ip_announce_value(&s_ip_announce_buf[3]);
  n6_meta_announce(s_ip_announce_buf, META_BOARD_IPV4, 8U);
}

/* Writes the announce as one whole frame. Returns its length, or 0. */
uint32_t NeuroN6_IpAnnounceFrame(uint8_t *out, uint32_t cap)
{
  const vnd_frame_hdr_t hdr = { VND_MAGIC, 0U, sizeof(s_ip_announce_buf), 0U, 0U };
  const uint32_t len = sizeof(hdr) + sizeof(s_ip_announce_buf);
  if (out == NULL || cap < len || n6_active_ipv4_be() == 0U) return 0U;

  memcpy(out, &hdr, sizeof(hdr));
  out[sizeof(hdr)]      = META_BOARD_IPV4;
  out[sizeof(hdr) + 1U] = 8U;
  out[sizeof(hdr) + 2U] = 0U;
  ip_announce_value(&out[sizeof(hdr) + 3U]);
  return len;
}

/* SoftAP announce: one META_BOARD_SOFTAP TLV. It starts only with no network
 * host attached, and vnd_tx_pump() drops it if one attaches part way. */
#define SOFTAP_ANNOUNCE_INTERVAL_MS  1000U
#define META_BOARD_SOFTAP            0x24U

static uint32_t s_last_softap_announce_ms = 0U;

static inline void softap_announce_service(void)
{
  if (vnd_tx_active || s_net != NULL) return;
  const uint32_t now = HAL_GetTick();
  if ((now - s_last_softap_announce_ms) < SOFTAP_ANNOUNCE_INTERVAL_MS) return;
  const uint32_t len = MayaW476_SoftApAnnounce(&s_softap_announce_buf[3],
                                               sizeof(s_softap_announce_buf) - 3U);
  if (len == 0U || !n6_meta_announce(s_softap_announce_buf, META_BOARD_SOFTAP, (uint16_t)len)) {
    return;
  }
  s_last_softap_announce_ms = now;
}

/* ISP tuning mode announce */
#define ISP_ANNOUNCE_INTERVAL_MS  1000U
#define META_VD66_ISP_ENABLED     0x40U

static uint8_t  s_isp_announce_buf[4];
static uint32_t s_last_isp_announce_ms = 0U;

static inline bool isp_announce_should_emit(void)
{
  if (g_isp_tuning_mode == 0u) return false;
  if (vnd_tx_active) return false;
  uint32_t now = HAL_GetTick();
  if ((now - s_last_isp_announce_ms) < ISP_ANNOUNCE_INTERVAL_MS) return false;
  s_last_isp_announce_ms = now;
  return true;
}

static inline void isp_announce_emit(void)
{
  s_isp_announce_buf[3] = g_isp_tuning_mode;
  n6_meta_announce(s_isp_announce_buf, META_VD66_ISP_ENABLED, 1U);
}

/* Camera control capability announce */
#define CAM_CAPS_ANNOUNCE_INTERVAL_MS  1000U
#define META_CAM_CAPS                  0x64U

static uint8_t  s_cam_caps_buf[4];
static uint32_t s_last_cam_caps_ms = 0U;

static inline bool cam_caps_announce_should_emit(void)
{
  if (n6_sensor_control_caps() == 0u) return false;
  if (vnd_tx_active) return false;
  uint32_t now = HAL_GetTick();
  if ((now - s_last_cam_caps_ms) < CAM_CAPS_ANNOUNCE_INTERVAL_MS) return false;
  s_last_cam_caps_ms = now;
  return true;
}

static inline void cam_caps_announce_emit(void)
{
  s_cam_caps_buf[3] = (uint8_t)(n6_sensor_control_caps() & 0xFFu);
  n6_meta_announce(s_cam_caps_buf, META_CAM_CAPS, 1U);
}

/* Live-tuning descriptor announce */
#define TUNE_ANNOUNCE_INTERVAL_MS  1000U
#define META_TUNE_DESC             0x65U

static uint8_t  s_tune_buf[3U + N6_TUNE_DESC_MAX_BYTES];
static uint32_t s_last_tune_announce_ms = 0U;

static inline bool tune_announce_should_emit(void)
{
  if (n6_tune_count() == 0u) return false;
  if (vnd_tx_active) return false;
  uint32_t now = HAL_GetTick();
  if ((now - s_last_tune_announce_ms) < TUNE_ANNOUNCE_INTERVAL_MS) return false;
  s_last_tune_announce_ms = now;
  return true;
}

static inline void tune_announce_emit(void)
{
  const uint32_t vlen = n6_tune_build_descriptor(&s_tune_buf[3],
                                                 (uint32_t)sizeof(s_tune_buf) - 3U);
  if (vlen == 0u) return;

  n6_meta_announce(s_tune_buf, META_TUNE_DESC, (uint16_t)vlen);
}

/* Applies an ISP0 packet to the PIPE1 ISP. RAW8 and manual mode only.
 *   [0..3]: magic
 *   [4..9]: shift, mult for R, then G, then B. Gain = mult * 2^shift / 128.
 *   [10]: black level
 *   [11]: demosaic sharpness, 0..7
 *   [12]: flags, bit0 gamma enable
 *   [13]: reserved
 *   [14..31]: CCM, 9 x int16 LE, RR RG RB GR GG GB BR BG BB, 256 = 1.0. Optional. */
static void n6_isp_apply_isp0(const uint8_t *p, uint32_t len)
{
  if (g_isp_tuning_mode != 1u) return;
  if (g_n6_sensor_format.csi_data_type != DCMIPP_DT_RAW8) return;
  if (len < 13u) return;

  uint8_t sharp = p[11]; if (sharp > 7u) sharp = 7u;

  DCMIPP_ExposureConfTypeDef wb = {
    .ShiftRed   = p[4], .MultiplierRed   = p[5],
    .ShiftGreen = p[6], .MultiplierGreen = p[7],
    .ShiftBlue  = p[8], .MultiplierBlue  = p[9],
  };
  (void)HAL_DCMIPP_PIPE_SetISPExposureConfig(&hdcmipp, DCMIPP_PIPE1, &wb);
  (void)HAL_DCMIPP_PIPE_EnableISPExposure(&hdcmipp, DCMIPP_PIPE1);

  DCMIPP_BlackLevelConfTypeDef blc = {
    .RedCompBlackLevel = p[10], .GreenCompBlackLevel = p[10], .BlueCompBlackLevel = p[10],
  };
  (void)HAL_DCMIPP_PIPE_SetISPBlackLevelCalibrationConfig(&hdcmipp, DCMIPP_PIPE1, &blc);
  (void)HAL_DCMIPP_PIPE_EnableISPBlackLevelCalibration(&hdcmipp, DCMIPP_PIPE1);

  DCMIPP_RawBayer2RGBConfTypeDef demo = {0};
  demo.RawBayerType  = g_n6_raw_bayer_order;
  demo.PeakStrength  = sharp;
  demo.EdgeStrength  = sharp;
  demo.VLineStrength = sharp;
  demo.HLineStrength = sharp;
  (void)HAL_DCMIPP_PIPE_SetISPRawBayer2RGBConfig(&hdcmipp, DCMIPP_PIPE1, &demo);
  (void)HAL_DCMIPP_PIPE_EnableISPRawBayer2RGB(&hdcmipp, DCMIPP_PIPE1);

  /* P1GMCR written directly: the HAL has no gamma disable call. */
  if ((p[12] & 0x01u) != 0u) SET_BIT(DCMIPP->P1GMCR, DCMIPP_P1GMCR_ENABLE);
  else                       CLEAR_BIT(DCMIPP->P1GMCR, DCMIPP_P1GMCR_ENABLE);

  if (len >= 32u) {
    int16_t m[9];
    for (int i = 0; i < 9; i++) {
      m[i] = (int16_t)((uint16_t)p[14 + i * 2] | ((uint16_t)p[15 + i * 2] << 8));
    }
    DCMIPP_ColorConversionConfTypeDef ccm = {
      .RR = m[0], .RG = m[1], .RB = m[2],
      .GR = m[3], .GG = m[4], .GB = m[5],
      .BR = m[6], .BG = m[7], .BB = m[8],
    };
    (void)HAL_DCMIPP_PIPE_SetISPColorConversionConfig(&hdcmipp, DCMIPP_PIPE1, &ccm);
    (void)HAL_DCMIPP_PIPE_EnableISPColorConversion(&hdcmipp, DCMIPP_PIPE1);
  }
}

/* Watchdog and boot report. The HAL IWDG module is not compiled in, so the
 * registers are written directly. */
static char s_boot_line[256];

/* Thermal pump counters: frames sent, and frames skipped for want of an idle slot. */
static volatile uint32_t g_dbg_th_tx = 0u;
static volatile uint32_t g_dbg_th_skip = 0u;

/* Called from USB1_OTG_HS_IRQHandler to wake n6_usb_task. The USB IRQ priority
 * must be at or below configMAX_SYSCALL_INTERRUPT_PRIORITY. */
void NeuroN6_UsbIrqKick(void)
{
  TaskHandle_t h = s_n6_usb_handle;
  if (h == NULL) return;
  BaseType_t woken = pdFALSE;
  vTaskNotifyGiveFromISR(h, &woken);
  portYIELD_FROM_ISR(woken);
}

/* Cycle count when the watchdog's registers settle. It differs from boot to
 * boot; vnd_meta.c reads it. */
uint32_t g_n6_boot_seed = 0u;

static void n6_watchdog_start(void)
{
  IWDG->KR  = 0xCCCCu;                    /* start, which also enables LSI */
  IWDG->KR  = 0x5555u;                    /* unlock PR, RLR, EWCR */
  IWDG->PR  = 6u;                         /* LSI 32 kHz / 256 = 125 Hz */
  IWDG->RLR = 1000u;                      /* 1000 ticks, 8 s */
  /* Early-wakeup interrupt 125 ticks, 1 s, before the reset. */
  IWDG->EWCR = IWDG_EWCR_EWIE | 125u;
  for (volatile uint32_t i = 0u;
       (IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU | IWDG_SR_EWU)) && i < 1000000u; i++) {}
  g_n6_boot_seed = DWT->CYCCNT;
  IWDG->KR  = 0xAAAAu;                    /* reload and feed */
  HAL_NVIC_SetPriority(IWDG_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(IWDG_IRQn);
}

/* IWDG early-wakeup ISR body, given the interrupted context's exception frame. It
 * does not feed the watchdog. */
__attribute__((used)) void n6_iwdg_ewi_body(uint32_t *frame)
{
  PWR->DBPCR |= PWR_DBPCR_DBP;
  TAMP->BKP25R = 8u;          /* tag 8, WDG-EWI */
  TAMP->BKP26R = frame[6];    /* stacked PC */
  TAMP->BKP27R = frame[5];    /* stacked LR */
  TAMP->BKP28R = frame[7];    /* stacked xPSR. Bits 8:0 are the interrupted exception number. */
  /* The thread context is stacked on the PSP. */
  {
    uint32_t psp = __get_PSP();
    uint32_t tpc = 0u, tlr = 0u;
    if ((psp >= 0x34000000u) && (psp <= 0x34200000u - 32u) && ((psp & 7u) == 0u)) {
      tpc = ((uint32_t *)psp)[6];
      tlr = ((uint32_t *)psp)[5];
    }
    TAMP->BKP29R = tpc;       /* thread PC, 0 when unavailable */
    TAMP->BKP30R = tlr;       /* thread LR */
    TAMP->BKP31R = SCB->ICSR; /* pending and active exception state */
  }
  /* First four characters of the running task's name. */
  {
    uint32_t nm = 0u;
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
      TaskHandle_t h = xTaskGetCurrentTaskHandle();
      const char *s = (h != NULL) ? pcTaskGetName(h) : NULL;
      if (s != NULL) {
        for (uint32_t i = 0u; (i < 4u) && (s[i] != '\0'); i++) {
          nm |= ((uint32_t)(uint8_t)s[i]) << (8u * i);
        }
      }
    }
    TAMP->BKP23R = nm;
  }
  TAMP->BKP24R = N6_CRASH_MAGIC;
  IWDG->ICR = IWDG_ICR_EWIC;
}

/* Passes the interrupted context's frame, on MSP or PSP, to the C body. b, not bl,
 * keeps EXC_RETURN in LR. */
__attribute__((naked)) void IWDG_IRQHandler(void)
{
  __asm volatile(
      "tst lr, #4          \n"
      "ite eq              \n"
      "mrseq r0, msp       \n"
      "mrsne r0, psp       \n"
      "b n6_iwdg_ewi_body  \n");
}

/* Called from the fault handlers. The magic is written last, so a partial record
 * never validates. */
static void n6_crash_record_save(uint32_t tag, uint32_t pc, uint32_t lr)
{
  PWR->DBPCR |= PWR_DBPCR_DBP;
  TAMP->BKP25R = tag;
  TAMP->BKP26R = pc;
  TAMP->BKP27R = lr;
  TAMP->BKP28R = SCB->CFSR;
  TAMP->BKP24R = N6_CRASH_MAGIC;
}

/* FreeRTOS configASSERT handler: saves a crash record and resets. It runs in task
 * or ISR context. */
void n6_configassert_fail(void)
{
  __disable_irq();
  PWR->DBPCR |= PWR_DBPCR_DBP;   /* backup-domain write access */
  n6_crash_record_save(N6_CRASH_TAG_CONFIGASSERT,
                       (uint32_t)__builtin_return_address(0), 0u);
  n6_boot_press_clear();
  NVIC_SystemReset();
  for (;;) { }                   /* in case a debugger blocks the reset */
}

static void n6_boot_report_build(void)
{
  const uint32_t rsr = s_boot_rsr;
  const uint32_t sbf = s_boot_sbf;

  /* Order matters: POR also sets BOR and PIN, and a software reset also sets PIN. */
  const char *cause = "unknown";
  if      ((rsr & RCC_RSR_IWDGRSTF) && (sbf != 0u)) cause = "wake from standby";
  else if (rsr & RCC_RSR_IWDGRSTF) cause = "watchdog (the firmware stopped responding)";
  else if (rsr & RCC_RSR_WWDGRSTF) cause = "window watchdog";
  else if (rsr & RCC_RSR_LCKRSTF)  cause = "CPU lockup (double fault)";
  else if ((sbf != 0u) && !(rsr & RCC_RSR_SFTRSTF)) cause = "wake from standby";
  else if (rsr & RCC_RSR_PORRSTF)  cause = "power-on";
  else if (rsr & RCC_RSR_SFTRSTF)  cause = "software reset (upload or reboot)";
  else if (rsr & RCC_RSR_PINRSTF)  cause = "reset pin";
  else if (rsr & RCC_RSR_BORRSTF)  cause = "brown-out (supply voltage dropped)";

  char crash[192] = "";
  if (TAMP->BKP24R == N6_CRASH_MAGIC) {
    static const char *const k_tags[] = {
      "CPUFault", "HardFault", "MemManage", "BusFault",
      "UsageFault", "NMI", "SecureFault", "Error_Handler(HAL)",
      "WDG-EWI(starved)",  /* tag 8. The 4th word is xPSR, not CFSR. */
      "configASSERT"       /* tag 9 */
    };
    const uint32_t tag = TAMP->BKP25R;
    int n = snprintf(crash, sizeof(crash),
             " | prev crash: %s pc=0x%08lX lr=0x%08lX %s=0x%08lX",
             (tag <= 9u) ? k_tags[tag] : "?",
             (unsigned long)TAMP->BKP26R, (unsigned long)TAMP->BKP27R,
             (tag == 8u) ? "xpsr" : "cfsr", (unsigned long)TAMP->BKP28R);
    if (tag == 8u && n > 0 && (size_t)n < sizeof(crash)) {
      char task[5] = "";
      const uint32_t nm = TAMP->BKP23R;
      for (uint32_t i = 0u; i < 4u; i++) {
        const char c = (char)((nm >> (8u * i)) & 0xFFu);
        task[i] = (c >= 0x20 && c < 0x7F) ? c : '\0';
        if (task[i] == '\0') break;
      }
      snprintf(crash + n, sizeof(crash) - (size_t)n,
               " tpc=0x%08lX tlr=0x%08lX icsr=0x%08lX task=%s",
               (unsigned long)TAMP->BKP29R, (unsigned long)TAMP->BKP30R,
               (unsigned long)TAMP->BKP31R, (task[0] != '\0') ? task : "?");
    }
    HAL_PWR_EnableBkUpAccess();
    TAMP->BKP24R = 0u;
  }

  snprintf(s_boot_line, sizeof(s_boot_line), "[BOOT] reset: %s%s", cause, crash);
}

#if !defined(NEURON6_DEBUG_CONSOLE) || (NEURON6_DEBUG_CONSOLE != 0)
/* Dumps LTDC and PIPE0 state. The LTDC underrun flags are read, then cleared. */
static void lcd_debug_dump(void)
{
  if (hltdc.State == HAL_LTDC_STATE_RESET) return;   /* the LTDC is unclocked */

  char b[256];
  uint32_t isr  = LTDC->ISR;
  uint32_t cps1 = LTDC->CPSR;

  snprintf(b, sizeof b,
    "[LCDDBG] LTDC EN=%lu ISR=%08lX[FUWIF=%lu FUIF=%lu FURIF=%lu TERRIF=%lu] CPSR=Y%lu/X%lu CDSR=%lX\r\n",
    (unsigned long)((LTDC->GCR & LTDC_GCR_LTDCEN_Msk) ? 1u : 0u), (unsigned long)isr,
    (unsigned long)!!(isr & LTDC_ISR_FUWIF_Msk), (unsigned long)!!(isr & LTDC_ISR_FUIF_Msk),
    (unsigned long)!!(isr & LTDC_ISR_FURIF_Msk), (unsigned long)!!(isr & LTDC_ISR_TERRIF_Msk),
    (unsigned long)(cps1 & LTDC_CPSR_CYPOS_Msk),
    (unsigned long)((cps1 & LTDC_CPSR_CXPOS_Msk) >> LTDC_CPSR_CXPOS_Pos),
    (unsigned long)LTDC->CDSR);
  dbg_print(b);

  snprintf(b, sizeof b,
    "[LCDDBG] L0cam LEN=%lu CFBAR=%08lX(pipe2buf=%08lX wire=%s) PFCR=%lX CFBLR=%08lX CFBLNR=%lX WHP=%08lX WVP=%08lX\r\n",
    (unsigned long)(LTDC_Layer1->CR & LTDC_LxCR_LEN_Msk),
    (unsigned long)LTDC_Layer1->CFBAR, (unsigned long)(uint32_t)pipe2buf,
    (LTDC_Layer1->CFBAR == (uint32_t)pipe2buf) ? "ok" : "MISMATCH",
    (unsigned long)LTDC_Layer1->PFCR, (unsigned long)LTDC_Layer1->CFBLR,
    (unsigned long)LTDC_Layer1->CFBLNR, (unsigned long)LTDC_Layer1->WHPCR,
    (unsigned long)LTDC_Layer1->WVPCR);
  dbg_print(b);

  snprintf(b, sizeof b,
    "[LCDDBG] L1ovl LEN=%lu CFBAR=%08lX(front=%08lX back=%08lX) PFCR=%lX\r\n",
    (unsigned long)(LTDC_Layer2->CR & LTDC_LxCR_LEN_Msk),
    (unsigned long)LTDC_Layer2->CFBAR,
    (unsigned long)(uint32_t)g_ovl_front, (unsigned long)(uint32_t)g_ovl_back,
    (unsigned long)LTDC_Layer2->PFCR);
  dbg_print(b);

  uint32_t cps2 = LTDC->CPSR;   /* second sample: Y moves while the LTDC scans */
  uint32_t p0sr = DCMIPP->P0SR;
  snprintf(b, sizeof b,
    "[LCDDBG] P0 CPTACT=%lu P0SR=%lX AR1=%08lX wire=%s P0FSCR=%lX | cam %lux%lu | CPSR2 Y=%lu\r\n",
    (unsigned long)!!(DCMIPP->CMSR1 & DCMIPP_CMSR1_P0CPTACT_Msk),
    (unsigned long)p0sr,
    (unsigned long)DCMIPP->P0PPM0AR1,
    (DCMIPP->P0PPM0AR1 == (uint32_t)pipe2buf) ? "ok" : "MISMATCH",
    (unsigned long)DCMIPP->P0FSCR,
    (unsigned long)usb_frame_width, (unsigned long)usb_frame_height,
    (unsigned long)(cps2 & LTDC_CPSR_CYPOS_Msk));
  dbg_print(b);

  LTDC->ICR = LTDC_ICR_CFUWIF_Msk | LTDC_ICR_CFUIF_Msk | LTDC_ICR_CTERRIF_Msk | LTDC_ICR_CLIF_Msk;
}
#endif  /* NEURON6_DEBUG_CONSOLE */

/* Network host */

/* A host handed over and not yet taken by n6_usb_task. */
static const n6_net_link_t *volatile s_net_pending = NULL;

/* Command frames from the network host, reassembled across reads. */
static uint8_t  s_net_rx[sizeof(vnd_frame_hdr_t) + N6_HOST_CMD_MAX];
static uint32_t s_net_rx_len = 0u;

bool NeuroN6_NetAttach(const n6_net_link_t *link)
{
  if (link == NULL || link->send == NULL || link->recv == NULL || link->closed == NULL) {
    return false;
  }
  if (s_n6_usb_handle == NULL) return false;

  taskENTER_CRITICAL();
  const bool free_slot = (s_net == NULL && s_net_pending == NULL);
  if (free_slot) s_net_pending = link;
  taskEXIT_CRITICAL();

  if (free_slot) xTaskNotifyGive(s_n6_usb_handle);
  return free_slot;
}

static void n6_net_detach(void)
{
  const n6_net_link_t *const link = s_net;
  if (link == NULL) return;
  s_net = NULL;
  s_net_rx_len = 0u;
  n6_host_session_start();
  dbg_print("[NET] host detached - stream back on USB\r\n");
  link->closed();
}

/* Called once per n6_usb_task pass. */
static void n6_net_service(void)
{
  if (s_net == NULL) {
    const n6_net_link_t *const link = s_net_pending;
    if (link == NULL) return;
    /* A frame part way out to the USB reader is cut off here. */
    if (vnd_tx_active && vnd_off != 0u && vnd_off < vnd_total) {
      vnd_usb_owed = vnd_total - vnd_off;
    }
    s_net_pending = NULL;
    s_net = link;
    s_net_rx_len = 0u;
    n6_host_session_start();
    s_net_progress_ms = HAL_GetTick();
    dbg_print("[NET] host attached - stream moved to the network\r\n");
    return;
  }

  /* The stall clock runs only while a frame waits. */
  if (!vnd_tx_active) s_net_progress_ms = HAL_GetTick();

  const uint32_t space = (uint32_t)sizeof(s_net_rx) - s_net_rx_len;
  const int32_t n = s_net->recv(&s_net_rx[s_net_rx_len], space);
  if (n < 0) { n6_net_detach(); return; }
  s_net_rx_len += ((uint32_t)n < space) ? (uint32_t)n : space;

  while (s_net_rx_len >= sizeof(vnd_frame_hdr_t)) {
    vnd_frame_hdr_t h;
    memcpy(&h, s_net_rx, sizeof(h));
    if (h.magic != VND_MAGIC || h.meta_len != 0u || h.payload_len > N6_HOST_CMD_MAX) {
      dbg_print("[NET] host sent something other than a command - dropped\r\n");
      n6_net_detach();
      return;
    }
    const uint32_t total = sizeof(h) + h.payload_len;
    if (s_net_rx_len < total) break;
    if (h.payload_len != 0u) {
      n6_host_command(&s_net_rx[sizeof(h)], h.payload_len, false);
    }
    s_net_rx_len -= total;
    memmove(s_net_rx, &s_net_rx[total], s_net_rx_len);
  }
}

static void n6_usb_task(void *arg)
{
  (void)arg;

#if !defined(NEURON6_DEBUG_CONSOLE) || (NEURON6_DEBUG_CONSOLE != 0)
  static bool diag_done = false;
#endif

  /* True while the vendor TX in flight owns a JPEG buffer. */
  static bool s_tx_counts = false;

  /* Started here, not in setup(), so sketch bring-up has no time limit. */
  n6_watchdog_start();
  n6_boot_report_build();

  /* The boot pump stops at its next pass. */
  while (s_n6_boot_pump_handle != NULL && !s_n6_boot_pump_done) {
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  for (;;) {
    IWDG->KR = 0xAAAAu;   /* feed the watchdog */
    tud_task_ext(0, false);   /* tud_task() blocks under OPT_OS_FREERTOS */
    vnd_tx_pump();
    n6_net_service();

    /* Publishes the boot line, then repeats it after the first frame is sent. */
    {
      static uint8_t s_boot_pub = 0u;
      static uint32_t s_tx0_ms = 0u;
      if (s_tx0_ms == 0u && g_tx_seq != 0u) s_tx0_ms = HAL_GetTick() | 1u;
      const uint32_t tx_ms = (s_tx0_ms != 0u) ? (HAL_GetTick() - s_tx0_ms) : 0u;
      if (s_boot_pub == 0u) {
        (void)vnd_meta_publish_debug_text(s_boot_line);
        s_boot_pub++;
      } else if ((s_boot_pub == 1u && tx_ms > 1000u) ||
                 (s_boot_pub == 2u && tx_ms > 6000u) ||
                 (s_boot_pub == 3u && tx_ms > 16000u)) {
        char replay[sizeof(s_boot_line) + 12];
        snprintf(replay, sizeof(replay), "%s [replay]", s_boot_line);
        (void)vnd_meta_publish_debug_text(replay);
        s_boot_pub++;
      }
    }
    /* State-based, not edge-based: vnd_tx_reset() can run inside tud_task()'s
     * callbacks. */
    if (s_tx_counts && !vnd_tx_active) {
      g_tx_seq++;
      s_tx_counts = false;
    }

    /* Slots outstanding for 1 s with the link idle and nothing queued are reclaimed. */
    {
      static uint32_t s_jam_since_ms = 0u;
      const bool jammed = !vnd_tx_active && (g_enc_seq != g_tx_seq) &&
                          g_n6_video_usb_queue != NULL &&
                          uxQueueMessagesWaiting(g_n6_video_usb_queue) == 0u &&
                          n6_video_queue() == g_n6_video_usb_queue;
      if (!jammed) {
        s_jam_since_ms = 0u;
      } else if (s_jam_since_ms == 0u) {
        s_jam_since_ms = HAL_GetTick();
      } else if ((HAL_GetTick() - s_jam_since_ms) > 1000u) {
        char jb[96];
        snprintf(jb, sizeof(jb),
                 "[JAMBREAK] reclaimed %lu leaked tx slot(s) (enc=%lu tx=%lu)\r\n",
                 (unsigned long)(g_enc_seq - g_tx_seq),
                 (unsigned long)g_enc_seq, (unsigned long)g_tx_seq);
        dbg_detail(jb);
        g_tx_seq = g_enc_seq;
        s_tx_counts = false;
        s_jam_since_ms = 0u;
      }
    }

    if (ip_announce_should_emit()) {
      ip_announce_emit();
    }

    softap_announce_service();

    if (isp_announce_should_emit()) {
      isp_announce_emit();
    }

    if (cam_caps_announce_should_emit()) {
      cam_caps_announce_emit();
    }

    if (tune_announce_should_emit()) {
      tune_announce_emit();
    }

    {
      static bool s_cdc_was_up = false;
      const bool cdc_up = tud_cdc_n_connected(0);
      if (cdc_up && !s_cdc_was_up) {
        s_cdc_connects++;
        dbg_print(s_boot_line);
        dbg_print("\r\n");
        for (uint32_t i = 0u; i < s_notice_count; i++)
        {
          dbg_print(s_notices[i]);
        }
      }
      s_cdc_was_up = cdc_up;
      s_cdc_up = cdc_up ? 1u : 0u;
    }

#if !defined(NEURON6_DEBUG_CONSOLE) || (NEURON6_DEBUG_CONSOLE != 0)
    /* The camera dump, at the first connection. A sketch with no camera prints
     * nothing: without HAL_DCMIPP_Init the CSI is unclocked and reading it
     * bus-faults. */
    if (!diag_done && tud_cdc_n_connected(0)) {
      diag_done = true;
      if (hdcmipp.State != HAL_DCMIPP_STATE_RESET) {
        char dbg[160];
        snprintf(dbg, sizeof(dbg),
          "[DIAG] hdcmipp.State=%d pipe0=%d pipe1=%d pipe2=%d\r\n",
          (int)hdcmipp.State,
          (int)hdcmipp.PipeState[DCMIPP_PIPE0],
          (int)hdcmipp.PipeState[DCMIPP_PIPE1],
          (int)hdcmipp.PipeState[DCMIPP_PIPE2]);
        dbg_print(dbg);
        /* Non-blocking: this task feeds the watchdog. */
        if (n6_i2c_trylock()) { n6_dump_ov5640_regs(); n6_i2c_unlock(); }
        n6_dump_csi_regs("");
      }
    }

    /* 1 Hz telemetry */
    {
      static uint32_t s_camdbg_last_ms = 0;
      uint32_t now_camdbg = HAL_GetTick();
      if ((now_camdbg - s_camdbg_last_ms) >= 1000U) {
        uint32_t camdbg_ivl = now_camdbg - s_camdbg_last_ms;
        s_camdbg_last_ms = now_camdbg;
        char cdbg[256];

        n6_audio_debug_dump();

        /* The CSI and VENC are unclocked until HAL_DCMIPP_Init. */
        if (hdcmipp.State != HAL_DCMIPP_STATE_RESET) {
        snprintf(cdbg, sizeof(cdbg),
          "[CAMDBG] %lux%lu pipe1=%lu enc_ok=%lu enc_fail=%lu jpg=%lu "
          "p1err=%lu dcerr=%lu csierr=%lu code=0x%lX ln=%lu ffi=%ld cm2=0x%lX st=%d p1=%d\r\n",
          (unsigned long)usb_frame_width, (unsigned long)usb_frame_height,
          (unsigned long)g_dbg_pipe1_frames, (unsigned long)g_dbg_encode_ok,
          (unsigned long)g_dbg_encode_fail,
          (unsigned long)g_dbg_last_jpeg_sz, (unsigned long)g_dbg_pipe1_err,
          (unsigned long)g_dbg_dcmipp_err, (unsigned long)g_dbg_csi_line_err,
          (unsigned long)g_dbg_last_err_code,
          (unsigned long)g_dbg_pipe1_line_events, (long)g_dbg_ff_init_ret,
          (unsigned long)g_dbg_cmsr2_sticky,
          (int)hdcmipp.State, (int)hdcmipp.PipeState[DCMIPP_PIPE1]);
        dbg_print(cdbg);

        lcd_debug_dump();

        static uint32_t s_last_pipe1 = 0, s_last_enc = 0, s_last_tx = 0, s_last_thtx = 0;
        uint32_t d_pipe1 = g_dbg_pipe1_frames - s_last_pipe1;
        uint32_t d_enc   = g_dbg_encode_ok    - s_last_enc;
        uint32_t d_tx    = g_dbg_tx_started   - s_last_tx;
        uint32_t d_thtx  = g_dbg_th_tx        - s_last_thtx;
        s_last_pipe1 = g_dbg_pipe1_frames;
        s_last_enc   = g_dbg_encode_ok;
        s_last_tx    = g_dbg_tx_started;
        s_last_thtx  = g_dbg_th_tx;
        if (camdbg_ivl == 0U) { camdbg_ivl = 1U; }
        snprintf(cdbg, sizeof(cdbg),
          "[RATE] cap=%lufps enc=%lufps tx=%lufps enc_ms=%lu (dpipe1=%lu denc=%lu dtx=%lu / %lums) thtx=%lu thskip=%lu\r\n",
          (unsigned long)((d_pipe1 * 1000UL) / camdbg_ivl),
          (unsigned long)((d_enc   * 1000UL) / camdbg_ivl),
          (unsigned long)((d_tx    * 1000UL) / camdbg_ivl),
          (unsigned long)g_dbg_last_enc_ms,
          (unsigned long)d_pipe1, (unsigned long)d_enc, (unsigned long)d_tx,
          (unsigned long)camdbg_ivl,
          (unsigned long)d_thtx, (unsigned long)g_dbg_th_skip);
        dbg_print(cdbg);
        snprintf(cdbg, sizeof(cdbg),
          "[LBDBG] wr_mbl=%lu cb=%lu enc_ms=%lu jpg=%lu lbstart=%ld lbenc=%ld instant=0x%08lX rdMax=%lu wrMax=%lu gapMax=%lu/%lu\r\n",
          (unsigned long)g_dcmipp_wr_mblines,
          (unsigned long)g_dbg_lb_cb_calls, (unsigned long)g_dbg_last_enc_ms,
          (unsigned long)g_dbg_last_jpeg_sz,
          (long)g_dbg_lb_start_ret, (long)g_dbg_lb_enc_rv, (unsigned long)g_dbg_lb_instant,
          (unsigned long)g_dbg_lb_rd_max, (unsigned long)g_dbg_lb_wr_max,
          (unsigned long)g_dbg_lb_gap_max, (unsigned long)(g_venc_band_depth * 2u));
        dbg_print(cdbg);
        snprintf(cdbg, sizeof(cdbg),
          "[LBGEO] band=0x%08lX sz=%lu pitch=%lu depth=%lu lbinit=%ld | instant hw=%lu lb=%lu lat=%lu d=%lu rd=%lu wr=%lu\r\n",
          (unsigned long)(uint32_t)g_venc_band, (unsigned long)g_venc_band_size,
          (unsigned long)g_venc_band_pitch, (unsigned long)g_venc_band_depth,
          (long)g_dbg_lb_init_ret,
          (unsigned long)!!(g_dbg_lb_instant & (1u<<29)), (unsigned long)!!(g_dbg_lb_instant & (1u<<28)),
          (unsigned long)!!(g_dbg_lb_instant & (1u<<27)), (unsigned long)((g_dbg_lb_instant >> 18) & 0x1FFu),
          (unsigned long)((g_dbg_lb_instant >> 9) & 0x1FFu), (unsigned long)(g_dbg_lb_instant & 0x1FFu));
        dbg_print(cdbg);
        {
          uint32_t s1 = CSI->SR1;
          snprintf(cdbg, sizeof(cdbg),
            "[CSILIVE] SR1=%08lX ERR1=%08lX ERR2=%08lX | clkACT=%lu L0[HS=%lu SYNC=%lu] L1[HS=%lu SYNC=%lu] | errL0[sot=%lu ss=%lu ctl=%lu] errL1[sot=%lu ss=%lu ctl=%lu]\r\n",
            (unsigned long)s1, (unsigned long)CSI->ERR1, (unsigned long)CSI->ERR2,
            (unsigned long)!!(s1 & CSI_SR1_ACTCLF),
            (unsigned long)!!(s1 & CSI_SR1_ACTDL0F), (unsigned long)!!(s1 & CSI_SR1_SYNCDL0F),
            (unsigned long)!!(s1 & CSI_SR1_ACTDL1F), (unsigned long)!!(s1 & CSI_SR1_SYNCDL1F),
            (unsigned long)!!(s1 & CSI_SR1_ESOTDL0F), (unsigned long)!!(s1 & CSI_SR1_ESOTSYNCDL0F), (unsigned long)!!(s1 & CSI_SR1_ECTRLDL0F),
            (unsigned long)!!(s1 & CSI_SR1_ESOTDL1F), (unsigned long)!!(s1 & CSI_SR1_ESOTSYNCDL1F), (unsigned long)!!(s1 & CSI_SR1_ECTRLDL1F));
          dbg_print(cdbg);
        }
        if (g_dbg_venc_print) {
          g_dbg_venc_print = 0;
          uint32_t sr1 = g_dbg_swreg_lo[1];
          const char *vst = (sr1 & 0x10u) ? "HW_RESET" : (sr1 & 0x08u) ? "ERROR" :
                            (sr1 & 0x20u) ? "BUFF_FULL" : (sr1 & 0x40u) ? "HW_TIMEOUT" :
                            (sr1 & 0x04u) ? "FRAME_READY" : "none";
          snprintf(cdbg, sizeof(cdbg),
            "[VENCDUMP] swreg1=0x%08lX(%s) | jeinit=%ld jesps=%ld cm2=0x%08lX\r\n",
            (unsigned long)sr1, vst,
            (long)g_dbg_ff_jeinit, (long)g_dbg_ff_jesps, (unsigned long)g_dbg_cmsr2_sticky);
          dbg_print(cdbg);
          for (uint32_t b = 0; b < 64u; b += 8u) {
            snprintf(cdbg, sizeof(cdbg),
              "[VSWR %02lu] %08lX %08lX %08lX %08lX %08lX %08lX %08lX %08lX\r\n", (unsigned long)b,
              (unsigned long)g_dbg_swreg_lo[b+0], (unsigned long)g_dbg_swreg_lo[b+1],
              (unsigned long)g_dbg_swreg_lo[b+2], (unsigned long)g_dbg_swreg_lo[b+3],
              (unsigned long)g_dbg_swreg_lo[b+4], (unsigned long)g_dbg_swreg_lo[b+5],
              (unsigned long)g_dbg_swreg_lo[b+6], (unsigned long)g_dbg_swreg_lo[b+7]);
            dbg_print(cdbg);
          }
          for (uint32_t b = 0; b < 16u; b += 8u) {
            snprintf(cdbg, sizeof(cdbg),
              "[VSWR %lu] %08lX %08lX %08lX %08lX %08lX %08lX %08lX %08lX\r\n", (unsigned long)(290u + b),
              (unsigned long)g_dbg_swreg_hi[b+0], (unsigned long)g_dbg_swreg_hi[b+1],
              (unsigned long)g_dbg_swreg_hi[b+2], (unsigned long)g_dbg_swreg_hi[b+3],
              (unsigned long)g_dbg_swreg_hi[b+4], (unsigned long)g_dbg_swreg_hi[b+5],
              (unsigned long)g_dbg_swreg_hi[b+6], (unsigned long)g_dbg_swreg_hi[b+7]);
            dbg_print(cdbg);
          }
        }
        }   /* hdcmipp.State != HAL_DCMIPP_STATE_RESET */
      }
    }
#endif  /* NEURON6_DEBUG_CONSOLE: 1 Hz telemetry block */

    uint8_t ch;
    for (uint32_t rx_n = 0u;
         (rx_n < 256u) && n6_serial_rx_open() && (tud_cdc_n_read(0, &ch, 1) == 1);
         rx_n++) {
      NeuroN6_SerialRxSink(ch);
#if !defined(NEURON6_DEBUG_CONSOLE) || (NEURON6_DEBUG_CONSOLE != 0)
      /* A console key runs only when it is a whole line, or the FIFO is empty
       * after it. */
      uint8_t kc = 0u;
      if ((ch == (uint8_t)'\r') || (ch == (uint8_t)'\n')) {
        if (s_con_len == 1u) kc = s_con_key;
        s_con_len = 0u; s_con_key = 0u;
      } else {
        if (s_con_len < 255u) s_con_len++;
        if (s_con_len == 1u) {
          s_con_key = ch;
          if (tud_cdc_n_available(0) == 0u) { kc = ch; s_con_key = 0u; s_con_len = 0u; }
        } else {
          s_con_key = 0u;
        }
      }
      if ((kc == (uint8_t)'r' || kc == (uint8_t)'v' || kc == (uint8_t)'d') &&
          hdcmipp.State == HAL_DCMIPP_STATE_RESET) {
        dbg_print("[DIAG] no camera; command ignored.\r\n");
      } else if (kc == (uint8_t)'b') {
        reboot_to_bootloader();
      } else if (kc == (uint8_t)'X') {   /* watchdog test */
        dbg_print("[WDGTEST] USB task halted; the watchdog resets the board in 8 s.\r\n");
        taskENTER_CRITICAL();           /* nothing can feed the watchdog */
        for (;;) {}
      } else if (kc == (uint8_t)'o') {   /* toggle the LCD overlay */
        if (hltdc.State == HAL_LTDC_STATE_RESET) {
          dbg_print("[LCD] no panel; 'o' ignored.\r\n");
        } else {
          g_lcd_overlay_on = g_lcd_overlay_on ? 0u : 1u;
          if (g_lcd_overlay_on) { LTDC_Layer2->CR |=  LTDC_LxCR_LEN_Msk; }
          else                  { LTDC_Layer2->CR &= ~LTDC_LxCR_LEN_Msk; }
          HAL_LTDC_Reload(&hltdc, LTDC_RELOAD_VERTICAL_BLANKING);
          dbg_print(g_lcd_overlay_on
            ? "[LCD] overlay on\r\n"
            : "[LCD] overlay off\r\n");
        }
      } else if (kc == (uint8_t)'p') {   /* PSRAM read-bandwidth probe */
        volatile const uint32_t *psr = (volatile const uint32_t *)0x70000000u;  /* PSRAM */
        uint32_t acc = 0u;
        const uint32_t words = 16384u / 4u;                  /* 16 KB */
        uint32_t c0 = DWT->CYCCNT;
        for (uint32_t i = 0u; i < words; i++) { acc += psr[i]; }
        uint32_t cyc  = DWT->CYCCNT - c0;
        uint32_t hclk = HAL_RCC_GetHCLKFreq();
        uint32_t mbps = cyc ? (uint32_t)((uint64_t)16384u * hclk / cyc / 1000000u) : 0u;
        char pb[200];
        snprintf(pb, sizeof(pb),
          "[PSRAMBW] 16 KB CPU read: %lu cycles at HCLK %lu Hz, %lu MB/s, single-beat | MUXEN=%lu acc=%08lX\r\n",
          (unsigned long)cyc, (unsigned long)hclk, (unsigned long)mbps,
          (unsigned long)((XSPIM->CR & XSPIM_CR_MUXEN_Msk) ? 1u : 0u), (unsigned long)acc);
        dbg_print(pb);
      } else if (kc == (uint8_t)'i') {   /* toggle NN inference */
        g_nn_inference_on = g_nn_inference_on ? 0u : 1u;
        dbg_print(g_nn_inference_on
          ? "[NN] inference on\r\n"
          : "[NN] inference off; detections hold their last values\r\n");
      } else if (kc == (uint8_t)'q') {   /* toggle the NPU's NoC QoS floor */
        static uint32_t s_npu_qos_saved   = 0xFFFFFFFFu;   /* not yet captured */
        static uint8_t  s_npu_qos_floored = 0u;
        /* The SYSCFG clock is gated after boot. */
        __HAL_RCC_SYSCFG_CLK_ENABLE();
        if (s_npu_qos_saved == 0xFFFFFFFFu) { s_npu_qos_saved = SYSCFG->NPUNICQOSCR; }
        if (!s_npu_qos_floored) {
          SYSCFG->NPUNICQOSCR &= ~(SYSCFG_NPUNICQOSCR_NPU1_ARQOSR_Msk | SYSCFG_NPUNICQOSCR_NPU1_ARQOSW_Msk |
                                   SYSCFG_NPUNICQOSCR_NPU2_ARQOSR_Msk | SYSCFG_NPUNICQOSCR_NPU2_ARQOSW_Msk);
          s_npu_qos_floored = 1u;
        } else {
          SYSCFG->NPUNICQOSCR = s_npu_qos_saved;
          s_npu_qos_floored = 0u;
        }
        char qb[184];
        snprintf(qb, sizeof(qb),
          "[QOS] NPUNICQOSCR=%08lX, NPU QoS %s\r\n",
          (unsigned long)SYSCFG->NPUNICQOSCR, s_npu_qos_floored ? "lowered" : "restored");
        dbg_print(qb);
      } else if (kc == (uint8_t)'r') {   /* dump the OV5640 and CSI registers */
        if (n6_i2c_trylock()) { n6_dump_ov5640_regs(); n6_i2c_unlock(); }
        n6_dump_csi_regs("");
      } else if (kc == (uint8_t)'v') {   /* VENC line-buffer feasibility */
        char vb[200];
        uint32_t c[8];
        encoder_jpeg_caps(c);
        snprintf(vb, sizeof(vb),
          "[VENCCFG] rgb=%lu jpeg=%lu instant=%lu loopback=%lu irqenh=%lu maxEncW=%lu busBits=%lu id=0x%08lX\r\n",
          (unsigned long)c[0], (unsigned long)c[1], (unsigned long)c[2], (unsigned long)c[3],
          (unsigned long)c[4], (unsigned long)c[5], (unsigned long)(c[6] * 32u), (unsigned long)c[7]);
        dbg_print(vb);
        uint32_t band = 0; void *addr = 0;
        int fit = encoder_jpeg_band_probe(usb_frame_width, 32u, &band, &addr);
        snprintf(vb, sizeof(vb),
          "[VENCBUD] %lux%lu band(32line)=%lu bytes fit=%s addr=%p\r\n",
          (unsigned long)usb_frame_width, (unsigned long)usb_frame_height,
          (unsigned long)band, fit ? "yes" : "no", addr);
        dbg_print(vb);
      } else if (kc == (uint8_t)'d') {   /* dump the PIPE1 and VENC registers */
        char d[200];
        snprintf(d, sizeof(d),
          "[DREG] %lux%lu st=%d p1st=%d | P1FSCR=%08lX P1FCTCR=%08lX P1SR=%08lX\r\n",
          (unsigned long)usb_frame_width, (unsigned long)usb_frame_height,
          (int)hdcmipp.State, (int)hdcmipp.PipeState[DCMIPP_PIPE1],
          (unsigned long)DCMIPP->P1FSCR, (unsigned long)DCMIPP->P1FCTCR, (unsigned long)DCMIPP->P1SR);
        dbg_print(d);
        snprintf(d, sizeof(d),
          "[DREG] P1PPCR=%08lX fmt=0x%lX | P1PPM0PR(pitch)=%08lX P1PPM0AR1=%08lX\r\n",
          (unsigned long)DCMIPP->P1PPCR, (unsigned long)(DCMIPP->P1PPCR & 0xFu),
          (unsigned long)DCMIPP->P1PPM0PR, (unsigned long)DCMIPP->P1PPM0AR1);
        dbg_print(d);
        snprintf(d, sizeof(d),
          "[DREG] P1YUVCR=%08lX yuv_en=%lu | P1CRSZR=%08lX P1DSCR=%08lX P1DSSZR=%08lX\r\n",
          (unsigned long)DCMIPP->P1YUVCR, (unsigned long)(DCMIPP->P1YUVCR & 1u),
          (unsigned long)DCMIPP->P1CRSZR, (unsigned long)DCMIPP->P1DSCR, (unsigned long)DCMIPP->P1DSSZR);
        dbg_print(d);
        snprintf(d, sizeof(d),
          "[DREG] CMCR=%08lX CMSR2=%08lX CMIER=%08lX | ffi=%ld\r\n",
          (unsigned long)DCMIPP->CMCR, (unsigned long)DCMIPP->CMSR2, (unsigned long)DCMIPP->CMIER,
          (long)g_dbg_ff_init_ret);
        dbg_print(d);
        uint32_t vc[8];
        encoder_jpeg_caps(vc);
        snprintf(d, sizeof(d),
          "[DREG VENC] rgb=%lu jpeg=%lu instant=%lu loopback=%lu maxEncW=%lu busBits=%lu id=0x%08lX\r\n",
          (unsigned long)vc[0], (unsigned long)vc[1], (unsigned long)vc[2], (unsigned long)vc[3],
          (unsigned long)vc[5], (unsigned long)(vc[6] * 32u), (unsigned long)vc[7]);
        dbg_print(d);
        snprintf(d, sizeof(d),
          "[DREG MEM] mjpeg0=%08lX mjpeg1=%08lX gmax=%lu | band=%08lX bandSz=%lu\r\n",
          (unsigned long)(uint32_t)mjpeg_buf0, (unsigned long)(uint32_t)mjpeg_buf1, (unsigned long)g_mjpeg_max,
          (unsigned long)(uint32_t)g_venc_band, (unsigned long)g_venc_band_size);
        dbg_print(d);
      } else if (kc == (uint8_t)'e') {   /* arm a VENC swreg snapshot */
        g_dbg_venc_arm = 1u;
        dbg_print("[VENC] armed: the next encode prints the VENC registers.\r\n");
      }
#endif  /* NEURON6_DEBUG_CONSOLE */
    }

    /* Microphone audio goes first, as a meta-only frame. NeuroN6_MetaAnnounce()
     * can take the transmitter first; the chunk then waits for the next pass. */
    {
      static const uint8_t *s_audio_tlv = NULL;
      static uint32_t       s_audio_len = 0u;
      if (!vnd_tx_active &&
          (s_audio_tlv != NULL || n6_audio_drain(&s_audio_tlv, &s_audio_len))) {
        if (vnd_tx_start_frame(s_audio_tlv, s_audio_len, NULL, 0u, 0u)) {
          g_dbg_tx_started++;
          s_tx_counts = false;
          s_audio_tlv = NULL;
        } else if (!vnd_tx_active) {
          s_audio_tlv = NULL;   /* not sendable */
        }
      }
    }

    /* Without a capture task, a meta-only frame is posted whenever a slot changes. */
    if (s_n6_capture_handle == NULL && g_n6_video_usb_queue != NULL) {
      static uint32_t s_last_meta_pub = 0u;
      uint32_t meta_pub = vnd_meta_pub_seq();
      bool meta_changed = (meta_pub != s_last_meta_pub);
      if (meta_changed && !vnd_tx_active &&
          uxQueueMessagesWaiting(g_n6_video_usb_queue) == 0u) {
        const uint8_t *pm = NULL;
        uint32_t pml = 0u;
        if (vnd_meta_get(&pm, &pml)) {
          n6_frame_msg_t pmsg;
          pmsg.jpeg_ptr = NULL; pmsg.jpeg_len = 0u;
          pmsg.meta_ptr = pm;   pmsg.meta_len = pml;
          if (xQueueSend(g_n6_video_usb_queue, &pmsg, 0) == pdTRUE) {
            s_last_meta_pub = meta_pub;
          }
        } else {
          s_last_meta_pub = meta_pub;
        }
      }
    }

    vnd_meta_debug_replay_step(vnd_tx_active ? 0 : 1);

    /* Thermal frames go out as meta-only frames, ahead of the next camera JPEG. */
    if (g_n6_video_usb_queue != NULL && !vnd_tx_active) {
      static uint32_t s_last_thermal_pub = 0u;
      uint32_t th_pub = vnd_meta_thermal_pub_seq();
      if (th_pub != s_last_thermal_pub) {
        if (th_pub - s_last_thermal_pub > 1u) {
          g_dbg_th_skip += (th_pub - s_last_thermal_pub) - 1u;
        }
        const uint8_t *th_ptr = NULL; uint32_t th_len = 0u;
        if (vnd_meta_get_slot(N6_META_SLOT_THERMAL, &th_ptr, &th_len)) {
          if (vnd_tx_start_frame(th_ptr, th_len, NULL, 0u, 0u)) {
            g_dbg_tx_started++;
            g_dbg_th_tx++;
            s_tx_counts = false;
            s_last_thermal_pub = th_pub;
          } else if (!vnd_tx_active) {
            s_last_thermal_pub = th_pub;   /* not sendable */
          }
        } else {
          s_last_thermal_pub = th_pub;
        }
      }
    }

    n6_frame_msg_t msg;
    if (!vnd_tx_active && g_n6_video_usb_queue &&
        xQueueReceive(g_n6_video_usb_queue, &msg, 0) == pdTRUE) {
      /* flags bit0 set means the frame carries a JPEG. */
      uint32_t _vflags = (msg.jpeg_len > 0u) ? 1u : 0u;
      if (vnd_tx_start_frame(msg.meta_ptr, msg.meta_len,
                             msg.jpeg_ptr, msg.jpeg_len, _vflags)) {
        g_dbg_tx_started++;
        s_tx_counts = (msg.jpeg_len > 0u);
      } else if (!vnd_tx_active ||
                 xQueueSendToFront(g_n6_video_usb_queue, &msg, 0) != pdTRUE) {
        /* Not sendable, or no room to wait: dropped, and its JPEG slot freed. */
        if (msg.jpeg_len > 0u) g_tx_seq++;
      }
    }

    if (vnd_tx_active) {
      /* Woken by NeuroN6_UsbIrqKick. */
      (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1));
    } else {
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
}

void NeuroN6_VisionTasks_Start(void)
{
  if (s_n6_capture_handle != NULL && s_n6_usb_handle != NULL) return;

  /* Depth 2: the two frames the ping-pong gate allows outstanding. */
  if (g_n6_video_usb_queue == NULL) {
    g_n6_video_usb_queue = xQueueCreate(2, sizeof(n6_frame_msg_t));
  }
  if (g_n6_video_eth_queue == NULL) {
    g_n6_video_eth_queue = xQueueCreate(2, sizeof(n6_frame_msg_t));
  }

  if (s_n6_capture_handle == NULL) {
    (void)xTaskCreate(n6_capture_task, "n6_capture",
                      4096u / sizeof(StackType_t),
                      NULL, tskIDLE_PRIORITY + 3u, &s_n6_capture_handle);
  }
  if (s_n6_usb_handle == NULL) {
    (void)xTaskCreate(n6_usb_task, "n6_usb",
                      8192u / sizeof(StackType_t),
                      NULL, tskIDLE_PRIORITY + 3u, &s_n6_usb_handle);
  }
}

/* Starts the USB task and its queue without the capture task. Idempotent.
 * The Ethernet queue is made too: EthernetN6 reads it once a client connects. */
void NeuroN6_StreamUsbOnlyStart(void)
{
  if (g_n6_video_eth_queue == NULL) {
    g_n6_video_eth_queue = xQueueCreate(2, sizeof(n6_frame_msg_t));
  }
  if (s_n6_usb_handle != NULL) return;

  if (g_n6_video_usb_queue == NULL) {
    g_n6_video_usb_queue = xQueueCreate(2, sizeof(n6_frame_msg_t));
  }
  (void)xTaskCreate(n6_usb_task, "n6_usb",
                    8192u / sizeof(StackType_t),
                    NULL, tskIDLE_PRIORITY + 3u, &s_n6_usb_handle);
}

/* HAL callbacks */

void HAL_DCMIPP_PIPE_FrameEventCallback(DCMIPP_HandleTypeDef *hdcmipp, uint32_t Pipe)
{
  (void)hdcmipp;

  if (Pipe == DCMIPP_PIPE1)
  {
    g_dbg_pipe1_frames++;
    pipe1FrameReceived = 1;
    /* Zero tells the capture task that no frame is arriving. */
    g_dcmipp_wr_mblines = 0u;
    {
      const uint32_t now = HAL_GetTick();
      if (s_pipe1_last_end_ms != 0u) s_pipe1_frame_ms = now - s_pipe1_last_end_ms;
      s_pipe1_last_end_ms = now;
    }
    if ((s_capture_awaits_frame_end != 0u) && (s_n6_capture_handle != NULL)) {
      BaseType_t woken = pdFALSE;
      vTaskNotifyGiveFromISR(s_n6_capture_handle, &woken);   /* DCMIPP_IRQn is priority 5 */
      portYIELD_FROM_ISR(woken);
    }
  }
  if (Pipe == DCMIPP_PIPE2)
  {
    /* pipe1ready is released by the code that armed the pipe. */
    cameraFrameReceived = 1;
  }
}

/* One PIPE1 LINE event is depth*16 lines, so g_dcmipp_wr_mblines advances by depth. */
void HAL_DCMIPP_PIPE_LineEventCallback(DCMIPP_HandleTypeDef *h, uint32_t Pipe)
{
  (void)h;
  if (Pipe == DCMIPP_PIPE1) {
    g_dbg_pipe1_line_events++;
    uint32_t wr = g_dcmipp_wr_mblines + g_venc_band_depth;
    if ((g_venc_total_mblines != 0u) && (wr > g_venc_total_mblines)) wr = g_venc_total_mblines;
    g_dcmipp_wr_mblines = wr;
    /* Samples the VENC rdPtr against the producer, to detect a torn frame. */
    const uint32_t reg = LL_VENC_ReadRegister(497u);   /* [17:9] rows read, [8:0] rows handed over */
    const uint32_t rd  = (reg >> 9) & 0x1FFu;
    const uint32_t fed = reg & 0x1FFu;
    if (rd  > g_dbg_lb_rd_max)  g_dbg_lb_rd_max  = rd;
    if (wr  > g_dbg_lb_wr_max)  g_dbg_lb_wr_max  = wr;
    /* rd at 0 with every row handed over means the picture is finished. */
    if ((rd != 0u) || (fed < g_venc_total_mblines)) {
      const uint32_t gap = (wr > rd) ? (wr - rd) : (rd - wr);
      if (gap > g_dbg_lb_gap_max) g_dbg_lb_gap_max = gap;
    }
  }
}

/* DCMIPP and CSI error IRQs */
void HAL_DCMIPP_PIPE_ErrorCallback(DCMIPP_HandleTypeDef *h, uint32_t Pipe)
{
  g_dbg_cmsr2_sticky |= DCMIPP->CMSR2;   /* the flags self-clear */
  if (Pipe == DCMIPP_PIPE1) g_dbg_pipe1_err++;
  g_dbg_last_err_code = h->ErrorCode;
}

void HAL_DCMIPP_ErrorCallback(DCMIPP_HandleTypeDef *h)
{
  g_dbg_cmsr2_sticky |= DCMIPP->CMSR2;
  g_dbg_dcmipp_err++;
  g_dbg_last_err_code = h->ErrorCode;
}

void HAL_DCMIPP_CSI_LineErrorCallback(DCMIPP_HandleTypeDef *h, uint32_t DataLane)
{
  (void)DataLane;
  g_dbg_cmsr2_sticky |= DCMIPP->CMSR2;
  g_dbg_csi_line_err++;
  g_dbg_last_err_code = h->ErrorCode;
}

/* Error handling */

/* The fault report, sent over USB-CDC in polled mode. */
static char g_err_dump[3072];

static void n6_err_build_dump(const char *tag, uint32_t pc, uint32_t lr, uint32_t sp, uint32_t tick)
{
  uint32_t off = 0;
  #define DUMPF(...) do { if (off < sizeof(g_err_dump) - 1u) { \
      int _n = snprintf(g_err_dump + off, sizeof(g_err_dump) - off, __VA_ARGS__); \
      if (_n > 0) off += (uint32_t)_n; } } while (0)

  DUMPF("\r\n=== [ERRDUMP] Neuro N6 %s  tick=%lu ms ===\r\n", tag, (unsigned long)tick);
  DUMPF("PC=0x%08lX LR=0x%08lX SP=0x%08lX\r\n",
        (unsigned long)pc, (unsigned long)lr, (unsigned long)sp);

  DUMPF("[SCB] CFSR=%08lX HFSR=%08lX DFSR=%08lX MMFAR=%08lX BFAR=%08lX\r\n",
        (unsigned long)SCB->CFSR, (unsigned long)SCB->HFSR, (unsigned long)SCB->DFSR,
        (unsigned long)SCB->MMFAR, (unsigned long)SCB->BFAR);
  DUMPF("[SCB] SHCSR=%08lX ICSR=%08lX VECTACT=%lu AIRCR=%08lX CCR=%08lX VTOR=%08lX\r\n",
        (unsigned long)SCB->SHCSR, (unsigned long)SCB->ICSR,
        (unsigned long)(SCB->ICSR & 0x1FFu), (unsigned long)SCB->AIRCR,
        (unsigned long)SCB->CCR, (unsigned long)SCB->VTOR);

  DUMPF("[CLK] SYS=%lu HCLK=%lu PLL1=%lu CSI=%lu DCMIPP=%lu Hz\r\n",
        (unsigned long)HAL_RCC_GetSysClockFreq(), (unsigned long)HAL_RCC_GetHCLKFreq(),
        (unsigned long)HAL_RCCEx_GetPLL1CLKFreq(),
        (unsigned long)HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_CSI),
        (unsigned long)HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_DCMIPP));

  DUMPF("[HAL] dcmipp.State=%d ErrorCode=0x%08lX pipe[0/1/2]=%d/%d/%d | i2c1.State=%d ErrorCode=0x%08lX\r\n",
        (int)hdcmipp.State, (unsigned long)hdcmipp.ErrorCode,
        (int)hdcmipp.PipeState[0], (int)hdcmipp.PipeState[1], (int)hdcmipp.PipeState[2],
        (int)hi2c1.State, (unsigned long)hi2c1.ErrorCode);

  /* An unclocked DCMIPP would bus-fault here. */
  if (hdcmipp.State != HAL_DCMIPP_STATE_RESET) {
    DUMPF("[DCMIPP] CMSR1=%08lX CMSR2=%08lX CMIER=%08lX CMCR=%08lX\r\n",
          (unsigned long)DCMIPP->CMSR1, (unsigned long)DCMIPP->CMSR2,
          (unsigned long)DCMIPP->CMIER, (unsigned long)DCMIPP->CMCR);
    DUMPF("[DCMIPP] P0FSCR=%08lX P1FSCR=%08lX P1PPCR=%08lX P1FCTCR=%08lX\r\n",
          (unsigned long)DCMIPP->P0FSCR, (unsigned long)DCMIPP->P1FSCR,
          (unsigned long)DCMIPP->P1PPCR, (unsigned long)DCMIPP->P1FCTCR);
    uint32_t s1 = CSI->SR1;
    DUMPF("[CSI] CR=%08lX PCR=%08lX SR0=%08lX SR1=%08lX ERR1=%08lX ERR2=%08lX\r\n",
          (unsigned long)CSI->CR, (unsigned long)CSI->PCR, (unsigned long)CSI->SR0,
          (unsigned long)s1, (unsigned long)CSI->ERR1, (unsigned long)CSI->ERR2);
    DUMPF("[CSIDEC] clkACT=%lu L0[HS=%lu SYNC=%lu] L1[HS=%lu SYNC=%lu]\r\n",
          (unsigned long)!!(s1 & CSI_SR1_ACTCLF),
          (unsigned long)!!(s1 & CSI_SR1_ACTDL0F), (unsigned long)!!(s1 & CSI_SR1_SYNCDL0F),
          (unsigned long)!!(s1 & CSI_SR1_ACTDL1F), (unsigned long)!!(s1 & CSI_SR1_SYNCDL1F));
  } else {
    DUMPF("[DCMIPP] not clocked (State==RESET) - fault preceded HAL_DCMIPP_Init\r\n");
  }

  DUMPF("[APP] res=%lux%lu lb_hw=%u transport=%d csi_lane=%d band=0x%08lX\r\n",
        (unsigned long)usb_frame_width, (unsigned long)usb_frame_height,
        (unsigned)g_venc_lb_hw, (int)g_n6_active_transport,
        (int)_neuron6_csi_lane_mapping, (unsigned long)(uint32_t)g_venc_band);
  DUMPF("[DBG] pipe1=%lu csierr=%lu dcerr=%lu lastcode=0x%08lX\r\n",
        (unsigned long)g_dbg_pipe1_frames, (unsigned long)g_dbg_csi_line_err,
        (unsigned long)g_dbg_dcmipp_err, (unsigned long)g_dbg_last_err_code);

  /* Stack window: 64 bytes, all inside the AXISRAM the stacks use. */
  if (sp >= 0x34000000u && sp <= 0x34200000u - 64u) {
    const uint32_t *stk = (const uint32_t *)sp;
    for (uint32_t i = 0; i < 16u; i += 4u)
      DUMPF("[STK+%02lu] %08lX %08lX %08lX %08lX\r\n", (unsigned long)(i * 4u),
            (unsigned long)stk[i], (unsigned long)stk[i + 1u],
            (unsigned long)stk[i + 2u], (unsigned long)stk[i + 3u]);
  }
  DUMPF("=== [ERRDUMP] end ===\r\n");
  #undef DUMPF
}

/* Set while a fault is reported. A fault during the report only blinks. */
static volatile uint8_t s_in_err = 0u;

/* Sends g_err_dump repeatedly and never returns. The caller has set s_in_err,
 * disabled IRQs and built the dump. */
static void n6_err_emit_and_loop(void)
{
  tud_disconnect();
  for (volatile uint32_t i = 0; i < 8000000u; i++) {}
  tud_connect();

  for (;;) {
    /* The device stack is polled, as IRQs are off. */
    for (uint32_t k = 0; k < 4000u; k++) { tud_int_handler(BOARD_DEVICE_RHPORT_NUM); tud_task_ext(0, false); }

    if (tud_cdc_n_connected(0)) {
      uint32_t total = (uint32_t)strlen(g_err_dump), sent = 0, guard = 0;
      while (sent < total && guard++ < 4000000u) {
        tud_int_handler(BOARD_DEVICE_RHPORT_NUM); tud_task_ext(0, false);
        uint32_t avail = tud_cdc_n_write_available(0);
        if (avail) {
          uint32_t chunk = total - sent; if (chunk > avail) chunk = avail;
          sent += tud_cdc_n_write(0, (const uint8_t *)g_err_dump + sent, chunk);
          tud_cdc_n_write_flush(0);
        }
        if ((guard & 0x3FFFFu) == 0u) HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
      }
    }
    HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
  }
}

void Error_Handler(void)
{
  /* The caller and SP are captured first. */
  uint32_t lr = (uint32_t)__builtin_return_address(0);
  uint32_t sp; __asm volatile ("mov %0, sp" : "=r" (sp));
  uint32_t tick = HAL_GetTick();

  if (s_in_err) {
    __disable_irq();
    for (;;) { HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin); for (volatile uint32_t i = 0; i < 150000u; i++) {} }
  }
  s_in_err = 1u;

  __disable_irq();
  /* A HAL error has no exception frame, so PC is the caller's LR. */
  n6_crash_record_save(N6_CRASH_TAG_HAL, lr, lr);
  n6_err_build_dump("Error_Handler (HAL)", lr, lr, sp, tick);
  n6_err_emit_and_loop();
}

/* Called by the fault vectors in stm32n6xx_it.c.
 *   frame: r0, r1, r2, r3, r12, LR, PC, xPSR
 *   tag: 1 HardFault, 2 MemManage, 3 BusFault, 4 UsageFault, 5 NMI, 6 SecureFault */
void NeuroN6_FaultEntry(uint32_t *frame, uint32_t exc_return, uint32_t tag)
{
  (void)exc_return;
  uint32_t tick = HAL_GetTick();

  if (s_in_err) {
    __disable_irq();
    for (;;) { HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin); for (volatile uint32_t i = 0; i < 150000u; i++) {} }
  }
  s_in_err = 1u;
  __disable_irq();

  static const char *const k_tags[] = {
    "CPUFault", "HardFault", "MemManage", "BusFault", "UsageFault", "NMI", "SecureFault"
  };
  const char *tag_s = (tag <= 6u) ? k_tags[tag] : "CPUFault";

  uint32_t pc = 0u, lr = 0u, sp = (uint32_t)frame;
  if (sp >= 0x34000000u && sp <= 0x34200000u - 32u) {   /* the 32-byte frame is in AXISRAM */
    pc = frame[6];
    lr = frame[5];
  }
  n6_crash_record_save(tag, pc, lr);
  n6_err_build_dump(tag_s, pc, lr, sp, tick);
  n6_err_emit_and_loop();
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif

/* Overlay buffers */

/* Draws that waited for a committed reload to retire. */
static volatile uint32_t g_ovl_stalls = 0u;

uint32_t OVL_GetStallCount(void) { return g_ovl_stalls; }

/* Returns the buffer the CPU draws into, or NULL without an LCD. It waits up to
 * 50 ms for a reload requested by OVL_Commit() to retire. */
uint16_t *OVL_GetBackBuffer(void)
{
  if (g_ovl_back == 0) return NULL;

  const uint32_t t0 = HAL_GetTick();
  bool stalled = false;

  while ((hltdc.Instance->SRCR & (LTDC_SRCR_VBR | LTDC_SRCR_IMR)) != 0u) {
    stalled = true;
    if ((HAL_GetTick() - t0) > 50u) break;
  }

  if (stalled) g_ovl_stalls++;
  return g_ovl_back;
}

/* Shows the back buffer at the next vertical blank and swaps the pointers. */
void OVL_Commit(void)
{
  SCB_CleanDCache_by_Addr((uint32_t *)g_ovl_back,
                          (int32_t)ALIGN_UP(OVL_BYTES, DCACHE_ALIGN));

  HAL_LTDC_SetAddress_NoReload(&hltdc, (uint32_t)g_ovl_back, 1);
  HAL_LTDC_Reload(&hltdc, LTDC_RELOAD_VERTICAL_BLANKING);

  uint16_t *tmp = g_ovl_front;
  g_ovl_front   = g_ovl_back;
  g_ovl_back    = tmp;
}

/* Arduino hooks */

/* Runs between NeuroN6_Setup() and xTaskCreate(). A sketch or variant may override it. */
__attribute__((weak)) void initVariant(void) { }

/* 1200-bps touch: DTR dropping with the baud at 1200 resets to DFU. */
static volatile uint32_t s_cdc_last_baud = 0U;

void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const *p_line_coding)
{
  (void)itf;
  if (p_line_coding != NULL) {
    s_cdc_last_baud = p_line_coding->bit_rate;
  }
}

void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
  (void)itf;
  (void)rts;
  if (!dtr && s_cdc_last_baud == 1200U) {
    reboot_to_bootloader();
  }
}

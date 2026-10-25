/**
  ******************************************************************************
  * @file    NeuroVisionThermal.h
  * @brief   FLIR Lepton 3.1R thermal camera driver: CCI on I2C1, VoSPI on SPI1.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * SPDX-License-Identifier: MIT
  ******************************************************************************
  */

#ifndef NEUROVISION_THERMAL_H
#define NEUROVISION_THERMAL_H

#include <stdint.h>

/* CCI, the I2C control interface */

/* The HAL takes the 8-bit form. */
#define NVT_I2C_ADDR_7BIT   (0x2A)
#define NVT_I2C_ADDR_8BIT   (NVT_I2C_ADDR_7BIT << 1)   /* 0x54 */

/* CCI registers: 16-bit addresses, 16-bit big-endian data. */
#define NVT_REG_POWER          0x0000
#define NVT_REG_STATUS         0x0002
#define NVT_REG_COMMAND        0x0004
#define NVT_REG_DATA_LENGTH    0x0006
#define NVT_REG_DATA_0         0x0008   /* DATA 0 to 15, 2 apart */
#define NVT_REG_DATA_BLOCK     0xF800   /* for transfers over 16 words */

/* STATUS bits. */
#define NVT_STATUS_BUSY        (1u << 0)  /* a command is executing */
#define NVT_STATUS_BOOT_MODE   (1u << 1)  /* booted from internal OTP */
#define NVT_STATUS_BOOT_STATUS (1u << 2)  /* boot complete */
/* STATUS[15:8] holds the signed command result once BUSY clears: 0 is LEP_OK,
 * a negative value is a LEP error. */

/* COMMAND word:
 *   bits [1:0]   type: 0 GET, 1 SET, 2 RUN
 *   bits [11:8]  module: AGC 0x1, SYS 0x2, VID 0x3, OEM 0x8, RAD 0xE
 *   0x4000       protection bit, set for OEM and RAD */
#define NVT_CMD_SYS_PING        0x0202   /* RUN: ping */
#define NVT_CMD_SYS_STATUS      0x0204   /* GET: camera status, 4 words */
#define NVT_CMD_SYS_SERIAL      0x0208   /* GET: serial number, 4 words */
#define NVT_CMD_SYS_UPTIME      0x020C   /* GET: uptime in ms, 2 words */
#define NVT_CMD_SYS_FPA_TEMP_K  0x0214   /* GET: focal-plane temp, K*100, 1 word */
#define NVT_CMD_SYS_AUX_TEMP_K  0x0210   /* GET: housing temp, K*100, 1 word */
#define NVT_CMD_SYS_RUN_FFC     0x0242   /* RUN: flat-field correction */
#define NVT_CMD_AGC_ENABLE_GET  0x0100   /* GET: AGC enable, 2 words */
#define NVT_CMD_AGC_ENABLE_SET  0x0101   /* SET: AGC enable, 2 words */

/* Library result codes, outside the LEP result range. */
#define NVT_OK              0
#define NVT_ERR_I2C      (-1000)   /* an I2C transfer failed */
#define NVT_ERR_TIMEOUT  (-1001)   /* BUSY did not clear in time */
#define NVT_ERR_STREAMING (-1002)  /* CCI refused until videoEnd() */

/* VoSPI. A 160x120 frame is 4 segments of 60 packets of 164 bytes: a 2-byte
 * ID, a 2-byte CRC and 80 16-bit big-endian pixels. Low nibble 0xF in the
 * first ID byte marks a discard packet. Packet 20 of a segment carries the
 * segment number, 1 to 4, in ID bits [14:12]. */
#define NVT_VOSPI_WIDTH               160
#define NVT_VOSPI_HEIGHT              120
#define NVT_VOSPI_PIXELS              (NVT_VOSPI_WIDTH * NVT_VOSPI_HEIGHT)   /* 19200 */
#define NVT_VOSPI_PACKET_BYTES        164
#define NVT_VOSPI_PACKET_PIXELS       80
#define NVT_VOSPI_PACKETS_PER_SEGMENT 60
#define NVT_VOSPI_SEGMENTS            4

/* Neuro Studio streaming. One metadata TLV, little-endian:
 *   u8 type, u16 length
 *   u16 width, u16 height
 *   u8 format, u8 flags, u16 reserved
 *   u16 pixels[width x height], row-major
 * A sketch that calls streamFrame(), streamAnnounce() or serviceBegin() must
 * include <PostProcess.h>. */
#define NVT_META_THERMAL_FRAME   0x30   /* TLV type, META_THERMAL_FRAME */
#define NVT_THERMAL_FMT_RAW16    0      /* format: 16-bit LE pixels */
#define NVT_THERMAL_FLAG_TLINEAR (1u << 0) /* flags bit 0: centikelvin pixels */
#define NVT_THERMAL_TLV_BYTES    (3 + 8 + (NVT_VOSPI_PIXELS * 2))  /* 38411 */

/* videoLastError() codes. The KERCLK codes are not fatal. */
#define NVT_VIDEO_OK              0
#define NVT_VIDEO_ERR_KERCLK_CFG  1   /* HAL_RCCEx_PeriphCLKConfig failed */
#define NVT_VIDEO_ERR_KERCLK_ZERO 2   /* PCLK2 read as 0, 200 MHz assumed */
#define NVT_VIDEO_ERR_SPI_INIT    3   /* HAL_SPI_Init failed */
#define NVT_VIDEO_ERR_DMA_BUILD   4   /* GPDMA node build failed */
#define NVT_VIDEO_ERR_DMA_INIT    5   /* HAL_DMAEx_List_Init failed */
#define NVT_VIDEO_ERR_DMA_ATTR    6   /* HAL_DMA_ConfigChannelAttributes failed */
#define NVT_VIDEO_ERR_DMA_LINK    7   /* HAL_DMAEx_List_LinkQ failed */
#define NVT_VIDEO_ERR_SEM         8   /* semaphore creation failed */
#define NVT_VIDEO_ERR_DMA_START   9   /* HAL_SPI_Receive_DMA failed */

#ifdef __cplusplus

/** @brief  FLIR Lepton 3.1R driver. Every method blocks, so none may be
 *          called from an interrupt handler. */
class NeuroVisionThermal {
public:
  NeuroVisionThermal() : _lastResult(NVT_OK) {}

  /** @brief  Waits for boot, about 1.5 s after powerOn(). It polls only from
   *          2 s after powerOn(). False on timeout. */
  bool begin(uint32_t bootTimeoutMs = 3000);

  /** @brief  Power-cycles and resets the Lepton. Steady I2C traffic to any
   *          device while it boots, about 1.5 s, can stop the boot. */
  void powerOn(void);

  void readControlPins(bool &pwrdwnHigh, bool &resetHigh);

  /** @brief  True when the Lepton acknowledges its I2C address. */
  bool isPresent(void);

  /** @brief  Probes 0x08 to 0x77 and returns the number that acknowledge.
   *          foundAddrs7 may be NULL. 0x2A is skipped while video is active. */
  static uint8_t scanBus(uint8_t *foundAddrs7, uint8_t maxFound);

  bool readStatusRegister(uint16_t &status);

  /** @brief  True once STATUS shows boot complete and not busy. */
  bool isBooted(void);

  /** @brief  Polls STATUS until BUSY clears. */
  bool waitReady(uint32_t timeoutMs = 1000);

  bool readRegister(uint16_t reg, uint16_t &value);
  bool writeRegister(uint16_t reg, uint16_t value);

  /* CCI commands. command is the full COMMAND word. Each returns the LEP
   * result, 0 for LEP_OK, or an NVT_ERR code. */
  int16_t getAttribute(uint16_t command, uint16_t *outWords, uint16_t lenWords);
  int16_t setAttribute(uint16_t command, const uint16_t *inWords, uint16_t lenWords);
  int16_t runCommand(uint16_t command);

  /* Command wrappers. The bool ones return true on NVT_OK. */
  bool ping(void);
  bool getSerialNumber(uint16_t serialWords[4]);
  bool getUptimeMs(uint32_t &uptimeMs);
  bool getFpaTemperatureK100(uint16_t &kelvinX100); /* focal plane */
  bool getAuxTemperatureK100(uint16_t &kelvinX100); /* housing */
  bool getAgcEnabled(bool &enabled);
  int16_t runFfc(void);                         /* flat-field correction */

  /** @brief  Result of the most recent CCI command. */
  int16_t lastResult(void) const { return _lastResult; }

  /* VoSPI video on SPI1: SCK PA5, MISO PB4, MOSI PA7, CS PF4 */

  /** @brief  Sets up SPI1 and its DMA. Capture starts on the first readFrame(),
   *          and CCI calls are refused from here until videoEnd(). */
  bool videoBegin(void);

  /** @brief  Stops capture and releases SPI1. */
  void videoEnd(void);

  /** @brief  Idles CS and SCK for 300 ms, so the Lepton restarts at segment 1,
   *          packet 0. readFrame() does this when sync is lost. */
  void videoResync(void);

  /** @brief  SCK chosen by videoBegin(): the fastest at or below 20 MHz. */
  uint32_t videoSckHz(void) const;

  /** @brief  HAL_RCC_GetPCLK2Freq() as read by videoBegin(). */
  uint32_t videoKernelHz(void) const;

  /** @brief  NVT_VIDEO_ERR_* code of the last failure. */
  int videoLastError(void) const;

  /** @brief  Capture counters. The two half counts restart with each capture
   *          start. A half is half of the DMA ring. */
  void videoStats(uint32_t &halvesProduced, uint32_t &halvesConsumed,
                  uint32_t &segments, uint32_t &frames,
                  uint32_t &restarts) const;

  /** @brief  First four bytes of the last parsed half. */
  uint32_t videoLastRaw(void) const;

  /** @brief  Discard packets parsed since boot. */
  uint32_t videoDiscards(void) const;

  /** @brief  SPI error count and the last HAL ErrorCode. */
  uint32_t videoSpiErrors(void) const;
  uint32_t videoSpiErrCode(void) const;

  /** @brief  Rotates frames 180 degrees in readFrame(). */
  void setFlip180(bool on);
  bool flip180(void) const;

  /** @brief  Waits for the next frame of NVT_VOSPI_PIXELS row-major pixels, one
   *          every 115 ms. Pixels are centikelvin with TLinear, 14-bit counts
   *          without it, or 0 to 255 with AGC. */
  bool readFrame(uint16_t *frame, uint32_t timeoutMs = 2000);

  /* Neuro Studio streaming */

  /** @brief  For a board with no camera, in place of Vision.begin(). Starts the
   *          USB stream and returns the result of videoBegin(). */
  bool streamBegin(void);

  /** @brief  Publishes one frame. tlinear: the pixels are centikelvin. */
  bool streamFrame(const uint16_t *frame, bool tlinear = true);

  /** @brief  Publishes the TLV header with no pixels, which tells Neuro Studio
   *          a thermal stream is coming. */
  bool streamAnnounce(bool tlinear = true);

  /* Capture, publishing and recovery on one task, beside Vision.begin(). */

  /** @brief  Powers the Lepton up and starts the task. Returns true when the
   *          first frame arrives within firstFrameCapMs. */
  bool serviceBegin(bool tlinear = true, uint32_t firstFrameCapMs = 5000);

private:
  int16_t _lastResult;
  bool _readData(uint16_t *outWords, uint16_t lenWords);
  bool _writeData(const uint16_t *inWords, uint16_t lenWords);
};

#endif /* __cplusplus */

#endif /* NEUROVISION_THERMAL_H */

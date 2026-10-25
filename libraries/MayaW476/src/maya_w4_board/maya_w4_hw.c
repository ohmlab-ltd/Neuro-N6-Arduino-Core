/**
 ******************************************************************************
 * @file    maya_w4_hw.c
 * @brief   MAYA-W476 setup: SDMMC1, control GPIO and IW610 firmware download.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "stm32n6xx_hal.h"
#include "maya_w4_pinmap.h"
#include "wifi_bt_config.h"
#include "wlan.h"
#include "n6_host_contract.h"

#include <stdint.h>
#include <stdio.h>

extern void NeuroN6_DbgPrint(const char *msg);
extern void NeuroN6_Notice(const char *msg);     /* kept and replayed to each host */

/* IW610 firmware: a pointer into memory-mapped XSPI flash, its length, and the
 * CRC32 of the file it was written from. */
extern const unsigned char *wlan_fw_bin;
extern const unsigned int    wlan_fw_bin_len;
extern const unsigned int    wlan_fw_bin_crc;

SDIO_HandleTypeDef hsdio1;

extern void STM32N6_SDIO_IRQHandler(void);

/* Overrides the weak SDMMC1 vector. */
void SDMMC1_IRQHandler(void)
{
  STM32N6_SDIO_IRQHandler();
}

/* SDMMC1 kernel clock, from IC4, and pins:
 *   PC8 D0  PC9 D1  PC10 D2  PC11 D3  PC12 CK  PH2 CMD */
void HAL_SDIO_MspInit(SDIO_HandleTypeDef *hsdio)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  if (hsdio->Instance == SDMMC1)
  {
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SDMMC1;
    PeriphClkInitStruct.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_IC4;
    PeriphClkInitStruct.ICSelection[RCC_IC4].ClockSelection = RCC_ICCLKSOURCE_PLL1;
    /* 800 MHz PLL1 / 7 = 114.29 MHz; with CLKDIV 7, 8.163 MHz on the pin.
     * Do not change it without retesting emissions. */
    PeriphClkInitStruct.ICSelection[RCC_IC4].ClockDivider   = 7;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      NeuroN6_DbgPrint("[MayaW476] WARN: SDMMC1 clock config (IC4<-PLL1/7) "
                       "FAILED - WiFi will fall back to USB.\r\n");
    }

    __HAL_RCC_SDMMC1_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    /* HIGH, not VERY_HIGH: emissions were tested with this slew rate. */
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_SDMMC1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
  }
}

void HAL_SDIO_MspDeInit(SDIO_HandleTypeDef *hsdio)
{
  if (hsdio->Instance == SDMMC1)
  {
    /* Read back first, so the last write retires before the clock stops. */
    (void)READ_REG(hsdio->Instance->POWER);
    __DSB();
    __HAL_RCC_SDMMC1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12);
    HAL_GPIO_DeInit(GPIOH, GPIO_PIN_2);
  }
}

static void maya_w4_gpio_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /* Levels are written before the pins become outputs. */
  HAL_GPIO_WritePin(NB_WAKE_IN_MAYA_GPIO_Port,      NB_WAKE_IN_MAYA_Pin,      GPIO_PIN_RESET);
  HAL_GPIO_WritePin(PDn_pin_MAYA_GPIO_Port,         PDn_pin_MAYA_Pin,         GPIO_PIN_SET);

  GPIO_InitStruct.Pin  = NB_WAKE_OUT_MAYA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NB_WAKE_OUT_MAYA_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin   = NB_WAKE_IN_MAYA_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NB_WAKE_IN_MAYA_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin  = WL_WAKE_OUT_MAYA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(WL_WAKE_OUT_MAYA_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin   = PDn_pin_MAYA_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PDn_pin_MAYA_GPIO_Port, &GPIO_InitStruct);

  /* WL_WAKE_IN high keeps the WLAN awake while the stack runs. */
  HAL_GPIO_WritePin(WL_WAKE_IN_MAYA_GPIO_Port, WL_WAKE_IN_MAYA_Pin, GPIO_PIN_SET);
  GPIO_InitStruct.Pin   = WL_WAKE_IN_MAYA_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(WL_WAKE_IN_MAYA_GPIO_Port, &GPIO_InitStruct);
}

/* CRC32 of the firmware image as it reads from flash. The reads are volatile,
 * so each call reads the flash again. */
static uint32_t maya_w4_fw_crc(void)
{
  const volatile uint8_t *fw = (const volatile uint8_t *)wlan_fw_bin;
  uint8_t  chunk[256];
  uint32_t crc = 0U;

  for (uint32_t off = 0U; off < wlan_fw_bin_len; )
  {
    uint32_t n = wlan_fw_bin_len - off;
    if (n > sizeof(chunk)) n = sizeof(chunk);
    for (uint32_t i = 0U; i < n; i++) chunk[i] = fw[off + i];
    crc = n6_host_crc32(crc, chunk, n);
    off += n;
  }
  return crc;
}

/* Returns 0 once the IW610 firmware is running, else a negative code per
 * stage. Must run in FreeRTOS task context. */
int maya_w4_sdio_bringup(void)
{
  char dbg[256];

  /* Before FirstLightSDIO, which pulses PDn. */
  maya_w4_gpio_init();

  uint8_t cccr_rev = 0U;
  maya_w4_status_t st = MAYA_W4_FirstLightSDIO(&cccr_rev);
  if (st != MAYA_W4_OK)
  {
    snprintf(dbg, sizeof(dbg),
             "[MayaW476] the WiFi module did not answer on SDIO (status %d), so "
             "WiFi is off.\r\n", (int)st);
    NeuroN6_Notice(dbg);
    return -11;
  }
  {
    const volatile uint8_t *fw = (const volatile uint8_t *)wlan_fw_bin;
    uint32_t first = ((uint32_t)fw[0]) |
                     ((uint32_t)fw[1] << 8) |
                     ((uint32_t)fw[2] << 16) |
                     ((uint32_t)fw[3] << 24);
    uint32_t last = ((uint32_t)fw[wlan_fw_bin_len - 4U]) |
                    ((uint32_t)fw[wlan_fw_bin_len - 3U] << 8) |
                    ((uint32_t)fw[wlan_fw_bin_len - 2U] << 16) |
                    ((uint32_t)fw[wlan_fw_bin_len - 1U] << 24);
    if ((first == 0xFFFFFFFFUL) || (first == 0x00000000UL) ||
        (last == 0xFFFFFFFFUL)) {
      NeuroN6_Notice("[MayaW476] the WiFi firmware in flash is blank or cut short, so "
                     "WiFi is off.\r\n");
      return -12;
    }
  }

  /* The module refuses an image that differs from the signed file. A second
   * read tells a different image from a flash that is misread. */
  {
    const uint32_t crc = maya_w4_fw_crc();
    if (crc != wlan_fw_bin_crc)
    {
      const uint32_t again = maya_w4_fw_crc();
      if (again != crc)
      {
        snprintf(dbg, sizeof(dbg),
                 "[MayaW476] WARN: the WiFi firmware reads from flash differently "
                 "each time (CRC %08lX, then %08lX; the file is %08lX). Update the "
                 "bootloader if the uploader reports a release older than v2.6.1.\r\n",
                 (unsigned long)crc, (unsigned long)again,
                 (unsigned long)wlan_fw_bin_crc);
      }
      else
      {
        snprintf(dbg, sizeof(dbg),
                 "[MayaW476] WARN: the WiFi firmware in flash is not the file this "
                 "core ships (CRC %08lX, the file is %08lX). Upload the sketch with "
                 "Tools > Model weights > Upload every time; if this line stays, the "
                 "flash is being misread.\r\n",
                 (unsigned long)crc, (unsigned long)wlan_fw_bin_crc);
      }
      NeuroN6_Notice(dbg);
    }
  }

  /* Downloads the firmware over CMD53. */
  int wlan_rc = wlan_init(wlan_fw_bin, wlan_fw_bin_len);
  if (wlan_rc != WM_SUCCESS)
  {
    snprintf(dbg, sizeof(dbg),
             "[MayaW476] wlan_init FAILED (rc=%d). Is the IW610 firmware "
             "flashed to XSPI 0x91E00000?\r\n", wlan_rc);
    NeuroN6_Notice(dbg);
    return -12;
  }

  NeuroN6_DbgPrint("[MayaW476] wlan_init OK - IW610 firmware running.\r\n");
  return 0;
}

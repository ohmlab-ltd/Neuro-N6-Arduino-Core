/**
 ******************************************************************************
 * @file    ethernetif.c
 * @brief   lwIP netif for the STM32N6 ETH1 MAC and the LAN8742A RMII PHY.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

/* Enables the ETH HAL module for this file only. The core keeps it off. */
#define HAL_ETH_MODULE_ENABLED
#include "stm32n6xx_hal.h"

#include "lwip/opt.h"
#include "lwip/def.h"
#include "lwip/pbuf.h"
#include "lwip/netif.h"
#include "lwip/tcpip.h"
#include "lwip/netifapi.h"
#include "lwip/ip4_addr.h"
#include "netif/etharp.h"

#include "FreeRTOS.h"
#include "task.h"

#include "eth_if.h"
#include "N6lwip.h"
#include <string.h>
#include <stdio.h>

extern void NeuroN6_DbgPrint(const char *s);
extern void NeuroN6_Notice(const char *s);     /* kept and replayed to each host */

/* DMA memory. It sits in .axisram_nc, which MPU_Config marks non-cacheable. */
#define ETH_RX_POOL_CNT   12u      /* >= ETH_DMA_RX_CH_CNT*ETH_RX_DESC_CNT + slack */
#define ETH_RX_BUF_SZ   1536u      /* >= RxBuffLen of 1524, 32-aligned */
#define ETH_TX_BUF_SZ   1536u

ETH_DMADescTypeDef DMARxDscrTab[ETH_DMA_RX_CH_CNT][ETH_RX_DESC_CNT] __attribute__((section(".axisram_nc")));
ETH_DMADescTypeDef DMATxDscrTab[ETH_DMA_TX_CH_CNT][ETH_TX_DESC_CNT] __attribute__((section(".axisram_nc")));

static uint8_t s_rx_pool[ETH_RX_POOL_CNT][ETH_RX_BUF_SZ] __attribute__((section(".axisram_nc"), aligned(32)));
static uint8_t s_tx_buf[ETH_TX_BUF_SZ]                    __attribute__((section(".axisram_nc"), aligned(32)));

static ETH_HandleTypeDef heth1;
static ETH_TxPacketConfig s_txcfg;
static struct netif       s_eth_netif;
static uint8_t            s_mac[6];      /* eth_mac_from_uid() */

static volatile int s_link_up    = 0;
static volatile int s_netif_up   = 0;

/* RX buffer pool and the HAL_ETH Rx callbacks. They override weak symbols,
 * which relies on USE_HAL_ETH_REGISTER_CALLBACKS == 0. */
static uint8_t *s_rx_free[ETH_RX_POOL_CNT];
static int      s_rx_free_top = -1;                     /* -1: pool not initialised */
static uint8_t *s_rx_bufs[ETH_RX_DESC_CNT + 1];         /* buffers linked for the current frame */
static uint32_t s_rx_nbuf = 0;
static uint32_t s_rx_len  = 0;

static void rx_pool_free(uint8_t *b)
{
  if (b != NULL && s_rx_free_top >= 0 && s_rx_free_top < (int)ETH_RX_POOL_CNT) {
    s_rx_free[s_rx_free_top++] = b;
  }
}

void HAL_ETH_RxAllocateCallback(uint8_t **buff)
{
  if (s_rx_free_top < 0) {                               /* first call, inside HAL_ETH_Start */
    for (uint32_t i = 0; i < ETH_RX_POOL_CNT; i++) s_rx_free[i] = s_rx_pool[i];
    s_rx_free_top = (int)ETH_RX_POOL_CNT;
  }
  if (s_rx_free_top > 0) { *buff = s_rx_free[--s_rx_free_top]; }
  else                   { *buff = NULL; }
}

void HAL_ETH_RxLinkCallback(void **pStart, void **pEnd, uint8_t *buff, uint16_t Length)
{
  if (*pStart == NULL) *pStart = buff;
  *pEnd = buff;
  if (s_rx_nbuf < (sizeof(s_rx_bufs) / sizeof(s_rx_bufs[0]))) s_rx_bufs[s_rx_nbuf++] = buff;
  s_rx_len += Length;
}

/* ETH1 clocks and RMII pins.
 *   PD1 MDC   PD12 MDIO
 *   PF5 REF_CLK  PF10 CRS_DV  PF11 TX_EN  PF12 TXD0  PF13 TXD1  PF14 RXD0  PF15 RXD1
 *   PG15 RMII_REF_CLK */
static void eth_hw_msp(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  {
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ETH1;
    PeriphClkInitStruct.Eth1ClockSelection   = RCC_ETH1CLKSOURCE_HCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
      NeuroN6_Notice("[EthernetN6] ETH1 PeriphCLK config failed\r\n");
    }

    /* Without the MAC, TX and RX clocks the MDIO state machine does not run and
     * PHY reads return 0x0000. */
    __HAL_RCC_ETH1_CLK_ENABLE();
    __HAL_RCC_ETH1MAC_CLK_ENABLE();
    __HAL_RCC_ETH1TX_CLK_ENABLE();
    __HAL_RCC_ETH1RX_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    GPIO_InitStruct.Pin       = GPIO_PIN_1 | GPIO_PIN_12;               /* MDC, MDIO */
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_10 | GPIO_PIN_5 | GPIO_PIN_14 | GPIO_PIN_15
                              | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH1;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_15;                            /* RMII_REF_CLK */
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH1;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
  }

}

/* Called by HAL_ETH_Init when heth->State == RESET. eth_mac_init() also calls
 * eth_hw_msp() directly. */
void HAL_ETH_MspInit(ETH_HandleTypeDef *heth)
{
  if (heth->Instance == ETH1) eth_hw_msp();
}

/* LAN8742A nRST is on PE9. It is driven high and held, so the PHY runs from
 * its power-on reset. */
static void eth_phy_reset(void)
{
  __HAL_RCC_GPIOE_CLK_ENABLE();
  GPIO_InitTypeDef phy_rst = {0};
  phy_rst.Pin   = GPIO_PIN_9;
  phy_rst.Mode  = GPIO_MODE_OUTPUT_PP;
  phy_rst.Pull  = GPIO_NOPULL;
  phy_rst.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &phy_rst);

  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
  HAL_Delay(5);
  HAL_Delay(50);                                        /* PHY boot, straps latch */
}

/* Register dump for a failed HAL_ETH_Init.
 *   clkEN: the ETH1, MAC, TX and RX clock enables.
 *   CCIPR2: ETH1SEL, the RMII/MII interface bits.
 *   DMAMR: bit 0 is SWR, 1 while the DMA software reset is asserted.
 *   MACCR: 0 or 0xFFFFFFFF means register access is blocked or the clock is off.
 *   GPIOx AFR: the RMII pins read 0xB, AF11. */
static void eth_dump_regs(const char *tag)
{
  char d[190];
  (void)snprintf(d, sizeof(d),
    "[ETHDBG] %s clkEN ETH1=%d MAC=%d TX=%d RX=%d | HCLK=%lu CCIPR2=0x%08lX\r\n", tag,
    (int)__HAL_RCC_ETH1_IS_CLK_ENABLED(),   (int)__HAL_RCC_ETH1MAC_IS_CLK_ENABLED(),
    (int)__HAL_RCC_ETH1TX_IS_CLK_ENABLED(), (int)__HAL_RCC_ETH1RX_IS_CLK_ENABLED(),
    (unsigned long)HAL_RCC_GetHCLKFreq(), (unsigned long)RCC->CCIPR2);
  NeuroN6_DbgPrint(d);
  (void)snprintf(d, sizeof(d), "[ETHDBG] %s ETH1 DMAMR=0x%08lX MACCR=0x%08lX\r\n", tag,
    (unsigned long)ETH1->DMAMR, (unsigned long)ETH1->MACCR);
  NeuroN6_DbgPrint(d);
  (void)snprintf(d, sizeof(d),
    "[ETHDBG] %s D MODER=%08lX AFR=%08lX/%08lX | F MODER=%08lX AFR=%08lX/%08lX | G MODER=%08lX AFRH=%08lX\r\n", tag,
    (unsigned long)GPIOD->MODER, (unsigned long)GPIOD->AFR[0], (unsigned long)GPIOD->AFR[1],
    (unsigned long)GPIOF->MODER, (unsigned long)GPIOF->AFR[0], (unsigned long)GPIOF->AFR[1],
    (unsigned long)GPIOG->MODER, (unsigned long)GPIOG->AFR[1]);
  NeuroN6_DbgPrint(d);
}

/* A locally administered unicast address from a hash of the chip's 96-bit
 * unique ID, the same on every boot. */
static void eth_mac_from_uid(void)
{
  const uint32_t uid[3] = { HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2() };
  const uint8_t *p = (const uint8_t *)uid;
  uint64_t h = 14695981039346656037ULL;              /* FNV-1a, 64-bit */
  for (unsigned i = 0u; i < sizeof(uid); i++) {
    h ^= p[i];
    h *= 1099511628211ULL;
  }
  s_mac[0] = 0x02u;
  for (unsigned i = 1u; i < 6u; i++) s_mac[i] = (uint8_t)(h >> (8u * (i - 1u)));
}

/* Returns 0, or -1 when HAL_ETH_Init fails. */
static int eth_mac_init(void)
{
  eth_mac_from_uid();

  /* RIF: the ETH1 DMA master needs CID1, secure and privileged, to reach its
   * descriptors. Without it HAL_ETH_Init's DMA reset does not complete. */
  __HAL_RCC_RIFSC_CLK_ENABLE();
  {
    RIMC_MasterConfig_t rimc = {0};
    rimc.MasterCID = RIF_CID_1;
    rimc.SecPriv   = RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV;
    (void)HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_ETH1, &rimc);
  }

  /* The MAC clocks must be on before HAL_ETH_Init's software reset. */
  eth_hw_msp();

  heth1.Instance          = ETH1;
  heth1.Init.MACAddr      = (uint8_t *)s_mac;
  heth1.Init.MediaInterface = HAL_ETH_RMII_MODE;
  for (int ch = 0; ch < ETH_DMA_CH_CNT; ch++) {
    heth1.Init.TxDesc[ch] = DMATxDscrTab[ch];
    heth1.Init.RxDesc[ch] = DMARxDscrTab[ch];
  }
  heth1.Init.RxBuffLen = 1524;

  if (HAL_ETH_Init(&heth1) != HAL_OK) {
    char d[100];
    (void)snprintf(d, sizeof(d),
      "[EthernetN6] HAL_ETH_Init err=0x%08lX DMAMR=0x%08lX (SWR %s)\r\n",
      (unsigned long)heth1.ErrorCode,
      (unsigned long)heth1.Instance->DMAMR,
      (heth1.Instance->DMAMR & ETH_DMAMR_SWR) ? "STUCK -> clock/refclk" : "cleared -> later step");
    NeuroN6_DbgPrint(d);
    eth_dump_regs("post-fail");
    return -1;
  }

  memset(&s_txcfg, 0, sizeof(s_txcfg));
  s_txcfg.Attributes  = ETH_TX_PACKETS_FEATURES_CRCPAD;   /* MAC appends the FCS, lwIP does the checksums */
  s_txcfg.CRCPadCtrl  = ETH_CRC_PAD_INSERT;
  return 0;
}

/* Every wait is bounded, so a missing cable cannot hang begin(). */
static void eth_link_bringup(void)
{
  uint32_t v = 0;

  /* The slowest MDC divider, /324. With a faster one the pulled-up MDIO line
   * does not slew in time and the MAC samples 0x0000. */
  MODIFY_REG(heth1.Instance->MACMDIOAR, ETH_MACMDIOAR_CR, ETH_MACMDIOAR_CR_DIV324);

  /* Up to 5 s for link: BSR bit 2, read twice because it latches low. */
  TickType_t t0 = xTaskGetTickCount();
  do {
    HAL_ETH_ReadPHYRegister(&heth1, 0U, 1U, &v);
    HAL_ETH_ReadPHYRegister(&heth1, 0U, 1U, &v);
    s_link_up = (v & 0x0004U) ? 1 : 0;
    if (!s_link_up) vTaskDelay(pdMS_TO_TICKS(100));
  } while (!s_link_up && (xTaskGetTickCount() - t0) < pdMS_TO_TICKS(5000));

  /* Auto-negotiation: BCR bit 12 enables it and bit 9 restarts it. Up to 4 s
   * for BSR bit 5 to report it complete. */
  HAL_ETH_ReadPHYRegister(&heth1, 0U, 0U, &v);
  HAL_ETH_WritePHYRegister(&heth1, 0U, 0U, v | 0x1200U);
  t0 = xTaskGetTickCount();
  int an_done = 0;
  do {
    HAL_ETH_ReadPHYRegister(&heth1, 0U, 1U, &v);
    HAL_ETH_ReadPHYRegister(&heth1, 0U, 1U, &v);
    s_link_up = (v & 0x0004U) ? 1 : 0;
    an_done   = (v & 0x0020U) ? 1 : 0;
    if (!an_done) vTaskDelay(pdMS_TO_TICKS(100));
  } while (!an_done && (xTaskGetTickCount() - t0) < pdMS_TO_TICKS(4000));

  /* Speed and duplex: LAN8742A SCSR, reg 31, HCDSPEED bits [4:2]. */
  uint32_t speed = 100U, full = 1U;
  if (s_link_up) {
    HAL_ETH_ReadPHYRegister(&heth1, 0U, 31U, &v);
    speed = ((v >> 2) & 0x2U) ? 100U : 10U;
    full  = ((v >> 2) & 0x4U) ? 1U : 0U;
  }

  ETH_MACConfigTypeDef mc = {0};
  HAL_ETH_GetMACConfig(&heth1, &mc);
  mc.DuplexMode = full          ? ETH_FULLDUPLEX_MODE : ETH_HALFDUPLEX_MODE;
  mc.Speed      = (speed == 100U) ? ETH_SPEED_100M     : ETH_SPEED_10M;
  HAL_ETH_SetMACConfig(&heth1, &mc);
  HAL_ETH_Start(&heth1);   /* polling mode */
}

/* lwIP TX. It runs under the lwIP core lock, so the single s_tx_buf is safe. */
static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
  (void)netif;
  if (p->tot_len > ETH_TX_BUF_SZ) return ERR_MEM;
  pbuf_copy_partial(p, s_tx_buf, p->tot_len, 0);

  ETH_BufferTypeDef tb;
  tb.buffer = s_tx_buf;
  tb.len    = p->tot_len;
  tb.next   = NULL;

  s_txcfg.TxDMACh    = 0u;
  s_txcfg.Attributes = ETH_TX_PACKETS_FEATURES_CRCPAD;
  s_txcfg.Length     = p->tot_len;
  s_txcfg.TxBuffer   = &tb;

  return (HAL_ETH_Transmit(&heth1, &s_txcfg, 100U) == HAL_OK) ? ERR_OK : ERR_IF;
}

/* RX task. Polls HAL_ETH_ReadData and refreshes the link flag once a second. */
static void eth_rx_task(void *arg)
{
  (void)arg;
  TickType_t link_tick = xTaskGetTickCount();

  for (;;) {
    int got = 0;
    for (;;) {
      uint8_t *pkt = NULL;
      s_rx_nbuf = 0;
      s_rx_len  = 0;
      if (HAL_ETH_ReadData(&heth1, (void **)&pkt) == HAL_OK && pkt != NULL) {
        got = 1;
        /* MTU 1500 + 14 < RxBuffLen 1524, so every frame is a single buffer. */
        if (s_rx_nbuf == 1u && s_rx_len > 0u && s_rx_len <= ETH_RX_BUF_SZ) {
          struct pbuf *p = pbuf_alloc(PBUF_RAW, (u16_t)s_rx_len, PBUF_POOL);
          if (p != NULL) {
            pbuf_take(p, s_rx_bufs[0], (u16_t)s_rx_len);
            if (s_eth_netif.input(p, &s_eth_netif) != ERR_OK) pbuf_free(p);
          }
        }
        for (uint32_t i = 0; i < s_rx_nbuf; i++) rx_pool_free(s_rx_bufs[i]);
      } else {
        break;
      }
    }
    if (!got) vTaskDelay(pdMS_TO_TICKS(1));

    if ((xTaskGetTickCount() - link_tick) > pdMS_TO_TICKS(1000)) {
      link_tick = xTaskGetTickCount();
      uint32_t v = 0;
      HAL_ETH_ReadPHYRegister(&heth1, 0U, 1U, &v);
      HAL_ETH_ReadPHYRegister(&heth1, 0U, 1U, &v);
      s_link_up = (v & 0x0004U) ? 1 : 0;
    }
  }
}

/* netif init callback, run by netif_add under the lwIP core lock. */
static err_t ethernetif_init(struct netif *netif)
{
  netif->name[0]   = 'e';
  netif->name[1]   = 'n';
  netif->output    = etharp_output;
  netif->linkoutput = low_level_output;
  netif->hwaddr_len = 6;
  memcpy(netif->hwaddr, s_mac, 6);
  netif->mtu       = 1500;
  netif->flags     = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP;
  return ERR_OK;
}

/* Overridden by eth_video_start() in eth_video.c. */
__attribute__((weak)) void eth_video_start(void) { }


int eth_link_ready(void) { return (s_netif_up && s_link_up) ? 1 : 0; }

int eth_netif_start(uint32_t ip_be, uint32_t mask_be, uint32_t gw_be)
{
  /* Started once: a second netif_add of the same netif corrupts lwIP's list. */
  if (s_netif_up) return 0;

  /* Shared with the WiFi library: lwIP runs one tcpip thread. */
  if (n6_lwip_tcpip_start() != 0) {
    NeuroN6_Notice("[EthernetN6] the lwIP thread did not start.\r\n");
    return -1;
  }

  /* The order matters: HAL_ETH_Init, then the PHY reset, then the link. */
  if (eth_mac_init() != 0) {
    NeuroN6_Notice("[EthernetN6] HAL_ETH_Init FAILED\r\n");
    return -1;
  }
  eth_phy_reset();
  eth_link_bringup();

  ip4_addr_t ip, mask, gw;
  IP4_ADDR(&ip,   (ip_be   >> 24) & 0xFFu, (ip_be   >> 16) & 0xFFu, (ip_be   >> 8) & 0xFFu, ip_be   & 0xFFu);
  IP4_ADDR(&mask, (mask_be >> 24) & 0xFFu, (mask_be >> 16) & 0xFFu, (mask_be >> 8) & 0xFFu, mask_be & 0xFFu);
  IP4_ADDR(&gw,   (gw_be   >> 24) & 0xFFu, (gw_be   >> 16) & 0xFFu, (gw_be   >> 8) & 0xFFu, gw_be   & 0xFFu);

  if (netifapi_netif_add(&s_eth_netif, &ip, &mask, &gw, NULL, ethernetif_init, tcpip_input) != ERR_OK) {
    NeuroN6_Notice("[EthernetN6] netif_add FAILED\r\n");
    return -1;
  }
  netifapi_netif_set_default(&s_eth_netif);
  netifapi_netif_set_up(&s_eth_netif);
  s_netif_up = 1;

  xTaskCreate(eth_rx_task, "eth_rx", 1024u, NULL, tskIDLE_PRIORITY + 2u, NULL);
  eth_video_start();
  return 0;
}

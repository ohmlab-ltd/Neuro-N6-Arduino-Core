/*
 * BenchAdc
 *
 * On-board ADC diagnostic. Prints to the CDC console and to the vendor stream
 * as META_DEBUG_TEXT, which `n6bench.py stream` shows as DBG lines.
 *
 * Hardware: Neuro N6, no camera. The wire check expects A5 wired to GND and A0
 * to 1V8. The pin map drives the pads LOW and needs nothing wired.
 *
 * TUN0 id 0 starts a run: 1 full report, 2 channel scan, 3 pin map, 4 wire
 * check, 5 ASV and AVMEN toggles, 6 ADC12 secure, 7 ADC12 non-secure. A full
 * report also runs 15 s after boot.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */
#include <Arduino.h>
#include "NeuroN6.h"
#include "NeuroN6_Tune.h"
#include "stm32n6xx_hal.h"
#include <stdio.h>
#include <string.h>

static int32_t s_cmd = 0;               /* TUN0 id 0 */
static uint8_t s_ann[3 + 200];          /* 3-byte TLV header written by the core */
static char    s_line[200];

static void out(const char *s)
{
  Serial.println(s);
  size_t n = strlen(s);
  if (n > 200u) { n = 200u; }
  memcpy(&s_ann[3], s, n);
  for (int tries = 0; tries < 40; tries++) {
    if (NeuroN6_MetaAnnounce(s_ann, 0x17, (uint16_t)n)) { break; }
    delay(1);
  }
  delay(6);
}
#define P(...) do { snprintf(s_line, sizeof s_line, __VA_ARGS__); out(s_line); } while (0)

struct apin_t { const char *name; uint32_t ard; GPIO_TypeDef *port; uint16_t mask; uint8_t adc; uint8_t chan; };
/* Channel numbers as in the core's pin table, pins_arduino.c. */
static const apin_t k_apins[] = {
  { "A0/PF13", A0, GPIOF, GPIO_PIN_13, 2u, 2u  },
  { "A1/PF11", A1, GPIOF, GPIO_PIN_11, 1u, 2u  },
  { "A2/PF12", A2, GPIOF, GPIO_PIN_12, 1u, 6u  },
  { "A3/PF3",  A3, GPIOF, GPIO_PIN_3,  1u, 16u },
  { "A4/PF14", A4, GPIOF, GPIO_PIN_14, 2u, 6u  },
  { "A5/PG15", A5, GPIOG, GPIO_PIN_15, 1u, 7u  },
};

static const uint32_t k_chan[] = {
  ADC_CHANNEL_0,  ADC_CHANNEL_1,  ADC_CHANNEL_2,  ADC_CHANNEL_3,  ADC_CHANNEL_4,
  ADC_CHANNEL_5,  ADC_CHANNEL_6,  ADC_CHANNEL_7,  ADC_CHANNEL_8,  ADC_CHANNEL_9,
  ADC_CHANNEL_10, ADC_CHANNEL_11, ADC_CHANNEL_12, ADC_CHANNEL_13, ADC_CHANNEL_14,
  ADC_CHANNEL_15, ADC_CHANNEL_16, ADC_CHANNEL_17, ADC_CHANNEL_18, ADC_CHANNEL_19,
};

static ADC_HandleTypeDef s_h1, s_h2;
static bool s_ok1, s_ok2;

static void dump_state(const char *tag)
{
  const uint32_t s1 = PWR->SVMCR1, s2 = PWR->SVMCR2, s3 = PWR->SVMCR3;
  P("[%s] PWR SVMCR1=%08lx VDDIO4SV=%d | SVMCR2=%08lx VDDIO5SV=%d | SVMCR3=%08lx VDDIO2SV=%d VDDIO3SV=%d ASV=%d AVMEN=%d ARDY=%d",
    tag, (unsigned long)s1, (s1 & PWR_SVMCR1_VDDIO4SV) ? 1 : 0,
    (unsigned long)s2, (s2 & PWR_SVMCR2_VDDIO5SV) ? 1 : 0,
    (unsigned long)s3, (s3 & PWR_SVMCR3_VDDIO2SV) ? 1 : 0, (s3 & PWR_SVMCR3_VDDIO3SV) ? 1 : 0,
    (s3 & PWR_SVMCR3_ASV) ? 1 : 0, (s3 & PWR_SVMCR3_AVMEN) ? 1 : 0, (s3 & PWR_SVMCR3_ARDY) ? 1 : 0);

  const bool vrefclk = (RCC->APB4ENR1 & RCC_APB4ENR1_VREFBUFEN) != 0u;
  if (vrefclk) {
    const uint32_t csr = VREFBUF->CSR;
    P("[%s] VREFBUF clk=1 CSR=%08lx ENVR=%d HIZ=%d VRR=%d VRS=%lu (HIZ=1/ENVR=0 = pin driven externally, expected)",
      tag, (unsigned long)csr, (csr & VREFBUF_CSR_ENVR) ? 1 : 0, (csr & VREFBUF_CSR_HIZ) ? 1 : 0,
      (csr & VREFBUF_CSR_VRR) ? 1 : 0, (unsigned long)((csr & VREFBUF_CSR_VRS_Msk) >> VREFBUF_CSR_VRS_Pos));
  } else {
    P("[%s] VREFBUF clk=0 (CSR not read)", tag);
  }

  const uint32_t ccipr1 = RCC->CCIPR1;
  P("[%s] RCC CCIPR1=%08lx ADC12SEL=%lu ADCPRE=%lu (div %lu) | AHB1ENR ADC12EN=%d | HCLK=%lu adc_ker=%lu SystemCoreClock=%lu",
    tag, (unsigned long)ccipr1,
    (unsigned long)((ccipr1 & RCC_CCIPR1_ADC12SEL_Msk) >> RCC_CCIPR1_ADC12SEL_Pos),
    (unsigned long)((ccipr1 & RCC_CCIPR1_ADCPRE_Msk) >> RCC_CCIPR1_ADCPRE_Pos),
    (unsigned long)(((ccipr1 & RCC_CCIPR1_ADCPRE_Msk) >> RCC_CCIPR1_ADCPRE_Pos) + 1u),
    (RCC->AHB1ENR & RCC_AHB1ENR_ADC12EN) ? 1 : 0,
    (unsigned long)HAL_RCC_GetHCLKFreq(), (unsigned long)HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_ADC),
    (unsigned long)SystemCoreClock);

  __HAL_RCC_RIFSC_CLK_ENABLE();
  const uint32_t attr = HAL_RIF_RISC_GetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_ADC12);
  P("[%s] RIFSC ADC12 attr=%08lx sec=%d priv=%d", tag, (unsigned long)attr,
    (attr & RIF_ATTRIBUTE_SEC) ? 1 : 0, (attr & RIF_ATTRIBUTE_PRIV) ? 1 : 0);
}

static void pad_mode(unsigned i, uint32_t mode, uint32_t pull)
{
  GPIO_InitTypeDef g = {0};
  g.Pin  = k_apins[i].mask;
  g.Mode = mode;
  g.Pull = pull;
  HAL_GPIO_Init(k_apins[i].port, &g);
}

static void pads_analog(void)
{
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  for (unsigned i = 0; i < 6u; i++) { pad_mode(i, GPIO_MODE_ANALOG, GPIO_NOPULL); }
  P("[pads] GPIOF MODER=%08lx GPIOG MODER=%08lx (11 = analog) PUPDR F=%08lx",
    (unsigned long)GPIOF->MODER, (unsigned long)GPIOG->MODER, (unsigned long)GPIOF->PUPDR);
}

static void arduino_path(void)
{
  analogReadResolution(12);
  int v[6];
  for (unsigned i = 0; i < 6u; i++) { v[i] = analogRead(k_apins[i].ard); }
  P("[api] analogRead 12-bit: A0=%d A1=%d A2=%d A3=%d A4=%d A5=%d", v[0], v[1], v[2], v[3], v[4], v[5]);
}

static bool hal_setup(ADC_HandleTypeDef *h, ADC_TypeDef *inst, const char *tag)
{
  memset(h, 0, sizeof *h);
  h->Instance = inst;
  if (LL_ADC_IsEnabled(inst) != 0u) {
    LL_ADC_Disable(inst);
    const uint32_t t0 = millis();
    while (LL_ADC_IsEnabled(inst) != 0u && (millis() - t0) < 20u) { }
    P("[%s] was enabled -> disabled=%d", tag, LL_ADC_IsEnabled(inst) ? 0 : 1);
  }
  h->Init.Resolution            = ADC_RESOLUTION_12B;
  h->Init.ScanConvMode          = ADC_SCAN_DISABLE;
  h->Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
  h->Init.LowPowerAutoWait      = DISABLE;
  h->Init.ContinuousConvMode    = DISABLE;
  h->Init.NbrOfConversion       = 1;
  h->Init.DiscontinuousConvMode = DISABLE;
  h->Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  h->Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
  h->Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  h->Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
  h->Init.OversamplingMode      = DISABLE;

  HAL_StatusTypeDef st = HAL_ADC_Init(h);
  P("[%s] HAL_ADC_Init=%d state=%08lx err=%08lx CR=%08lx ISR=%08lx CFGR1=%08lx", tag, (int)st,
    (unsigned long)h->State, (unsigned long)h->ErrorCode, (unsigned long)inst->CR,
    (unsigned long)inst->ISR, (unsigned long)inst->CFGR1);
  if (st != HAL_OK) { return false; }

  st = HAL_ADCEx_Calibration_Start(h, ADC_SINGLE_ENDED);
  P("[%s] Calibration=%d factor=%lu CR=%08lx ISR=%08lx (ADEN=%d ADRDY=%d)", tag, (int)st,
    (unsigned long)LL_ADC_GetCalibrationFactor(inst, LL_ADC_SINGLE_ENDED),
    (unsigned long)inst->CR, (unsigned long)inst->ISR,
    (inst->CR & ADC_CR_ADEN) ? 1 : 0, (inst->ISR & ADC_ISR_ADRDY) ? 1 : 0);
  /* Calibration leaves the ADC enabled, and ConfigChannel refuses while it is. */
  const HAL_StatusTypeDef sst = HAL_ADC_Stop(h);
  P("[%s] HAL_ADC_Stop after calibration=%d -> ADEN=%d (ConfigChannel needs 0)", tag, (int)sst,
    (inst->CR & ADC_CR_ADEN) ? 1 : 0);
  return true;
}

/* Returns the raw 12-bit value, or a negative step code. */
static int32_t hal_read(ADC_HandleTypeDef *h, uint32_t chan)
{
  ADC_ChannelConfTypeDef c = {0};
  c.Channel      = chan;
  c.Rank         = ADC_REGULAR_RANK_1;
  c.SamplingTime = ADC_SAMPLETIME_246CYCLES_5;
  c.SingleDiff   = ADC_SINGLE_ENDED;
  c.OffsetNumber = ADC_OFFSET_NONE;
  if (HAL_ADC_ConfigChannel(h, &c) != HAL_OK) { (void)HAL_ADC_Stop(h); return -1; }
  if (HAL_ADC_Start(h) != HAL_OK) { (void)HAL_ADC_Stop(h); return -2; }
  if (HAL_ADC_PollForConversion(h, 10u) != HAL_OK) { (void)HAL_ADC_Stop(h); return -3; }
  const int32_t v = (int32_t)HAL_ADC_GetValue(h);
  (void)HAL_ADC_Stop(h);
  return v;
}

static void scan(ADC_HandleTypeDef *h, int32_t v[20])
{
  for (unsigned i = 0; i < 20u; i++) { v[i] = hal_read(h, k_chan[i]); }
}

static void print_scan(const char *tag, const int32_t v[20])
{
  P("[%s] ch0-9 : %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld", tag,
    (long)v[0], (long)v[1], (long)v[2], (long)v[3], (long)v[4], (long)v[5], (long)v[6], (long)v[7], (long)v[8], (long)v[9]);
  P("[%s] ch10-19: %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld", tag,
    (long)v[10], (long)v[11], (long)v[12], (long)v[13], (long)v[14], (long)v[15], (long)v[16], (long)v[17], (long)v[18], (long)v[19]);
}

static void scan_all(void)
{
  int32_t v[20];
  if (s_ok1) { scan(&s_h1, v); print_scan("ADC1", v); }
  if (s_ok2) { scan(&s_h2, v); print_scan("ADC2", v); }
  if (s_ok1) {
    P("[ADC1] after scan: PCSEL=%08lx SQR1=%08lx SMPR1=%08lx SMPR2=%08lx CR=%08lx CCR=%08lx",
      (unsigned long)ADC1->PCSEL, (unsigned long)ADC1->SQR1, (unsigned long)ADC1->SMPR1,
      (unsigned long)ADC1->SMPR2, (unsigned long)ADC1->CR, (unsigned long)ADC12_COMMON->CCR);
  }
}

static void read_apins(const char *tag)
{
  int32_t v[6];
  for (unsigned i = 0; i < 6u; i++) {
    ADC_HandleTypeDef *h = (k_apins[i].adc == 1u) ? &s_h1 : &s_h2;
    const bool ok = (k_apins[i].adc == 1u) ? s_ok1 : s_ok2;
    v[i] = ok ? hal_read(h, k_chan[k_apins[i].chan]) : -9;
  }
  const int32_t vref = s_ok1 ? hal_read(&s_h1, ADC_CHANNEL_VREFINT) : -9;
  P("[%s] A0=%ld A1=%ld A2=%ld A3=%ld A4=%ld A5=%ld | VREFINT=%ld (%ld mV if VREF+=1.8V) | ARDY=%d", tag,
    (long)v[0], (long)v[1], (long)v[2], (long)v[3], (long)v[4], (long)v[5],
    (long)vref, (long)((vref > 0) ? (vref * 1800L) / 4095L : -1),
    (PWR->SVMCR3 & PWR_SVMCR3_ARDY) ? 1 : 0);
}

/* OTP110 is VREFINT_CAL. Its BSEC shadow reads 0 until OTPCR.ADDR = 110 is
 * written with PROG and PPLOCK clear, which reads the fuse into the shadow. */
static uint32_t otp_reload_vrefint_cal(void)
{
  __HAL_RCC_BSEC_CLK_ENABLE();
  const uint32_t before = BSEC->FVRw[110];
  uint32_t t0 = millis();
  while ((BSEC->OTPSR & BSEC_OTPSR_BUSY) != 0u && (millis() - t0) < 20u) { }
  BSEC->OTPCR = (110u << BSEC_OTPCR_ADDR_Pos) & BSEC_OTPCR_ADDR_Msk;   /* PROG=0, PPLOCK=0 */
  t0 = millis();
  while ((BSEC->OTPSR & BSEC_OTPSR_BUSY) != 0u && (millis() - t0) < 20u) { }
  const uint32_t sr = BSEC->OTPSR;
  const uint32_t after = BSEC->FVRw[110];
  P("[otp] OTP110 shadow before=%08lx after reload=%08lx OTPSR=%08lx (BUSY=%d ERR=%d)",
    (unsigned long)before, (unsigned long)after, (unsigned long)sr,
    (sr & BSEC_OTPSR_BUSY) ? 1 : 0, (sr & BSEC_OTPSR_OTPERR) ? 1 : 0);
  return after;
}

static void internals(void)
{
  const int32_t vref = s_ok1 ? hal_read(&s_h1, ADC_CHANNEL_VREFINT) : -9;
  const int32_t vbat = s_ok2 ? hal_read(&s_h2, ADC_CHANNEL_VBAT)    : -9;
  const int32_t core = s_ok2 ? hal_read(&s_h2, ADC_CHANNEL_VDDCORE) : -9;
  P("[int] VREFINT(ADC1)=%ld VBAT(ADC2)=%ld VDDCORE(ADC2)=%ld CCR=%08lx", (long)vref, (long)vbat, (long)core,
    (unsigned long)ADC12_COMMON->CCR);
  const uint32_t word = otp_reload_vrefint_cal();
  const uint32_t cal  = word & 0xFFFFu;
  if (vref > 0 && cal > 100u && cal < 4095u) {
    P("[int] VREFINT_CAL=%lu (raw at VREF+=1800mV, factory) -> measured VREF+ = %ld mV, VREFINT = %ld mV",
      (unsigned long)cal, (long)((1800L * (long)cal) / vref), (long)((1800L * (long)cal) / 4095L));
  } else {
    P("[int] VREFINT_CAL unusable (%lu); VREFINT=%ld mV if VREF+=1800mV", (unsigned long)cal,
      (long)((vref > 0) ? (vref * 1800L) / 4095L : -1));
  }
}

/* A pad reaches the ADC only in analog mode, where the weak pulls are off.
 * A VREFINT read before each channel starts the sample-and-hold from 0.8 V. */
static void scan_precharged(ADC_HandleTypeDef *h, int32_t v[20])
{
  for (unsigned i = 0; i < 20u; i++) {
    (void)hal_read(&s_h1, ADC_CHANNEL_VREFINT);
    v[i] = hal_read(h, k_chan[i]);
  }
}

static void pin_map(void)
{
  int32_t base1[20], base2[20], v1[20], v2[20];
  pads_analog();
  delay(5);
  if (s_ok1) { scan_precharged(&s_h1, base1); }
  if (s_ok2) { scan_precharged(&s_h2, base2); }
  print_scan("map base ADC1", base1);
  print_scan("map base ADC2", base2);
  for (unsigned i = 0; i < 6u; i++) {
    HAL_GPIO_WritePin(k_apins[i].port, k_apins[i].mask, GPIO_PIN_RESET);
    pad_mode(i, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
    delay(2);
    pad_mode(i, GPIO_MODE_ANALOG, GPIO_NOPULL);
    if (s_ok1) { scan_precharged(&s_h1, v1); }
    if (s_ok2) { scan_precharged(&s_h2, v2); }
    char hits[120]; hits[0] = 0; size_t n = 0;
    for (unsigned c = 0; c < 20u; c++) {
      if (s_ok1 && (base1[c] - v1[c]) > 500) { n += snprintf(hits + n, sizeof hits - n, " ADC1_INP%u(%ld->%ld)", c, (long)base1[c], (long)v1[c]); }
      if (s_ok2 && (base2[c] - v2[c]) > 500) { n += snprintf(hits + n, sizeof hits - n, " ADC2_INP%u(%ld->%ld)", c, (long)base2[c], (long)v2[c]); }
      if (n >= sizeof hits - 30) { break; }
    }
    P("[map] %s grounded ->%s | table says ADC%u_INP%u", k_apins[i].name, hits[0] ? hits : " (no channel moved)",
      k_apins[i].adc, k_apins[i].chan);
    delay(50);
  }
}

/* IDR with the pull-up then the pull-down: a grounded pin reads 0/0, a floating one 1/0. */
static void wire_check(void)
{
  static const unsigned idx[2] = { 5u, 0u };
  for (unsigned k = 0; k < 2u; k++) {
    const unsigned i = idx[k];
    pad_mode(i, GPIO_MODE_INPUT, GPIO_PULLUP);
    delay(2);
    const int up = HAL_GPIO_ReadPin(k_apins[i].port, k_apins[i].mask);
    const uint32_t moder_in = k_apins[i].port->MODER, pupdr_up = k_apins[i].port->PUPDR;
    pad_mode(i, GPIO_MODE_INPUT, GPIO_PULLDOWN);
    delay(2);
    const int dn = HAL_GPIO_ReadPin(k_apins[i].port, k_apins[i].mask);
    pad_mode(i, GPIO_MODE_ANALOG, GPIO_NOPULL);
    delay(1);
    ADC_HandleTypeDef *h = (k_apins[i].adc == 1u) ? &s_h1 : &s_h2;
    const int32_t v = hal_read(h, k_chan[k_apins[i].chan]);
    P("[wire] %s: IDR pull-up=%d pull-down=%d | MODER(input)=%08lx PUPDR(up)=%08lx | back to analog: ADC%u_INP%u=%ld | SECCFGR=%08lx PRIVCFGR=%08lx LOCK=%08lx",
      k_apins[i].name, up, dn, (unsigned long)moder_in, (unsigned long)pupdr_up,
      k_apins[i].adc, k_apins[i].chan, (long)v,
      (unsigned long)k_apins[i].port->SECCFGR, (unsigned long)k_apins[i].port->PRIVCFGR,
      (unsigned long)k_apins[i].port->RCFGLOCKR);
  }
}

static uint32_t otp_word(uint32_t idx)
{
  __HAL_RCC_BSEC_CLK_ENABLE();
  uint32_t t0 = millis();
  while ((BSEC->OTPSR & BSEC_OTPSR_BUSY) != 0u && (millis() - t0) < 20u) { }
  BSEC->OTPCR = (idx << BSEC_OTPCR_ADDR_Pos) & BSEC_OTPCR_ADDR_Msk;   /* PROG=0: read */
  t0 = millis();
  while ((BSEC->OTPSR & BSEC_OTPSR_BUSY) != 0u && (millis() - t0) < 20u) { }
  return BSEC->FVRw[idx];
}

/* ASV is the VDDA18ADC supply-valid bit, AVMEN its monitor. */
static void pwr_experiments(void)
{
  read_apins("pwr before");
  CLEAR_BIT(PWR->SVMCR3, PWR_SVMCR3_ASV);  (void)PWR->SVMCR3; delay(10);
  read_apins("pwr ASV=0");
  SET_BIT(PWR->SVMCR3, PWR_SVMCR3_ASV);    (void)PWR->SVMCR3; delay(10);
  read_apins("pwr ASV=1 again");
  CLEAR_BIT(PWR->SVMCR3, PWR_SVMCR3_AVMEN); (void)PWR->SVMCR3; delay(10);
  read_apins("pwr AVMEN=0");
  SET_BIT(PWR->SVMCR3, PWR_SVMCR3_AVMEN);   (void)PWR->SVMCR3; delay(10);
  read_apins("pwr AVMEN=1 again");
  P("[otp] OTP124(HCONF)=%08lx OTP125=%08lx OTP110=%08lx | PWR SVMCR1=%08lx SVMCR2=%08lx SVMCR3=%08lx CR1=%08lx",
    (unsigned long)otp_word(124u), (unsigned long)otp_word(125u), (unsigned long)otp_word(110u),
    (unsigned long)PWR->SVMCR1, (unsigned long)PWR->SVMCR2, (unsigned long)PWR->SVMCR3, (unsigned long)PWR->CR1);
}

/* The GPIO pads are secure and ADC12 boots non-secure. An external channel
 * reads a constant mid-scale until ADC12 is marked secure in the RIFSC. */
static void rifsc_set(uint32_t attr, const char *tag)
{
  __HAL_RCC_RIFSC_CLK_ENABLE();
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_ADC12, attr);
  const uint32_t rd = HAL_RIF_RISC_GetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_ADC12);
  P("[rif] ADC12 attr set=%lu read=%lu (%s)", (unsigned long)attr, (unsigned long)rd, tag);
  s_ok1 = hal_setup(&s_h1, ADC1, "rif ADC1");
  s_ok2 = hal_setup(&s_h2, ADC2, "rif ADC2");
  read_apins(tag);
  wire_check();
}

static void full_report(void)
{
  P("[bench] ---- full report, uptime=%lu ms ----", (unsigned long)millis());
  dump_state("p0");
  pads_analog();
  arduino_path();
  s_ok1 = hal_setup(&s_h1, ADC1, "ADC1");
  s_ok2 = hal_setup(&s_h2, ADC2, "ADC2");
  scan_all();
  internals();
  read_apins("hal");
  wire_check();
  /* pin_map() drives the pads LOW: cmd 3 only, with no wires on the header. */
  P("[bench] ---- report done; live line every 2 s; TUN0 id0: 1 full, 2 scan, 3 map (drives pads LOW!) ----");
}

static uint32_t s_last_live = 0;

void setup()
{
  memset(s_ann, 0, sizeof s_ann);
  n6_tune_int("cmd", &s_cmd, 0, 9);
  const uint32_t t0 = millis();
  while (s_cmd == 0 && (millis() - t0) < 15000u) { delay(10); }
  P("[bench] BenchAdc up, uptime=%lu ms (serial=%d cmd=%ld)", (unsigned long)millis(), Serial ? 1 : 0, (long)s_cmd);
  s_cmd = 0;
  full_report();
}

void loop()
{
  if (s_cmd == 1) { s_cmd = 0; full_report(); }
  else if (s_cmd == 2) { s_cmd = 0; scan_all(); internals(); }
  else if (s_cmd == 3) { s_cmd = 0; pin_map(); }
  else if (s_cmd == 4) { s_cmd = 0; wire_check(); }
  else if (s_cmd == 5) { s_cmd = 0; pwr_experiments(); }
  else if (s_cmd == 6) { s_cmd = 0; rifsc_set(RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV, "ADC12 secure"); }
  else if (s_cmd == 7) { s_cmd = 0; rifsc_set(0u, "ADC12 non-secure"); }
  if ((millis() - s_last_live) >= 2000u) {
    s_last_live = millis();
    if (s_ok1 || s_ok2) { read_apins("live"); }
  }
  delay(10);
}

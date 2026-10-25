/**
 ******************************************************************************
 * @file    wiring_analog.c
 * @brief   analogRead (ADC1/ADC2), analogWrite (TIM PWM), tone (TIM3).
 ******************************************************************************
 * Copyright (c) 2025-2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include <stdio.h>
#include "wiring_digital.h"
#include "wiring_analog.h"

extern void NeuroN6_DbgPrint(const char *msg);
extern uint32_t SystemCoreClock;

static void analog_warn(const char *what, uint32_t num)
{
  char msg[96];
  snprintf(msg, sizeof msg, "[WIRING] %s (%lu)\r\n", what, (unsigned long)num);
  NeuroN6_DbgPrint(msg);
}

/* analogRead */

static ADC_HandleTypeDef s_hadc1 = { .Instance = ADC1 };
static ADC_HandleTypeDef s_hadc2 = { .Instance = ADC2 };
static bool s_adc_ready[2];       /* [0]=ADC1 [1]=ADC2 */
static volatile int s_read_bits = 10;   /* Arduino default 0..1023 */

static const uint32_t s_adc_chan_lut[] = {
  ADC_CHANNEL_0,  ADC_CHANNEL_1,  ADC_CHANNEL_2,  ADC_CHANNEL_3,
  ADC_CHANNEL_4,  ADC_CHANNEL_5,  ADC_CHANNEL_6,  ADC_CHANNEL_7,
  ADC_CHANNEL_8,  ADC_CHANNEL_9,  ADC_CHANNEL_10, ADC_CHANNEL_11,
  ADC_CHANNEL_12, ADC_CHANNEL_13, ADC_CHANNEL_14, ADC_CHANNEL_15,
  ADC_CHANNEL_16, ADC_CHANNEL_17, ADC_CHANNEL_18, ADC_CHANNEL_19,
};

void analogReadResolution(int bits)
{
  if (bits >= 8 && bits <= 16) { s_read_bits = bits; }
}

static bool adc_lazy_init(uint8_t adc_num)
{
  ADC_HandleTypeDef *h = (adc_num == 1u) ? &s_hadc1 : &s_hadc2;
  if (s_adc_ready[adc_num - 1u]) { return true; }

  /* The ADC prescaler is RCC's AdcDivider: 25 MHz from a 400 MHz HCLK. */
  RCC_PeriphCLKInitTypeDef pclk = {0};
  pclk.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  pclk.AdcClockSelection    = RCC_ADCCLKSOURCE_HCLK;
  pclk.AdcDivider           = 16u;
  if (HAL_RCCEx_PeriphCLKConfig(&pclk) != HAL_OK) { return false; }

  __HAL_RCC_ADC12_CLK_ENABLE();

  /* The N6 routes an ADC input from a GPIO pad only when the ADC's RIFSC
   * security attribute matches the pad's. The pads boot secure. */
  __HAL_RCC_RIFSC_CLK_ENABLE();
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_ADC12, RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);

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

  if (HAL_ADC_Init(h) != HAL_OK) { return false; }

  if (HAL_ADCEx_Calibration_Start(h, ADC_SINGLE_ENDED) != HAL_OK) {
    analog_warn("ADC calibration failed", adc_num);
    /* Continues uncalibrated. */
  }

  /* Calibration leaves ADEN=1, and HAL_ADC_ConfigChannel refuses while the
   * ADC is enabled. */
  (void)HAL_ADC_Stop(h);

  s_adc_ready[adc_num - 1u] = true;
  return true;
}

int analogRead(uint32_t pin)
{
  /* Read once: another task may change it during the conversion. */
  const int bits = s_read_bits;
  const PinDescription *d = pin_desc(pin);
  if (d == NULL || d->adc_num == 0xFFu) { return 0; }

  if (!adc_lazy_init(d->adc_num)) {
    analog_warn("analogRead: ADC init failed for pin", pin);
    return 0;
  }

  ADC_HandleTypeDef *h = (d->adc_num == 1u) ? &s_hadc1 : &s_hadc2;

  pin_enable_gpio_clock(d->port);
  GPIO_InitTypeDef init = {0};
  init.Pin  = d->mask;
  init.Mode = GPIO_MODE_ANALOG;
  init.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(d->port, &init);

  ADC_ChannelConfTypeDef ch = {0};
  ch.Channel      = s_adc_chan_lut[d->adc_chan];
  ch.Rank         = ADC_REGULAR_RANK_1;
  ch.SamplingTime = ADC_SAMPLETIME_246CYCLES_5;   /* long, for high-Z sources */
  ch.SingleDiff   = ADC_SINGLE_ENDED;
  ch.OffsetNumber = ADC_OFFSET_NONE;
  /* Every error path must leave the ADC disabled, or the next call's
   * ConfigChannel is refused. */
  if (HAL_ADC_ConfigChannel(h, &ch) != HAL_OK) {
    analog_warn("analogRead: channel config failed for pin", pin);
    (void)HAL_ADC_Stop(h);
    return 0;
  }

  if (HAL_ADC_Start(h) != HAL_OK) {
    analog_warn("analogRead: start failed for pin", pin);
    (void)HAL_ADC_Stop(h);
    return 0;
  }
  if (HAL_ADC_PollForConversion(h, 10u) != HAL_OK) {   /* ms */
    HAL_ADC_Stop(h);
    return 0;
  }
  uint32_t raw = HAL_ADC_GetValue(h);   /* 12-bit */
  HAL_ADC_Stop(h);

  if (bits == 12) { return (int)raw; }
  if (bits > 12)  { return (int)(raw << (bits - 12)); }
  return (int)(raw >> (12 - bits));
}

/* Timer kernel-clock measurement, shared by PWM and tone. */

static void tim_clk_enable(TIM_TypeDef *tim)
{
  if      (tim == TIM1)  { __HAL_RCC_TIM1_CLK_ENABLE();  }
  else if (tim == TIM2)  { __HAL_RCC_TIM2_CLK_ENABLE();  }
  else if (tim == TIM3)  { __HAL_RCC_TIM3_CLK_ENABLE();  }
  else if (tim == TIM15) { __HAL_RCC_TIM15_CLK_ENABLE(); }
  else if (tim == TIM17) { __HAL_RCC_TIM17_CLK_ENABLE(); }
}

/* Measures a timer's kernel clock against the DWT cycle counter. */
static uint32_t tim_measure_clk_hz(TIM_TypeDef *tim)
{
  tim_clk_enable(tim);

  uint32_t saved_cr1 = tim->CR1;
  tim->CR1 = 0;             /* stop */
  tim->PSC = 0;
  tim->ARR = 0xFFFFFFFFu;   /* 16-bit timers keep the low half */
  tim->EGR = TIM_EGR_UG;    /* latch PSC */
  tim->CNT = 0;
  tim->CR1 = TIM_CR1_CEN;

  /* 50 us window: 20,000 ticks at 400 MHz, inside the 16-bit counters. With
   * interrupts masked, no preemption can stretch it past a counter wrap. */
  uint32_t cyc_per_us = SystemCoreClock / 1000000u;
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  uint32_t t0 = DWT->CYCCNT;
  uint32_t c0 = tim->CNT;
  while ((DWT->CYCCNT - t0) < (50u * cyc_per_us)) { }
  uint32_t elapsed_cyc = DWT->CYCCNT - t0;
  uint32_t ticks = (tim->CNT - c0) & 0xFFFFu;
  __set_PRIMASK(primask);

  tim->CR1 = saved_cr1;

  if (elapsed_cyc == 0u) { return 0; }
  return (uint32_t)(((uint64_t)ticks * SystemCoreClock) / elapsed_cyc);
}

/* analogWrite */

static int      s_write_bits = 8;      /* Arduino default 0..255 */
static uint32_t s_pwm_hz     = 1000u;

typedef struct {
  TIM_HandleTypeDef htim;
  uint32_t          clk_hz;   /* measured kernel clock */
  bool              inited;
} pwm_tim_state_t;

static pwm_tim_state_t s_pwm[4];   /* TIM1, TIM2, TIM15, TIM17 */

static pwm_tim_state_t *pwm_state_for(TIM_TypeDef *tim)
{
  if (tim == TIM1)  { return &s_pwm[0]; }
  if (tim == TIM2)  { return &s_pwm[1]; }
  if (tim == TIM15) { return &s_pwm[2]; }
  if (tim == TIM17) { return &s_pwm[3]; }
  return NULL;
}

void analogWriteResolution(int bits)
{
  if (bits >= 4 && bits <= 16) { s_write_bits = bits; }
}

void analogWriteFrequency(uint32_t hz)
{
  if (hz >= 1u && hz <= 1000000u) { s_pwm_hz = hz; }
  /* Applies at the next analogWrite call. */
  for (unsigned i = 0; i < 4u; i++) { s_pwm[i].inited = false; }
}

static const uint32_t s_tim_chan_lut[] = {
  0, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4,
};

void analogWrite(uint32_t pin, int value)
{
  const PinDescription *d = pin_desc(pin);
  if (d == NULL) { return; }

  int maxv = (1 << s_write_bits) - 1;
  if (value < 0) { value = 0; }
  if (value > maxv) { value = maxv; }

  if (d->pwm_tim == NULL) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, (value >= (maxv + 1) / 2) ? HIGH : LOW);
    return;
  }

  pwm_tim_state_t *st = pwm_state_for(d->pwm_tim);
  if (st == NULL) { return; }

  if (!st->inited) {
    tim_clk_enable(d->pwm_tim);
    if (st->clk_hz == 0u) { st->clk_hz = tim_measure_clk_hz(d->pwm_tim); }
    if (st->clk_hz == 0u) { return; }

    uint32_t period = st->clk_hz / s_pwm_hz;
    uint32_t psc = 0;
    while ((period / (psc + 1u)) > 0xFFFFu) { psc++; }

    st->htim.Instance               = d->pwm_tim;
    st->htim.Init.Prescaler         = psc;
    st->htim.Init.CounterMode       = TIM_COUNTERMODE_UP;
    st->htim.Init.Period            = (period / (psc + 1u)) - 1u;
    st->htim.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    st->htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&st->htim) != HAL_OK) { return; }
    st->inited = true;
  }

  uint32_t chan = s_tim_chan_lut[d->pwm_chan];
  uint32_t arr  = st->htim.Init.Period;
  uint32_t ccr  = (uint32_t)(((uint64_t)value * (arr + 1u)) / (uint32_t)(maxv + 1));

  TIM_OC_InitTypeDef oc = {0};
  oc.OCMode     = TIM_OCMODE_PWM1;
  oc.Pulse      = ccr;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&st->htim, &oc, chan) != HAL_OK) { return; }

  pin_enable_gpio_clock(d->port);
  GPIO_InitTypeDef init = {0};
  init.Pin       = d->mask;
  init.Mode      = GPIO_MODE_AF_PP;
  init.Pull      = GPIO_NOPULL;
  init.Speed     = GPIO_SPEED_FREQ_LOW;
  init.Alternate = d->pwm_af;
  HAL_GPIO_Init(d->port, &init);

  HAL_TIM_PWM_Start(&st->htim, chan);   /* sets BDTR.MOE on TIM1/15/17 */
}

/* tone and noTone: the TIM3 update interrupt toggles the pin. */

#define TONE_NVIC_PRIO 8u

static volatile uint32_t s_tone_pin = NOT_A_PIN_NUM;
static volatile uint32_t s_tone_halfcycles_left;   /* 0 = indefinite */
static uint32_t s_tim3_clk_hz;

void tone(uint32_t pin, unsigned int frequency, unsigned long duration_ms)
{
  const PinDescription *d = pin_desc(pin);
  if (d == NULL || frequency == 0u) { return; }

  /* One tone at a time: a different pin is ignored until noTone(). */
  if (s_tone_pin != NOT_A_PIN_NUM && s_tone_pin != pin) { return; }

  pinMode(pin, OUTPUT);

  tim_clk_enable(TIM3);
  if (s_tim3_clk_hz == 0u) { s_tim3_clk_hz = tim_measure_clk_hz(TIM3); }
  if (s_tim3_clk_hz == 0u) { return; }

  uint32_t half = s_tim3_clk_hz / (2u * frequency);   /* ticks per half period */
  uint32_t psc = 0;
  while ((half / (psc + 1u)) > 0xFFFFu) { psc++; }

  /* 0 means indefinite, so a duration shorter than one half-cycle plays one
   * full cycle. */
  uint32_t halfcycles = 0u;
  if (duration_ms != 0u) {
    halfcycles = (uint32_t)(((uint64_t)duration_ms * frequency * 2u) / 1000u);
    if (halfcycles == 0u) { halfcycles = 2u; }
  }

  s_tone_pin = pin;
  s_tone_halfcycles_left = halfcycles;

  TIM3->CR1 = 0;
  TIM3->PSC = psc;
  TIM3->ARR = (half / (psc + 1u)) - 1u;
  TIM3->CNT = 0;
  TIM3->EGR = TIM_EGR_UG;
  TIM3->SR  = 0;
  TIM3->DIER = TIM_DIER_UIE;

  HAL_NVIC_SetPriority(TIM3_IRQn, TONE_NVIC_PRIO, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);

  TIM3->CR1 = TIM_CR1_CEN;
}

void noTone(uint32_t pin)
{
  if (s_tone_pin != pin) { return; }
  TIM3->CR1  = 0;
  TIM3->DIER = 0;
  HAL_NVIC_DisableIRQ(TIM3_IRQn);
  s_tone_pin = NOT_A_PIN_NUM;
  digitalWrite(pin, LOW);
}

void TIM3_IRQHandler(void)
{
  TIM3->SR = 0;   /* clear UIF */
  uint32_t pin = s_tone_pin;
  if (pin == NOT_A_PIN_NUM) { return; }

  digitalToggle(pin);

  uint32_t left = s_tone_halfcycles_left;
  if (left != 0u) {
    if (--left == 0u) {
      TIM3->CR1  = 0;
      TIM3->DIER = 0;
      s_tone_pin = NOT_A_PIN_NUM;
      digitalWrite(pin, LOW);
    }
    s_tone_halfcycles_left = left;
  }
}

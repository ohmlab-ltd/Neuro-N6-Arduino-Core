/**
 * @file    NeuroN6_PowerSave.cpp
 * @brief   Sleep modes, clock profiles and peripheral power-down.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#include "NeuroN6_PowerSave.h"

#include "NeuroN6_app.h"
#include "stm32n6xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"

extern "C" bool tud_disconnect(void);
extern "C" bool tud_mounted(void);

/* HAL tick counter. */
extern "C" __IO uint32_t uwTick;

NeuroN6PowerSave PowerSave;

/* Idle-task WFI */
static volatile bool     s_idleWfiOn    = false;

/* Sleep stops every peripheral clock whose sleep enable bit is 0, its reset
 * value. Copying the run enables before each WFI leaves Sleep stopping only
 * the CPU. */
static void ps_sleep_keeps_run_clocks(void)
{
  RCC->BUSLPENSR   = RCC->BUSENR;
  RCC->MISCLPENSR  = RCC->MISCENR;
  RCC->MEMLPENSR   = RCC->MEMENR;
  RCC->AHB1LPENSR  = RCC->AHB1ENR;
  RCC->AHB2LPENSR  = RCC->AHB2ENR;
  RCC->AHB3LPENSR  = RCC->AHB3ENR;
  RCC->AHB4LPENSR  = RCC->AHB4ENR;
  RCC->AHB5LPENSR  = RCC->AHB5ENR;
  RCC->APB1LPENSR1 = RCC->APB1ENR1;
  RCC->APB1LPENSR2 = RCC->APB1ENR2;
  RCC->APB2LPENSR  = RCC->APB2ENR;
  RCC->APB3LPENSR  = RCC->APB3ENR;
  RCC->APB4LPENSR1 = RCC->APB4ENR1;
  RCC->APB4LPENSR2 = RCC->APB4ENR2;
  RCC->APB5LPENSR  = RCC->APB5ENR;
}

void NeuroN6PowerSave::idleWfiEnable()
{
  ps_sleep_keeps_run_clocks();
  s_idleWfiOn = true;
}
void NeuroN6PowerSave::idleWfiDisable() { s_idleWfiOn = false; }

extern "C" void vApplicationIdleHook(void)
{
  if (!s_idleWfiOn) return;
  if (((SCB->SCR & (SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk)) == 0u) &&
      (__get_BASEPRI() == 0u) && (__get_PRIMASK() == 0u)) {
    ps_sleep_keeps_run_clocks();
    __DSB();
    __WFI();
    __ISB();
  }
}

/* Bare handle. The FSBL initialises the RTC, and HAL_RTC_Init would disturb
 * its double-tap reset timestamp. */
static RTC_HandleTypeDef s_hrtc;

/* RTC wakeup interrupt. The app is CPU-secure, so it arrives on RTC_S_IRQn,
 * or on RTC_IRQn when the RIF routes it non-secure. */
static void ps_rtc_irq_clear(void)
{
  WRITE_REG(RTC->SCR, RTC_SCR_CWUTF);
  NVIC_ClearPendingIRQ(RTC_S_IRQn);
  NVIC_ClearPendingIRQ(RTC_IRQn);
}
extern "C" void RTC_S_IRQHandler(void) { ps_rtc_irq_clear(); }
extern "C" void RTC_IRQHandler(void)   { ps_rtc_irq_clear(); }

/* RTC preparation */
void NeuroN6PowerSave::rtcPrepare()
{
  if (_rtcReady) return;

  __HAL_RCC_RTC_CLK_ENABLE();
  __HAL_RCC_RTCAPB_CLK_ENABLE();
  /* The RTC wakes the CPU from Stop only with its two PERxLPEN bits set. */
  __HAL_RCC_RTC_CLK_SLEEP_ENABLE();
  __HAL_RCC_RTCAPB_CLK_SLEEP_ENABLE();
  HAL_PWR_EnableBkUpAccess();

  s_hrtc.Instance = RTC;
  s_hrtc.State    = HAL_RTC_STATE_READY;
  s_hrtc.Lock     = HAL_UNLOCKED;
  __HAL_RTC_WRITEPROTECTION_DISABLE(&s_hrtc);

  uint32_t rtcsel = READ_BIT(RCC->CCIPR7, RCC_CCIPR7_RTCSEL);
  if (rtcsel == RCC_CCIPR7_RTCSEL_0)      _rtcClkHz = 32768u;  /* LSE */
  else if (rtcsel == RCC_CCIPR7_RTCSEL_1) _rtcClkHz = 32000u;  /* LSI */
  else                                    _rtcClkHz = 32000u;  /* HSE or off */

  HAL_NVIC_SetPriority(RTC_S_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(RTC_S_IRQn);
  HAL_NVIC_SetPriority(RTC_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(RTC_IRQn);

  /* The calendar runs at 1 Hz, so (PREDIV_A + 1) x (PREDIV_S + 1) is the RTC
   * clock in Hz. BYPSHAD makes SSR readable directly. */
  {
    uint32_t prer = READ_REG(RTC->PRER);
    uint32_t a = (prer & RTC_PRER_PREDIV_A_Msk) >> RTC_PRER_PREDIV_A_Pos;
    uint32_t s = (prer & RTC_PRER_PREDIV_S_Msk) >> RTC_PRER_PREDIV_S_Pos;
    uint32_t hz = (a + 1u) * (s + 1u);
    if (hz > 10000u && hz < 50000u) { _rtcClkHz = hz; }
    SET_BIT(RTC->CR, RTC_CR_BYPSHAD);
  }

  /* The RTC wakeup leaves Stop through EXTI line 17, secure, or 18,
   * non-secure. */
  SET_BIT(EXTI->IMR1, (1UL << 17) | (1UL << 18));

  _rtcReady = true;
}

void NeuroN6PowerSave::begin()
{
  if (_begun) return;

  NeuroN6_SetRunLedEnabled(false);

  /* The DBGMCU bits survive a reset and keep the debug domain clocked in
   * Sleep, Stop and Standby. */
  CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_SLEEP | DBGMCU_CR_DBG_STOP | DBGMCU_CR_DBG_STANDBY);
  HAL_PWREx_DisableMonitoring();
  HAL_PWREx_DisableVDDCOREMonitoring();

  rtcPrepare();
  _begun = true;
}

/* The core's n6_usb task arms the IWDG, about 8 s on LSI. It keeps counting
 * in Stop and Standby, and only a reset stops it. */
void NeuroN6PowerSave::iwdgFeed()
{
  IWDG->KR = 0xAAAAu;
}

/* Clock profiles */

/* PLL parameters, the same as the FSBL's. */
static void ps_pll1_params(RCC_OscInitTypeDef &o)
{
  o.PLL1.PLLSource = RCC_PLLSOURCE_HSI;  /* 64/2*25 = 800 MHz */
  o.PLL1.PLLM = 2;  o.PLL1.PLLN = 25;  o.PLL1.PLLFractional = 0;
  o.PLL1.PLLP1 = 1; o.PLL1.PLLP2 = 1;
}
static void ps_pll2_params(RCC_OscInitTypeDef &o)
{
  o.PLL2.PLLSource = RCC_PLLSOURCE_HSI;  /* 64/8*125 = 1000 MHz */
  o.PLL2.PLLM = 8;  o.PLL2.PLLN = 125; o.PLL2.PLLFractional = 0;
  o.PLL2.PLLP1 = 1; o.PLL2.PLLP2 = 1;
}
static void ps_pll3_params(RCC_OscInitTypeDef &o)
{
  o.PLL3.PLLSource = RCC_PLLSOURCE_HSI;  /* 64/8*225 /2 = 900 MHz */
  o.PLL3.PLLM = 8;  o.PLL3.PLLN = 225; o.PLL3.PLLFractional = 0;
  o.PLL3.PLLP1 = 1; o.PLL3.PLLP2 = 2;
}
static void ps_pll4_params(RCC_OscInitTypeDef &o)
{
  o.PLL4.PLLSource = RCC_PLLSOURCE_HSE;  /* 48/6*200 /4 = 400 MHz */
  o.PLL4.PLLM = 6;  o.PLL4.PLLN = 200; o.PLL4.PLLFractional = 0;
  o.PLL4.PLLP1 = 4; o.PLL4.PLLP2 = 1;
}

/* Order: CPU to HSI, enable the PLLs the target needs, switch the tree, then
 * disable the unused PLLs. A running PLL is not re-enabled: the HAL cycles
 * it. */
bool NeuroN6PowerSave::applyClockTree(PsClockProfile p, uint32_t cpuDiv)
{
  const bool wantPll1 = (p != PS_CLOCK_ECO);
  const bool wantNpu  = (p == PS_CLOCK_FULL) && !_npuOff;

  {
    RCC_ClkInitTypeDef ck = {};
    ck.ClockType    = RCC_CLOCKTYPE_CPUCLK;
    ck.CPUCLKSource = RCC_CPUCLKSOURCE_HSI;
    if (HAL_RCC_ClockConfig(&ck) != HAL_OK) return false;
  }

  {
    RCC_OscInitTypeDef osc = {};
    osc.OscillatorType = RCC_OSCILLATORTYPE_NONE;
    osc.PLL1.PLLState = RCC_PLL_NONE;
    osc.PLL2.PLLState = RCC_PLL_NONE;
    osc.PLL3.PLLState = RCC_PLL_NONE;
    osc.PLL4.PLLState = RCC_PLL_NONE;

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) == 0U) {
      osc.OscillatorType |= RCC_OSCILLATORTYPE_HSE;   /* PLL4 source */
      osc.HSEState = RCC_HSE_ON;
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PLL4RDY) == 0U) {
      osc.PLL4.PLLState = RCC_PLL_ON;  ps_pll4_params(osc);
    }
    if (wantPll1 && (__HAL_RCC_GET_FLAG(RCC_FLAG_PLL1RDY) == 0U)) {
      osc.PLL1.PLLState = RCC_PLL_ON;  ps_pll1_params(osc);
    }
    if (wantNpu && (__HAL_RCC_GET_FLAG(RCC_FLAG_PLL2RDY) == 0U)) {
      osc.PLL2.PLLState = RCC_PLL_ON;  ps_pll2_params(osc);
    }
    if (wantNpu && (__HAL_RCC_GET_FLAG(RCC_FLAG_PLL3RDY) == 0U)) {
      osc.PLL3.PLLState = RCC_PLL_ON;  ps_pll3_params(osc);
    }
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) return false;
  }

  {
    RCC_ClkInitTypeDef ck = {};
    ck.ClockType = RCC_CLOCKTYPE_CPUCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                   RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                   RCC_CLOCKTYPE_PCLK5 | RCC_CLOCKTYPE_PCLK4;

    if (wantPll1) {
      ck.CPUCLKSource = RCC_CPUCLKSOURCE_IC1;
      ck.IC1Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
      ck.IC1Selection.ClockDivider   = cpuDiv;               /* 800/div MHz */
    } else {
      ck.CPUCLKSource = RCC_CPUCLKSOURCE_HSI;                /* ECO: 64 MHz */
      ck.IC1Selection.ClockSelection = RCC_ICCLKSOURCE_PLL4; /* parked on PLL4 */
      ck.IC1Selection.ClockDivider   = 1;
    }

    ck.SYSCLKSource = RCC_SYSCLKSOURCE_IC2_IC6_IC11;
    /* AXI stays on PLL4 at 400 MHz in every profile. */
    ck.IC2Selection.ClockSelection  = RCC_ICCLKSOURCE_PLL4;
    ck.IC2Selection.ClockDivider    = 1;
    if (wantNpu) {
      ck.IC6Selection.ClockSelection  = RCC_ICCLKSOURCE_PLL2;   /* 1000 MHz */
      ck.IC6Selection.ClockDivider    = 1;
      ck.IC11Selection.ClockSelection = RCC_ICCLKSOURCE_PLL3;   /* 900 MHz */
      ck.IC11Selection.ClockDivider   = 1;
    } else {
      /* IC6 and IC11 move to PLL4/4, 100 MHz, before PLL2 and PLL3 stop. */
      ck.IC6Selection.ClockSelection  = RCC_ICCLKSOURCE_PLL4;
      ck.IC6Selection.ClockDivider    = 4;
      ck.IC11Selection.ClockSelection = RCC_ICCLKSOURCE_PLL4;
      ck.IC11Selection.ClockDivider   = 4;
    }

    ck.AHBCLKDivider  = RCC_HCLK_DIV2;   /* HCLK 200 MHz */
    ck.APB1CLKDivider = RCC_APB1_DIV1;
    ck.APB2CLKDivider = RCC_APB2_DIV1;
    ck.APB4CLKDivider = RCC_APB4_DIV1;
    ck.APB5CLKDivider = RCC_APB5_DIV1;
    if (HAL_RCC_ClockConfig(&ck) != HAL_OK) return false;
  }

  {
    RCC_OscInitTypeDef osc = {};
    osc.OscillatorType = RCC_OSCILLATORTYPE_NONE;
    osc.PLL1.PLLState = RCC_PLL_NONE;
    osc.PLL2.PLLState = RCC_PLL_NONE;
    osc.PLL3.PLLState = RCC_PLL_NONE;
    osc.PLL4.PLLState = RCC_PLL_NONE;

    bool any = false;
    if (!wantPll1 && (__HAL_RCC_GET_FLAG(RCC_FLAG_PLL1RDY) != 0U)) {
      osc.PLL1.PLLState = RCC_PLL_OFF; any = true;
    }
    if (!wantNpu && (__HAL_RCC_GET_FLAG(RCC_FLAG_PLL2RDY) != 0U)) {
      osc.PLL2.PLLState = RCC_PLL_OFF; any = true;
    }
    if (!wantNpu && (__HAL_RCC_GET_FLAG(RCC_FLAG_PLL3RDY) != 0U)) {
      osc.PLL3.PLLState = RCC_PLL_OFF; any = true;
    }
    if (any) { (void)HAL_RCC_OscConfig(&osc); }
  }

  /* millis() is the FreeRTOS tick, and SysTick counts the CPU clock:
   * HAL_RCC_ClockConfig has set SystemCoreClock, so the reload follows it. */
  if ((SysTick->CTRL & SysTick_CTRL_ENABLE_Msk) != 0u) {
    taskENTER_CRITICAL();
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    SysTick->LOAD = (SystemCoreClock / configTICK_RATE_HZ) - 1u;
    SysTick->VAL  = 0u;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
    taskEXIT_CRITICAL();
  }

  return true;
}

bool NeuroN6PowerSave::setClockProfile(PsClockProfile p)
{
  uint32_t div = 1;
  switch (p) {
    case PS_CLOCK_FULL:   div = 1; break;
    case PS_CLOCK_CRUISE: div = 2; break;
    case PS_CLOCK_LOW:    div = 8; break;
    case PS_CLOCK_ECO:    div = 1; break;   /* unused: CPU on HSI */
  }
  if (!applyClockTree(p, div)) return false;
  _profile = p;
  _cpuDiv  = div;
  return true;
}

bool NeuroN6PowerSave::setCpuDivider(uint32_t div)
{
  if (div < 1u) div = 1u;
  if (div > 256u) div = 256u;
  PsClockProfile p = (_profile == PS_CLOCK_ECO) ? PS_CLOCK_LOW : _profile;
  if (!applyClockTree(p, div)) return false;
  _profile = p;
  _cpuDiv  = div;
  return true;
}

uint32_t NeuroN6PowerSave::cpuClockHz() const
{
  return SystemCoreClock;
}

void NeuroN6PowerSave::npuRamOff()
{
  /* SRAMSD powers the array off. */
  __HAL_RCC_RAMCFG_CLK_ENABLE();
  SET_BIT(RAMCFG_SRAM3_AXI->CR, RAMCFG_CR_SRAMSD);
  SET_BIT(RAMCFG_SRAM4_AXI->CR, RAMCFG_CR_SRAMSD);
  SET_BIT(RAMCFG_SRAM5_AXI->CR, RAMCFG_CR_SRAMSD);
  SET_BIT(RAMCFG_SRAM6_AXI->CR, RAMCFG_CR_SRAMSD);
}

void NeuroN6PowerSave::npuOff()
{
  _npuOff = true;
  (void)applyClockTree(_profile, _cpuDiv);
  npuRamOff();
}

bool NeuroN6PowerSave::psramSleep()
{
  XSPI_TypeDef *x = XSPI2;

  /* Leaves memory-mapped mode. */
  uint32_t guard = 1000000u;
  SET_BIT(x->CR, XSPI_CR_ABORT);
  while ((READ_BIT(x->CR, XSPI_CR_ABORT) != 0u) && --guard) { }
  if (guard == 0u) return false;
  guard = 1000000u;
  while ((READ_BIT(x->SR, XSPI_SR_BUSY) != 0u) && --guard) { }
  if (guard == 0u) return false;
  WRITE_REG(x->FCR, XSPI_FCR_CTEF | XSPI_FCR_CTCF | XSPI_FCR_CSMF | XSPI_FCR_CTOF);
  MODIFY_REG(x->CR, XSPI_CR_FMODE, 0u);           /* indirect write mode */

  /* Bare handle: DCR and the timing registers hold the FSBL's configuration. */
  XSPI_HandleTypeDef h = {};
  h.Instance = x;
  h.State    = HAL_XSPI_STATE_READY;

  XSPI_RegularCmdTypeDef cmd = {};
  uint8_t tx[2] = { 0xC0u, 0xC0u };  /* APS51208N MR6 = 0xC0: Deep Power Down */
  cmd.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
  cmd.InstructionMode    = HAL_XSPI_INSTRUCTION_8_LINES;
  cmd.InstructionWidth   = HAL_XSPI_INSTRUCTION_8_BITS;
  cmd.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
  cmd.Instruction        = 0xC0u;                 /* Mode Register Write */
  cmd.AddressMode        = HAL_XSPI_ADDRESS_8_LINES;
  cmd.AddressWidth       = HAL_XSPI_ADDRESS_32_BITS;
  cmd.AddressDTRMode     = HAL_XSPI_ADDRESS_DTR_ENABLE;
  cmd.Address            = 0x00000006u;           /* MR6 */
  cmd.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  cmd.DataMode           = HAL_XSPI_DATA_8_LINES;
  cmd.DataDTRMode        = HAL_XSPI_DATA_DTR_ENABLE;
  cmd.DataLength         = 2;
  cmd.DummyCycles        = 0;                     /* register writes: latency 1 */
  cmd.DQSMode            = HAL_XSPI_DQS_DISABLE;
#if defined(XSPI_CCR_SIOO)
  cmd.SIOOMode           = HAL_XSPI_SIOO_INST_EVERY_CMD;
#endif

  bool ok = (HAL_XSPI_Command(&h, &cmd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK) &&
            (HAL_XSPI_Transmit(&h, tx, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK);

  /* NCS is released before the controller goes off. The carrier's pull-up
   * R223 then holds RAM_CS idle. */
  guard = 1000000u;
  SET_BIT(x->CR, XSPI_CR_ABORT);
  while ((READ_BIT(x->CR, XSPI_CR_ABORT) != 0u) && --guard) { }
  CLEAR_BIT(x->CR, XSPI_CR_EN);
  (void)READ_REG(x->CR);   /* the write retires before the clock stops */
  __DSB();
  __HAL_RCC_XSPI2_CLK_DISABLE();

  return ok;
}

bool NeuroN6PowerSave::norSleep()
{
  /* MX25UM octal DTR opcodes are 16 bits, the command and its complement:
   * 0xB9 becomes 0xB946. */
  XSPI_TypeDef *x = XSPI1;

  uint32_t guard = 1000000u;
  SET_BIT(x->CR, XSPI_CR_ABORT);
  while ((READ_BIT(x->CR, XSPI_CR_ABORT) != 0u) && --guard) { }
  if (guard == 0u) return false;
  guard = 1000000u;
  while ((READ_BIT(x->SR, XSPI_SR_BUSY) != 0u) && --guard) { }
  if (guard == 0u) return false;
  WRITE_REG(x->FCR, XSPI_FCR_CTEF | XSPI_FCR_CTCF | XSPI_FCR_CSMF | XSPI_FCR_CTOF);
  MODIFY_REG(x->CR, XSPI_CR_FMODE, 0u);

  XSPI_HandleTypeDef h = {};
  h.Instance = x;
  h.State    = HAL_XSPI_STATE_READY;

  XSPI_RegularCmdTypeDef cmd = {};
  cmd.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
  cmd.Instruction        = 0xB946u;               /* Deep Power Down */
  cmd.InstructionMode    = HAL_XSPI_INSTRUCTION_8_LINES;
  cmd.InstructionWidth   = HAL_XSPI_INSTRUCTION_16_BITS;
  cmd.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_ENABLE;
  cmd.AddressMode        = HAL_XSPI_ADDRESS_NONE;
  cmd.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  cmd.DataMode           = HAL_XSPI_DATA_NONE;
  cmd.DummyCycles        = 0;
  cmd.DQSMode            = HAL_XSPI_DQS_DISABLE;
#if defined(XSPI_CCR_SIOO)
  cmd.SIOOMode           = HAL_XSPI_SIOO_INST_EVERY_CMD;
#endif
  bool ok = (HAL_XSPI_Command(&h, &cmd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK);

  /* The rising NCS edge latches deep power down. The controller stays
   * enabled, unlike psramSleep, so that NCS is driven high. */
  guard = 1000000u;
  SET_BIT(x->CR, XSPI_CR_ABORT);
  while ((READ_BIT(x->CR, XSPI_CR_ABORT) != 0u) && --guard) { }

  return ok;
}

/* External watchdog */
void NeuroN6PowerSave::externalWatchdogPause() { _wdPaused = true; }

void NeuroN6PowerSave::externalWatchdogHeartbeatTimeout(uint32_t timeoutMs)
{
  _wdBeatTo = timeoutMs;
  _wdBeatMs = rtcTimeMs();
}

void NeuroN6PowerSave::externalWatchdogHeartbeat() { _wdBeatMs = rtcTimeMs(); }

bool NeuroN6PowerSave::wdHeartbeatLost()
{
  if (_wdBeatTo == 0u) return false;
  return ((rtcTimeMs() - _wdBeatMs + 86400000u) % 86400000u) > _wdBeatTo;
}

void NeuroN6PowerSave::externalWatchdogKick()
{
  if (_wdPort == nullptr || _wdPaused || wdHeartbeatLost()) return;

  /* One falling edge per call, held high for the 50 ns WDI needs. sleepStop
   * calls this with interrupts masked and the CPU on HSI, so the pulse is a
   * NOP loop. */
  _wdPort->BSRR = (uint32_t)_wdMask;
  for (volatile uint32_t i = 0; i < 200u; i++) { __NOP(); }
  _wdPort->BSRR = (uint32_t)_wdMask << 16;

  _wdLastMs = rtcTimeMs();
}

void NeuroN6PowerSave::externalWatchdogArmPin(uint32_t pin)
{
  const PinDescription *d = pin_desc(pin);
  if (d == nullptr || d->port == nullptr) return;

  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);        /* SET1 = 0, SET0 = 1: watchdog disabled */

  _wdArmPort = d->port;
  _wdArmMask = d->mask;
  _wdArmed   = false;
}

void NeuroN6PowerSave::externalWatchdogArm()
{
  if (_wdArmPort == nullptr || _wdArmed) return;

  _wdArmPort->BSRR = (uint32_t)_wdArmMask;   /* SET1 = 1: watchdog enabled */

  /* The device ignores WDI for 150 microseconds after enabling. */
  const uint32_t t0 = rtcTimeMs();
  while (((rtcTimeMs() - t0 + 86400000u) % 86400000u) < 2u) { }

  /* The first pulse after an enable has no lower bound. */
  externalWatchdogKick();
  _wdArmed = true;
}

void NeuroN6PowerSave::externalWatchdogService()
{
  if (_wdPort == nullptr || _wdPaused || wdHeartbeatLost()) return;

  /* An early pulse is a fault on a window watchdog. */
  const uint32_t now = rtcTimeMs();
  if (((now - _wdLastMs + 86400000u) % 86400000u) < _wdKickMs) return;
  externalWatchdogKick();
}

/* n6_extwdg task: services the watchdog while the board is awake. */
static void ps_wd_task(void *arg)
{
  NeuroN6PowerSave *ps = static_cast<NeuroN6PowerSave *>(arg);
  for (;;) {
    ps->externalWatchdogService();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void NeuroN6PowerSave::externalWatchdogBegin(uint32_t pin, uint32_t windowLoMs,
                                             uint32_t windowHiMs, uint32_t targetMs)
{
  const PinDescription *d = pin_desc(pin);
  if (d == nullptr || d->port == nullptr) return;
  if (windowHiMs <= windowLoMs || windowLoMs < 5u) return;

  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);        /* idles low */

  _wdPort   = d->port;
  _wdMask   = d->mask;
  /* The target is clamped for an LSI of 28.8 to 33.6 kHz against the 32 kHz
   * the calendar assumes. */
  _wdKickMs = (targetMs != 0u) ? targetMs : ((windowLoMs + windowHiMs) / 2u);
  {
    const uint32_t loSafe = (windowLoMs * 33600u) / 32000u;   /* fast LSI */
    const uint32_t hiSafe = (windowHiMs * 28800u) / 32000u;   /* slow LSI */
    if (_wdKickMs < loSafe) _wdKickMs = loSafe;
    if (_wdKickMs > hiSafe) _wdKickMs = hiSafe;
  }

  rtcPrepare();
  externalWatchdogKick();        /* the first edge starts the window */

  (void)xTaskCreate(ps_wd_task, "n6_extwdg", 384u, this,
                    configMAX_PRIORITIES - 3, nullptr);
}

uint32_t NeuroN6PowerSave::gpioFloatingToAnalog()
{
  /* MODER: 00 input, 01 output, 10 alternate function, 11 analog. */
  __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE(); __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE(); __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE(); __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPION_CLK_ENABLE(); __HAL_RCC_GPIOO_CLK_ENABLE();
  __HAL_RCC_GPIOP_CLK_ENABLE(); __HAL_RCC_GPIOQ_CLK_ENABLE();

  GPIO_TypeDef *const ports[] = { GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF,
                                  GPIOG, GPIOH, GPION, GPIOO, GPIOP, GPIOQ };
  uint32_t converted = 0;

  for (uint32_t p = 0; p < (sizeof(ports) / sizeof(ports[0])); p++) {
    GPIO_TypeDef *g = ports[p];
    uint32_t moder = READ_REG(g->MODER);
    uint32_t pupdr = READ_REG(g->PUPDR);

    for (uint32_t pin = 0; pin < 16u; pin++) {
      const uint32_t shift = pin * 2u;
      if (((moder >> shift) & 0x3u) != 0x0u) continue;   /* not an input */
      moder |= (0x3u << shift);                          /* analog */
      pupdr &= ~(0x3u << shift);                         /* no pull */
      converted++;
    }

    /* PUPDR first: a pin must not be analog with a pull. */
    WRITE_REG(g->PUPDR, pupdr);
    WRITE_REG(g->MODER, moder);
  }
  return converted;
}

uint32_t NeuroN6PowerSave::rtcTimeMs()
{
  rtcPrepare();   /* sets BYPSHAD */

  uint32_t prer = READ_REG(RTC->PRER);
  uint32_t S = (prer & RTC_PRER_PREDIV_S_Msk) >> RTC_PRER_PREDIV_S_Pos;

  /* Direct reads: SSR, TR, SSR until the two SSR values match. */
  uint32_t ssr, tr, ssr2;
  do {
    ssr  = READ_REG(RTC->SSR) & 0xFFFFu;
    tr   = READ_REG(RTC->TR);
    ssr2 = READ_REG(RTC->SSR) & 0xFFFFu;
  } while (ssr != ssr2);

  uint32_t sec = (tr & 0xFu) + 10u * ((tr >> 4) & 0x7u);
  uint32_t min = ((tr >> 8) & 0xFu) + 10u * ((tr >> 12) & 0x7u);
  uint32_t hr  = ((tr >> 16) & 0xFu) + 10u * ((tr >> 20) & 0x3u);
  uint32_t secOfDay = hr * 3600u + min * 60u + sec;

  uint32_t msInSec = 0;
  if (S > 0u && ssr <= S) {
    msInSec = ((S - ssr) * 1000u) / (S + 1u);   /* SSR counts down */
  }
  return secOfDay * 1000u + msInSec;
}

/* Peripheral gating */
void NeuroN6PowerSave::periphOff(uint32_t mask)
{
  /* Each running peripheral is read back first, so its last write retires
   * before the clock stops. */
  if ((mask & PS_PERIPH_DCMIPP) && __HAL_RCC_DCMIPP_IS_CLK_ENABLED()) {
    (void)READ_REG(DCMIPP->CMCR);
    __DSB();
    __HAL_RCC_DCMIPP_CLK_DISABLE();
  }
  if ((mask & PS_PERIPH_CSI) && __HAL_RCC_CSI_IS_CLK_ENABLED()) {
    (void)READ_REG(CSI->PRCR);
    __DSB();
    __HAL_RCC_CSI_CLK_DISABLE();
  }
  if ((mask & PS_PERIPH_I2C1) && __HAL_RCC_I2C1_IS_CLK_ENABLED()) {
    (void)READ_REG(I2C1->CR1);
    __DSB();
    __HAL_RCC_I2C1_CLK_DISABLE();
  }
  if ((mask & PS_PERIPH_MDF1) && __HAL_RCC_MDF1_IS_CLK_ENABLED()) {
    (void)READ_REG(MDF1->CKGCR);
    __DSB();
    __HAL_RCC_MDF1_CLK_DISABLE();
  }
}

void NeuroN6PowerSave::periphOn(uint32_t mask)
{
  if (mask & PS_PERIPH_CSI)    __HAL_RCC_CSI_CLK_ENABLE();
  if (mask & PS_PERIPH_DCMIPP) __HAL_RCC_DCMIPP_CLK_ENABLE();
  if (mask & PS_PERIPH_I2C1)   __HAL_RCC_I2C1_CLK_ENABLE();
  if (mask & PS_PERIPH_MDF1)   __HAL_RCC_MDF1_CLK_ENABLE();
}

void NeuroN6PowerSave::usbOff()
{
  /* The OTG AHB clock stays on: the n6_usb task goes on polling TinyUSB. */
  (void)tud_disconnect();
  HAL_Delay(2);
  HAL_PWREx_DisableVddUSB();
  HAL_PWREx_DisableVddUSBVMEN();
}

/* Board sensors */
static void ps_i2c_write8(uint8_t addr8, uint8_t reg, uint8_t val)
{
  n6_i2c_lock();
  (void)HAL_I2C_Mem_Write(&hi2c1, addr8, reg, I2C_MEMADD_SIZE_8BIT, &val, 1u, 25u);
  n6_i2c_unlock();
}

void NeuroN6PowerSave::imuSleep()
{
  /* LSM6DSL: CTRL1_XL, 0x10, and CTRL2_G, 0x11, set to 0 power both blocks
   * down. Both SA0 addresses are written. */
  for (uint8_t a7 = 0x6Au; a7 <= 0x6Bu; a7++) {
    ps_i2c_write8((uint8_t)(a7 << 1), 0x10u, 0x00u);
    ps_i2c_write8((uint8_t)(a7 << 1), 0x11u, 0x00u);
  }
}

void NeuroN6PowerSave::magSleep()
{
  /* MMC5603NJ: ODR, 0x1A, and Control 2, 0x1D, set to 0 clear continuous
   * mode. */
  ps_i2c_write8(0x60u /*0x30<<1*/, 0x1Au, 0x00u);
  ps_i2c_write8(0x60u,             0x1Du, 0x00u);
}

void NeuroN6PowerSave::micSleep()
{
  /* The MP34DT05 sleeps without a clock. PE2, MDF1_CCK0, is driven low and
   * the data pin PB2 is set to analog. */
  periphOff(PS_PERIPH_MDF1);

  GPIO_InitTypeDef g = {};
  __HAL_RCC_GPIOE_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);
  g.Pin = GPIO_PIN_2; g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &g);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);

  __HAL_RCC_GPIOB_CLK_ENABLE();
  g.Pin = GPIO_PIN_2; g.Mode = GPIO_MODE_ANALOG; g.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &g);
}

void NeuroN6PowerSave::sensorsSleep()
{
  imuSleep();
  magSleep();
  micSleep();
}

/* FLIR Lepton: PWR_DWN_L = PD3, RESET_L = PD9. */
void NeuroN6PowerSave::thermalPower(bool on)
{
  GPIO_InitTypeDef g = {};
  __HAL_RCC_GPIOD_CLK_ENABLE();
  g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW;
  g.Pin = GPIO_PIN_3 | GPIO_PIN_9;
  HAL_GPIO_Init(GPIOD, &g);

  if (on) {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);  /* hold reset */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);    /* power on */
    HAL_Delay(5);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);    /* release reset */
  } else {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);  /* assert reset */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);  /* power down */
  }
}

/* Sleep modes */
/* Stop progress marker in backup register DR4, which survives an IWDG reset:
 *   0xC1 chunk-loop top     0xC2 WUT armed       0xC3 woke from WFI
 *   0xC4 WUT deactivated    0xC5 clocks restored 0xC6 ticks credited
 *   0xC0 sleepStop returned */
static void ps_crumb(uint32_t v)
{
  HAL_RTCEx_BKUPWrite(&s_hrtc, RTC_BKP_DR4, v);
}

uint32_t NeuroN6PowerSave::wutTicksForMs(uint32_t ms) const
{
  /* WUT clock = rtcclk / 16, 2000 Hz on LSI. */
  uint64_t t = ((uint64_t)ms * (_rtcClkHz / 16u)) / 1000u;
  if (t < 1u) t = 1u;
  if (t > 65535u) t = 65535u;
  return (uint32_t)t;
}

bool NeuroN6PowerSave::sleepStop(uint32_t ms)
{
  if (ms == 0u) return true;
  /* Stop loses a mounted USB connection until the next reset. */
  if (tud_mounted()) {
    sleepLight(ms);
    return false;
  }
  rtcPrepare();

  const bool schedulerUp =
      (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED);

  markerSet(PS_MARKER_AWAKE, false);
  if (schedulerUp) vTaskSuspendAll();

  (void)HAL_PWREx_ControlStopModeVoltageScaling(
      _svosLow ? PWR_REGULATOR_STOP_VOLTAGE_SCALE5
               : PWR_REGULATOR_STOP_VOLTAGE_SCALE3);

  const uint32_t savedDemcr = CoreDebug->DEMCR;
  CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;

  uint32_t remaining = ms;
  uint32_t slept     = 0;
  bool     full      = true;

  uint32_t chunkCap = _chunkMs;
  if (_wdPort != nullptr && _wdKickMs < chunkCap) chunkCap = _wdKickMs;

  while (remaining > 0u) {
    uint32_t chunk = (remaining > chunkCap) ? chunkCap : remaining;

    /* The chunk ends when the watchdog pulse is next due. */
    if (_wdPort != nullptr) {
      const uint32_t since =
          (rtcTimeMs() - _wdLastMs + 86400000u) % 86400000u;
      uint32_t until = (_wdKickMs > since) ? (_wdKickMs - since) : 0u;
      if (until < 50u) {
        externalWatchdogKick();
        until = _wdKickMs;
      }
      if (until < chunk) chunk = until;
    }

    /* A pending interrupt makes WFI return at once, with no Stop. The USB OTG
     * IRQ does this while a host is attached. */
    bool rtcWake = false;
    for (int attempt = 0; attempt < 3 && !rtcWake; attempt++) {
      __disable_irq();
      ps_crumb(0xC1u);

      (void)HAL_RTCEx_SetWakeUpTimer_IT(&s_hrtc, wutTicksForMs(chunk) - 1u,
                                        RTC_WAKEUPCLOCK_RTCCLK_DIV16, 0u);
      SET_BIT(PWR->CPUCR, PWR_CPUCR_CSSF); /* clears the Stop and Standby flags */
      ps_crumb(0xC2u);
      iwdgFeed();

      HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);

      /* Awake on HSI with PRIMASK set: the wake IRQ is pending, not taken. */
      iwdgFeed();
      /* service(), not kick(): an early edge is a fault. */
      externalWatchdogService();
      ps_crumb(0xC3u);
      rtcWake = (READ_REG(RTC->SR) & RTC_SR_WUTF) != 0u;

      (void)HAL_RTCEx_DeactivateWakeUpTimer(&s_hrtc);
      ps_rtc_irq_clear();
      ps_crumb(0xC4u);

      __enable_irq();
      /* Pended ISRs run here. */
      __ISB();

      if (rtcWake && (remaining - chunk) == 0u) {
        /* HAL timeouts cannot fire here: HAL_GetTick() is the suspended
         * FreeRTOS tick. */
        (void)applyClockTree(_profile, _cpuDiv);
        ps_crumb(0xC5u);
      }
    }

    if (!rtcWake) {
      (void)applyClockTree(_profile, _cpuDiv);
      full = false;
      break;
    }

    slept     += chunk;
    remaining -= chunk;
  }

  iwdgFeed();

  /* delayMicroseconds() reads the DWT cycle counter. */
  CoreDebug->DEMCR = savedDemcr;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  uwTick += slept;
  if (schedulerUp) {
    (void)xTaskResumeAll();
    /* In 64 bits: pdMS_TO_TICKS() wraps past 4,294,967 ms. */
    (void)xTaskCatchUpTicks((TickType_t)(((uint64_t)slept * configTICK_RATE_HZ) / 1000u));
  }
  ps_crumb(0xC6u);

  markerSet(PS_MARKER_AWAKE, true);
  ps_crumb(0xC0u);
  return full;
}

void NeuroN6PowerSave::sleepLight(uint32_t ms)
{
  ps_sleep_keeps_run_clocks();
  uint32_t t0 = millis();
  while ((millis() - t0) < ms) {
    __WFI();   /* wakes on every tick or interrupt */
  }
}

/* The longest standby(), in ms. The IWDG times it: 4096 ticks of LSI/1024 is
 * about 131 s. */
#define PS_STANDBY_MAX_MS     100000u
#define PS_LSI_HZ             32000u

/* Waits up to 60 ms, more than one period of the IWDG's slowest prescaler,
 * for its register updates. False if they do not finish. */
static bool ps_iwdg_idle(void)
{
  const uint32_t t0  = DWT->CYCCNT;
  const uint32_t lim = (SystemCoreClock / 1000u) * 60u;
  while ((IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU | IWDG_SR_WVU | IWDG_SR_EWU)) != 0u) {
    if ((DWT->CYCCNT - t0) > lim) return false;
  }
  return true;
}

/* Writes one IWDG register, then waits 1 ms (the update flags rise a few LSI
 * cycles after the write) and for the update to end. */
static bool ps_iwdg_write(volatile uint32_t *reg, uint32_t value)
{
  *reg = value;
  const uint32_t t0 = DWT->CYCCNT;
  while ((DWT->CYCCNT - t0) < (SystemCoreClock / 1000u)) { }
  return ps_iwdg_idle();
}

/* Sets the IWDG to reset the board after about ms milliseconds. The early
 * warning, one tick below the new reload value, rises within two ticks only
 * when the counter holds that value; a Standby entry before then keeps the
 * old one. True when it rose. */
static bool ps_iwdg_timer(uint32_t ms)
{
  uint32_t pr = 0u;                       /* LSI / (4 << pr) */
  uint64_t ticks = ((uint64_t)ms * PS_LSI_HZ) / 4000u;
  while (ticks > 4096u && pr < 8u) {
    pr++;
    ticks = ((uint64_t)ms * PS_LSI_HZ) / ((4000u) << pr);
  }
  if (ticks > 4096u) ticks = 4096u;
  if (ticks < 3u) ticks = 3u;
  const uint32_t rlr     = (uint32_t)ticks - 1u;
  const uint32_t tick_us = ((4u << pr) * 1000000u) / PS_LSI_HZ;

  if ((IWDG->SR & IWDG_SR_ONF) == 0u) IWDG->KR = 0xCCCCu;   /* also starts LSI */
  IWDG->KR = 0x5555u;
  if (!ps_iwdg_idle()) return false;
  if (!ps_iwdg_write(&IWDG->PR, pr)) return false;
  if (!ps_iwdg_write(&IWDG->RLR, rlr)) return false;
  if (!ps_iwdg_write(&IWDG->EWCR, IWDG_EWCR_EWIE | (rlr - 1u))) return false;
  IWDG->ICR = IWDG_ICR_EWIC;
  IWDG->KR  = 0xAAAAu;

  const uint32_t t0  = DWT->CYCCNT;
  const uint32_t lim = (SystemCoreClock / 1000000u) * (4u * tick_us + 10000u);
  while ((IWDG->SR & IWDG_SR_EWIF) == 0u) {
    if ((DWT->CYCCNT - t0) > lim) return false;
  }
  IWDG->ICR = IWDG_ICR_EWIC;
  return ((IWDG->PR & IWDG_PR_PR) == pr) && ((IWDG->RLR & IWDG_RLR_RL) == rlr);
}

/* Sets the IWDG to end the Standby and enters it, with interrupts masked.
 * Anything that fails resets the board instead. */
static void ps_standby_enter(uint32_t ms)
{
  bool timed = false;
  for (int attempt = 0; attempt < 3 && !timed; attempt++) timed = ps_iwdg_timer(ms);
  if (!timed) NVIC_SystemReset();

  /* The CPU leaves Standby at INITSVTORCR, which SystemInit sets to this
   * image's table in AXISRAM, lost in Standby. From the boot ROM, the
   * register's reset value, the board boots afresh. */
  RCC->APB4ENSR2 = RCC_APB4ENSR2_SYSCFGENS;
  (void)RCC->APB4ENR2;
  SYSCFG->INITSVTORCR = BOOTROM_BASE_S;
  if (SYSCFG->INITSVTORCR != BOOTROM_BASE_S) NVIC_SystemReset();

  SET_BIT(PWR->CPUCR, PWR_CPUCR_CSSF);   /* clears the Stop and Standby flags */
  __HAL_RCC_CLEAR_RESET_FLAGS();         /* the next boot reports the wake */

  /* A pending interrupt makes WFI return at once, with no Standby. */
  SysTick->CTRL = 0u;
  SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;
  for (uint32_t i = 0u; i < (sizeof(NVIC->ICER) / sizeof(NVIC->ICER[0])); i++) {
    NVIC->ICER[i] = 0xFFFFFFFFu;
    NVIC->ICPR[i] = 0xFFFFFFFFu;
  }

  HAL_PWR_EnterSTANDBYMode();
  /* Reached only when Standby was not entered. */
  NVIC_SystemReset();
}

void NeuroN6PowerSave::standby(uint32_t ms)
{
  markerSet(PS_MARKER_AWAKE, false);
  /* A pending bootloader request would take the wake into DFU: go there now. */
  if (n6_boot_dfu_pending()) NVIC_SystemReset();
  /* The wake is a reset through the FSBL, which must not take it for the
   * second press of a double tap. */
  n6_boot_press_clear();
  if (ms > PS_STANDBY_MAX_MS) ms = PS_STANDBY_MAX_MS;
  __disable_irq();
  ps_standby_enter(ms);
}

/* Phase markers */
void NeuroN6PowerSave::markerConfig(PsMarker m, uint32_t arduinoPin)
{
  if ((int)m < PS_MARKER_COUNT) _markerPin[m] = arduinoPin;
}

void NeuroN6PowerSave::markersBegin()
{
  for (int i = 0; i < PS_MARKER_COUNT; i++) {
    pinMode(_markerPin[i], OUTPUT);
    digitalWrite(_markerPin[i], LOW);
  }
  _markersOn = true;
  digitalWrite(_markerPin[PS_MARKER_AWAKE], HIGH);
}

void NeuroN6PowerSave::markerSet(PsMarker m, bool level)
{
  if (!_markersOn || (int)m >= PS_MARKER_COUNT) return;
  digitalWrite(_markerPin[m], level ? HIGH : LOW);
}

void NeuroN6PowerSave::markerPulse(PsMarker m, uint32_t microseconds)
{
  if (!_markersOn || (int)m >= PS_MARKER_COUNT) return;
  digitalWrite(_markerPin[m], HIGH);
  delayMicroseconds(microseconds);
  digitalWrite(_markerPin[m], LOW);
}

/* The on-board LEDs are active-low. */
void NeuroN6PowerSave::ledBlink(uint32_t ledPin, uint32_t ms)
{
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);    /* on */
  delay(ms);
  digitalWrite(ledPin, HIGH);   /* off */
}


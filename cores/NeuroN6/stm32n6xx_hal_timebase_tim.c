/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32n6xx_hal_timebase_tim.c
  * @brief   HAL time base based on the hardware TIM.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "stm32n6xx_hal.h"
#include "stm32n6xx_hal_tim.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef        htim16;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  This function configures the TIM16 as a time base source.
  *         The time source is configured  to have 1ms time base with a dedicated
  *         Tick interrupt priority.
  * @note   This function is called  automatically at the beginning of program after
  *         reset by HAL_Init() or at any time when clock is configured, by HAL_RCC_ClockConfig().
  * @param  TickPriority: Tick interrupt priority.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  uint32_t              uwTimclock = 0;
  uint32_t              uwPrescalerValue = 0;

  /*Configure the TIM16 IRQ priority */
  HAL_NVIC_SetPriority(TIM16_IRQn, TickPriority ,0);
  /* Enable the TIM16 global Interrupt */
  HAL_NVIC_EnableIRQ(TIM16_IRQn);

  /* Enable TIM16 clock */
  __HAL_RCC_TIM16_CLK_ENABLE();

  /* TIM16 counts the timer kernel clock, the system bus clock divided by
     TIMPRE: 400 MHz on this board, twice PCLK2. 0 means unresolved. */
  uwTimclock = HAL_RCCEx_GetTIMGFreq();
  if (uwTimclock < 1000000U)
  {
    uwTimclock = 400000000U;
  }

  /* Compute the prescaler value to have TIM16 counter clock equal to 1MHz */
  uwPrescalerValue = (uint32_t) ((uwTimclock / 1000000U) - 1U);

  /* Initialize TIM16 */
  htim16.Instance = TIM16;

  /* Initialize TIMx peripheral as follow:
   * Period = [(TIM16CLK/1000) - 1]. to have a (1/1000) s time base.
   * Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
   * ClockDivision = 0
   * Counter direction = Up
   */
  htim16.Init.Period = (1000000U / 1000U) - 1U;
  htim16.Init.Prescaler = uwPrescalerValue;
  htim16.Init.ClockDivision = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;

  if(HAL_TIM_Base_Init(&htim16) == HAL_OK)
  {
    /* Start the TIM time Base generation in interrupt mode */
    return HAL_TIM_Base_Start_IT(&htim16);
  }

  /* Return function status */
  return HAL_ERROR;
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called when TIM16 interrupt took place, inside
  *         HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to
  *         increment a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM16) {
    HAL_IncTick();
  }
}

/**
  * @brief  Suspend Tick increment.
  * @note   Disable the tick increment by disabling TIM16 update interrupt.
  * @param  None
  * @retval None
  */
void HAL_SuspendTick(void)
{
  /* Disable TIM16 update Interrupt */
  __HAL_TIM_DISABLE_IT(&htim16, TIM_IT_UPDATE);
}

/**
  * @brief  Resume Tick increment.
  * @note   Enable the tick increment by Enabling TIM16 update interrupt.
  * @param  None
  * @retval None
  */
void HAL_ResumeTick(void)
{
  /* Enable TIM16 Update interrupt */
  __HAL_TIM_ENABLE_IT(&htim16, TIM_IT_UPDATE);
}

/* Overrides the weak HAL_GetTick: once the scheduler runs, the FreeRTOS tick,
 * 1 ms, is the time base, offset to carry on from uwTick. */
#include "FreeRTOS.h"
#include "task.h"

static uint32_t kernel_ticks(void)
{
  if (xPortIsInsideInterrupt() == pdTRUE) {
    return (uint32_t)xTaskGetTickCountFromISR();
  }
  return (uint32_t)xTaskGetTickCount();
}

uint32_t HAL_GetTick(void)
{
  /* uwTick at the scheduler's start, where the kernel tick starts from 0. */
  static volatile uint32_t s_base;
  static volatile uint8_t  s_base_set;

  if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
    return uwTick;
  }
  if (s_base_set == 0u) {
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (s_base_set == 0u) {
      /* Both have counted milliseconds since the scheduler started. */
      s_base     = uwTick - kernel_ticks();
      s_base_set = 1u;
    }
    __set_PRIMASK(primask);
  }
  return s_base + kernel_ticks();
}


/**
  ******************************************************************************
  * @file    stm32n6xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "main.h"
#include "stm32n6xx_it.h"
#include "tusb_config.h"
#include "tusb.h"

/* External peripheral handles */
extern PCD_HandleTypeDef    hpcd_USB_OTG_HS1;
extern DCMIPP_HandleTypeDef hdcmipp;
extern TIM_HandleTypeDef    htim16;

/* Streams an [ERRDUMP] over CDC and never returns. */
extern void NeuroN6_FaultEntry(uint32_t *frame, uint32_t exc_return, uint32_t tag);

/* Naked: LR still holds EXC_RETURN. r0 = MSP or PSP per EXC_RETURN bit 2,
 * r1 = EXC_RETURN, r2 = the fault tag. */
#define N6_FAULT_HANDLER(name, tagnum)                            \
  __attribute__((naked)) void name(void)                          \
  {                                                               \
    __asm volatile (                                              \
      "movs r2, #" #tagnum "          \n"                         \
      "tst  lr, #4                    \n"                         \
      "ite  eq                        \n"                         \
      "mrseq r0, msp                  \n"                         \
      "mrsne r0, psp                  \n"                         \
      "mov  r1, lr                    \n"                         \
      "ldr  r3, =NeuroN6_FaultEntry   \n"                         \
      "bx   r3                        \n"                         \
    );                                                            \
  }

N6_FAULT_HANDLER(HardFault_Handler,   1)
N6_FAULT_HANDLER(MemManage_Handler,   2)
N6_FAULT_HANDLER(BusFault_Handler,    3)
N6_FAULT_HANDLER(UsageFault_Handler,  4)
N6_FAULT_HANDLER(NMI_Handler,         5)
N6_FAULT_HANDLER(SecureFault_Handler, 6)

/* SVC_Handler, PendSV_Handler and SysTick_Handler are owned by FreeRTOS. */

void DebugMon_Handler(void) {}

/* STM32N6xx peripheral interrupt handlers. */

void TIM16_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim16);
}

void USB1_OTG_HS_IRQHandler(void)
{
  tud_int_handler(BOARD_DEVICE_RHPORT_NUM);
  /* Wakes n6_usb_task only when an event was queued: the dwc2 TX-FIFO-empty
   * interrupt fires once per packet and queues nothing. */
  if (tud_task_event_ready()) {
    extern void NeuroN6_UsbIrqKick(void);
    NeuroN6_UsbIrqKick();
  }
}

void DCMIPP_IRQHandler(void)
{
  HAL_DCMIPP_IRQHandler(&hdcmipp);
}

void CSI_IRQHandler(void)
{
  HAL_DCMIPP_CSI_IRQHandler(&hdcmipp);
}

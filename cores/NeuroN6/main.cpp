/**
  ******************************************************************************
  * @file    main.cpp
  * @brief   Arduino entry point for the Neuro N6 board.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * SPDX-License-Identifier: MIT
  ******************************************************************************
  */

#include <Arduino.h>
#include <NeuroN6.h>

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
}

void setup(void);
void loop(void);

/* Sketch task stack. FreeRTOS sizes stacks in StackType_t words. */
#define ARDUINO_TASK_STACK_BYTES  (16u * 1024u)
#define ARDUINO_TASK_STACK_WORDS  (ARDUINO_TASK_STACK_BYTES / sizeof(StackType_t))

#define ARDUINO_TASK_PRIORITY     (tskIDLE_PRIORITY + 2u)

#ifdef NEURON6_BOOTDIAG
extern "C" void n6_bootdiag_mark(int stage);
#endif

static void arduino_loop_task(void *arg)
{
  (void)arg;
#ifdef NEURON6_BOOTDIAG
  n6_bootdiag_mark(5);
#endif
  setup();
  for (;;) {
    NeuroN6_App();
    loop();
  }
}

/* Weak, empty by default. */
extern "C" void initVariant(void);

int main(void)
{
  /* Must run before the scheduler starts. */
  NeuroN6_Setup();

  initVariant();

  /* USB is served from the scheduler start, through setup(). */
  NeuroN6_UsbBootPumpStart();

  TaskHandle_t arduino_handle = NULL;
  BaseType_t ok = xTaskCreate(arduino_loop_task,
                              "ArduinoLoop",
                              ARDUINO_TASK_STACK_WORDS,
                              NULL,
                              ARDUINO_TASK_PRIORITY,
                              &arduino_handle);
  if (ok != pdPASS || arduino_handle == NULL) {
    for (;;) {
      HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
      for (volatile uint32_t i = 0; i < 800000; i++) {}
    }
  }

#ifdef NEURON6_BOOTDIAG
  n6_bootdiag_mark(4);
#endif

  vTaskStartScheduler();

  /* Reached only when the scheduler cannot start. */
  for (;;) {
    HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
    for (volatile uint32_t i = 0; i < 800000; i++) {}
  }
}

/* FreeRTOS application hooks */

extern "C" {

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  (void)pcTaskName;
  for (;;) {
    HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
    for (volatile uint32_t i = 0; i < 800000; i++) {}
  }
}

static StaticTask_t s_idle_tcb;
static StackType_t  s_idle_stack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t  **ppxIdleTaskStackBuffer,
                                   uint32_t      *pulIdleTaskStackSize)
{
  *ppxIdleTaskTCBBuffer   = &s_idle_tcb;
  *ppxIdleTaskStackBuffer = s_idle_stack;
  *pulIdleTaskStackSize   = configMINIMAL_STACK_SIZE;
}

#if (configUSE_TIMERS == 1)
static StaticTask_t s_timer_tcb;
static StackType_t  s_timer_stack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t  **ppxTimerTaskStackBuffer,
                                    uint32_t      *pulTimerTaskStackSize)
{
  *ppxTimerTaskTCBBuffer   = &s_timer_tcb;
  *ppxTimerTaskStackBuffer = s_timer_stack;
  *pulTimerTaskStackSize   = configTIMER_TASK_STACK_DEPTH;
}
#endif

} /* extern "C" */

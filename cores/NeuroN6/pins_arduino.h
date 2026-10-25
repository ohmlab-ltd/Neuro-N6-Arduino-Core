/**
 ******************************************************************************
 * @file    pins_arduino.h
 * @brief   Board pin map for the Neuro N6 carrier.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef PINS_ARDUINO_H
#define PINS_ARDUINO_H

#include "main.h"
#include "stm32n6xx_hal.h"

/* Arduino pin numbers, indices into g_APinDescription[]. D2-D4, D7 and D8 are
 * not routed. */
#define PIN_D0            0u   /* PD0, header RX  */
#define PIN_D1            1u   /* PD1, header TX  */
#define PIN_D5            5u   /* PE9  */
#define PIN_D6            6u   /* PF5  */
#define PIN_D9            9u   /* PD12 */
#define PIN_D10          10u   /* PF10 */
#define PIN_D11          11u   /* PF15 */
#define PIN_D12          12u   /* PA3  */
#define PIN_D13          13u   /* PG14 */
#define PIN_A0           14u   /* PF13 */
#define PIN_A1           15u   /* PF11 */
#define PIN_A2           16u   /* PF12 */
#define PIN_A3           17u   /* PF3  */
#define PIN_A4           18u   /* PF14 */
#define PIN_A5           19u   /* PG15 */
#define PIN_WIRE_SDA     20u   /* PD15, header SDA, 3V3 with 4.7k pull-ups, I2C2 = Wire1 */
#define PIN_WIRE_SCL     21u   /* PD14, header SCL, I2C2 = Wire1 */
/* The header I2C pins belong to Wire1. Wire is the on-board sensor bus, I2C1
 * on PE6/PH9. */
#define PIN_WIRE1_SDA    PIN_WIRE_SDA
#define PIN_WIRE1_SCL    PIN_WIRE_SCL
#define PIN_SPI_MOSI     22u   /* PD7  */
#define PIN_SPI_MISO     23u   /* PD11 */
#define PIN_SPI_SCK      24u   /* PF2  */
#define PIN_LEDR         25u   /* PE11, RGB LED red, active low */
#define PIN_LEDG         26u   /* PD6, RGB LED green, active low */
#define PIN_LEDB         27u   /* PC1, RGB LED blue, active low */
#define PIN_BTN_USER     28u   /* PG8, USER button, external 10k pulldown, reads high while pressed */

/* Not macros: CMSIS-DSP's PID structs have members named A0, A1 and A2. */
#ifdef __cplusplus
static const uint32_t A0 = PIN_A0;
static const uint32_t A1 = PIN_A1;
static const uint32_t A2 = PIN_A2;
static const uint32_t A3 = PIN_A3;
static const uint32_t A4 = PIN_A4;
static const uint32_t A5 = PIN_A5;

static const uint32_t D0  = PIN_D0;
static const uint32_t D1  = PIN_D1;
static const uint32_t D5  = PIN_D5;
static const uint32_t D6  = PIN_D6;
static const uint32_t D9  = PIN_D9;
static const uint32_t D10 = PIN_D10;
static const uint32_t D11 = PIN_D11;
static const uint32_t D12 = PIN_D12;
static const uint32_t D13 = PIN_D13;

static const uint32_t SDA  = PIN_WIRE_SDA;
static const uint32_t SCL  = PIN_WIRE_SCL;
static const uint32_t SDA1 = PIN_WIRE1_SDA;
static const uint32_t SCL1 = PIN_WIRE1_SCL;
static const uint32_t MOSI = PIN_SPI_MOSI;
static const uint32_t MISO = PIN_SPI_MISO;
static const uint32_t SCK  = PIN_SPI_SCK;
static const uint32_t SS   = PIN_D10;   /* default CS */
#endif /* __cplusplus */

#define LED_BUILTIN PIN_LEDG   /* a macro, for #ifdef LED_BUILTIN */
#ifdef __cplusplus
static const uint32_t LEDR = PIN_LEDR;
static const uint32_t LEDG = PIN_LEDG;
static const uint32_t LEDB = PIN_LEDB;
static const uint32_t BTN_USER = PIN_BTN_USER;
static const uint32_t USER_BTN = PIN_BTN_USER;
#endif /* __cplusplus */

#define PINS_COUNT           29u
#define NUM_DIGITAL_PINS     29u
#define NUM_ANALOG_INPUTS    6u

#define analogInputToDigitalPin(p) ((uint32_t)(p) < 6u ? (uint32_t)(p) + PIN_A0 : NOT_A_PIN_NUM)
#define digitalPinToInterrupt(p)   (p)
#define digitalPinHasPWM(p)        (pinHasPwm(p))

#define NOT_A_PIN_NUM 0xFFFFFFFFu

#define LED_BUILTIN_GPIO_Port   GREEN_LED_GPIO_Port
#define LED_BUILTIN_Pin         GREEN_LED_Pin

#define LEDG_GPIO_Port          GREEN_LED_GPIO_Port
#define LEDG_Pin                GREEN_LED_Pin
#define LEDR_GPIO_Port          RED_LED_GPIO_Port
#define LEDR_Pin                RED_LED_Pin
#define LEDB_GPIO_Port          BLUE_LED_GPIO_Port
#define LEDB_Pin                BLUE_LED_Pin

/* LED_ON and LED_OFF name the logic level, not the visual state. */
#define LED_ON(led)             HAL_GPIO_WritePin(led##_GPIO_Port, led##_Pin, GPIO_PIN_SET)
#define LED_OFF(led)            HAL_GPIO_WritePin(led##_GPIO_Port, led##_Pin, GPIO_PIN_RESET)
#define LED_TOGGLE(led)         HAL_GPIO_TogglePin (led##_GPIO_Port, led##_Pin)

/* Weak in NeuroN6.c. Runs after NeuroN6_Setup() and before the scheduler
 * starts. */
#ifdef __cplusplus
extern "C" {
#endif
void initVariant(void);
#ifdef __cplusplus
}
#endif

#endif /* PINS_ARDUINO_H */

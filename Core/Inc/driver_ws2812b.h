/**
 *******************************************************************************
 * @file           : driver_ws2812b.c
 * @brief          : Driver for addressable LED (WS2812B).
 *******************************************************************************
 *******************************************************************************
 * @note:
 * Based on Phil’s Lab "STM32 + RGB LEDs Firmware Tutorial (TIM + DMA) - Phil's
 * Lab #136" (https://youtu.be/MqbJTj0Cw6o).
 *******************************************************************************
 */

#ifndef __DRIVER_WS2812B_H
#define __DRIVER_WS2812B_H

/*
 * Includes.
 */

#include "stm32l4xx_hal.h"

/*
 * Definitions.
 */

#define LED_COUNT 5

#define WS2812B_TIM htim1
#define WS2812B_TIM_CHANNEL TIM_CHANNEL_1

// Based on 80 MHz peripheral clock at 1-1 pre-scaler and 100-1 ARR with time
// period of 1.25 us.
#define WS2812B_LO_VAL_DUTY 32     // 0.4 us.
#define WS2812B_HI_VAL_DUTY 64     // 0.8 us.
#define WS2812B_RST_VAL_PERIODS 40 // 50 us.
#define WS2812B_BITS_PER_LED 24    // 8 bytes for R, G, B.

#define WS2812B_DMA_BUF_LEN                                                    \
  ((LED_COUNT * WS2812B_BITS_PER_LED) + WS2812B_RST_VAL_PERIODS)

typedef union {
  struct {
    // Bit order: G7, G6, ... G0, R7, R6, ... R0, B7, B6, ... B0.
    uint8_t g;
    uint8_t r;
    uint8_t b;
  } colour;

  uint32_t data;

} WS2812B_LED_DATA_RGB;

/*
 * Variables.
 */

extern WS2812B_LED_DATA_RGB WS2812B_LED_DATA[LED_COUNT];
extern uint8_t WS2812B_DMA_BUF[WS2812B_DMA_BUF_LEN];
extern volatile uint8_t WS2812B_DMA_COMPLETE_FLAG;

/*
 * Functions.
 */

HAL_StatusTypeDef WS2812B_Init();

void WS2812B_Set_Colour(uint8_t index, uint8_t r, uint8_t g, uint8_t b);

HAL_StatusTypeDef WS2812B_Update();

// TODO: Callback for HAL function, call within:
//  void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim).
void WS2812B_Callback();

#endif

/**
  ******************************************************************************
  * @file           : driver_ws2812b.c
  * @brief          : Driver for addressable LED (WS2812B).
  ******************************************************************************
  * @note:
  * Based on Phil’s Lab "STM32 + RGB LEDs Firmware Tutorial (TIM + DMA) - Phil's Lab #136" (https://youtu.be/MqbJTj0Cw6o).
  ******************************************************************************
  */

/*
 * Includes.
 */

#include "driver_ws2812b.h"

/*
 * Variables.
 */

extern TIM_HandleTypeDef htim1;
WS2812B_LED_DATA_RGB WS2812B_LED_DATA[LED_COUNT];
uint8_t WS2812B_DMA_BUF[WS2812B_DMA_BUF_LEN];
volatile uint8_t WS2812B_DMA_COMPLETE_FLAG;

/*
 * Functions.
 */

HAL_StatusTypeDef WS2812B_Init() {
  // Initialize PWM timer.
  const HAL_StatusTypeDef hal_status = HAL_TIM_PWM_Init(&WS2812B_TIM);

  // Clear DMA buffer.
  for (uint16_t buffer_i = 0; buffer_i < WS2812B_DMA_BUF_LEN; buffer_i++) {
    WS2812B_DMA_BUF[buffer_i] = 0;
  }

  // Set DMA ready flag.
  WS2812B_DMA_COMPLETE_FLAG = 1;

  return hal_status;
}

void WS2812B_Set_Colour(const uint8_t index, const uint8_t r, const uint8_t g,
                        const uint8_t b) {
  WS2812B_LED_DATA[index].colour.r = r;
  WS2812B_LED_DATA[index].colour.g = g;
  WS2812B_LED_DATA[index].colour.b = b;
}

HAL_StatusTypeDef WS2812B_Update() {
  // Check if previous DMA transfer has been completed.
  if (!WS2812B_DMA_COMPLETE_FLAG) {
    return HAL_BUSY;
  }

  uint16_t buffer_i = 0;

  // For each LED.
  for (uint8_t led = 0; led < LED_COUNT; led++) {

    // Loop through all 24 data bits.
    for (uint8_t bits = 0; bits < WS2812B_BITS_PER_LED; bits++, buffer_i++) {

      // Calculate total bit shift.
      const uint8_t byte = bits / 8 * 8;  // Calculate byte offset as bit count.
      const uint8_t bit = 7 - bits % 8;  // Calculate bit position.
      const uint8_t bit_index = byte + bit;  // Calculate total bit shift.

      // Update DMA buffer to match LED data.
      if (WS2812B_LED_DATA[led].data >> bit_index & 0x01)  // If bit is set.
        WS2812B_DMA_BUF[buffer_i] = WS2812B_HI_VAL_DUTY;  // High.
      else
        WS2812B_DMA_BUF[buffer_i] = WS2812B_LO_VAL_DUTY;  // Low.
    }

    // Reset is already set to zero during WS2812B_Init.
  }

  // Attempt DMA transfer.
  const HAL_StatusTypeDef hal_status = HAL_TIM_PWM_Start_DMA(
          &WS2812B_TIM,
          WS2812B_TIM_CHANNEL,
          WS2812B_DMA_BUF,
          WS2812B_DMA_BUF_LEN
  );

  // If DMA transfer started successfully.
  if (hal_status == HAL_OK) {
    WS2812B_DMA_COMPLETE_FLAG = 0; // Clear DMA flag.
  }

  return hal_status;
}

// TODO: Callback for HAL function, call within:
//  void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim).
void WS2812B_Callback() {
  HAL_TIM_PWM_Stop_DMA(&WS2812B_TIM, WS2812B_TIM_CHANNEL);
  WS2812B_DMA_COMPLETE_FLAG = 1;
}

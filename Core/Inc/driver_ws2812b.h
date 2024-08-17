/**
 *******************************************************************************
 * @file:  driver_ws2812b.h
 * @brief: Driver for addressable LED (WS2812B).
 *******************************************************************************
 *******************************************************************************
 * @note:
 * Based on Phil’s Lab "STM32 + RGB LEDs Firmware Tutorial (TIM + DMA) - Phil's
 * Lab #136" (https://youtu.be/MqbJTj0Cw6o).
 *******************************************************************************
 */

#ifndef __DRIVER_WS2812B_H
#define __DRIVER_WS2812B_H

/* Includes. */

#include "stm32l4xx_hal.h"

/* Definitions. */

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

/**
 * @brief  Union for representing WS2812B LED color data.
 *
 * This union is designed to store and manipulate the color data for a single
 * WS2812B LED. The color data can be accessed either as individual RGB
 * components (red, green, blue) or as a combined 32-bit value.
 *
 * The structure within the union allows for easy manipulation of the red,
 * green, and blue color channels individually, while the `data` field provides
 * a way to handle the entire 24-bit color value as a single unit.
 *
 * The bit order in the `data` field is:
 * - G7, G6, ..., G0 (Green channel, most to least significant bit).
 * - R7, R6, ..., R0 (Red channel, most to least significant bit).
 * - B7, B6, ..., B0 (Blue channel, most to least significant bit).
 *
 * Usage example:
 * @code
 * WS2812B_LED_DATA_RGB led;
 *
 * // Set individual color components.
 * led.colour.r = 255; // Max red intensity.
 * led.colour.g = 128; // Half green intensity.
 * led.colour.b = 64;  // Quarter blue intensity.
 *
 * // Alternatively, set the entire color using the data field.
 * led.data = 0xFF8040; // RGB color: (255, 128, 64).
 * @endcode
 */
typedef union {
  struct {
    uint8_t g;
    uint8_t r;
    uint8_t b;
  } colour;

  uint32_t data;

} WS2812B_LED_DATA_RGB;

/* Variables. */

extern WS2812B_LED_DATA_RGB WS2812B_LED_DATA[LED_COUNT];
extern uint8_t WS2812B_DMA_BUF[WS2812B_DMA_BUF_LEN];
extern volatile uint8_t WS2812B_DMA_COMPLETE_FLAG;

/* Functions. */

/**
 * @brief  Initializes the WS2812B LED control system.
 * @retval HAL_StatusTypeDef: Returns the status of the initialization process.
 */
HAL_StatusTypeDef WS2812B_Init();

/**
 * @brief  Sets the color of a specific WS2812B LED.
 * @param  index: The index of the LED in the array (0-based index).
 * @param  r: The red component of the color (0-255).
 * @param  g: The green component of the color (0-255).
 * @param  b: The blue component of the color (0-255).
 * @retval None
 *
 * Example Usage:
 * @code
 * // Set the first LED to bright red.
 * WS2812B_Set_Colour(0, 255, 0, 0);
 *
 * // Set the second LED to green.
 * WS2812B_Set_Colour(1, 0, 255, 0);
 * @endcode
 */
void WS2812B_Set_Colour(uint8_t index, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief  Updates the WS2812B LEDs with the latest color data.
 * @retval HAL_StatusTypeDef: Returns the status of the operation.
 */
HAL_StatusTypeDef WS2812B_Update();

/**
 * @brief  Callback function to handle the completion of WS2812B data transfer.
 *
 * Example usage within `HAL_TIM_PWM_PulseFinishedCallback()`:
 * @code
 * void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
 *   WS2812B_Callback();
 * }
 * @endcode
 *
 * @retval None
 */
void WS2812B_Callback();

#endif

/**
 *******************************************************************************
 * @file:  driver_ws2812b.c
 * @brief: Driver for addressable LED (WS2812B).
 *******************************************************************************
 * @note:
 * Based on Phil’s Lab "STM32 + RGB LEDs Firmware Tutorial (TIM + DMA) - Phil's
 * Lab #136" (https://youtu.be/MqbJTj0Cw6o).
 *******************************************************************************
 */

/* Includes. */

#include "driver_ws2812b.h"

/* Variables. */

extern TIM_HandleTypeDef htim1;
WS2812B_LED_DATA_RGB WS2812B_LED_DATA[LED_COUNT];
uint8_t WS2812B_DMA_BUF[WS2812B_DMA_BUF_LEN];
volatile uint8_t WS2812B_DMA_COMPLETE_FLAG;

/* Functions. */

/**
 * @brief  Initializes the WS2812B LED control system.
 * @retval HAL_StatusTypeDef: Returns the status of the initialization process.
 *         - `HAL_OK`: The initialization was successful.
 *         - Other `HAL_StatusTypeDef` values indicate a failure during
 *           initialization.
 *
 * - PWM Timer Initialization: Calls `HAL_TIM_PWM_Init()` to initialize the PWM
 *   timer (`WS2812B_TIM`). This timer is used to generate the PWM signals
 *   necessary for controlling the WS2812B LEDs.
 *
 * - DMA Buffer Clearing: Clears the `WS2812B_DMA_BUF` by setting all elements
 *   to 0. This ensures that the DMA buffer starts in a known state, preventing
 *   any unintended signals from being sent to the LEDs during the first update.
 *
 * - DMA Ready Flag: Sets the `WS2812B_DMA_COMPLETE_FLAG` to 1, indicating that
 *   the system is ready for a DMA transfer. This flag is crucial for
 *   synchronizing updates to the LEDs, ensuring that data is only sent when the
 *   system is ready.
 *
 * Implementation Notes:
 * - Ensure that `WS2812B_TIM` is properly configured and that the
 *   `WS2812B_DMA_BUF_LEN` is correct for the number of LEDs (`LED_COUNT`) and
 *   the bits per LED (`WS2812B_BITS_PER_LED`).
 * - The function should be called during system initialization before any
 *   attempts to update the LEDs.
 *
 * Potential Issues:
 * - If `HAL_TIM_PWM_Init()` fails, the function will return an error code, and
 *   the LED system will not be able to operate correctly.
 * - The DMA buffer must be properly sized using the `WS2812B_DMA_BUF_LEN` macro
 *   to prevent data corruption or timing issues.
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

/**
 * @brief  Sets the color of a specific WS2812B LED.
 * @param  index: The index of the LED in the array (0-based index).
 * @param  r: The red component of the color (0-255).
 * @param  g: The green component of the color (0-255).
 * @param  b: The blue component of the color (0-255).
 * @retval None
 *
 * This function updates the RGB color values for the specified LED in the
 * WS2812B_LED_DATA array. The changes will take effect the next time the LED
 * data is transmitted.
 *
 * @Note: Ensure that the index provided does not exceed the array bounds,
 * as this will result in undefined behavior.
 */
void WS2812B_Set_Colour(const uint8_t index, const uint8_t r, const uint8_t g,
                        const uint8_t b) {
  WS2812B_LED_DATA[index].colour.r = r;
  WS2812B_LED_DATA[index].colour.g = g;
  WS2812B_LED_DATA[index].colour.b = b;
}

/**
 * @brief  Updates the WS2812B LEDs with the latest color data.
 * @retval HAL_StatusTypeDef: Returns the status of the operation.
 *         - `HAL_OK`: The DMA transfer started successfully.
 *         - `HAL_BUSY`: The previous DMA transfer is still in progress.
 *
 * - DMA Completion Check: First, it checks if the previous DMA transfer has
 *   completed by checking the `WS2812B_DMA_COMPLETE_FLAG`. If the flag is not
 *   set, indicating that the previous transfer is still ongoing, the function
 *   returns `HAL_BUSY` without starting a new transfer.
 *
 * - Data Preparation: If the DMA is ready, the function then prepares the data
 *   for each LED by iterating through the `WS2812B_LED_DATA` array. For each
 *   LED, it calculates the corresponding bits and updates the DMA buffer
 *   (`WS2812B_DMA_BUF`) with the appropriate duty cycle values
 *   (`WS2812B_HI_VAL_DUTY` for a high bit, `WS2812B_LO_VAL_DUTY` for a low
 *   bit).
 *
 * - DMA Transfer Start: After preparing the data, the function attempts to
 *   start a new DMA transfer using `HAL_TIM_PWM_Start_DMA()`. If the transfer
 *   is started successfully, the `WS2812B_DMA_COMPLETE_FLAG` is cleared,
 *   signaling that a new transfer is in progress.
 *
 * Implementation Notes:
 * - The function assumes that the DMA buffer has been initialized correctly by
 *   `WS2812B_Init()`.
 * - The reset condition of the LEDs is handled by the initialization routine,
 *   so no additional steps are required here.
 */
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
      const uint8_t byte = bits / 8 * 8; // Calculate byte offset as bit count.
      const uint8_t bit = 7 - bits % 8;  // Calculate bit position.
      const uint8_t bit_index = byte + bit; // Calculate total bit shift.

      // Update DMA buffer to match LED data.
      if (WS2812B_LED_DATA[led].data >> bit_index & 0x01) // If bit is set.
        WS2812B_DMA_BUF[buffer_i] = WS2812B_HI_VAL_DUTY;  // High.
      else
        WS2812B_DMA_BUF[buffer_i] = WS2812B_LO_VAL_DUTY; // Low.
    }

    // Reset is already set to zero during WS2812B_Init.
  }

  // Attempt DMA transfer.
  const HAL_StatusTypeDef hal_status = HAL_TIM_PWM_Start_DMA(
      &WS2812B_TIM, WS2812B_TIM_CHANNEL, WS2812B_DMA_BUF, WS2812B_DMA_BUF_LEN);

  // If DMA transfer started successfully.
  if (hal_status == HAL_OK) {
    WS2812B_DMA_COMPLETE_FLAG = 0; // Clear DMA flag.
  }

  return hal_status;
}

/**
 * @brief  Callback function to handle the completion of WS2812B LED data.
 *
 * This function should be called within the
 * `HAL_TIM_PWM_PulseFinishedCallback()` to handle the completion of a PWM
 * pulses for WS2812B LEDs. The function stops the DMA (Direct Memory Access)
 * transfer for the WS2812B LEDs and sets a flag to indicate that the data
 * transfer is complete.
 *
 * This callback is essential for managing the timing and synchronization of
 * WS2812B LED control, ensuring that the system only proceeds after the current
 * data transfer is fully completed.
 *
 * @retval None
 */
void WS2812B_Callback() {
  HAL_TIM_PWM_Stop_DMA(&WS2812B_TIM, WS2812B_TIM_CHANNEL);
  WS2812B_DMA_COMPLETE_FLAG = 1;
}

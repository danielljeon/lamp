# lamp

STM32L432KC driving WS2812B LEDs.

---

<details markdown="1">
  <summary>Table of Contents</summary>

- [1 Overview](#1-overview)
    - [1.1 Bill of Materials (BOM)](#11-bill-of-materials-bom)
    - [1.2 Pin Configurations](#12-pin-configurations)
    - [1.3 Clock Configurations](#13-clock-configurations)
- [2 WS2812B](#2-ws2812b)
    - [2.1 Clocks](#21-clocks)
    - [2.2 Pulse Width Modulation (PWM) Timer](#22-pulse-width-modulation-pwm-timer)
        - [2.2.1 Timer Calculations](#221-timer-calculations)
    - [2.3 Direct Memory Access (DMA)](#23-direct-memory-access-dma)
    - [2.4 Nested Vectored Interrupt Controller (NVIC)](#24-nested-vectored-interrupt-controller-nvic)
    - [2.5 WS2812B Driver](#25-ws2812b-driver)
        - [2.5.1 PWM Duty Cycle Calculations](#251-pwm-duty-cycle-calculations)
        - [2.5.2 Reset Code Time Periods Calculation](#252-reset-code-time-periods-calculation)

</details>

---

## 1 Overview

### 1.1 Bill of Materials (BOM)

| Manufacturer Part Number | Manufacturer       | Description         | Quantity | Notes     |
|--------------------------|--------------------|---------------------|---------:|-----------|
| NUCLEO-L432KC            | STMicroelectronics | Nucleo-64 board     |        1 | Dev (DNP) |
| STM32F446RE              | STMicroelectronics | 32-bit MCU          |        1 |           |
| WS2812B                  | (Various)          | Addressable RGB LED |        5 |           |

### 1.2 Pin Configurations

<details markdown="1">
  <summary>CubeMX Pinout</summary>

![CubeMX Pinout.png](docs/CubeMX%20Pinout.png)

</details>

<details markdown="1">
  <summary>Pin & Peripherals Table</summary>

| STM32L432KC | Peripheral     | Config | Connection            | Notes |
|-------------|----------------|--------|-----------------------|-------|
| PB3         | SYS_JTDO-SWO   |        | SWD/JTAG (ie: TC2050) |       |
| PA14        | SYS_JTCK-SWCLK |        | SWD/JTAG (ie: TC2050) |       |
| PA13        | SYS_JTMS-SWDIO |        | SWD/JTAG (ie: TC2050) |       |
| PA8         | TIM1_CH1       | PWM    | WS2812B Pin 1: DIN    |       |

</details>

### 1.3 Clock Configurations

```
16 MHz High Speed Internal (HSI)
↓
Phase-Locked Loop Main (PLLM)
↓
80 MHz SYSCLK
↓
80 MHz HCLK
↓
 → 80 MHz APB1 (Maxed) → 80 MHz APB1 Timer
 → 80 MHz APB2 (Maxed) → 80 MHz APB2 Timer
```

---

## 2 WS2812B

Individually addressable RGB LED with an integrated control circuit over a
series single-wire data protocol.

### 2.1 Clocks

APB2: 80 MHz (Clock for TIM1CH[1:4]).

All subsequent calculations assume an 80 MHz peripheral clock on the PWM timer
channel (STM32L432KC).

### 2.2 Pulse Width Modulation (PWM) Timer

A Pulse Width Modulation (PWM) timer is utilized generate data signals to the
WS2812B.

PA8 → Timer 1 Channel 1 → PWM Generation CH1.

```c
htim1.Instance = TIM1;
htim1.Init.Prescaler = 1-1;
htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
htim1.Init.Period = 100-1;
htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
htim1.Init.RepetitionCounter = 0;
htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
```

#### 2.2.1 Timer Calculations

Given the PWM equation:

$$f_{PWM} = \frac{f_{TIM}}{ \left( ARR + 1 \right) \times \left( PSC + 1
\right) }$$

- $f_{TIM} = 80 \space \mathrm{MHz}$
    - Defined by the PWM channel's peripheral clock.
- $ARR = 100 - 1$
    - Counter period, aka Auto Reload Register (ARR) of 100 is used to simplify
      the translation of duty cycle percentages.
- $f_{PWM} = 800 \space \mathrm{kHz}$
    - As specified by the WS2812B datasheet, the target data transfer time
      period is 1.25 µs.
    - Calculating for required PWM frequency:
        - $f_{PWM} = \frac{1}{1.25 \times 10 ^{-6} \space \mathrm{s}} = 800
          \space \mathrm{kHz}$

Thus, the prescaler, $PSC = 1 - 1$.

### 2.3 Direct Memory Access (DMA)

Direct Memory Access (DMA) is used to transfer the color data for the WS2812B
LEDs directly from memory to the PWM timer's registers without requiring CPU
overhead.

On CubeMX, configure `TIM1_CH1` for a DMA channel:

- Memory to Peripheral.
- Normal request with memory increment addressing, data width "Byte".

### 2.4 Nested Vectored Interrupt Controller (NVIC)

Nested Vectored Interrupt Controller (NVIC) is used to efficiently manage the
interrupt generated by the DMA controller upon the completion of a data
transfer. This allows the system to update the PWM signals for the WS2812B LEDs
with minimal CPU overhead, enabling efficient and responsive control of the
LEDs.

HAL_TIM_PWM_PulseFinishedCallback() is called withing part of the Interrupt
Service Routine (ISR) for PWM DMA.

On CubeMX, enable NVIC for TIM1.

### 2.5 WS2812B Driver

The WS2812B driver is made of 2 files:

1. [driver_ws2812b.h](Core/Inc/driver_ws2812b.h).
2. [driver_ws2812b.c](Core/Src/driver_ws2812b.c).

```
ws2812b_init(): Initialize DMA, flags, timers, etc.
↓
ws2812b_set_colour(): Set struct values for (R, G, B) colours.
↓
ws2812b_update(): Initialize DMA buffer and trigger PWM DMA transfer.
↓
ws2812b_callback(): Called in ISR for end of PWM, stop DMA transfer.
```

#### 2.5.1 PWM Duty Cycle Calculations

Duty cycle required for PWM control:

$$D = \frac{PW}{T} \times 100$$

- $D$ = Duty cycle, required calculation.
- $PW$ = Pulse width (active time), as defined by the datasheet.
- $T$ = Total signal time period, 1.25 µs, as defined by the datasheet.

| Operation | $PW$   | Margin  | $D$ |
|-----------|--------|---------|-----|
| 0 code    | 0.4 µs | ±150 ns | 32% |
| 1 code    | 0.8 µs | ±150 ns | 64% |

#### 2.5.2 Reset Code Time Periods Calculation

The datasheet requires a low signal of > 50 µs. Thus, the number of full (low)
cycles is given by:

$$50 \space \mathrm{\mu s} \div 1.25 \space \mathrm{\mu s} = 40$$

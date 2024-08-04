# lamp

STM32L432KC driving WS2812B LEDs.

---

<details markdown="1">
  <summary>Table of Contents</summary>

- [1 WS2812B](#1-ws2812b)
- [1.1 PWM Timer Setting Calculations](#11-pwm-timer-setting-calculations)
- [1.2 PWM Duty Cycle Calculations](#12-pwm-duty-cycle-calculations)
- [1.3 Reset Code Time Periods Calculation](#13-reset-code-time-periods-calculation)

</details>

---

## 1 WS2812B

All further calculations are assumed using a 80 MHz peripheral clock on the PWM
timer channel (STM32L432KC).

### 1.1 PWM Timer Setting Calculations

As specified by the datasheet, aiming for a total data transfer time period of
1.25 us.

Calculating for required PWM frequency:

$$\frac{1}{1.25 \times 10 ^{-6} \space \mathrm{s}} = 800 \space \mathrm{kHz}$$

Given the PWM equation:

$$f_{PWM} = \frac{f_{TIM}}{ \left( ARR + 1 \right) \times \left( PSC + 1
\right)}$$

- $f_{TIM} = 80 \times 10^6$ defined by the PWM channel's clock.

$$ARR = 100 - 1$$

$$PSC = 1 - 1$$

ARR of 100 is used to simplify the translation of duty cycle percentages.

### 1.2 PWM Duty Cycle Calculations

Duty cycle required for PWM control:

$$D = \frac{PW}{T} \times 100$$

- $D$ = Duty cycle.
- $PW$ = Pulse width active time.
- $T$ = Total signal time period. (Recall, 1.25 us)

| Operation | $PW$   | Margin  | $D$ |
|-----------|--------|---------|-----|
| 0 code    | 0.4 us | ±150 ns | 32% |
| 1 code    | 0.8 us | ±150 ns | 64% |

### 1.3 Reset Code Time Periods Calculation

The datasheet requires a low signal of > 50 us. Thus, the number of full (low)
cycles is given by:

$$50 \space \mathrm{\mu s} \div 1.25 \space \mathrm{\mu s} = 40$$

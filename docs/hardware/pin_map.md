## PWM Outputs

| Channel | STM32 Pin | Timer                    |
| ------- | --------- | ------------------------ |
| CH1     | PA0       | TIM2_CH1                 |
| CH2     | PA1       | TIM2_CH2                 |
| CH3     | PB6       | TIM4_CH1                 |
| CH4     | PB7       | TIM4_CH2                 |
| CH5     | PB8       | TIM4_CH3                 |
| CH6     | PB9       | TIM4_CH4                 |
| CH7     | PB4       | TIM3_CH1 — partial remap |
| CH8     | PB5       | TIM3_CH2 — partial remap |

### Timer configuration

All three timers use:

```text
Timer clock = 72 MHz
Prescaler   = 71
Counter     = 1 MHz
ARR         = 19999
Frequency   = 50 Hz
```

Therefore:

```text
1 timer tick = 1 µs
```

and:

```text
CCR = 1000 → 1000 µs
CCR = 1500 → 1500 µs
CCR = 2000 → 2000 µs
```

TIM3 must use its partial alternate-function remapping to place CH1/CH2 on PB4/PB5.

PB3 is not used as a PWM output.

The SWJ debug interface will be configured for **Serial Wire** during STM32 initialization so ST-Link SWD remains available while the required remapped GPIO functionality is used.

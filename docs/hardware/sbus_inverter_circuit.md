# SBUS Inverter Circuit

## Purpose

SBUS uses inverted UART logic.

The STM32F103 USART2 generates normal UART polarity on PA2. An external NPN transistor provides the required electrical inversion.

## Circuit

```text
STM32 PA2
   │
  1 kΩ
   │
   ├──── Base
   │      NPN
   │
  NPN
   │
   ├──── Collector ───── SBUS Signal
   │
  10 kΩ
   │
  3.3 V
   │
  Emitter
   │
  GND
```

## Connections

```text
PA2 ── 1 kΩ ── NPN Base
NPN Emitter ── GND
NPN Collector ── SBUS output
SBUS output ── 10 kΩ ── 3.3 V
```

## Firmware

USART2 configuration:

```text
Baud rate : 100000
Data      : 8 bits
Parity    : Even
Stop bits : 2
Polarity  : Normal UART
```

The firmware must **not** attempt to perform software inversion.

The transistor performs the electrical inversion required by SBUS.

## Important

The SBUS output voltage level is 3.3 V with the specified pull-up.

If a connected device requires a different electrical level, the appropriate level-shifting arrangement must be evaluated separately.

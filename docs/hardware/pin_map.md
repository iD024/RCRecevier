# RC Receiver V2 — Pin Map

## MCU

STM32F103C8T6 Blue Pill.

## NRF24L01+

| Function | STM32 Pin |
| -------- | --------- |
| SCK      | PA5       |
| MISO     | PA6       |
| MOSI     | PA7       |
| CSN      | PA4       |
| CE       | PB11      |
| IRQ      | PB0       |

## PWM

| Channel | Pin | Timer    |
| ------- | --- | -------- |
| CH1     | PA0 | TIM2_CH1 |
| CH2     | PA1 | TIM2_CH2 |
| CH3     | PB6 | TIM4_CH1 |
| CH4     | PB7 | TIM4_CH2 |
| CH5     | PB8 | TIM4_CH3 |
| CH6     | PB9 | TIM4_CH4 |
| CH7     | PB3 | TBD      |
| CH8     | PB4 | TBD      |

CH7 and CH8 require confirmation during STM32 peripheral/pin configuration.

## UART

| Interface | Function | TX   | RX   | Configuration |
| --------- | -------- | ---- | ---- | ------------- |
| USART1    | Debug    | PA9  | PA10 | 115200        |
| USART2    | SBUS     | PA2  | —    | 100000, 8E2   |
| USART3    | IBUS     | PB10 | —    | 115200, 8N1   |

## Other

| Function    | Pin  | Configuration       |
| ----------- | ---- | ------------------- |
| Status LED  | PC13 | Active LOW          |
| Bind Button | PB12 | Active LOW, pull-up |

## Hardware Notes

The SBUS output uses an external NPN transistor inverter because the STM32F103 USART does not provide the required hardware signal inversion.

The NRF24L01+ requires local power-supply decoupling:

* 100 nF ceramic capacitor
* 10 µF electrolytic capacitor

Place both close to the NRF24 VCC/GND pins.

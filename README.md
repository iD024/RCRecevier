# RC Receiver V2

Professional STM32F103C8T6-based RC receiver for a custom NRF24L01+ radio link.

## Hardware

* MCU: STM32F103C8T6 Blue Pill
* MCU: ARM Cortex-M3, 72 MHz
* Radio: NRF24L01+
* Radio protocol: Enhanced ShockBurst
* RC channels: 8
* Servo outputs: 8-channel PWM
* Digital outputs: SBUS and IBUS
* Debug interface: USART1
* Build system: PlatformIO
* Framework: STM32Cube HAL
* Language: C++17

## Firmware Architecture

```text
Application
     │
     ▼
Radio / Output / Diagnostics
     │
     ▼
Protocol
     │
     ▼
Channel Processing
     │
     ▼
Drivers
     │
     ▼
STM32 HAL / Hardware
```

The firmware uses a cooperative superloop at 200 Hz with a 5 ms loop period.

No RTOS is used.

## Project Structure

```text
RCReceiver/
├── include/
│   └── config/
├── src/
│   ├── core/
│   ├── drivers/
│   ├── protocol/
│   ├── channel/
│   ├── output/
│   ├── failsafe/
│   ├── storage/
│   ├── telemetry/
│   └── diagnostics/
├── docs/
├── test/
├── platformio.ini
└── README.md
```

## Development Status

### Version 1

* [X] M1 — Project Scaffold & Configuration
* [X] M2 — STM32 Initialization
* [X] M3 — NRF24 Driver
* [ ] M4 — Communication Protocol
* [ ] M5 — Packet Decoder
* [ ] M6 — Channel Processing
* [ ] M7 — PWM Output
* [ ] M8 — Failsafe

### Version 2

* [ ] M9 — SBUS Output
* [ ] M10 — IBUS Output
* [ ] M11 — Telemetry
* [ ] M12 — Flash Configuration Storage
* [ ] M13 — Binding
* [ ] M14 — Diagnostics Console
* [ ] M15 — Integration Testing

See the project design specification and implementation plan for the detailed architecture and development sequence.

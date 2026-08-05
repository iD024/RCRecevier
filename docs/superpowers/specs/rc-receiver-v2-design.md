# RC Receiver V2 — Design Specification

**Date:** 2026-08-05  
**Status:** Approved  
**Author:** RC Receiver Firmware Team  
**Project:** RC Receiver V2 (STM32F103C8T6 + NRF24L01+)

---

## 1. Project Overview

A professional-grade STM32-based RC receiver capable of receiving commands from a custom
ESP32 transmitter over an NRF24L01 radio link.

This document covers **Version 1 (Milestones 1–8)** and **Version 2 (Milestones 9–15)**.
The architecture is intentionally designed to support future V3+ features (frequency hopping,
CRSF, OTA, AES encryption) without major refactoring.

### Goals

- Receive 8-channel RC data at ≤10ms latency end-to-end
- Output servo PWM (8 channels), SBUS, and IBUS simultaneously
- Professional embedded firmware suitable for GitHub and PCB production
- Modular, testable, zero-global-variable architecture

---

## 2. Target Hardware

| Component | Part | Notes |
|---|---|---|
| MCU | STM32F103C8T6 | ARM Cortex-M3, 72 MHz, 64KB Flash, 20KB RAM |
| Radio | NRF24L01+ | 2.4 GHz, Enhanced ShockBurst mode |
| Interface | SPI1 | 8 MHz SCK |
| Debug UART | USART1 | 115200 baud, PA9/PA10 |
| SBUS Output | USART2 | 100000 baud, inverted logic, PA2 |
| IBUS Output | USART3 | 115200 baud, PB10 |
| PWM | TIM2+TIM3+TIM4 | 8 channels, 50Hz |
| Storage | Internal Flash | HAL EEPROM Emulation, 2 pages |
| Status LED | PC13 | Active LOW (Blue Pill onboard LED) |
| Bind Button | PB12 | Active LOW, internal pull-up |

---

## 3. Pin Allocation Map

```
STM32F103C8T6 Pin Map
==============================
Function     Pin    Notes
------------------------------
SPI1 SCK     PA5    NRF24 clock
SPI1 MISO    PA6    NRF24 data in
SPI1 MOSI    PA7    NRF24 data out
NRF24 CSN    PA4    GPIO output, active LOW
NRF24 CE     PB11   GPIO output, active HIGH (moved off PB1 to avoid TIM3_CH4 conflict)
NRF24 IRQ    PB0    EXTI, falling edge

PWM CH1      PA0    TIM2_CH1, Servo 1
PWM CH2      PA1    TIM2_CH2, Servo 2
PWM CH3      PB6    TIM4_CH1, Servo 3
PWM CH4      PB7    TIM4_CH2, Servo 4
PWM CH5      PB8    TIM4_CH3, Servo 5
PWM CH6      PB9    TIM4_CH4, Servo 6
PWM CH7      PB3    TIM2_CH2-remap or GPIO (TBD in CubeMX), Servo 7
PWM CH8      PB4    TIM3_CH1-partial-remap (PB4), Servo 8

NOTE: Channels 7-8 will use TIM2/TIM3 partial remaps confirmed in Milestone 2.
TIM4 fully provides CH3-CH6 on PB6-PB9 with no AF conflicts.

USART1 TX    PA9    Debug console output
USART1 RX    PA10   Debug console input
USART2 TX    PA2    SBUS output (external inverter circuit required)
USART3 TX    PB10   IBUS output

Status LED   PC13   Active LOW, onboard Blue Pill LED
Bind Button  PB12   Active LOW, internal pull-up
```

> **NOTE:** The alternate function conflicts on PA6/PA7 between SPI1 and TIM3 will be
> definitively resolved during Milestone 2 using CubeMX's pin conflict checker.
> See Section 9 (Hardware Notes) for the SBUS inverter circuit.

---

## 4. Architecture

### 4.1 Layered Architecture

```
APPLICATION LAYER
  main.cpp — System Init -> Cooperative Superloop
       |
  _____|__________________________
  |              |               |
RADIO TASK    OUTPUT TASK    DIAGNOSTICS TASK
(5ms budget)  (20ms)         (100ms)
  |              |
PROTOCOL      CHANNEL PROCESSING
LAYER         - Scaling / Deadband / Filter
- Packet
- CRC-16
- Validator
  |
DRIVER LAYER
- NRF24 / SPI / GPIO / Timers / UART
```

### 4.2 Superloop Timing Budget

The main loop runs at 5ms intervals (200Hz). Within each loop iteration:

```
5ms Superloop Tick
[Radio Poll: ~0.5ms] [Channel Process: ~0.1ms] [Output Update: ~0.2ms] [Failsafe: ~0.05ms] [Diagnostics throttled: ~0.1ms every 100ms]
```

### 4.3 Data Flow

```
NRF24 Radio (hardware ESB)
     |
     | SPI read (on IRQ or poll)
     v
RawPacket (32 bytes)
     |
     | Protocol::Decoder::validate()
     v
DecodedPacket { header, channels[8], crc }
     |
     | ChannelProcessor::process()
     v
ChannelData { us_values[8], scaled[8] }
     |
     |------------------------------------------
     v                                          v
PWM Output (TIM2/3/4)                    SBUS Output (USART2)
                                         IBUS Output (USART3)
```

---

## 5. Packet Protocol

### 5.1 Packet Format (32 bytes fixed)

```
Byte  0    : MAGIC_BYTE_0      (0xRC = 0xAC)
Byte  1    : MAGIC_BYTE_1      (0x24)
Byte  2    : PROTOCOL_VERSION  (0x01)
Byte  3    : PACKET_TYPE       (enum: CONTROL=0x01, BIND=0x02, TELEMETRY_REQ=0x03)
Byte  4-7  : TRANSMITTER_ID    (uint32_t, unique TX hardware ID)
Byte  8-11 : RECEIVER_ID       (uint32_t, 0xFFFFFFFF = broadcast during bind)
Byte  12   : SEQUENCE_NUMBER   (uint8_t, rolling counter)
Byte  13   : FLAGS             (uint8_t, bit0=failsafe, bit1=bind, bits2-7=reserved)
Byte  14-15: CHANNEL_0         (uint16_t, 1000-2000 us)
Byte  16-17: CHANNEL_1         (uint16_t)
Byte  18-19: CHANNEL_2         (uint16_t)
Byte  20-21: CHANNEL_3         (uint16_t)
Byte  22-23: CHANNEL_4         (uint16_t)
Byte  24-25: CHANNEL_5         (uint16_t)
Byte  26-27: CHANNEL_6         (uint16_t)
Byte  28-29: CHANNEL_7         (uint16_t)
Byte  30-31: CRC_16            (CRC-16/CCITT-FALSE over bytes 0-29)
```

Total: 32 bytes — fits exactly in one NRF24 Enhanced ShockBurst payload.

### 5.2 Protocol Design Decisions

- **Fixed 32-byte packets**: NRF24 ESB works best with fixed payload size.
- **CRC-16/CCITT-FALSE (poly 0x1021)**: Stronger than CRC-8 (used by older FlySky).
- **Versioned**: PROTOCOL_VERSION byte allows future changes without breaking compatibility.
- **Receiver ID**: Enables binding. 0xFFFFFFFF = broadcast during bind phase.
- **Sequence number**: Enables packet loss detection for telemetry metrics.

---

## 6. Folder Structure

```
RCReceiver/
|-- platformio.ini
|-- README.md
|-- docs/
|   |-- superpowers/
|   |   |-- specs/        <- This document
|   |   `-- plans/        <- Implementation plans
|   |-- hardware/
|   |   |-- pin_map.md
|   |   `-- sbus_inverter_circuit.md
|   `-- architecture/
|       `-- data_flow.md
|-- src/
|   |-- main.cpp
|   |-- core/system/
|   |   |-- System.hpp
|   |   `-- System.cpp
|   |-- drivers/
|   |   |-- nrf24/
|   |   |-- spi/
|   |   |-- gpio/
|   |   |-- timer/
|   |   `-- uart/
|   |-- protocol/
|   |   |-- Packet.hpp
|   |   |-- PacketDecoder.hpp/.cpp
|   |   |-- CRC16.hpp/.cpp
|   |   |-- Binding.hpp/.cpp
|   |-- output/
|   |   |-- pwm/
|   |   |-- sbus/
|   |   `-- ibus/
|   |-- channel/
|   |   |-- ChannelProcessor.hpp/.cpp
|   |-- failsafe/
|   |   |-- Failsafe.hpp/.cpp
|   |-- storage/
|   |   |-- ConfigStore.hpp/.cpp
|   |-- telemetry/
|   |   |-- TelemetryFrame.hpp
|   |   `-- Telemetry.hpp
|   `-- diagnostics/
|       |-- DebugConsole.hpp/.cpp
|-- include/config/
|   |-- HardwareConfig.hpp
|   `-- FirmwareConfig.hpp
`-- test/
    `-- (unit tests, host-side where possible)
```

---

## 7. State Machine

```
Power On Reset
      |
      v
   [BOOT] -- System init OK -->
      |
      v
  [NORMAL] <-- bind complete -- [BINDING] <-- bind button held
      |
      | No packet for > FAILSAFE_TIMEOUT_MS
      v
  [FAILSAFE]
      |
      | Valid packet received
      v
  [NORMAL] (resume)
```

---

## 8. Version 1 Milestones

| Milestone | Name | Key Deliverable |
|---|---|---|
| M1 | System Architecture | This document + folder scaffold |
| M2 | STM32 Initialization | CubeMX config + HAL init code |
| M3 | NRF24 Driver | SPI + NRF24 class, ESB mode |
| M4 | Communication Protocol | Packet struct + CRC-16 |
| M5 | Packet Decoder | Validation + reject corrupted packets |
| M6 | Channel Processing | 1000-2000us conversion + filtering |
| M7 | PWM Output | 8-channel servo PWM via TIM2/3/4 |
| M8 | Failsafe | Timeout detection + throttle cut |

---

## 9. Version 2 Milestones

| Milestone | Name | Key Deliverable |
|---|---|---|
| M9  | SBUS Output | 100k inverted UART, 25ms frame |
| M10 | IBUS Output | 115200 UART, checksum |
| M11 | Telemetry Framework | RSSI, packet loss, voltage in ACK payload |
| M12 | Flash Config Storage | HAL EEPROM emulation |
| M13 | Binding System | Bind button, ID pairing, flash storage |
| M14 | Diagnostics Console | Serial debug with metrics |
| M15 | Integration Testing | Stress test, timing validation |

---

## 10. Hardware Notes

### SBUS Inverter Circuit

SBUS uses inverted UART logic. STM32F103 has no hardware signal inversion.
A single NPN transistor (e.g., 2N2222 or BC547) is used:

```
USART2 TX (PA2) --[1kOhm]--+-- Base (NPN)
                            |
GND ------------------------+-- Emitter

3.3V --[10kOhm]-------------- Collector --> SBUS Signal Output
```

**IMPORTANT:** The STM32 firmware sends **normal (non-inverted) UART** on PA2 at 100000 baud.
The transistor circuit performs the electrical inversion. Do NOT configure the USART for
any software inversion. The STM32F103 does not support hardware UART polarity inversion.

This gives correct SBUS polarity (idle HIGH, active LOW) at 3.3V logic.
Use a level shifter on the collector pull-up if 5V SBUS is required.

### NRF24L01+ Power Supply Decoupling

Place 100nF ceramic + 10uF electrolytic between VCC and GND of the NRF24 module.
Power supply noise is the #1 cause of NRF24 unreliability.

---

## 11. Coding Standards

| Rule | Policy |
|---|---|
| Language | C++17 |
| Namespaces | Every module in RC:: namespace (e.g. RC::Protocol, RC::Output) |
| Global variables | Forbidden. Use class members. |
| Magic numbers | All constants in HardwareConfig.hpp or FirmwareConfig.hpp as constexpr |
| Interrupt handlers | Minimal — set a flag only, never block |
| Error handling | enum class Result return types, not exceptions |
| Naming | PascalCase types, camelCase variables, SCREAMING_SNAKE constants |
| File guards | #pragma once |

---

## 12. Future Expansion Points (V3+ Reserved)

- **Frequency Hopping**: setChannel() abstracted behind RadioDriver interface
- **Encryption**: Packet framing supports AES-128 payload extension
- **CRSF Protocol**: output/ directory structure accepts crsf/ module cleanly
- **OTA Firmware Update**: Flash storage uses upper pages, preserving bootloader area
- **Multi-receiver**: Receiver ID scheme supports 2^32 unique receivers per transmitter

---

*End of Design Specification — RC Receiver V2*

# STM32F103 NRF24L01+ RC Receiver (`RCReceiver`)

Production-ready firmware for an 8-channel 2.4 GHz RC receiver based on the STM32F103C8T6 (Blue Pill) MCU and NRF24L01+ transceiver. Designed for low latency and high reliability, it features concurrent PWM, SBUS, and IBUS outputs, non-blocking failsafe handling, dynamic binding, bi-directional telemetry via ACK payloads, and persistent configuration stored in flash memory.

---

## Overview & Key Features

* **Target MCU:** STM32F103C8T6 ARM Cortex-M3 running at 72 MHz.
* **RF Link:** NRF24L01+ transceiver connected via SPI1 using Enhanced ShockBurst / custom 32-byte packet framing with CRC16-CCITT verification.
* **Multi-Protocol Simultaneous Output:**
  * **PWM:** 8 independent hardware timer channels (50 Hz standard RC servo PWM, 1000–2000 µs pulse width).
  * **SBUS:** 100,000 baud, 8E2 inverted UART frame generator (USART2).
  * **IBUS:** 115,200 baud, 8N1 serial stream (USART3).
* **Dynamic Binding Protocol:** Runtime transmitter pairing over RF broadcast. Persists Transmitter ID, Receiver ID, RF channel, and address to flash.
* **Non-Blocking Failsafe:** Configurable 500 ms signal loss detection. Automatically drops throttle to 1000 µs, centers control surfaces to 1500 µs, and sets protocol failsafe flags on SBUS/IBUS.
* **Bi-Directional Telemetry:** Sends receiver voltage, RSSI, link quality (LQI), packet loss, and CPU load back to the transmitter using NRF24 dynamic ACK payloads.
* **Persistent Configuration:** Flash emulation on page 63 (`0x0800FC00`) with magic signature (`0x43464731`) and CRC32 verification.
* **Superloop Architecture:** Non-blocking 200 Hz (5 ms) core tick timing ensuring low jitter without RTOS overhead.

---

## Pinout & Hardware Connections

### NRF24L01+ Transceiver (SPI1)

| Signal | STM32 Pin | Peripheral | Description |
| :--- | :--- | :--- | :--- |
| **CSN** | `PA4` | GPIO Output | SPI Chip Select (Active Low) |
| **SCK** | `PA5` | `SPI1_SCK` | SPI Clock |
| **MISO** | `PA6` | `SPI1_MISO` | SPI Master In / Slave Out |
| **MOSI** | `PA7` | `SPI1_MOSI` | SPI Master Out / Slave In |
| **CE** | `PB11` | GPIO Output | Radio Chip Enable |
| **IRQ** | `PB0` | GPIO Input | Interrupt Request (optional polling/EXTI) |

### Protocol & Serial Outputs

| Interface | STM32 Pin | Peripheral | Configuration | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **Debug CLI** | `PA9` (TX), `PA10` (RX) | `USART1` | 115200 8N1 | Diagnostic console & logging |
| **SBUS Output** | `PA2` (TX) | `USART2` | 100000 8E2 | Inverted serial output for flight controllers |
| **IBUS Output** | `PB10` (TX) | `USART3` | 115200 8N1 | Standard serial output (FlySky IBUS protocol) |

### 8-Channel Servo PWM Outputs

| Channel | STM32 Pin | Timer Peripheral | Pulse Range | Default Function |
| :--- | :--- | :--- | :--- | :--- |
| **CH1** | `PA0` | `TIM2_CH1` | 1000–2000 µs | Roll / Aileron |
| **CH2** | `PA1` | `TIM2_CH2` | 1000–2000 µs | Pitch / Elevator |
| **CH3** | `PB6` | `TIM4_CH1` | 1000–2000 µs | Throttle |
| **CH4** | `PB7` | `TIM4_CH2` | 1000–2000 µs | Yaw / Rudder |
| **CH5** | `PB8` | `TIM4_CH3` | 1000–2000 µs | Aux 1 / Flight Mode |
| **CH6** | `PB9` | `TIM4_CH4` | 1000–2000 µs | Aux 2 |
| **CH7** | `PB4` | `TIM3_CH1` (Partial Remap) | 1000–2000 µs | Aux 3 |
| **CH8** | `PB5` | `TIM3_CH2` (Partial Remap) | 1000–2000 µs | Aux 4 |

### Status Controls & SWD Debug

| Signal | STM32 Pin | Type | Notes |
| :--- | :--- | :--- | :--- |
| **Status LED** | `PC13` | Output (Active-Low) | Onboard Blue Pill LED |
| **Bind Button** | `PB12` | Input (Pull-Up) | Short to GND to trigger bind mode |
| **SWDIO** | `PA13` | Debug | ST-Link / J-Link SWD Data line |
| **SWCLK** | `PA14` | Debug | ST-Link / J-Link SWD Clock line |

---

## Firmware Architecture

```
                       ┌──────────────────────┐
                       │  NRF24 Transceiver   │
                       └──────────┬───────────┘
                                  │ SPI1
                                  ▼
                       ┌──────────────────────┐
                       │   NRF24 Driver &     │
                       │ PacketDecoder (CRC16)│
                       └──────────┬───────────┘
                                  │ Valid Control Frame
                                  ▼
                       ┌──────────────────────┐
                       │  ChannelProcessor    │
                       │ (Filtering/Deadband) │
                       └──────────┬───────────┘
                                  │
                   ┌──────────────┴──────────────┐
                   │                             │
    Valid Packet Received                  Signal Loss (>500ms)
                   │                             │
                   ▼                             ▼
       ┌──────────────────────┐       ┌──────────────────────┐
       │   Active Channels    │       │ Failsafe Controller  │
       └──────────┬───────────┘       │ (Preset Safe Values) │
                  │                   └──────────┬───────────┘
                  └──────────────┬───────────────┘
                                 │
        ┌────────────────────────┼────────────────────────┐
        ▼                        ▼                        ▼
┌──────────────┐         ┌──────────────┐         ┌──────────────┐
│  PWM Output  │         │ SBUS Output  │         │ IBUS Output  │
│ (TIM2/3/4)   │         │ (USART2 100k)│         │(USART3 115.2k)│
└──────────────┘         └──────────────┘         └──────────────┘
```

The application runs a 200 Hz superloop structure:
1. **Radio Ingestion:** Non-blocking FIFO check on the NRF24L01+. Incoming 32-byte frames are validated against CRC16-CCITT and checked for matching `boundTransmitterId`.
2. **Channel Processing:** Applies exponential smoothing ($\alpha = 0.8$) and configurable deadband filters to input channels.
3. **Failsafe Monitor:** Tracks milliseconds elapsed since the last valid packet. If timeout exceeds 500 ms, channel values transition to pre-programmed safe positions and protocol failsafe flags are raised.
4. **Protocol Dispatch:** Timers update PWM match registers, while USART2 and USART3 transmit SBUS and IBUS frames.
5. **Telemetry Feedback:** Telemetry stats (VCC, RSSI, LQI) are staged into the NRF24 TX ACK payload pipe.

---

## Build & Flash Instructions

### Prerequisites
* [PlatformIO IDE](https://platformio.org/) (VS Code extension) or PlatformIO Core CLI.
* ST-Link V2, J-Link, or compatible SWD programmer.
* GCC ARM Embedded toolchain (automatically managed by PlatformIO).

### Building the Project

```bash
# Clone the repository
git clone https://github.com/your-username/RCReceiver.git
cd RCReceiver

# Build firmware for STM32F103C8
pio run -e bluepill_f103c8

# Run unit tests on host machine (Native C++ environment)
pio run -e native
```

### Flashing via ST-Link

1. Connect ST-Link V2 to the Blue Pill SWD header (`3V3`, `GND`, `SWDIO`, `SWCLK`).
2. Run the upload command:

```bash
pio run -e bluepill_f103c8 --target upload
```

---

## Configuration & Pairing / Binding

### Dynamic Binding Procedure
1. Hold down the **Bind Button** (`PB12` to GND) while powering on the receiver, or power on without a valid bound configuration.
2. The Status LED enters rapid blinking mode to indicate bind search.
3. Power on the transmitter in binding mode.
4. Upon receiving a valid Bind packet (`0x02`), the receiver saves the Transmitter ID, assigned RF Channel, and RF Pipe Address to flash page 63, then automatically resets into active receiver mode.

### LED Status Indicators

| LED Pattern | Receiver State | Description |
| :--- | :--- | :--- |
| **Solid ON (Brief 15ms pulse OFF)** | Active / Normal | Receiving valid RF packets from bound transmitter. |
| **5 Hz Blink (100ms ON / 100ms OFF)** | Failsafe Active | RF link lost; failsafe fallback positions applied. |
| **Rapid Blinking** | Binding Mode | Searching for binding transmitter. |
| **3 Rapid Blinks + 1s Pause** | Hardware Error | NRF24 SPI communications failed; check wiring/power. |

### Configurable Failsafe
Failsafe defaults are defined in `FirmwareConfig.hpp` and can be customized per channel. On signal loss:
* Throttle (`CH3`) drops to `1000 µs`.
* Control surfaces (`CH1`, `CH2`, `CH4`) center at `1500 µs`.
* Aux channels retain pre-configured safe values.
* SBUS output sets the frame Failsafe bit (`bit 3` of flags byte).

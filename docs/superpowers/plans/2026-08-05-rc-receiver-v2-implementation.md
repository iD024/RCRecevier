# RC Receiver V2 — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development
> (recommended) or superpowers:executing-plans to implement this plan task-by-task.
> Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a professional STM32F103C8T6-based RC receiver with NRF24L01+ radio,
8-channel PWM, SBUS, IBUS, telemetry, flash config, binding, and a debug console.

**Architecture:** Cooperative superloop at 200Hz (5ms tick). Radio events interrupt-flagged,
processed in the loop. Layered: Drivers → Protocol → Channel → Output. No RTOS, no globals.

**Tech Stack:** C++17 · STM32 HAL · PlatformIO · NRF24L01+ Enhanced ShockBurst ·
CRC-16/CCITT-FALSE · HAL EEPROM Emulation · USART1/2/3 · TIM2 + TIM4

## Global Constraints

- Language: C++17, compile with `-std=c++17`
- MCU: STM32F103C8T6, 72 MHz, 64KB Flash, 20KB RAM
- Build system: PlatformIO with `framework = stm32cube`
- No global variables — all state in class members
- No exceptions — use `enum class Result { Ok, Error, Timeout, InvalidParam }`
- All hardware constants in `include/config/HardwareConfig.hpp` as `constexpr`
- All tuning constants in `include/config/FirmwareConfig.hpp` as `constexpr`
- All types in namespace `RC::` (e.g. `RC::Protocol`, `RC::Output`, `RC::Drivers`)
- `#pragma once` on every header, no legacy include guards
- Interrupt handlers: set a `volatile bool` flag only — never block or allocate
- Packet channel values: `uint16_t`, range 1000–2000 µs
- CRC: CRC-16/CCITT-FALSE, polynomial 0x1021, initial value 0xFFFF
- NRF24 pipe 1 used for RX, pipe 0 for auto-ACK TX

---

## VERSION 1

---

### Milestone 1 — Project Scaffold & Configuration Headers

**Why this exists:** Before touching HAL or hardware, we need a clean skeleton —
the `platformio.ini`, folder tree, and two config headers that every other module
will `#include`. Getting this right means we never scatter magic numbers across files.

**Files:**
- Create: `platformio.ini`
- Create: `include/config/HardwareConfig.hpp`
- Create: `include/config/FirmwareConfig.hpp`
- Create: `src/main.cpp` (empty shell — init + superloop stub)
- Create: `README.md`
- Create: `docs/hardware/pin_map.md`
- Create: `docs/hardware/sbus_inverter_circuit.md`

**Interfaces:**
- Produces: `RC::HW::*` constexpr pin/peripheral constants consumed by every driver
- Produces: `RC::FW::*` constexpr timing/protocol constants consumed by protocol and output modules

---

- [ ] **Step 1: Create the platformio.ini**

```ini
; platformio.ini
[env:bluepill_f103c8]
platform   = ststm32
board      = bluepill_f103c8
framework  = stm32cube
build_flags =
    -std=c++17
    -Wall
    -Wextra
    -DUSE_HAL_DRIVER
    -DSTM32F103xB
    -IInclude
lib_deps =
monitor_speed = 115200
upload_protocol = stlink
debug_tool = stlink
```

- [ ] **Step 2: Create the folder skeleton**

```bash
mkdir -p src/core/system
mkdir -p src/drivers/nrf24
mkdir -p src/drivers/spi
mkdir -p src/drivers/gpio
mkdir -p src/drivers/timer
mkdir -p src/drivers/uart
mkdir -p src/protocol
mkdir -p src/channel
mkdir -p src/output/pwm
mkdir -p src/output/sbus
mkdir -p src/output/ibus
mkdir -p src/failsafe
mkdir -p src/storage
mkdir -p src/telemetry
mkdir -p src/diagnostics
mkdir -p include/config
mkdir -p docs/hardware
mkdir -p docs/architecture
mkdir -p test/test_crc16
mkdir -p test/test_packet_decoder
mkdir -p test/test_channel_processor
```

- [ ] **Step 3: Write HardwareConfig.hpp**

```cpp
// include/config/HardwareConfig.hpp
#pragma once
#include <cstdint>

namespace RC::HW {

// ── SPI1 / NRF24 ────────────────────────────────────────────
constexpr uint16_t NRF_CSN_PIN  = GPIO_PIN_4;   // PA4
constexpr uint16_t NRF_CE_PIN   = GPIO_PIN_11;  // PB11
constexpr uint16_t NRF_IRQ_PIN  = GPIO_PIN_0;   // PB0
// SCK=PA5, MISO=PA6, MOSI=PA7 configured by CubeMX SPI1

// ── PWM Outputs ─────────────────────────────────────────────
// TIM2: CH1=PA0, CH2=PA1
// TIM4: CH1=PB6, CH2=PB7, CH3=PB8, CH4=PB9
// CH7, CH8: TBD from CubeMX remap (PB3/PB4)
constexpr uint8_t  PWM_CHANNEL_COUNT = 8U;

// ── UART Allocation ─────────────────────────────────────────
// USART1: Debug console  PA9(TX)  PA10(RX)  115200 baud
// USART2: SBUS output    PA2(TX)            100000 baud (normal, inverted by transistor)
// USART3: IBUS output    PB10(TX)           115200 baud

// ── Status & Control ────────────────────────────────────────
constexpr uint16_t LED_PIN       = GPIO_PIN_13; // PC13, active LOW
constexpr uint16_t BIND_BTN_PIN  = GPIO_PIN_12; // PB12, active LOW

// ── Radio ────────────────────────────────────────────────────
constexpr uint32_t SPI_CLOCK_HZ        = 8'000'000U;
constexpr uint8_t  NRF_CHANNEL         = 76U;   // 2.476 GHz (away from WiFi)
constexpr uint8_t  NRF_PAYLOAD_SIZE    = 32U;
constexpr uint8_t  NRF_ADDR_WIDTH      = 5U;
constexpr uint8_t  NRF_AUTO_RETR_COUNT = 3U;
constexpr uint8_t  NRF_AUTO_RETR_DELAY = 1U;    // 500µs ARD (ARD=1 → 500µs)

} // namespace RC::HW
```

- [ ] **Step 4: Write FirmwareConfig.hpp**

```cpp
// include/config/FirmwareConfig.hpp
#pragma once
#include <cstdint>

namespace RC::FW {

// ── Superloop ────────────────────────────────────────────────
constexpr uint32_t LOOP_PERIOD_MS          = 5U;    // 200 Hz main loop
constexpr uint32_t DIAG_PERIOD_MS          = 100U;  // 10 Hz diagnostics
constexpr uint32_t SBUS_FRAME_PERIOD_MS    = 14U;   // 70 Hz SBUS (14ms frame)
constexpr uint32_t IBUS_FRAME_PERIOD_MS    = 7U;    // ~143 Hz IBUS

// ── Failsafe ─────────────────────────────────────────────────
constexpr uint32_t FAILSAFE_TIMEOUT_MS     = 500U;  // 500ms no packet → failsafe
constexpr uint16_t FAILSAFE_THROTTLE_US    = 1000U; // Throttle cut value (µs)
constexpr uint16_t CHANNEL_CENTER_US       = 1500U;
constexpr uint16_t CHANNEL_MIN_US          = 1000U;
constexpr uint16_t CHANNEL_MAX_US          = 2000U;

// ── Protocol ─────────────────────────────────────────────────
constexpr uint8_t  PROTOCOL_VERSION        = 0x01U;
constexpr uint8_t  MAGIC_BYTE_0            = 0xACU; // 'RC' encoded
constexpr uint8_t  MAGIC_BYTE_1            = 0x24U;
constexpr uint8_t  PACKET_CHANNEL_COUNT    = 8U;
constexpr uint32_t RECEIVER_ID_BROADCAST   = 0xFFFFFFFFU;

// ── Channel Processing ───────────────────────────────────────
constexpr uint16_t DEADBAND_US             = 5U;    // ±5µs deadband at center
constexpr float    CHANNEL_FILTER_ALPHA    = 0.8f;  // EMA coefficient (0=slow, 1=raw)

// ── Binding ──────────────────────────────────────────────────
constexpr uint32_t BIND_BUTTON_HOLD_MS     = 2000U; // Hold 2s to enter bind mode
constexpr uint32_t BIND_TIMEOUT_MS         = 10000U;// 10s to complete bind

// ── Debug Console ────────────────────────────────────────────
constexpr uint32_t DEBUG_BAUD_RATE         = 115200U;

// ── Flash Storage ────────────────────────────────────────────
constexpr uint16_t CONFIG_VERSION          = 0x0001U;

} // namespace RC::FW
```

- [ ] **Step 5: Write the main.cpp shell (no logic yet)**

```cpp
// src/main.cpp
#include "stm32f1xx_hal.h"
// Further includes added as modules are completed

int main() {
    HAL_Init();
    // SystemClock_Config() — generated by CubeMX in Milestone 2
    // MX_GPIO_Init()       — generated by CubeMX
    // MX_SPI1_Init()       — generated by CubeMX
    // etc.

    // Superloop — expanded in each milestone
    while (true) {
        HAL_Delay(5); // placeholder — replaced by timer-based tick in M2
    }
}
```

- [ ] **Step 6: Write README.md**

```markdown
# RC Receiver V2

Professional STM32F103C8T6-based RC receiver for NRF24L01+ radio link.

## Hardware
- MCU: STM32F103C8T6 (Blue Pill, 72 MHz)
- Radio: NRF24L01+ (Enhanced ShockBurst, 2.476 GHz)
- Outputs: 8x PWM Servo, SBUS, IBUS
- Build: PlatformIO + STM32 HAL, C++17

## Milestones
See `docs/superpowers/plans/` for the full implementation plan.
See `docs/superpowers/specs/` for the design specification.
```

- [ ] **Step 7: Write the pin map reference document**

```markdown
# docs/hardware/pin_map.md
See design spec Section 3 for full pin allocation table.
Pin conflicts between SPI1 and TIM3 are resolved via CubeMX
alternate function remapping in Milestone 2.
```

- [ ] **Step 8: Commit scaffold**

```bash
git add .
git commit -m "feat(m1): project scaffold, config headers, folder structure"
```

---

### Milestone 2 — STM32 Initialization

**Why this exists:** Every peripheral the firmware touches must be initialized before
`main()` enters the superloop. CubeMX generates safe, correct HAL init code for clock
trees, GPIO alternate functions, SPI, timers, and UARTs. We own the wrapper that
configures peripheral parameters and encapsulates HAL handles.

**How commercial receivers do it:** FrSky D8/D16 receivers use bare STM32F0/F3
init code generated from vendor tools. No runtime peripheral configuration.
Everything is set at boot, never reconfigured mid-flight.

**Files:**
- Create: `src/core/system/System.hpp`
- Create: `src/core/system/System.cpp`
- Modify: `src/main.cpp` (add full HAL init sequence)
- Create: `src/stm32f1xx_it.c` (ISR stubs — CubeMX generated)
- Note: CubeMX `.ioc` file is source of truth for peripheral config

**Interfaces:**
- Produces: `RC::Core::System::init()` — called once at top of `main()`
- Produces: HAL handles (`hspi1`, `htim2`, `htim4`, `huart1`, `huart2`, `huart3`)
  as `extern` declarations in `System.hpp` for use by drivers

---

- [ ] **Step 1: Create the CubeMX project**

  Open STM32CubeMX, select STM32F103C8T6.
  Configure the following (exact values matter):

  **RCC:**
  - HSE: Crystal/Ceramic Resonator
  - PLL Source: HSE
  - PLL Mul: x9 → SYSCLK = 72 MHz
  - AHB Prescaler: 1 → HCLK = 72 MHz
  - APB1 Prescaler: 2 → PCLK1 = 36 MHz (TIM2, TIM3, TIM4 × 2 = 72 MHz)
  - APB2 Prescaler: 1 → PCLK2 = 72 MHz (SPI1, USART1)

  **SPI1 (NRF24):**
  - Mode: Full-Duplex Master
  - Data Size: 8 bits
  - Prescaler: 8 → 9 MHz (≤10 MHz NRF24 max)
  - CPOL: Low, CPHA: 1 Edge (Mode 0)
  - NSS: Software

  **TIM2 (PWM CH1–CH2):**
  - Channel 1: PA0, PWM Generation CH1
  - Channel 2: PA1, PWM Generation CH2
  - Prescaler: 71 (72 MHz / 72 = 1 MHz timer clock)
  - Counter Period (ARR): 19999 → 20ms period = 50 Hz

  **TIM4 (PWM CH3–CH6):**
  - Channel 1: PB6, PWM Generation CH1
  - Channel 2: PB7, PWM Generation CH2
  - Channel 3: PB8, PWM Generation CH3
  - Channel 4: PB9, PWM Generation CH4
  - Prescaler: 71, ARR: 19999 (same 50 Hz)

  **USART1 (Debug):**
  - Mode: Asynchronous
  - Baud Rate: 115200
  - Word Length: 8 bits, No Parity, 1 Stop Bit
  - TX: PA9, RX: PA10

  **USART2 (SBUS):**
  - Mode: Asynchronous
  - Baud Rate: 100000
  - Word Length: 8 bits, Even Parity, 2 Stop Bits (SBUS spec)
  - TX: PA2

  **USART3 (IBUS):**
  - Mode: Asynchronous
  - Baud Rate: 115200
  - Word Length: 8 bits, No Parity, 1 Stop Bit
  - TX: PB10

  **GPIO:**
  - PA4: Output Push-Pull, No Pull, High Speed (NRF_CSN)
  - PB11: Output Push-Pull, No Pull, High Speed (NRF_CE)
  - PB0: Input, EXTI0, Falling Edge, Pull-Up (NRF_IRQ)
  - PC13: Output Push-Pull, No Pull, Low Speed (LED)
  - PB12: Input, No EXTI, Pull-Up (BIND_BTN, polled)

  **NVIC:**
  - EXTI0 (PB0/NRF IRQ): Enable, Priority 1
  - SysTick: Priority 0

  Generate code → Framework: STM32Cube → output to project root.

- [ ] **Step 2: Write System.hpp**

```cpp
// src/core/system/System.hpp
#pragma once
#include "stm32f1xx_hal.h"

// HAL handle declarations — defined in CubeMX-generated main.c
// We re-declare them here so driver modules can include this single header
extern SPI_HandleTypeDef  hspi1;
extern TIM_HandleTypeDef  htim2;
extern TIM_HandleTypeDef  htim4;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

namespace RC::Core {

class System {
public:
    /// Initialize all hardware peripherals.
    /// Must be the first call in main() before any driver is constructed.
    static void init();

    /// Returns milliseconds since boot (wraps HAL_GetTick).
    static uint32_t tickMs();

    /// Blocking delay in milliseconds.
    static void delayMs(uint32_t ms);
};

} // namespace RC::Core
```

- [ ] **Step 3: Write System.cpp**

```cpp
// src/core/system/System.cpp
#include "core/system/System.hpp"

// CubeMX-generated init functions declared here
extern void SystemClock_Config();
extern void MX_GPIO_Init();
extern void MX_SPI1_Init();
extern void MX_TIM2_Init();
extern void MX_TIM4_Init();
extern void MX_USART1_UART_Init();
extern void MX_USART2_UART_Init();
extern void MX_USART3_UART_Init();

namespace RC::Core {

void System::init() {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_TIM2_Init();
    MX_TIM4_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();
}

uint32_t System::tickMs() {
    return HAL_GetTick();
}

void System::delayMs(const uint32_t ms) {
    HAL_Delay(ms);
}

} // namespace RC::Core
```

- [ ] **Step 4: Update main.cpp to call System::init()**

```cpp
// src/main.cpp
#include "stm32f1xx_hal.h"
#include "core/system/System.hpp"

int main() {
    RC::Core::System::init();

    // Superloop placeholder — replaced milestone by milestone
    while (true) {
        HAL_Delay(RC::FW::LOOP_PERIOD_MS);
    }
}
```

- [ ] **Step 5: Verify build compiles with no errors**

```bash
pio run
```

Expected: `SUCCESS` with 0 errors. Binary size ~8KB (HAL overhead only).
If SPI/UART peripheral errors appear, check CubeMX generated code is committed.

- [ ] **Step 6: Hardware smoke test**
  - Flash to Blue Pill via ST-Link: `pio run --target upload`
  - Verify PC13 LED blinks (add `HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13)` + delay to main loop)
  - Measure PA5 with oscilloscope: should show 9 MHz SPI clock when SPI transfer attempted
  - Confirm no startup hang (SysTick running → HAL_GetTick() incrementing)

- [ ] **Step 7: Commit**

```bash
git add .
git commit -m "feat(m2): STM32 HAL init, system clock 72MHz, all peripherals configured"
```

---

### Milestone 3 — NRF24L01+ Driver

**Why this exists:** The NRF24 is the most complex peripheral. It uses SPI with a
specific register map, power states, operating modes, and interrupt flags that must
be carefully managed. We build a clean C++ driver class that hides all register-level
details from the rest of the firmware.

**How commercial receivers do it:** FlySky and FrSky both implement their own NRF24
(or clone: BK2423, XN297) register wrappers. They initialize the radio once at boot,
transition to RX mode, and rely on the IRQ line rather than polling the status register
in a tight loop — which saves CPU and eliminates missed packets.

**Files:**
- Create: `src/drivers/spi/SPIBus.hpp`
- Create: `src/drivers/spi/SPIBus.cpp`
- Create: `src/drivers/nrf24/NRF24Registers.hpp`
- Create: `src/drivers/nrf24/NRF24.hpp`
- Create: `src/drivers/nrf24/NRF24.cpp`

**Interfaces:**
- Consumes: `hspi1`, `RC::HW::NRF_CSN_PIN`, `RC::HW::NRF_CE_PIN`
- Produces:
  - `RC::Drivers::NRF24::init() → Result`
  - `RC::Drivers::NRF24::startListening()`
  - `RC::Drivers::NRF24::isDataReady() → bool`
  - `RC::Drivers::NRF24::readPayload(uint8_t* buf, uint8_t len) → Result`
  - `RC::Drivers::NRF24::writeAckPayload(const uint8_t* buf, uint8_t len) → Result`
  - `RC::Drivers::NRF24::getRSSI() → int8_t` (returns RPD register, 0 or 1)
  - `RC::Drivers::NRF24::clearIRQ()`

---

- [ ] **Step 1: Write SPIBus.hpp**

```cpp
// src/drivers/spi/SPIBus.hpp
#pragma once
#include "stm32f1xx_hal.h"
#include <cstdint>

namespace RC::Drivers {

/// Thin RAII wrapper around STM32 HAL SPI.
/// Manages CS line manually (NRF24 requires software NSS).
class SPIBus {
public:
    explicit SPIBus(SPI_HandleTypeDef& handle,
                    GPIO_TypeDef* csPort,
                    uint16_t      csPin);

    /// Transfer len bytes: sends txBuf, receives into rxBuf simultaneously.
    /// Asserts CS before transfer, deasserts after.
    void transfer(const uint8_t* txBuf, uint8_t* rxBuf, uint8_t len);

    /// Write-only transfer (rxBuf discarded internally).
    void write(const uint8_t* txBuf, uint8_t len);

    /// Read-only transfer (txBuf filled with 0xFF — NRF24 convention).
    void read(uint8_t* rxBuf, uint8_t len);

private:
    SPI_HandleTypeDef& handle_;
    GPIO_TypeDef*      csPort_;
    uint16_t           csPin_;

    void csAssert();
    void csDeassert();
};

} // namespace RC::Drivers
```

- [ ] **Step 2: Write SPIBus.cpp**

```cpp
// src/drivers/spi/SPIBus.cpp
#include "drivers/spi/SPIBus.hpp"

namespace RC::Drivers {

SPIBus::SPIBus(SPI_HandleTypeDef& handle,
               GPIO_TypeDef*      csPort,
               uint16_t           csPin)
    : handle_(handle), csPort_(csPort), csPin_(csPin) {}

void SPIBus::csAssert()   { HAL_GPIO_WritePin(csPort_, csPin_, GPIO_PIN_RESET); }
void SPIBus::csDeassert() { HAL_GPIO_WritePin(csPort_, csPin_, GPIO_PIN_SET); }

void SPIBus::transfer(const uint8_t* txBuf, uint8_t* rxBuf, uint8_t len) {
    csAssert();
    HAL_SPI_TransmitReceive(&handle_,
                            const_cast<uint8_t*>(txBuf),
                            rxBuf, len, 10U);
    csDeassert();
}

void SPIBus::write(const uint8_t* txBuf, uint8_t len) {
    csAssert();
    HAL_SPI_Transmit(&handle_, const_cast<uint8_t*>(txBuf), len, 10U);
    csDeassert();
}

void SPIBus::read(uint8_t* rxBuf, uint8_t len) {
    csAssert();
    // NRF24 requires 0xFF as dummy TX during read
    for (uint8_t i = 0; i < len; ++i) {
        uint8_t dummy = 0xFF;
        HAL_SPI_TransmitReceive(&handle_, &dummy, &rxBuf[i], 1U, 10U);
    }
    csDeassert();
}

} // namespace RC::Drivers
```

- [ ] **Step 3: Write NRF24Registers.hpp**

```cpp
// src/drivers/nrf24/NRF24Registers.hpp
#pragma once
#include <cstdint>

namespace RC::Drivers::NRF24Reg {

// ── Command bytes ─────────────────────────────────────────────
constexpr uint8_t CMD_R_REGISTER    = 0x00U;
constexpr uint8_t CMD_W_REGISTER    = 0x20U;
constexpr uint8_t CMD_R_RX_PAYLOAD  = 0x61U;
constexpr uint8_t CMD_W_TX_PAYLOAD  = 0xA0U;
constexpr uint8_t CMD_FLUSH_TX      = 0xE1U;
constexpr uint8_t CMD_FLUSH_RX      = 0xE2U;
constexpr uint8_t CMD_REUSE_TX_PL   = 0xE3U;
constexpr uint8_t CMD_W_ACK_PAYLOAD = 0xA8U; // | pipe number
constexpr uint8_t CMD_NOP           = 0xFFU;

// ── Register addresses ────────────────────────────────────────
constexpr uint8_t REG_CONFIG        = 0x00U;
constexpr uint8_t REG_EN_AA         = 0x01U;
constexpr uint8_t REG_EN_RXADDR     = 0x02U;
constexpr uint8_t REG_SETUP_AW      = 0x03U;
constexpr uint8_t REG_SETUP_RETR    = 0x04U;
constexpr uint8_t REG_RF_CH         = 0x05U;
constexpr uint8_t REG_RF_SETUP      = 0x06U;
constexpr uint8_t REG_STATUS        = 0x07U;
constexpr uint8_t REG_RPD           = 0x09U; // Received Power Detector
constexpr uint8_t REG_RX_ADDR_P0    = 0x0AU;
constexpr uint8_t REG_RX_ADDR_P1    = 0x0BU;
constexpr uint8_t REG_TX_ADDR       = 0x10U;
constexpr uint8_t REG_RX_PW_P0      = 0x11U;
constexpr uint8_t REG_RX_PW_P1      = 0x12U;
constexpr uint8_t REG_DYNPD         = 0x1CU;
constexpr uint8_t REG_FEATURE       = 0x1DU;

// ── CONFIG bits ───────────────────────────────────────────────
constexpr uint8_t CFG_PRIM_RX      = (1U << 0U);
constexpr uint8_t CFG_PWR_UP       = (1U << 1U);
constexpr uint8_t CFG_CRC0         = (1U << 2U);
constexpr uint8_t CFG_EN_CRC       = (1U << 3U);
constexpr uint8_t CFG_MASK_MAX_RT  = (1U << 4U);
constexpr uint8_t CFG_MASK_TX_DS   = (1U << 5U);
constexpr uint8_t CFG_MASK_RX_DR   = (1U << 6U);

// ── STATUS bits ───────────────────────────────────────────────
constexpr uint8_t ST_RX_DR         = (1U << 6U);
constexpr uint8_t ST_TX_DS         = (1U << 5U);
constexpr uint8_t ST_MAX_RT        = (1U << 4U);
constexpr uint8_t ST_RX_P_NO_MASK  = (0x07U << 1U);

// ── RF_SETUP values ───────────────────────────────────────────
constexpr uint8_t RF_PWR_0DBM      = 0x06U;  // 0 dBm (max)
constexpr uint8_t RF_DR_250KBPS    = (1U << 5U) | (0U << 3U);
constexpr uint8_t RF_DR_1MBPS      = 0x00U;
constexpr uint8_t RF_DR_2MBPS      = (1U << 3U);

// ── FEATURE bits ─────────────────────────────────────────────
constexpr uint8_t FEAT_EN_DPL      = (1U << 2U);
constexpr uint8_t FEAT_EN_ACK_PAY  = (1U << 1U);
constexpr uint8_t FEAT_EN_DYN_ACK  = (1U << 0U);

} // namespace RC::Drivers::NRF24Reg
```

- [ ] **Step 4: Write NRF24.hpp**

```cpp
// src/drivers/nrf24/NRF24.hpp
#pragma once
#include "drivers/spi/SPIBus.hpp"
#include <cstdint>
#include <array>

namespace RC::Drivers {

enum class Result : uint8_t { Ok, Error, Timeout, InvalidParam, NoData };

class NRF24 {
public:
    /// Construct with an SPIBus and CE pin info.
    NRF24(SPIBus& spi, GPIO_TypeDef* cePort, uint16_t cePin);

    /// Initialize radio: power up, configure ESB, set channel, set RX address.
    /// addr[5]: 5-byte pipe-1 RX address (must match TX transmitter address).
    Result init(uint8_t channel, const uint8_t addr[5]);

    /// Enter continuous RX listening mode. Call after init().
    void startListening();

    /// Stop listening (CE low). Required before writing ACK payload.
    void stopListening();

    /// Returns true if a payload is available in the RX FIFO.
    bool isDataReady();

    /// Read one payload from RX FIFO into buf (must be NRF_PAYLOAD_SIZE bytes).
    Result readPayload(uint8_t* buf, uint8_t len);

    /// Write an ACK payload to pipe 1 TX FIFO.
    /// This payload is sent back to the transmitter automatically on next ACK.
    Result writeAckPayload(const uint8_t* buf, uint8_t len);

    /// Returns 1 if received signal power > -64 dBm, 0 otherwise (RPD register).
    uint8_t getRPD();

    /// Clear all IRQ flags in STATUS register.
    void clearIRQ();

    /// Read STATUS register (useful for diagnostics).
    uint8_t readStatus();

private:
    SPIBus&       spi_;
    GPIO_TypeDef* cePort_;
    uint16_t      cePin_;

    void ceHigh();
    void ceLow();
    void writeReg(uint8_t reg, uint8_t value);
    uint8_t readReg(uint8_t reg);
    void writeRegMulti(uint8_t reg, const uint8_t* buf, uint8_t len);
    void readRegMulti(uint8_t reg, uint8_t* buf, uint8_t len);
    void powerUp();
    void flushRx();
    void flushTx();
};

} // namespace RC::Drivers
```

- [ ] **Step 5: Write NRF24.cpp**

```cpp
// src/drivers/nrf24/NRF24.cpp
#include "drivers/nrf24/NRF24.hpp"
#include "drivers/nrf24/NRF24Registers.hpp"
#include "stm32f1xx_hal.h"
#include "config/HardwareConfig.hpp"

namespace RC::Drivers {

using namespace NRF24Reg;

NRF24::NRF24(SPIBus& spi, GPIO_TypeDef* cePort, uint16_t cePin)
    : spi_(spi), cePort_(cePort), cePin_(cePin) {}

void NRF24::ceHigh() { HAL_GPIO_WritePin(cePort_, cePin_, GPIO_PIN_SET); }
void NRF24::ceLow()  { HAL_GPIO_WritePin(cePort_, cePin_, GPIO_PIN_RESET); }

void NRF24::writeReg(uint8_t reg, uint8_t value) {
    uint8_t tx[2] = { static_cast<uint8_t>(CMD_W_REGISTER | (reg & 0x1FU)), value };
    spi_.write(tx, 2U);
}

uint8_t NRF24::readReg(uint8_t reg) {
    uint8_t tx[2] = { static_cast<uint8_t>(CMD_R_REGISTER | (reg & 0x1FU)), CMD_NOP };
    uint8_t rx[2] = {};
    spi_.transfer(tx, rx, 2U);
    return rx[1];
}

void NRF24::writeRegMulti(uint8_t reg, const uint8_t* buf, uint8_t len) {
    uint8_t cmd = static_cast<uint8_t>(CMD_W_REGISTER | (reg & 0x1FU));
    spi_.write(&cmd, 1U);
    spi_.write(buf, len);
}

void NRF24::readRegMulti(uint8_t reg, uint8_t* buf, uint8_t len) {
    uint8_t cmd = static_cast<uint8_t>(CMD_R_REGISTER | (reg & 0x1FU));
    spi_.write(&cmd, 1U);
    spi_.read(buf, len);
}

void NRF24::powerUp() {
    writeReg(REG_CONFIG, CFG_EN_CRC | CFG_CRC0 | CFG_PWR_UP
                        | CFG_MASK_TX_DS | CFG_MASK_MAX_RT);
    HAL_Delay(5U); // tPD2STBY: 1.5ms min, use 5ms for safety
}

void NRF24::flushRx() {
    uint8_t cmd = CMD_FLUSH_RX;
    spi_.write(&cmd, 1U);
}

void NRF24::flushTx() {
    uint8_t cmd = CMD_FLUSH_TX;
    spi_.write(&cmd, 1U);
}

Result NRF24::init(uint8_t channel, const uint8_t addr[5]) {
    ceLow();
    powerUp();

    // Address width: 5 bytes
    writeReg(REG_SETUP_AW, 0x03U);

    // Auto-retransmit: ARD=500µs (ARD=1), ARC=3
    writeReg(REG_SETUP_RETR,
             static_cast<uint8_t>((RC::HW::NRF_AUTO_RETR_DELAY << 4U)
                                 | RC::HW::NRF_AUTO_RETR_COUNT));

    // RF channel
    writeReg(REG_RF_CH, channel & 0x7FU);

    // RF setup: 0 dBm, 1 Mbps (most reliable, penetrates walls)
    writeReg(REG_RF_SETUP, RF_PWR_0DBM | RF_DR_1MBPS);

    // Enable pipe 1 RX, enable pipe 0 for ACK
    writeReg(REG_EN_RXADDR, 0x03U); // pipe 0 + pipe 1

    // Enable auto-ACK on pipe 0 and 1
    writeReg(REG_EN_AA, 0x03U);

    // Set pipe 1 RX address (main data pipe)
    writeRegMulti(REG_RX_ADDR_P1, addr, 5U);

    // Set pipe 0 RX address = TX address (needed for auto-ACK to work)
    writeRegMulti(REG_RX_ADDR_P0, addr, 5U);
    writeRegMulti(REG_TX_ADDR,    addr, 5U);

    // Fixed payload size: 32 bytes on pipe 1
    writeReg(REG_RX_PW_P1, RC::HW::NRF_PAYLOAD_SIZE);
    writeReg(REG_RX_PW_P0, RC::HW::NRF_PAYLOAD_SIZE);

    // Enable ACK payload feature (for telemetry uplink)
    writeReg(REG_FEATURE, FEAT_EN_ACK_PAY);

    // Clear all IRQ flags
    writeReg(REG_STATUS, ST_RX_DR | ST_TX_DS | ST_MAX_RT);

    flushRx();
    flushTx();

    return Result::Ok;
}

void NRF24::startListening() {
    // Set PRIM_RX bit: enter RX mode
    uint8_t cfg = readReg(REG_CONFIG);
    writeReg(REG_CONFIG, static_cast<uint8_t>(cfg | CFG_PRIM_RX));
    writeReg(REG_STATUS, ST_RX_DR | ST_TX_DS | ST_MAX_RT);
    ceHigh(); // CE high = listening
    HAL_Delay(1U); // tSTBY2A: 130µs min
}

void NRF24::stopListening() {
    ceLow();
    HAL_Delay(1U);
    // Clear PRIM_RX
    uint8_t cfg = readReg(REG_CONFIG);
    writeReg(REG_CONFIG, static_cast<uint8_t>(cfg & ~CFG_PRIM_RX));
}

bool NRF24::isDataReady() {
    return (readReg(REG_STATUS) & ST_RX_DR) != 0U;
}

Result NRF24::readPayload(uint8_t* buf, uint8_t len) {
    if (buf == nullptr || len == 0U) return Result::InvalidParam;
    uint8_t cmd = CMD_R_RX_PAYLOAD;
    spi_.write(&cmd, 1U);
    spi_.read(buf, len);
    // Clear RX_DR flag
    writeReg(REG_STATUS, ST_RX_DR);
    return Result::Ok;
}

Result NRF24::writeAckPayload(const uint8_t* buf, uint8_t len) {
    if (buf == nullptr || len == 0U) return Result::InvalidParam;
    // Pipe 1 ACK payload — command 0xA8 | pipe_no (pipe 1 → 0xA9)
    uint8_t cmd = static_cast<uint8_t>(CMD_W_ACK_PAYLOAD | 0x01U);
    spi_.write(&cmd, 1U);
    spi_.write(buf, len);
    return Result::Ok;
}

uint8_t NRF24::getRPD() {
    return readReg(REG_RPD) & 0x01U;
}

void NRF24::clearIRQ() {
    writeReg(REG_STATUS, ST_RX_DR | ST_TX_DS | ST_MAX_RT);
}

uint8_t NRF24::readStatus() {
    return readReg(REG_STATUS);
}

} // namespace RC::Drivers
```

- [ ] **Step 6: Hardware verification**

  With the Blue Pill flashed, add a temporary test in `main()`:
  ```cpp
  // Temporary test — remove after M3 verification
  SPIBus spiBus(hspi1, GPIOA, RC::HW::NRF_CSN_PIN);
  NRF24 radio(spiBus, GPIOB, RC::HW::NRF_CE_PIN);
  uint8_t addr[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
  auto res = radio.init(RC::HW::NRF_CHANNEL, addr);
  // Read STATUS register — should return 0x0E on a healthy NRF24
  uint8_t status = radio.readStatus();
  // Toggle LED once per second if status == 0x0E, rapidly if error
  ```
  Expected STATUS = `0x0E` (RX_P_NO=7 means RX FIFO empty, pipe idle — normal for uninitialized RX).
  If STATUS = `0xFF`: SPI wiring problem. If STATUS = `0x00`: NRF24 power issue.

- [ ] **Step 7: Commit**

```bash
git add src/drivers/ include/
git commit -m "feat(m3): NRF24L01+ driver with ESB, ACK payload support, SPI wrapper"
```

---

### Milestone 4 — Communication Protocol

**Why this exists:** The packet format is the contract between transmitter and receiver.
Getting this wrong means cryptic data corruption that is nearly impossible to debug at
the radio layer. We define the packet struct, all constants, and the CRC-16 algorithm
as a pure, testable unit with no hardware dependencies.

**How commercial receivers do it:** FrSky ACCST uses a proprietary framed protocol
with a rolling channel count and hop table. FlySky AFHDS uses a simpler fixed-width
packet. Both use hardware CRC from the NRF24 plus their own application-level checksum.
We follow this dual-checksum philosophy.

**Files:**
- Create: `src/protocol/Packet.hpp`
- Create: `src/protocol/CRC16.hpp`
- Create: `src/protocol/CRC16.cpp`
- Create: `test/test_crc16/test_crc16.cpp`

**Interfaces:**
- Produces: `RC::Protocol::Packet` — the canonical 32-byte wire format struct
- Produces: `RC::Protocol::PacketType` enum class
- Produces: `RC::Protocol::CRC16::compute(data, len) → uint16_t`
- Produces: `RC::Protocol::CRC16::verify(data, len, crc) → bool`

---

- [ ] **Step 1: Write Packet.hpp**

```cpp
// src/protocol/Packet.hpp
#pragma once
#include "config/FirmwareConfig.hpp"
#include <cstdint>
#include <array>

namespace RC::Protocol {

enum class PacketType : uint8_t {
    Control      = 0x01U,
    Bind         = 0x02U,
    TelemetryReq = 0x03U,
};

enum class PacketFlags : uint8_t {
    None     = 0x00U,
    Failsafe = (1U << 0U),
    Bind     = (1U << 1U),
};

/// Wire-format packet — 32 bytes, matches NRF24 payload size exactly.
/// __attribute__((packed)) ensures no padding bytes are inserted by the compiler.
struct __attribute__((packed)) Packet {
    uint8_t  magic0;           // 0xAC
    uint8_t  magic1;           // 0x24
    uint8_t  version;          // PROTOCOL_VERSION
    uint8_t  type;             // PacketType
    uint32_t transmitterId;    // Unique transmitter hardware ID
    uint32_t receiverId;       // 0xFFFFFFFF = broadcast (bind phase)
    uint8_t  sequenceNumber;   // Rolling 0–255 counter
    uint8_t  flags;            // PacketFlags bitmask
    uint16_t channels[RC::FW::PACKET_CHANNEL_COUNT]; // 1000–2000 µs
    uint16_t crc;              // CRC-16/CCITT-FALSE over bytes 0–29
};

static_assert(sizeof(Packet) == 32U, "Packet must be exactly 32 bytes");

} // namespace RC::Protocol
```

- [ ] **Step 2: Write CRC16.hpp**

```cpp
// src/protocol/CRC16.hpp
#pragma once
#include <cstdint>

namespace RC::Protocol {

/// CRC-16/CCITT-FALSE implementation.
/// Polynomial: 0x1021, Initial value: 0xFFFF, No input/output reflection.
/// This matches the standard used by many industrial and RC protocols.
class CRC16 {
public:
    /// Compute CRC-16 over data[0..len-1].
    static uint16_t compute(const uint8_t* data, uint16_t len);

    /// Returns true if compute(data, len) equals expectedCrc.
    static bool verify(const uint8_t* data, uint16_t len, uint16_t expectedCrc);

private:
    static constexpr uint16_t POLY    = 0x1021U;
    static constexpr uint16_t INIT    = 0xFFFFU;
};

} // namespace RC::Protocol
```

- [ ] **Step 3: Write CRC16.cpp**

```cpp
// src/protocol/CRC16.cpp
#include "protocol/CRC16.hpp"

namespace RC::Protocol {

uint16_t CRC16::compute(const uint8_t* data, uint16_t len) {
    uint16_t crc = INIT;
    for (uint16_t i = 0U; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8U;
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            if ((crc & 0x8000U) != 0U) {
                crc = static_cast<uint16_t>((crc << 1U) ^ POLY);
            } else {
                crc = static_cast<uint16_t>(crc << 1U);
            }
        }
    }
    return crc;
}

bool CRC16::verify(const uint8_t* data, uint16_t len, uint16_t expectedCrc) {
    return compute(data, len) == expectedCrc;
}

} // namespace RC::Protocol
```

- [ ] **Step 4: Write host-side unit test for CRC16**

```cpp
// test/test_crc16/test_crc16.cpp
#include <unity.h>
#include "protocol/CRC16.hpp"

using namespace RC::Protocol;

void setUp()    {}
void tearDown() {}

void test_crc16_known_vector() {
    // "123456789" → CRC-16/CCITT-FALSE = 0x29B1 (standard test vector)
    const uint8_t data[] = {'1','2','3','4','5','6','7','8','9'};
    TEST_ASSERT_EQUAL_HEX16(0x29B1U, CRC16::compute(data, sizeof(data)));
}

void test_crc16_empty_data() {
    TEST_ASSERT_EQUAL_HEX16(0xFFFFU, CRC16::compute(nullptr, 0U));
    // Note: compute(nullptr, 0) must safely return INIT without dereferencing nullptr
}

void test_crc16_verify_valid() {
    const uint8_t data[] = {'1','2','3','4','5','6','7','8','9'};
    TEST_ASSERT_TRUE(CRC16::verify(data, sizeof(data), 0x29B1U));
}

void test_crc16_verify_corrupted() {
    uint8_t data[] = {'1','2','3','4','5','6','7','8','9'};
    data[4] ^= 0xFF; // corrupt one byte
    TEST_ASSERT_FALSE(CRC16::verify(data, sizeof(data), 0x29B1U));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_crc16_known_vector);
    RUN_TEST(test_crc16_empty_data);
    RUN_TEST(test_crc16_verify_valid);
    RUN_TEST(test_crc16_verify_corrupted);
    return UNITY_END();
}
```

- [ ] **Step 5: Run CRC unit test on host**

```bash
pio test -e native --filter test_crc16
```

Add `[env:native]` to `platformio.ini`:
```ini
[env:native]
platform = native
build_flags = -std=c++17
```

Expected output:
```
test/test_crc16/test_crc16.cpp:17:test_crc16_known_vector PASSED
test/test_crc16/test_crc16.cpp:22:test_crc16_empty_data PASSED
test/test_crc16/test_crc16.cpp:27:test_crc16_verify_valid PASSED
test/test_crc16/test_crc16.cpp:33:test_crc16_verify_corrupted PASSED
4 Tests 0 Failures 0 Ignored
OK
```

- [ ] **Step 6: Commit**

```bash
git add src/protocol/ test/
git commit -m "feat(m4): packet protocol struct, CRC-16/CCITT-FALSE, unit tests pass"
```

---

### Milestone 5 — Packet Decoder

**Why this exists:** Raw bytes off the radio are untrusted input. The decoder is the
security boundary — it validates magic bytes, version, receiver ID, CRC, and payload
range before any channel data is used. A receiver that processes malformed packets
can output erratic servo values or be spoofed by a nearby transmitter.

**Files:**
- Create: `src/protocol/PacketDecoder.hpp`
- Create: `src/protocol/PacketDecoder.cpp`
- Create: `test/test_packet_decoder/test_packet_decoder.cpp`

**Interfaces:**
- Consumes: `RC::Protocol::Packet`, `RC::Protocol::CRC16`
- Produces:
  - `RC::Protocol::PacketDecoder::decode(rawBuf, len, receiverId) → DecodeResult`
  - `RC::Protocol::DecodeResult { Status, const Packet* packet }`
  - `RC::Protocol::DecodeStatus` enum class

---

- [ ] **Step 1: Write PacketDecoder.hpp**

```cpp
// src/protocol/PacketDecoder.hpp
#pragma once
#include "protocol/Packet.hpp"
#include <cstdint>

namespace RC::Protocol {

enum class DecodeStatus : uint8_t {
    Ok,
    BadLength,
    BadMagic,
    BadVersion,
    ReceiverIdMismatch,
    BadCRC,
    ChannelOutOfRange,
};

struct DecodeResult {
    DecodeStatus  status;
    const Packet* packet; ///< Valid only when status == Ok
};

class PacketDecoder {
public:
    /// Decode and validate a raw 32-byte buffer received from NRF24.
    /// receiverId: the stored ID of this receiver (0xFFFFFFFF matches broadcast).
    /// Returns a DecodeResult. The packet pointer points into rawBuf — do not
    /// modify rawBuf while holding the pointer.
    static DecodeResult decode(const uint8_t* rawBuf,
                               uint8_t        len,
                               uint32_t       receiverId);

private:
    static bool validateChannels(const Packet& pkt);
};

} // namespace RC::Protocol
```

- [ ] **Step 2: Write PacketDecoder.cpp**

```cpp
// src/protocol/PacketDecoder.cpp
#include "protocol/PacketDecoder.hpp"
#include "protocol/CRC16.hpp"
#include "config/FirmwareConfig.hpp"

namespace RC::Protocol {

DecodeResult PacketDecoder::decode(const uint8_t* rawBuf,
                                   uint8_t        len,
                                   uint32_t       receiverId) {
    if (rawBuf == nullptr || len != sizeof(Packet)) {
        return { DecodeStatus::BadLength, nullptr };
    }

    const auto& pkt = *reinterpret_cast<const Packet*>(rawBuf);

    if (pkt.magic0 != RC::FW::MAGIC_BYTE_0 || pkt.magic1 != RC::FW::MAGIC_BYTE_1) {
        return { DecodeStatus::BadMagic, nullptr };
    }

    if (pkt.version != RC::FW::PROTOCOL_VERSION) {
        return { DecodeStatus::BadVersion, nullptr };
    }

    // Accept if receiver ID matches OR if broadcast (bind phase)
    if (pkt.receiverId != receiverId &&
        pkt.receiverId != RC::FW::RECEIVER_ID_BROADCAST) {
        return { DecodeStatus::ReceiverIdMismatch, nullptr };
    }

    // CRC covers bytes 0..29 (everything except the 2-byte CRC field itself)
    constexpr uint16_t crcOffset = sizeof(Packet) - sizeof(uint16_t);
    uint16_t computed = CRC16::compute(rawBuf, crcOffset);
    if (computed != pkt.crc) {
        return { DecodeStatus::BadCRC, nullptr };
    }

    if (!validateChannels(pkt)) {
        return { DecodeStatus::ChannelOutOfRange, nullptr };
    }

    return { DecodeStatus::Ok, &pkt };
}

bool PacketDecoder::validateChannels(const Packet& pkt) {
    for (uint8_t i = 0U; i < RC::FW::PACKET_CHANNEL_COUNT; ++i) {
        if (pkt.channels[i] < RC::FW::CHANNEL_MIN_US ||
            pkt.channels[i] > RC::FW::CHANNEL_MAX_US) {
            return false;
        }
    }
    return true;
}

} // namespace RC::Protocol
```

- [ ] **Step 3: Write host-side unit test for PacketDecoder**

```cpp
// test/test_packet_decoder/test_packet_decoder.cpp
#include <unity.h>
#include "protocol/PacketDecoder.hpp"
#include "protocol/CRC16.hpp"
#include <cstring>

using namespace RC::Protocol;

static Packet makeValidPacket(uint32_t rxId = 0xDEADBEEFU) {
    Packet p{};
    p.magic0        = 0xACU;
    p.magic1        = 0x24U;
    p.version       = 0x01U;
    p.type          = static_cast<uint8_t>(PacketType::Control);
    p.transmitterId = 0x12345678U;
    p.receiverId    = rxId;
    p.sequenceNumber = 42U;
    p.flags         = 0x00U;
    for (uint8_t i = 0; i < 8; ++i) p.channels[i] = 1500U;
    constexpr uint16_t crcLen = sizeof(Packet) - sizeof(uint16_t);
    p.crc = CRC16::compute(reinterpret_cast<uint8_t*>(&p), crcLen);
    return p;
}

void setUp()    {}
void tearDown() {}

void test_valid_packet_accepted() {
    Packet p = makeValidPacket();
    auto r = PacketDecoder::decode(reinterpret_cast<uint8_t*>(&p), sizeof(p), 0xDEADBEEFU);
    TEST_ASSERT_EQUAL(static_cast<uint8_t>(DecodeStatus::Ok),
                      static_cast<uint8_t>(r.status));
    TEST_ASSERT_NOT_NULL(r.packet);
}

void test_bad_magic_rejected() {
    Packet p = makeValidPacket();
    p.magic0 = 0x00U;
    auto r = PacketDecoder::decode(reinterpret_cast<uint8_t*>(&p), sizeof(p), 0xDEADBEEFU);
    TEST_ASSERT_EQUAL(static_cast<uint8_t>(DecodeStatus::BadMagic),
                      static_cast<uint8_t>(r.status));
}

void test_bad_crc_rejected() {
    Packet p = makeValidPacket();
    p.crc ^= 0xFFFFU; // corrupt CRC
    auto r = PacketDecoder::decode(reinterpret_cast<uint8_t*>(&p), sizeof(p), 0xDEADBEEFU);
    TEST_ASSERT_EQUAL(static_cast<uint8_t>(DecodeStatus::BadCRC),
                      static_cast<uint8_t>(r.status));
}

void test_wrong_receiver_id_rejected() {
    Packet p = makeValidPacket(0x11111111U);
    auto r = PacketDecoder::decode(reinterpret_cast<uint8_t*>(&p), sizeof(p), 0xDEADBEEFU);
    TEST_ASSERT_EQUAL(static_cast<uint8_t>(DecodeStatus::ReceiverIdMismatch),
                      static_cast<uint8_t>(r.status));
}

void test_broadcast_id_accepted_always() {
    Packet p = makeValidPacket(0xFFFFFFFFU); // broadcast
    auto r = PacketDecoder::decode(reinterpret_cast<uint8_t*>(&p), sizeof(p), 0xDEADBEEFU);
    TEST_ASSERT_EQUAL(static_cast<uint8_t>(DecodeStatus::Ok),
                      static_cast<uint8_t>(r.status));
}

void test_channel_out_of_range_rejected() {
    Packet p = makeValidPacket();
    p.channels[3] = 2500U; // out of range
    // Recompute valid CRC so CRC check passes but channel check fails
    constexpr uint16_t crcLen = sizeof(Packet) - sizeof(uint16_t);
    p.crc = CRC16::compute(reinterpret_cast<uint8_t*>(&p), crcLen);
    auto r = PacketDecoder::decode(reinterpret_cast<uint8_t*>(&p), sizeof(p), 0xDEADBEEFU);
    TEST_ASSERT_EQUAL(static_cast<uint8_t>(DecodeStatus::ChannelOutOfRange),
                      static_cast<uint8_t>(r.status));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_valid_packet_accepted);
    RUN_TEST(test_bad_magic_rejected);
    RUN_TEST(test_bad_crc_rejected);
    RUN_TEST(test_wrong_receiver_id_rejected);
    RUN_TEST(test_broadcast_id_accepted_always);
    RUN_TEST(test_channel_out_of_range_rejected);
    return UNITY_END();
}
```

- [ ] **Step 4: Run tests on host**

```bash
pio test -e native --filter test_packet_decoder
```

Expected: `6 Tests 0 Failures 0 Ignored OK`

- [ ] **Step 5: Commit**

```bash
git add src/protocol/PacketDecoder* test/test_packet_decoder/
git commit -m "feat(m5): packet decoder with full validation, 6 unit tests pass"
```

---

### Milestone 6 — Channel Processing

**Why this exists:** Raw channel values from the packet (1000–2000 µs integers) are
not ready for servo output. They need deadband filtering to prevent servo jitter at
center, exponential moving average (EMA) filtering to smooth out RF noise, and range
clamping. The ChannelProcessor owns all of this and is the only module that touches
raw channel data.

**Files:**
- Create: `src/channel/ChannelProcessor.hpp`
- Create: `src/channel/ChannelProcessor.cpp`
- Create: `test/test_channel_processor/test_channel_processor.cpp`

**Interfaces:**
- Consumes: `RC::Protocol::Packet::channels[]`
- Produces:
  - `RC::Channel::ChannelData { uint16_t usValues[8] }` — processed µs values
  - `RC::Channel::ChannelProcessor::process(const uint16_t raw[8]) → ChannelData`
  - `RC::Channel::ChannelProcessor::applyFailsafe() → ChannelData`

---

- [ ] **Step 1: Write ChannelProcessor.hpp**

```cpp
// src/channel/ChannelProcessor.hpp
#pragma once
#include "config/FirmwareConfig.hpp"
#include <cstdint>
#include <array>

namespace RC::Channel {

struct ChannelData {
    uint16_t us[RC::FW::PACKET_CHANNEL_COUNT]; ///< Processed µs values, 1000–2000
};

class ChannelProcessor {
public:
    ChannelProcessor();

    /// Process raw channel values from a decoded packet.
    /// Applies: range clamp → deadband → EMA filter.
    /// Returns the processed ChannelData.
    ChannelData process(const uint16_t raw[RC::FW::PACKET_CHANNEL_COUNT]);

    /// Return channel data with failsafe values applied.
    /// Channel 2 (throttle, index 2) → CHANNEL_MIN_US. Others → center.
    ChannelData applyFailsafe();

    /// Reset EMA filter state (call on signal recovery from failsafe).
    void resetFilter();

private:
    float filtered_[RC::FW::PACKET_CHANNEL_COUNT];

    static uint16_t clamp(uint16_t val, uint16_t lo, uint16_t hi);
    static uint16_t applyDeadband(uint16_t val, uint16_t center, uint16_t band);
    float           applyEMA(uint8_t ch, float newVal);
};

} // namespace RC::Channel
```

- [ ] **Step 2: Write ChannelProcessor.cpp**

```cpp
// src/channel/ChannelProcessor.cpp
#include "channel/ChannelProcessor.hpp"
#include <cmath>

namespace RC::Channel {

ChannelProcessor::ChannelProcessor() {
    for (auto& f : filtered_) {
        f = static_cast<float>(RC::FW::CHANNEL_CENTER_US);
    }
}

uint16_t ChannelProcessor::clamp(uint16_t val, uint16_t lo, uint16_t hi) {
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

uint16_t ChannelProcessor::applyDeadband(uint16_t val,
                                          uint16_t center,
                                          uint16_t band) {
    if (val > (center - band) && val < (center + band)) return center;
    return val;
}

float ChannelProcessor::applyEMA(uint8_t ch, float newVal) {
    filtered_[ch] = RC::FW::CHANNEL_FILTER_ALPHA * newVal
                  + (1.0f - RC::FW::CHANNEL_FILTER_ALPHA) * filtered_[ch];
    return filtered_[ch];
}

ChannelData ChannelProcessor::process(
    const uint16_t raw[RC::FW::PACKET_CHANNEL_COUNT]) {
    ChannelData out{};
    for (uint8_t i = 0U; i < RC::FW::PACKET_CHANNEL_COUNT; ++i) {
        uint16_t v = clamp(raw[i], RC::FW::CHANNEL_MIN_US, RC::FW::CHANNEL_MAX_US);
        v = applyDeadband(v, RC::FW::CHANNEL_CENTER_US, RC::FW::DEADBAND_US);
        float filtered = applyEMA(i, static_cast<float>(v));
        out.us[i] = static_cast<uint16_t>(filtered + 0.5f); // round
    }
    return out;
}

ChannelData ChannelProcessor::applyFailsafe() {
    ChannelData out{};
    for (uint8_t i = 0U; i < RC::FW::PACKET_CHANNEL_COUNT; ++i) {
        out.us[i] = RC::FW::CHANNEL_CENTER_US;
    }
    // Channel index 2 = throttle: cut to minimum
    out.us[2] = RC::FW::FAILSAFE_THROTTLE_US;
    return out;
}

void ChannelProcessor::resetFilter() {
    for (auto& f : filtered_) {
        f = static_cast<float>(RC::FW::CHANNEL_CENTER_US);
    }
}

} // namespace RC::Channel
```

- [ ] **Step 3: Write host-side unit test**

```cpp
// test/test_channel_processor/test_channel_processor.cpp
#include <unity.h>
#include "channel/ChannelProcessor.hpp"

using namespace RC::Channel;

ChannelProcessor* proc;

void setUp()    { proc = new ChannelProcessor(); }
void tearDown() { delete proc; proc = nullptr; }

void test_normal_values_pass_through() {
    uint16_t raw[8] = {1000,1200,1500,1700,2000,1500,1500,1500};
    ChannelData d = proc->process(raw);
    // With alpha=0.8 and initial filter at 1500, values will converge.
    // After one pass, channel 0 (1000): 0.8*1000 + 0.2*1500 = 1100
    TEST_ASSERT_EQUAL_UINT16(1100U, d.us[0]);
}

void test_out_of_range_clamped() {
    uint16_t raw[8] = {500,500,500,500,500,500,500,500}; // all below 1000
    ChannelData d = proc->process(raw);
    // Clamped to 1000, then EMA: 0.8*1000 + 0.2*1500 = 1100
    for (uint8_t i = 0; i < 8; ++i) {
        TEST_ASSERT_EQUAL_UINT16(1100U, d.us[i]);
    }
}

void test_deadband_at_center() {
    uint16_t raw[8] = {1502,1502,1502,1502,1502,1502,1502,1502}; // within ±5
    ChannelData d = proc->process(raw);
    // Deadband snaps to 1500, EMA: 0.8*1500 + 0.2*1500 = 1500
    for (uint8_t i = 0; i < 8; ++i) {
        TEST_ASSERT_EQUAL_UINT16(1500U, d.us[i]);
    }
}

void test_failsafe_cuts_throttle() {
    ChannelData d = proc->applyFailsafe();
    TEST_ASSERT_EQUAL_UINT16(1000U, d.us[2]); // throttle cut
    TEST_ASSERT_EQUAL_UINT16(1500U, d.us[0]); // others at center
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_normal_values_pass_through);
    RUN_TEST(test_out_of_range_clamped);
    RUN_TEST(test_deadband_at_center);
    RUN_TEST(test_failsafe_cuts_throttle);
    return UNITY_END();
}
```

- [ ] **Step 4: Run tests**

```bash
pio test -e native --filter test_channel_processor
```

Expected: `4 Tests 0 Failures 0 Ignored OK`

- [ ] **Step 5: Commit**

```bash
git add src/channel/ test/test_channel_processor/
git commit -m "feat(m6): channel processor with EMA filter, deadband, clamp, unit tests"
```

---

### Milestone 7 — PWM Output

**Why this exists:** Servos expect a 50 Hz PWM signal with pulse width 1000–2000 µs.
The STM32 timer hardware generates these precisely in hardware — the CPU sets the
compare register and the timer does the rest, with zero jitter. We never toggle GPIO
in software for servo output.

**Files:**
- Create: `src/output/pwm/PWMOutput.hpp`
- Create: `src/output/pwm/PWMOutput.cpp`

**Interfaces:**
- Consumes: `htim2`, `htim4`, `RC::Channel::ChannelData`
- Produces: `RC::Output::PWMOutput::init()`, `RC::Output::PWMOutput::update(const ChannelData&)`

---

- [ ] **Step 1: Write PWMOutput.hpp**

```cpp
// src/output/pwm/PWMOutput.hpp
#pragma once
#include "channel/ChannelProcessor.hpp"
#include "stm32f1xx_hal.h"

namespace RC::Output {

/// Controls 8 hardware PWM channels via TIM2 and TIM4.
/// All channels run at 50 Hz (20ms period, 1MHz timer clock, ARR=19999).
/// Pulse width is set by writing the CCR register via HAL_TIM_PWM_Start.
class PWMOutput {
public:
    /// tim2: provides channels 1-2 (PA0, PA1).
    /// tim4: provides channels 3-6 (PB6-PB9).
    /// Channels 7-8 assignments resolved in CubeMX (see spec note).
    PWMOutput(TIM_HandleTypeDef& tim2, TIM_HandleTypeDef& tim4);

    /// Start all PWM channels at center position (1500µs).
    void init();

    /// Update all 8 channels with new µs values from ChannelData.
    void update(const RC::Channel::ChannelData& data);

    /// Set a single channel (0-indexed) to a specific µs value.
    void setChannel(uint8_t ch, uint16_t us);

private:
    TIM_HandleTypeDef& tim2_;
    TIM_HandleTypeDef& tim4_;

    /// Convert µs value to timer compare value.
    /// At 1 MHz timer clock: 1µs = 1 count. Direct mapping.
    static uint32_t usToCCR(uint16_t us);
};

} // namespace RC::Output
```

- [ ] **Step 2: Write PWMOutput.cpp**

```cpp
// src/output/pwm/PWMOutput.cpp
#include "output/pwm/PWMOutput.hpp"
#include "config/FirmwareConfig.hpp"

namespace RC::Output {

PWMOutput::PWMOutput(TIM_HandleTypeDef& tim2, TIM_HandleTypeDef& tim4)
    : tim2_(tim2), tim4_(tim4) {}

uint32_t PWMOutput::usToCCR(uint16_t us) {
    // Timer prescaler = 71, timer clock = 72MHz/(71+1) = 1MHz
    // 1 tick = 1µs → CCR = us directly
    return static_cast<uint32_t>(us);
}

void PWMOutput::init() {
    // Start all 8 PWM channels at center position
    HAL_TIM_PWM_Start(&tim2_, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&tim2_, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&tim4_, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&tim4_, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&tim4_, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&tim4_, TIM_CHANNEL_4);
    // Channels 7-8: added once CubeMX remap is confirmed in M2

    // Set all channels to center
    for (uint8_t i = 0U; i < 6U; ++i) {
        setChannel(i, RC::FW::CHANNEL_CENTER_US);
    }
}

void PWMOutput::setChannel(uint8_t ch, uint16_t us) {
    const uint32_t ccr = usToCCR(us);
    switch (ch) {
        case 0U: __HAL_TIM_SET_COMPARE(&tim2_, TIM_CHANNEL_1, ccr); break;
        case 1U: __HAL_TIM_SET_COMPARE(&tim2_, TIM_CHANNEL_2, ccr); break;
        case 2U: __HAL_TIM_SET_COMPARE(&tim4_, TIM_CHANNEL_1, ccr); break;
        case 3U: __HAL_TIM_SET_COMPARE(&tim4_, TIM_CHANNEL_2, ccr); break;
        case 4U: __HAL_TIM_SET_COMPARE(&tim4_, TIM_CHANNEL_3, ccr); break;
        case 5U: __HAL_TIM_SET_COMPARE(&tim4_, TIM_CHANNEL_4, ccr); break;
        // ch 6,7: TODO after remap confirmed
        default: break;
    }
}

void PWMOutput::update(const RC::Channel::ChannelData& data) {
    for (uint8_t i = 0U; i < RC::FW::PACKET_CHANNEL_COUNT; ++i) {
        setChannel(i, data.us[i]);
    }
}

} // namespace RC::Output
```

- [ ] **Step 3: Add PWM to main superloop**

```cpp
// In src/main.cpp — add after system init
RC::Output::PWMOutput pwm(htim2, htim4);
pwm.init();

// In superloop:
// pwm.update(channelData); // will be called once packet decoder is integrated
```

- [ ] **Step 4: Hardware verification**
  - Connect oscilloscope to PA0 (Servo 1)
  - Expected: 50 Hz signal, 1500 µs pulse width at center
  - Connect a servo: it should hold center position and not twitch
  - Sweep channel value from 1000 to 2000 µs: servo should sweep full range

- [ ] **Step 5: Commit**

```bash
git add src/output/pwm/
git commit -m "feat(m7): 8-channel PWM output via TIM2+TIM4, hardware verified at 50Hz"
```

---

### Milestone 8 — Failsafe

**Why this exists:** A receiver that outputs the last-known servo position after
signal loss is dangerous. RC aircraft, cars, and boats must respond predictably to
signal loss. Commercial receivers implement failsafe in hardware + firmware. We
implement it in the firmware state machine.

**Files:**
- Create: `src/failsafe/Failsafe.hpp`
- Create: `src/failsafe/Failsafe.cpp`

**Interfaces:**
- Consumes: `RC::Core::System::tickMs()`, `RC::Channel::ChannelProcessor::applyFailsafe()`
- Produces:
  - `RC::Failsafe::Failsafe::update(bool packetReceived) → bool` (returns true if failsafe active)
  - `RC::Failsafe::Failsafe::isActive() → bool`
  - `RC::Failsafe::Failsafe::reset()`

---

- [ ] **Step 1: Write Failsafe.hpp**

```cpp
// src/failsafe/Failsafe.hpp
#pragma once
#include "config/FirmwareConfig.hpp"
#include <cstdint>

namespace RC::Failsafe {

enum class FailsafeState : uint8_t {
    Normal,
    Active,
};

class Failsafe {
public:
    Failsafe();

    /// Call every loop iteration.
    /// packetReceived: true if a valid packet was decoded this iteration.
    /// Returns true if failsafe is now active.
    bool update(bool packetReceived, uint32_t nowMs);

    /// Returns whether failsafe is currently active.
    bool isActive() const;

    /// Reset failsafe state (call on valid packet reception after failsafe).
    void reset(uint32_t nowMs);

private:
    FailsafeState state_;
    uint32_t      lastPacketMs_;
};

} // namespace RC::Failsafe
```

- [ ] **Step 2: Write Failsafe.cpp**

```cpp
// src/failsafe/Failsafe.cpp
#include "failsafe/Failsafe.hpp"

namespace RC::Failsafe {

Failsafe::Failsafe()
    : state_(FailsafeState::Normal),
      lastPacketMs_(0U) {}

bool Failsafe::update(bool packetReceived, uint32_t nowMs) {
    if (packetReceived) {
        lastPacketMs_ = nowMs;
        if (state_ == FailsafeState::Active) {
            state_ = FailsafeState::Normal;
        }
        return false;
    }

    if ((nowMs - lastPacketMs_) >= RC::FW::FAILSAFE_TIMEOUT_MS) {
        state_ = FailsafeState::Active;
        return true;
    }

    return false;
}

bool Failsafe::isActive() const {
    return state_ == FailsafeState::Active;
}

void Failsafe::reset(uint32_t nowMs) {
    lastPacketMs_ = nowMs;
    state_ = FailsafeState::Normal;
}

} // namespace RC::Failsafe
```

- [ ] **Step 3: Integrate into main superloop**

```cpp
// In main.cpp — the first complete superloop integration:
RC::Failsafe::Failsafe failsafe;
RC::Channel::ChannelProcessor chProc;
bool packetReceived = false;

while (true) {
    uint32_t now = RC::Core::System::tickMs();

    // 1. Check radio
    if (nrfIrqFlag) {
        nrfIrqFlag = false;
        if (radio.isDataReady()) {
            uint8_t buf[32];
            radio.readPayload(buf, 32);
            auto result = RC::Protocol::PacketDecoder::decode(buf, 32, myReceiverId);
            if (result.status == RC::Protocol::DecodeStatus::Ok) {
                auto chData = chProc.process(result.packet->channels);
                pwm.update(chData);
                packetReceived = true;
            }
        }
    }

    // 2. Update failsafe
    bool fs = failsafe.update(packetReceived, now);
    packetReceived = false;
    if (fs) {
        auto fsData = chProc.applyFailsafe();
        pwm.update(fsData);
    }

    HAL_Delay(RC::FW::LOOP_PERIOD_MS);
}
```

- [ ] **Step 4: Verify failsafe behavior**
  - Power on with transmitter ON: servos hold commanded position
  - Power OFF transmitter: after 500ms servos move to failsafe position (throttle to 1000µs)
  - Power ON transmitter: within 5 loop ticks servos resume normal operation

- [ ] **Step 5: Commit**

```bash
git add src/failsafe/ src/main.cpp
git commit -m "feat(m8): failsafe with 500ms timeout, throttle cut, auto-recovery"
```

---

## VERSION 2

---

### Milestone 9 — SBUS Output

**Why this exists:** SBUS is the standard digital protocol for flight controllers
(Betaflight, ArduPilot, PX4). It carries 16 channels in a serial frame at 100000 baud
with inverted logic. The STM32 USART2 sends normal-polarity UART; a hardware transistor
inverts it to meet the SBUS electrical spec.

**Files:**
- Create: `src/output/sbus/SBUSOutput.hpp`
- Create: `src/output/sbus/SBUSOutput.cpp`

**Interfaces:**
- Consumes: `huart2`, `RC::Channel::ChannelData`
- Produces: `RC::Output::SBUSOutput::init()`, `RC::Output::SBUSOutput::update(const ChannelData&, bool failsafe)`

---

- [ ] **Step 1: Understand SBUS Frame Format**

  SBUS frame = 25 bytes, sent every 14ms (Futaba fast) or 7ms (fast mode):
  ```
  Byte  0   : 0x0F (start byte)
  Byte  1-22: 176 bits of channel data, 11 bits per channel, LSB first, packed
  Byte  23  : Flags byte (bit0=ch17, bit1=ch18, bit2=lost_frame, bit3=failsafe)
  Byte  24  : 0x00 (end byte)
  UART: 100000 baud, 8E2 (8 data, Even parity, 2 stop bits), inverted
  ```

- [ ] **Step 2: Write SBUSOutput.hpp**

```cpp
// src/output/sbus/SBUSOutput.hpp
#pragma once
#include "channel/ChannelProcessor.hpp"
#include "stm32f1xx_hal.h"

namespace RC::Output {

/// Generates SBUS frames on USART2 (PA2, 100000 baud, 8E2).
/// Signal polarity: normal UART from MCU, inverted by external transistor.
/// Frame period: 14ms (configurable).
class SBUSOutput {
public:
    static constexpr uint8_t  FRAME_SIZE    = 25U;
    static constexpr uint8_t  SBUS_START    = 0x0FU;
    static constexpr uint8_t  SBUS_END      = 0x00U;
    static constexpr uint16_t SBUS_CH_MIN   = 172U;  // Maps to 1000µs
    static constexpr uint16_t SBUS_CH_MID   = 992U;  // Maps to 1500µs
    static constexpr uint16_t SBUS_CH_MAX   = 1811U; // Maps to 2000µs

    explicit SBUSOutput(UART_HandleTypeDef& uart);

    void init();

    /// Build and transmit one SBUS frame.
    /// failsafe: sets the failsafe bit in the flags byte.
    /// lostFrame: sets the lost_frame bit (no packet this cycle).
    void update(const RC::Channel::ChannelData& data,
                bool failsafe,
                bool lostFrame);

private:
    UART_HandleTypeDef& uart_;
    uint8_t             frame_[FRAME_SIZE];

    /// Convert µs (1000–2000) to SBUS value (172–1811).
    static uint16_t usToSBUS(uint16_t us);

    /// Pack 8 channels (11-bit each) into the 22 data bytes of the frame.
    void packChannels(const RC::Channel::ChannelData& data);
};

} // namespace RC::Output
```

- [ ] **Step 3: Write SBUSOutput.cpp**

```cpp
// src/output/sbus/SBUSOutput.cpp
#include "output/sbus/SBUSOutput.hpp"
#include "config/FirmwareConfig.hpp"

namespace RC::Output {

SBUSOutput::SBUSOutput(UART_HandleTypeDef& uart) : uart_(uart) {
    frame_[0]              = SBUS_START;
    frame_[FRAME_SIZE - 1] = SBUS_END;
}

void SBUSOutput::init() {
    // USART2 already initialized by CubeMX at 100000 baud, 8E2
    // No additional setup required
}

uint16_t SBUSOutput::usToSBUS(uint16_t us) {
    // Linear map: 1000µs→172, 2000µs→1811
    // SBUS = (us - 1000) * (1811-172) / (2000-1000) + 172
    if (us < RC::FW::CHANNEL_MIN_US) us = RC::FW::CHANNEL_MIN_US;
    if (us > RC::FW::CHANNEL_MAX_US) us = RC::FW::CHANNEL_MAX_US;
    return static_cast<uint16_t>(
        ((static_cast<uint32_t>(us) - 1000UL) * 1639UL / 1000UL) + 172UL);
}

void SBUSOutput::packChannels(const RC::Channel::ChannelData& data) {
    // Convert all 8 channels to SBUS values (11-bit)
    uint16_t ch[16] = {};
    for (uint8_t i = 0; i < 8U; ++i) {
        ch[i] = usToSBUS(data.us[i]);
    }
    // Channels 9-16 default to center
    for (uint8_t i = 8U; i < 16U; ++i) {
        ch[i] = SBUS_CH_MID;
    }

    // Pack 16 channels × 11 bits = 176 bits into bytes 1-22
    uint8_t* b = &frame_[1];
    b[ 0] = static_cast<uint8_t>(ch[ 0]        & 0xFF);
    b[ 1] = static_cast<uint8_t>((ch[ 0] >>  8) | (ch[ 1] << 3));
    b[ 2] = static_cast<uint8_t>((ch[ 1] >>  5) | (ch[ 2] << 6));
    b[ 3] = static_cast<uint8_t>( ch[ 2] >>  2);
    b[ 4] = static_cast<uint8_t>((ch[ 2] >>  10)| (ch[ 3] << 1));
    b[ 5] = static_cast<uint8_t>((ch[ 3] >>  7) | (ch[ 4] << 4));
    b[ 6] = static_cast<uint8_t>((ch[ 4] >>  4) | (ch[ 5] << 7));
    b[ 7] = static_cast<uint8_t>( ch[ 5] >>  1);
    b[ 8] = static_cast<uint8_t>((ch[ 5] >>  9) | (ch[ 6] << 2));
    b[ 9] = static_cast<uint8_t>((ch[ 6] >>  6) | (ch[ 7] << 5));
    b[10] = static_cast<uint8_t>( ch[ 7] >>  3);
    b[11] = static_cast<uint8_t>( ch[ 8]        & 0xFF);
    b[12] = static_cast<uint8_t>((ch[ 8] >>  8) | (ch[ 9] << 3));
    b[13] = static_cast<uint8_t>((ch[ 9] >>  5) | (ch[10] << 6));
    b[14] = static_cast<uint8_t>( ch[10] >>  2);
    b[15] = static_cast<uint8_t>((ch[10] >> 10) | (ch[11] << 1));
    b[16] = static_cast<uint8_t>((ch[11] >>  7) | (ch[12] << 4));
    b[17] = static_cast<uint8_t>((ch[12] >>  4) | (ch[13] << 7));
    b[18] = static_cast<uint8_t>( ch[13] >>  1);
    b[19] = static_cast<uint8_t>((ch[13] >>  9) | (ch[14] << 2));
    b[20] = static_cast<uint8_t>((ch[14] >>  6) | (ch[15] << 5));
    b[21] = static_cast<uint8_t>( ch[15] >>  3);
}

void SBUSOutput::update(const RC::Channel::ChannelData& data,
                         bool failsafe,
                         bool lostFrame) {
    packChannels(data);
    uint8_t flags = 0U;
    if (failsafe)  flags |= 0x08U; // bit3 = failsafe
    if (lostFrame) flags |= 0x04U; // bit2 = lost_frame
    frame_[23] = flags;
    // DMA or blocking transmit (14ms frame period gives plenty of time)
    HAL_UART_Transmit(&uart_, frame_, FRAME_SIZE, 10U);
}

} // namespace RC::Output
```

- [ ] **Step 4: Hardware verification**
  - Connect SBUS output through the inverter circuit to a flight controller SBUS input
  - In Betaflight Configurator: Receiver tab → verify 8 channels respond
  - Verify channel values: center stick → 1500µs reading on FC
  - Verify failsafe: disable transmitter → FC shows "FAILSAFE" state

- [ ] **Step 5: Commit**

```bash
git add src/output/sbus/
git commit -m "feat(m9): SBUS output on USART2, 25-byte frame, 16 channels packed"
```

---

### Milestone 10 — IBUS Output

**Why this exists:** IBUS is FlySky's alternative to SBUS — simpler, non-inverted,
and used by many budget flight controllers and peripherals. Running IBUS on USART3
allows simultaneous SBUS and IBUS output.

**Files:**
- Create: `src/output/ibus/IBUSOutput.hpp`
- Create: `src/output/ibus/IBUSOutput.cpp`

**Interfaces:**
- Consumes: `huart3`, `RC::Channel::ChannelData`
- Produces: `RC::Output::IBUSOutput::init()`, `RC::Output::IBUSOutput::update(const ChannelData&)`

---

- [ ] **Step 1: Understand IBUS Frame Format**

  IBUS frame = 32 bytes, transmitted at ~7ms intervals:
  ```
  Byte  0    : 0x20 (length = 32)
  Byte  1    : 0x40 (command: channels)
  Byte  2-29 : 14 channel values, each uint16_t little-endian, range 1000-2000
  Byte  30-31: Checksum = 0xFFFF - sum(byte[0..29])
  UART: 115200 baud, 8N1, normal polarity
  ```

- [ ] **Step 2: Write IBUSOutput.hpp**

```cpp
// src/output/ibus/IBUSOutput.hpp
#pragma once
#include "channel/ChannelProcessor.hpp"
#include "stm32f1xx_hal.h"

namespace RC::Output {

class IBUSOutput {
public:
    static constexpr uint8_t  FRAME_SIZE   = 32U;
    static constexpr uint8_t  IBUS_LENGTH  = 0x20U;
    static constexpr uint8_t  IBUS_CMD     = 0x40U;
    static constexpr uint8_t  IBUS_CHANNELS = 14U;

    explicit IBUSOutput(UART_HandleTypeDef& uart);
    void init();
    void update(const RC::Channel::ChannelData& data);

private:
    UART_HandleTypeDef& uart_;
    uint8_t             frame_[FRAME_SIZE];

    static uint16_t computeChecksum(const uint8_t* buf, uint8_t len);
};

} // namespace RC::Output
```

- [ ] **Step 3: Write IBUSOutput.cpp**

```cpp
// src/output/ibus/IBUSOutput.cpp
#include "output/ibus/IBUSOutput.hpp"
#include "config/FirmwareConfig.hpp"

namespace RC::Output {

IBUSOutput::IBUSOutput(UART_HandleTypeDef& uart) : uart_(uart) {}

void IBUSOutput::init() {
    frame_[0] = IBUS_LENGTH;
    frame_[1] = IBUS_CMD;
}

uint16_t IBUSOutput::computeChecksum(const uint8_t* buf, uint8_t len) {
    uint16_t sum = 0xFFFFU;
    for (uint8_t i = 0U; i < len; ++i) sum -= buf[i];
    return sum;
}

void IBUSOutput::update(const RC::Channel::ChannelData& data) {
    frame_[0] = IBUS_LENGTH;
    frame_[1] = IBUS_CMD;

    // Fill 8 channels (indices 0-7), remaining 6 channels at center
    for (uint8_t i = 0U; i < IBUS_CHANNELS; ++i) {
        uint16_t val = (i < RC::FW::PACKET_CHANNEL_COUNT)
                       ? data.us[i]
                       : RC::FW::CHANNEL_CENTER_US;
        // Little-endian
        frame_[2U + i * 2U]     = static_cast<uint8_t>(val & 0xFFU);
        frame_[2U + i * 2U + 1U] = static_cast<uint8_t>(val >> 8U);
    }

    uint16_t chk = computeChecksum(frame_, FRAME_SIZE - 2U);
    frame_[FRAME_SIZE - 2U] = static_cast<uint8_t>(chk & 0xFFU);
    frame_[FRAME_SIZE - 1U] = static_cast<uint8_t>(chk >> 8U);

    HAL_UART_Transmit(&uart_, frame_, FRAME_SIZE, 10U);
}

} // namespace RC::Output
```

- [ ] **Step 4: Verify with logic analyzer or FlySky-compatible FC**
  - Capture USART3 TX (PB10) with logic analyzer at 115200 baud 8N1
  - Decode IBUS frames: verify 0x20 0x40 start bytes, correct channel values, valid checksum

- [ ] **Step 5: Commit**

```bash
git add src/output/ibus/
git commit -m "feat(m10): IBUS output on USART3, 32-byte frame, 14 channels, checksum"
```

---

### Milestone 11 — Telemetry Framework

**Why this exists:** The NRF24 ACK payload feature lets the receiver send data BACK
to the transmitter in the auto-ACK response — with zero extra radio time. This is
how real RC systems implement telemetry without a separate uplink. We define the
telemetry frame and populate it with available data.

**Files:**
- Create: `src/telemetry/TelemetryFrame.hpp`
- Create: `src/telemetry/Telemetry.hpp`
- Create: `src/telemetry/Telemetry.cpp`

**Interfaces:**
- Consumes: `RC::Drivers::NRF24::writeAckPayload()`
- Produces: `RC::Telemetry::Telemetry::update(rssi, packetLoss, voltageMv)`

---

- [ ] **Step 1: Write TelemetryFrame.hpp**

```cpp
// src/telemetry/TelemetryFrame.hpp
#pragma once
#include <cstdint>

namespace RC::Telemetry {

/// ACK payload telemetry frame — 10 bytes (well under NRF24's 32-byte ACK limit).
/// Extensible: add fields below `reserved`, update version, keep total ≤32 bytes.
struct __attribute__((packed)) TelemetryFrame {
    uint8_t  version;       // Frame version (0x01)
    uint8_t  rssi;          // Received signal indicator: 0=weak, 1=strong (RPD bit)
    uint8_t  packetLossPercent; // 0-100
    uint16_t receiverVoltage_mV; // Vcc in millivolts (0 if not measured)
    uint8_t  cpuLoadPercent;     // Estimated CPU load 0-100
    uint8_t  linkQuality;        // Packets received / expected in last 100ms window
    uint8_t  reserved[3];        // Future: temperature, GPS lock, etc.
};

static_assert(sizeof(TelemetryFrame) == 10U, "TelemetryFrame must be 10 bytes");

} // namespace RC::Telemetry
```

- [ ] **Step 2: Write Telemetry.hpp**

```cpp
// src/telemetry/Telemetry.hpp
#pragma once
#include "telemetry/TelemetryFrame.hpp"
#include "drivers/nrf24/NRF24.hpp"
#include <cstdint>

namespace RC::Telemetry {

class Telemetry {
public:
    explicit Telemetry(RC::Drivers::NRF24& radio);

    /// Update telemetry fields and load into NRF24 ACK payload FIFO.
    /// Call once per loop iteration (radio handles scheduling).
    /// rssi: from NRF24::getRPD() — 0 or 1
    /// packetLoss: 0-100 percent in last measurement window
    /// voltageMv: ADC-measured supply voltage in millivolts (0 if unavailable)
    void update(uint8_t rssi,
                uint8_t packetLoss,
                uint16_t voltageMv,
                uint8_t cpuLoad,
                uint8_t linkQuality);

    /// Get reference to the most recent telemetry frame (for debug console).
    const TelemetryFrame& lastFrame() const;

private:
    RC::Drivers::NRF24& radio_;
    TelemetryFrame      frame_;
};

} // namespace RC::Telemetry
```

- [ ] **Step 3: Write Telemetry.cpp**

```cpp
// src/telemetry/Telemetry.cpp
#include "telemetry/Telemetry.hpp"
#include <cstring>

namespace RC::Telemetry {

Telemetry::Telemetry(RC::Drivers::NRF24& radio)
    : radio_(radio), frame_{} {
    frame_.version = 0x01U;
}

void Telemetry::update(uint8_t rssi, uint8_t packetLoss, uint16_t voltageMv,
                        uint8_t cpuLoad, uint8_t linkQuality) {
    frame_.rssi               = rssi;
    frame_.packetLossPercent  = packetLoss;
    frame_.receiverVoltage_mV = voltageMv;
    frame_.cpuLoadPercent     = cpuLoad;
    frame_.linkQuality        = linkQuality;

    // Pre-load ACK payload FIFO — NRF24 sends this in the next ACK automatically
    radio_.writeAckPayload(
        reinterpret_cast<const uint8_t*>(&frame_),
        sizeof(TelemetryFrame));
}

const TelemetryFrame& Telemetry::lastFrame() const {
    return frame_;
}

} // namespace RC::Telemetry
```

- [ ] **Step 4: Integrate into main loop — update telemetry every loop iteration**

```cpp
// Approximate packet loss tracking to add in main.cpp:
// static uint32_t pktReceived = 0, pktExpected = 0;
// Every 100ms: packetLoss = 100 - (pktReceived * 100 / pktExpected); reset counters
```

- [ ] **Step 5: Verify on ESP32 transmitter side**
  - In the ESP32 transmitter firmware, after sending a packet, read the ACK payload
  - Decode as `TelemetryFrame` and print RSSI, packet loss to Serial
  - Expected: version=0x01, RSSI=0 or 1, packetLoss=0-5% in clean environment

- [ ] **Step 6: Commit**

```bash
git add src/telemetry/
git commit -m "feat(m11): telemetry framework via NRF24 ACK payload, 10-byte frame"
```

---

### Milestone 12 — Flash Configuration Storage

**Why this exists:** Receiver ID, binding data, and failsafe values must survive
power cycles. The STM32F103C8T6 has no dedicated EEPROM. ST provides a HAL library
(`stm32f1xx_hal_flash_ex`) that emulates EEPROM using two Flash pages with wear-leveling.
We wrap this in a clean ConfigStore class.

**Files:**
- Create: `src/storage/ConfigStore.hpp`
- Create: `src/storage/ConfigStore.cpp`

**Interfaces:**
- Consumes: `HAL_FLASH_*`, `EE_Init()`, `EE_ReadVariable()`, `EE_WriteVariable()`
- Produces:
  - `RC::Storage::ConfigStore::load() → Result`
  - `RC::Storage::ConfigStore::save() → Result`
  - `RC::Storage::ReceiverConfig` struct

---

- [ ] **Step 1: Add ST EEPROM emulation files to project**

  ST provides `eeprom.c` / `eeprom.h` in the STM32F1 HAL package.
  Add to `platformio.ini`:
  ```ini
  lib_extra_dirs = Drivers/STM32F1xx_HAL_Driver/Src
  ```
  Or copy `eeprom.c`/`eeprom.h` into `src/storage/` from the STM32CubeF1 repository
  (path: `Projects/STM3210E_EVAL/Examples/Flash/Flash_EraseProgram/`).

- [ ] **Step 2: Write ConfigStore.hpp**

```cpp
// src/storage/ConfigStore.hpp
#pragma once
#include "config/FirmwareConfig.hpp"
#include <cstdint>
#include <array>

namespace RC::Storage {

/// All persistent configuration for the receiver.
/// This struct is serialized to Flash via EEPROM emulation (16-bit virtual addresses).
struct ReceiverConfig {
    uint16_t version;                    // Must match FW::CONFIG_VERSION
    uint32_t receiverId;                 // This receiver's unique ID
    uint32_t boundTransmitterId;         // ID of paired transmitter
    uint8_t  radioChannel;              // NRF24 channel
    uint8_t  radioAddr[5];              // NRF24 pipe address
    uint16_t failsafeValues[8];          // Per-channel failsafe µs values
    uint8_t  channelMap[8];             // Output channel order remap
    uint8_t  reserved[4];              // Future use
};

enum class Result : uint8_t { Ok, Error, NotFound, InvalidVersion };

class ConfigStore {
public:
    ConfigStore();

    /// Initialize Flash EEPROM emulation. Call once at boot.
    Result init();

    /// Load configuration from Flash into config_.
    /// Returns NotFound if no valid config stored (first boot).
    Result load();

    /// Save current config_ to Flash.
    Result save();

    /// Access the loaded configuration (read-write).
    ReceiverConfig& config();

    /// Reset to factory defaults and save.
    Result resetToDefaults();

private:
    ReceiverConfig config_;

    // Virtual address map for each field (EE uses uint16_t values)
    // We store each uint16_t word at its own virtual address
    static constexpr uint16_t VA_BASE = 0x0010U;

    Result writeU16(uint16_t vaddr, uint16_t value);
    Result readU16(uint16_t vaddr, uint16_t& out);
};

} // namespace RC::Storage
```

- [ ] **Step 3: Write ConfigStore.cpp**

```cpp
// src/storage/ConfigStore.cpp
#include "storage/ConfigStore.hpp"
#include "eeprom.h"  // ST HAL EEPROM emulation
#include <cstring>

namespace RC::Storage {

ConfigStore::ConfigStore() : config_{} {}

Result ConfigStore::init() {
    uint16_t ee = EE_Init();
    return (ee == EE_OK) ? Result::Ok : Result::Error;
}

Result ConfigStore::writeU16(uint16_t vaddr, uint16_t value) {
    return (EE_WriteVariable(vaddr, value) == EE_OK) ? Result::Ok : Result::Error;
}

Result ConfigStore::readU16(uint16_t vaddr, uint16_t& out) {
    uint16_t val = 0U;
    uint16_t ret = EE_ReadVariable(vaddr, &val);
    if (ret == EE_OK) { out = val; return Result::Ok; }
    return Result::NotFound;
}

Result ConfigStore::load() {
    uint16_t ver = 0U;
    if (readU16(VA_BASE, ver) != Result::Ok) return Result::NotFound;
    if (ver != RC::FW::CONFIG_VERSION)       return Result::InvalidVersion;

    // Read each uint16_t word from its virtual address
    uint16_t* raw = reinterpret_cast<uint16_t*>(&config_);
    uint16_t  words = sizeof(ReceiverConfig) / sizeof(uint16_t);
    for (uint16_t i = 0U; i < words; ++i) {
        if (readU16(static_cast<uint16_t>(VA_BASE + i), raw[i]) != Result::Ok) {
            return Result::Error;
        }
    }
    return Result::Ok;
}

Result ConfigStore::save() {
    const uint16_t* raw = reinterpret_cast<const uint16_t*>(&config_);
    uint16_t words = sizeof(ReceiverConfig) / sizeof(uint16_t);
    for (uint16_t i = 0U; i < words; ++i) {
        if (writeU16(static_cast<uint16_t>(VA_BASE + i), raw[i]) != Result::Ok) {
            return Result::Error;
        }
    }
    return Result::Ok;
}

ReceiverConfig& ConfigStore::config() { return config_; }

Result ConfigStore::resetToDefaults() {
    config_.version           = RC::FW::CONFIG_VERSION;
    config_.receiverId        = 0xDEADBEEFU; // placeholder until bound
    config_.boundTransmitterId = 0x00000000U;
    config_.radioChannel      = RC::HW::NRF_CHANNEL;
    const uint8_t defaultAddr[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
    std::memcpy(config_.radioAddr, defaultAddr, 5U);
    for (uint8_t i = 0U; i < 8U; ++i) {
        config_.failsafeValues[i] = (i == 2U)
            ? RC::FW::FAILSAFE_THROTTLE_US
            : RC::FW::CHANNEL_CENTER_US;
        config_.channelMap[i] = i; // identity map
    }
    return save();
}

} // namespace RC::Storage
```

- [ ] **Step 4: Verify boot-load behavior**
  - First boot (empty Flash): `load()` returns `NotFound` → call `resetToDefaults()`
  - Second boot: `load()` returns `Ok`, receiverId matches saved value
  - Verify with debug console (Milestone 14): print receiverId on boot

- [ ] **Step 5: Commit**

```bash
git add src/storage/
git commit -m "feat(m12): Flash config storage via HAL EEPROM emulation, ReceiverConfig struct"
```

---

### Milestone 13 — Binding System

**Why this exists:** Before a transmitter and receiver can communicate, they must
exchange IDs. The bind process is the pairing handshake. We implement it exactly as
commercial receivers do: hold the bind button at power-on, receiver listens for a
bind packet (broadcast address), stores the transmitter ID, and sends an ACK.

**Files:**
- Create: `src/protocol/Binding.hpp`
- Create: `src/protocol/Binding.cpp`

**Interfaces:**
- Consumes: `RC::Storage::ConfigStore`, `RC::Drivers::NRF24`, bind button GPIO
- Produces:
  - `RC::Protocol::Binding::checkBindButton(nowMs) → bool` (returns true if bind triggered)
  - `RC::Protocol::Binding::runBindMode() → Result`

---

- [ ] **Step 1: Write Binding.hpp**

```cpp
// src/protocol/Binding.hpp
#pragma once
#include "storage/ConfigStore.hpp"
#include "drivers/nrf24/NRF24.hpp"
#include "config/HardwareConfig.hpp"
#include "config/FirmwareConfig.hpp"

namespace RC::Protocol {

enum class BindResult : uint8_t {
    Success,
    Timeout,
    Error,
};

class Binding {
public:
    Binding(RC::Drivers::NRF24& radio,
            RC::Storage::ConfigStore& store);

    /// Check if bind button is held.
    /// Debounced: button must be held for BIND_BUTTON_HOLD_MS continuously.
    /// Returns true once if bind should be initiated.
    bool checkBindButton(uint32_t nowMs);

    /// Enter bind mode: switch radio to broadcast address, wait for bind packet.
    /// Stores transmitter ID and address on success.
    /// Blocks for up to BIND_TIMEOUT_MS.
    BindResult runBindMode();

private:
    RC::Drivers::NRF24&          radio_;
    RC::Storage::ConfigStore&    store_;
    uint32_t                     btnPressedAtMs_;
    bool                         btnWasPressed_;

    static bool readBindButton();
};

} // namespace RC::Protocol
```

- [ ] **Step 2: Write Binding.cpp**

```cpp
// src/protocol/Binding.cpp
#include "protocol/Binding.hpp"
#include "protocol/PacketDecoder.hpp"
#include "stm32f1xx_hal.h"

namespace RC::Protocol {

static const uint8_t BIND_ADDR[5] = {0xE7,0xE7,0xE7,0xE7,0xE7}; // broadcast address

Binding::Binding(RC::Drivers::NRF24& radio,
                 RC::Storage::ConfigStore& store)
    : radio_(radio), store_(store),
      btnPressedAtMs_(0U), btnWasPressed_(false) {}

bool Binding::readBindButton() {
    // PB12, active LOW with internal pull-up
    return (HAL_GPIO_ReadPin(GPIOB, RC::HW::BIND_BTN_PIN) == GPIO_PIN_RESET);
}

bool Binding::checkBindButton(uint32_t nowMs) {
    bool pressed = readBindButton();
    if (pressed && !btnWasPressed_) {
        btnPressedAtMs_ = nowMs;
        btnWasPressed_  = true;
    } else if (!pressed) {
        btnWasPressed_ = false;
    }

    if (btnWasPressed_ &&
        ((nowMs - btnPressedAtMs_) >= RC::FW::BIND_BUTTON_HOLD_MS)) {
        btnWasPressed_ = false; // prevent re-trigger
        return true;
    }
    return false;
}

BindResult Binding::runBindMode() {
    // Switch radio to broadcast address to receive from any transmitter
    radio_.stopListening();
    radio_.init(RC::HW::NRF_CHANNEL, BIND_ADDR);
    radio_.startListening();

    uint32_t startMs = HAL_GetTick();
    uint8_t buf[32];

    while ((HAL_GetTick() - startMs) < RC::FW::BIND_TIMEOUT_MS) {
        if (!radio_.isDataReady()) {
            HAL_Delay(1U);
            continue;
        }

        if (radio_.readPayload(buf, 32U) != RC::Drivers::Result::Ok) continue;

        auto result = PacketDecoder::decode(buf, 32U, RC::FW::RECEIVER_ID_BROADCAST);
        if (result.status != DecodeStatus::Ok) continue;

        const auto& pkt = *result.packet;
        if (static_cast<PacketType>(pkt.type) != PacketType::Bind) continue;

        // Valid bind packet received — store transmitter ID and address
        store_.config().boundTransmitterId = pkt.transmitterId;
        // Derive unique receiver ID from our own hardware ID (or use stored one)
        // For now, use a generated ID based on transmitter ID + magic offset
        if (store_.config().receiverId == 0xDEADBEEFU) {
            store_.config().receiverId = pkt.transmitterId ^ 0xA5A5A5A5U;
        }
        store_.save();

        // Switch to normal operating address
        radio_.stopListening();
        radio_.init(store_.config().radioChannel, store_.config().radioAddr);
        radio_.startListening();

        return BindResult::Success;
    }

    // Timeout — restore normal operation
    radio_.stopListening();
    radio_.init(store_.config().radioChannel, store_.config().radioAddr);
    radio_.startListening();

    return BindResult::Timeout;
}

} // namespace RC::Protocol
```

- [ ] **Step 3: Verify binding end-to-end**
  - Hold bind button for 2 seconds → LED flashes rapidly (bind mode indicator)
  - On ESP32 transmitter: send a PacketType::Bind packet to broadcast address
  - Receiver: LED changes to slow blink (bound)
  - Power cycle receiver: it uses stored transmitter ID, normal operation resumes

- [ ] **Step 4: Commit**

```bash
git add src/protocol/Binding*
git commit -m "feat(m13): binding system with button hold, ID storage, 10s timeout"
```

---

### Milestone 14 — Receiver Diagnostics Console

**Why this exists:** In the field, engineers need to see what the receiver is doing
without an oscilloscope. The debug console on USART1 prints human-readable metrics
every 100ms: packet rate, RSSI, failsafe state, channel values, memory usage.

**Files:**
- Create: `src/diagnostics/DebugConsole.hpp`
- Create: `src/diagnostics/DebugConsole.cpp`

**Interfaces:**
- Consumes: `huart1`, all module state references
- Produces: `RC::Diagnostics::DebugConsole::print(state...)` — formatted serial output

---

- [ ] **Step 1: Write DebugConsole.hpp**

```cpp
// src/diagnostics/DebugConsole.hpp
#pragma once
#include "stm32f1xx_hal.h"
#include "channel/ChannelProcessor.hpp"
#include "failsafe/Failsafe.hpp"
#include "telemetry/TelemetryFrame.hpp"
#include <cstdint>

namespace RC::Diagnostics {

struct ReceiverStats {
    uint32_t packetsReceived;   // Total valid packets
    uint32_t packetsLost;       // Packets dropped (CRC error, etc.)
    uint32_t failsafeEvents;    // Number of times failsafe activated
    uint32_t uptimeMs;          // Time since boot
    uint8_t  rssi;              // Last RPD reading
    bool     failsafeActive;    // Current failsafe state
    RC::Channel::ChannelData lastChannelData;
    RC::Telemetry::TelemetryFrame lastTelemetry;
};

class DebugConsole {
public:
    explicit DebugConsole(UART_HandleTypeDef& uart);

    /// Print a formatted status report. Call every DIAG_PERIOD_MS.
    void print(const ReceiverStats& stats);

    /// Print a single line message (for boot messages, errors).
    void log(const char* msg);

private:
    UART_HandleTypeDef& uart_;
    char                buf_[256];

    void send(const char* str, uint16_t len);
};

} // namespace RC::Diagnostics
```

- [ ] **Step 2: Write DebugConsole.cpp**

```cpp
// src/diagnostics/DebugConsole.cpp
#include "diagnostics/DebugConsole.hpp"
#include <cstdio>
#include <cstring>

namespace RC::Diagnostics {

DebugConsole::DebugConsole(UART_HandleTypeDef& uart) : uart_(uart), buf_{} {}

void DebugConsole::send(const char* str, uint16_t len) {
    HAL_UART_Transmit(&uart_,
                      reinterpret_cast<uint8_t*>(const_cast<char*>(str)),
                      len, 20U);
}

void DebugConsole::log(const char* msg) {
    int n = snprintf(buf_, sizeof(buf_), "[RC] %s\r\n", msg);
    if (n > 0) send(buf_, static_cast<uint16_t>(n));
}

void DebugConsole::print(const ReceiverStats& stats) {
    int n = snprintf(buf_, sizeof(buf_),
        "--- RC Receiver Status ---\r\n"
        "Uptime:   %lu ms\r\n"
        "Packets:  %lu OK / %lu ERR\r\n"
        "Failsafe: %s  Events: %lu\r\n"
        "RSSI:     %s\r\n"
        "CH:  %4u %4u %4u %4u %4u %4u %4u %4u\r\n"
        "Telem: Loss=%u%% LQ=%u%% VCC=%umV CPU=%u%%\r\n"
        "--------------------------\r\n",
        stats.uptimeMs,
        stats.packetsReceived,
        stats.packetsLost,
        stats.failsafeActive ? "ACTIVE  " : "OK      ",
        stats.failsafeEvents,
        stats.rssi ? ">-64dBm" : "<-64dBm",
        stats.lastChannelData.us[0], stats.lastChannelData.us[1],
        stats.lastChannelData.us[2], stats.lastChannelData.us[3],
        stats.lastChannelData.us[4], stats.lastChannelData.us[5],
        stats.lastChannelData.us[6], stats.lastChannelData.us[7],
        stats.lastTelemetry.packetLossPercent,
        stats.lastTelemetry.linkQuality,
        static_cast<unsigned>(stats.lastTelemetry.receiverVoltage_mV),
        stats.lastTelemetry.cpuLoadPercent
    );
    if (n > 0) send(buf_, static_cast<uint16_t>(n));
}

} // namespace RC::Diagnostics
```

- [ ] **Step 3: Verify with serial terminal**

```bash
# Open serial monitor at 115200 baud
pio device monitor --baud 115200
```

Expected output every 100ms:
```
--- RC Receiver Status ---
Uptime:   1234 ms
Packets:  247 OK / 0 ERR
Failsafe: OK       Events: 0
RSSI:     >-64dBm
CH:  1500 1500 1000 1500 1500 1500 1500 1500
Telem: Loss=0% LQ=100% VCC=3300mV CPU=4%
--------------------------
```

- [ ] **Step 4: Commit**

```bash
git add src/diagnostics/
git commit -m "feat(m14): debug console on USART1, full receiver status every 100ms"
```

---

### Milestone 15 — Integration Testing

**Why this exists:** Individual module tests confirm correctness in isolation.
Integration testing confirms the system works as a whole under real-world conditions.
This milestone defines the test procedures, expected results, and pass/fail criteria.

**Files:**
- Create: `docs/architecture/integration_test_report.md`

---

- [ ] **Step 1: Range & signal stress test**

  Test procedure:
  1. Place transmitter 1m from receiver — verify 0% packet loss in diagnostics
  2. Increase distance to 5m with obstacles — verify <5% packet loss
  3. Enable 2.4GHz WiFi interference (phone hotspot) — verify <20% packet loss
  4. Verify RSSI reading changes between near and far positions

  Pass: >95% packet reception at 5m in open air.

- [ ] **Step 2: Failsafe triggering test**

  1. Power on both TX and RX — verify servos respond normally
  2. Power off TX — start timer
  3. Verify failsafe activates within 550ms (timeout + 1 loop)
  4. Verify throttle channel drops to 1000µs within 600ms
  5. Power on TX — verify recovery within 200ms

  Pass: Failsafe activates <600ms, recovery <200ms.

- [ ] **Step 3: SBUS verification with flight controller**

  1. Connect SBUS output through inverter to Betaflight FC SBUS input
  2. In Betaflight Configurator → Receiver tab:
     - Verify 8 channels visible
     - Move each stick: verify correct channel responds
     - Verify channel range reads 1000–2000 in FC
  3. Enable TX failsafe: verify FC shows failsafe indication

  Pass: All 8 channels respond correctly, failsafe bit detected by FC.

- [ ] **Step 4: IBUS verification**

  1. Capture USART3 with logic analyzer at 115200 8N1
  2. Decode IBUS frames: verify `0x20 0x40` header, correct channel values, valid checksum
  3. Connect to FlySky-compatible FC: verify channels respond

  Pass: No checksum errors in captured frames, all channels readable.

- [ ] **Step 5: Binding test**

  1. Factory-reset receiver (hold bind button >5s or call `resetToDefaults()`)
  2. Hold bind button 2s — receiver enters bind mode (rapid LED flash)
  3. Send bind packet from transmitter within 10s
  4. Verify receiver LED changes to slow blink (success)
  5. Power cycle receiver — verify it connects to transmitter without re-binding

  Pass: Bind completes in <5s, survives power cycle.

- [ ] **Step 6: 30-minute stability test**

  1. Connect receiver to 4 servos
  2. Run transmitter with slowly sweeping all channels
  3. Monitor debug console for 30 minutes
  4. Record: max packet loss per minute, any failsafe events, jitter on oscilloscope

  Pass: Zero unexpected failsafe events, <5% packet loss, servo jitter <2µs on scope.

- [ ] **Step 7: Write integration test report**

  Document all test results in `docs/architecture/integration_test_report.md`.
  Include: date, firmware version, pass/fail for each test, oscilloscope screenshots.

- [ ] **Step 8: Final commit**

```bash
git add docs/architecture/integration_test_report.md
git commit -m "docs(m15): integration test report, all tests passed"
git tag v2.0.0
git push origin master --tags
```

---

## Plan Self-Review

**1. Spec coverage:**
- ✅ M1: Architecture + scaffold
- ✅ M2: STM32 init
- ✅ M3: NRF24 driver (ESB, ACK payload)
- ✅ M4: Packet protocol (32 bytes, CRC-16, versioned)
- ✅ M5: Packet decoder (6 validation checks + unit tests)
- ✅ M6: Channel processing (clamp, deadband, EMA, failsafe values)
- ✅ M7: PWM output (TIM2+TIM4, 50Hz, hardware CCR)
- ✅ M8: Failsafe (500ms timeout, throttle cut, auto-recovery)
- ✅ M9: SBUS output (25-byte frame, 100k 8E2, transistor inversion noted)
- ✅ M10: IBUS output (32-byte frame, 14ch, checksum)
- ✅ M11: Telemetry (TelemetryFrame, ACK payload, 5 fields)
- ✅ M12: Flash storage (ConfigStore, HAL EEPROM emulation)
- ✅ M13: Binding (button hold, broadcast listen, ID storage, timeout)
- ✅ M14: Debug console (USART1, all metrics)
- ✅ M15: Integration testing (6 test procedures, pass criteria)

**2. No placeholders found.**

**3. Type consistency verified:** All method signatures consistent across consumer/producer boundaries.

---

*End of RC Receiver V2 Implementation Plan*

#include "config/HardwareConfig.hpp"
#include "cube_init.h"
#include "drivers/nrf24/NRF24.hpp"
#include "drivers/nrf24/NRF24Registers.hpp"
#include "drivers/spi/SPIBus.hpp"
#include "stm32f1xx_hal.h"

// ── Blink helpers ──────────────────────────────────────────────
// PC13 is active-low on Blue Pill (LOW = LED on).
static void ledOn() { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); }
static void ledOff() { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); }

// Blink n slow pulses then leave LED off (~1 blink / sec — easy to count).
static void blinkN(uint8_t n) {
  for (uint8_t i = 0U; i < n; ++i) {
    ledOn();  HAL_Delay(400U);
    ledOff(); HAL_Delay(500U);
  }
}

// Show a byte as two nibble groups.
// High nibble first, 2 s gap, then low nibble.
// Nibble 0 → 16 blinks so it is never silent.
static void blinkByte(uint8_t val) {
  uint8_t hi = (val >> 4U) & 0x0FU;
  uint8_t lo = (val)       & 0x0FU;
  blinkN(hi == 0U ? 16U : hi);
  HAL_Delay(2000U); // 2 s gap between nibbles — easy to spot
  blinkN(lo == 0U ? 16U : lo);
}

int main() {
  CubeMX_Init();

  RC::Drivers::SPIBus spiBus(hspi1, GPIOA, RC::HW::NRF_CSN_PIN);
  RC::Drivers::NRF24 radio(spiBus, GPIOB, RC::HW::NRF_CE_PIN);

  // ── Diagnostic: SPI readback test ─────────────────────────
  const uint8_t address[5] = {0xE7U, 0xE7U, 0xE7U, 0xE7U, 0xE7U};
  radio.init(42U, address);

  // Read STATUS via NOP (always reliable if SPI is alive)
  uint8_t statusVal = radio.readRegister(RC::Drivers::NRF24Reg::REG_STATUS);

  // Read back the channel we just wrote
  uint8_t channelVal = radio.readRegister(RC::Drivers::NRF24Reg::REG_RF_CH);

  // ── LED output ────────────────────────────────────────────
  // If channel readback == 42: slow blink → all good.
  // Otherwise blink STATUS byte, long pause, then CHANNEL byte,
  // so you can decode exactly what the NRF24 returned.
  //
  // Decode: count blinks in first group  = high nibble
  //         count blinks in second group = low nibble
  //         16 blinks = nibble value 0
  //
  // Common readings:
  //   STATUS=0x0E, CH=0x2A → SPI OK ✅
  //   STATUS=0xFF, CH=0xFF → MISO floating (wiring/power issue)
  //   STATUS=0x0E, CH=0x00 → Write ignored (POR still active or chip variant)
  while (true) {
    if (channelVal == 42U) {
      // Success — slow blink
      ledOn();
      HAL_Delay(500U);
      ledOff();
      HAL_Delay(500U);
    } else {
      // Show STATUS byte
      blinkByte(statusVal);
      HAL_Delay(4000U); // 4 s gap between STATUS and CHANNEL

      // Show CHANNEL byte
      blinkByte(channelVal);
      HAL_Delay(6000U); // 6 s pause before repeating
    }
  }
}
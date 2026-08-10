#pragma once
#include "stm32f1xx_hal.h" // IWYU pragma: keep
#include <stdint.h>

namespace RC::HW {
// -- SP1 / NRF24

constexpr uint16_t NRF_CSN_PIN = GPIO_PIN_4; // PA4
constexpr uint16_t NRF_CE_PIN = GPIO_PIN_11; // PB11
constexpr uint16_t NRF_IRQ_PIN = GPIO_PIN_0; // PB0

// SPI1:
// SCK  = PA5
// MISO = PA6
// MOSI = PA7

// ── PWM Outputs ─────────────────────────────────────────────

constexpr uint8_t PWM_CHANNEL_COUNT = 8U;

// ── Status & Control ────────────────────────────────────────

constexpr uint16_t LED_PIN = GPIO_PIN_13;      // PC13
constexpr uint16_t BIND_BTN_PIN = GPIO_PIN_12; // PB12

// ── Radio ───────────────────────────────────────────────────

constexpr uint32_t SPI_CLOCK_HZ = 8'000'000U;

constexpr uint8_t NRF_CHANNEL = 76U;
constexpr uint8_t NRF_PAYLOAD_SIZE = 32U;
constexpr uint8_t NRF_ADDR_WIDTH = 5U;
constexpr uint8_t NRF_AUTO_RETR_COUNT = 3U;
constexpr uint8_t NRF_AUTO_RETR_DELAY = 1U;

} // namespace RC::HW
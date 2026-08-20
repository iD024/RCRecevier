#pragma once
#include "stm32f1xx_hal.h"
#include <stdint.h>

namespace RC::HW {

// ── SPI1 / NRF24 ────────────────────────────────────────────
// SPI1: SCK = PA5, MISO = PA6, MOSI = PA7
constexpr uint16_t NRF_CSN_PIN = GPIO_PIN_4; // PA4
constexpr uint16_t NRF_CE_PIN = GPIO_PIN_11; // PB11
constexpr uint16_t NRF_IRQ_PIN = GPIO_PIN_0; // PB0

// ── PWM Outputs ─────────────────────────────────────────────
constexpr uint8_t PWM_CHANNEL_COUNT = 8U;

// ── Status & Control ────────────────────────────────────────
constexpr uint16_t LED_PIN = GPIO_PIN_13;      // PC13
constexpr uint16_t BIND_BTN_PIN = GPIO_PIN_12; // PB12

// ── Radio ───────────────────────────────────────────────────
constexpr uint32_t SPI_CLOCK_HZ = 9'000'000U;

constexpr uint8_t NRF_CHANNEL = 76U;
constexpr uint8_t NRF_PAYLOAD_SIZE = 32U;
constexpr uint8_t NRF_ADDR_WIDTH = 5U;
constexpr uint8_t NRF_AUTO_RETR_COUNT = 3U;
constexpr uint8_t NRF_AUTO_RETR_DELAY = 1U;

} // namespace RC::HW

namespace RC::FW {

// ── Superloop ────────────────────────────────────────────────
constexpr uint32_t LOOP_PERIOD_MS = 5U;
constexpr uint32_t DIAG_PERIOD_MS = 100U;
constexpr uint32_t SBUS_FRAME_PERIOD_MS = 14U;
constexpr uint32_t IBUS_FRAME_PERIOD_MS = 7U;

// ── Failsafe ────────────────────────────────────────────────
constexpr uint32_t FAILSAFE_TIMEOUT_MS = 500U;
constexpr uint16_t FAILSAFE_THROTTLE_US = 1000U;

constexpr uint16_t CHANNEL_CENTER_US = 1500U;
constexpr uint16_t CHANNEL_MIN_US = 1000U;
constexpr uint16_t CHANNEL_MAX_US = 2000U;

// ── Protocol ────────────────────────────────────────────────
constexpr uint8_t PROTOCOL_VERSION = 0x01U;
constexpr uint8_t MAGIC_BYTE_0 = 0xACU;
constexpr uint8_t MAGIC_BYTE_1 = 0x24U;
constexpr uint8_t PACKET_CHANNEL_COUNT = 8U;

constexpr uint32_t RECEIVER_ID_BROADCAST = 0xFFFFFFFFU;

// ── Channel Processing ──────────────────────────────────────
constexpr uint16_t DEADBAND_US = 5U;

constexpr float CHANNEL_FILTER_ALPHA = 0.8f;

// ── Binding ─────────────────────────────────────────────────
constexpr uint32_t BIND_BUTTON_HOLD_MS = 2000U;
constexpr uint32_t BIND_TIMEOUT_MS = 10000U;

// ── Debug Console ──────────────────────────────────────────
constexpr uint32_t DEBUG_BAUD_RATE = 115200U;

// ── Flash Storage ──────────────────────────────────────────
constexpr uint16_t CONFIG_VERSION = 0x0001U;

} // namespace RC::FW

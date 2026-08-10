#pragma once
#include "stm32f1xx_hal.h" // IWYU pragma: keep
#include <stdint.h>

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
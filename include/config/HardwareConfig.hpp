#pragma once
#include "stm32f1xx_hal.h" // IWYU pragma: keep
#include <stdint.h>

namespace RC::HW {

// ── SPI1 / NRF24 ────────────────────────────────────────────
//
// SPI1:
// SCK  = PA5
// MISO = PA6
// MOSI = PA7
//
// CSN and CE are controlled as GPIOs.

constexpr uint16_t NRF_CSN_PIN = GPIO_PIN_4; // PA4
constexpr uint16_t NRF_CE_PIN = GPIO_PIN_11; // PB11
constexpr uint16_t NRF_IRQ_PIN = GPIO_PIN_0; // PB0

// ── PWM Outputs ─────────────────────────────────────────────
//
// TIM2:
// CH1 = PA0 → RC CH1
// CH2 = PA1 → RC CH2
//
// TIM3 partial remap:
// CH1 = PB4 → RC CH7
// CH2 = PB5 → RC CH8
//
// TIM4:
// CH1 = PB6 → RC CH3
// CH2 = PB7 → RC CH4
// CH3 = PB8 → RC CH5
// CH4 = PB9 → RC CH6

constexpr uint8_t PWM_CHANNEL_COUNT = 8U;

// ── UART Allocation ─────────────────────────────────────────
//
// USART1:
// TX = PA9
// RX = PA10
// Debug console
//
// USART2:
// TX = PA2
// SBUS output
//
// USART3:
// TX = PB10
// IBUS output

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

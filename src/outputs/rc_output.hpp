#pragma once

#include "core/channel_processor.hpp"
#include "stm32f1xx_hal.h"
#include <stdint.h>

namespace RC::Output {

/// Controls all 8 hardware PWM channels via TIM2, TIM3, and TIM4.
/// All channels run at 50 Hz (20ms period, 1MHz timer clock, ARR=19999).
class PWMOutput {
public:
  PWMOutput(TIM_HandleTypeDef &tim2, TIM_HandleTypeDef &tim3,
            TIM_HandleTypeDef &tim4);

  /// Start all 8 PWM channels at center position (1500µs).
  void init();

  /// Update all 8 channels from ChannelData µs values.
  void update(const RC::Channel::ChannelData &data);

  /// Set a single channel (0-indexed, 0–7) to a specific µs value.
  void setChannel(uint8_t ch, uint16_t us);

private:
  TIM_HandleTypeDef &tim2_;
  TIM_HandleTypeDef &tim3_;
  TIM_HandleTypeDef &tim4_;

  static uint32_t usToCCR(uint16_t us);
};

/// SBUS Protocol Output
/// 100kbps, 8E2, inverted logic (hardware inversion required externally)
/// 25-byte frame: [0x0F, 22 bytes payload, flags, 0x00]
class SBUSOutput {
public:
  explicit SBUSOutput(UART_HandleTypeDef &huart);

  /// Convert channel data to SBUS frame and transmit via UART.
  void sendFrame(const RC::Channel::ChannelData &data, bool isFailsafe);

private:
  UART_HandleTypeDef &huart_;

  /// Converts 1000-2000µs range to SBUS 11-bit range
  static uint16_t usToSBUS(uint16_t us);
};

/// IBUS Protocol Output
/// 115200 baud, 8N1, standard UART logic
/// 32-byte frame: [0x20, 0x40, ch1_l, ch1_h, ..., ch14_l, ch14_h, chksum_l, chksum_h]
class IBUSOutput {
public:
  explicit IBUSOutput(UART_HandleTypeDef &huart);

  /// Convert channel data to IBUS frame and transmit via UART.
  void sendFrame(const RC::Channel::ChannelData &data);

private:
  UART_HandleTypeDef &huart_;
};

} // namespace RC::Output

// src/output/sbus/SBUSOutput.hpp
#pragma once
#include "channel/ChannelProcessor.hpp"
#include "stm32f1xx_hal.h" // IWYU pragma: keep
#include <stdint.h>

namespace RC::Output {

/// SBUS Protocol Output
/// 100kbps, 8E2, inverted logic (hardware inversion required externally)
/// 25-byte frame: [0x0F, 22 bytes payload, flags, 0x00]
class SBUSOutput {
public:
  explicit SBUSOutput(UART_HandleTypeDef &huart);

  /// Convert channel data to SBUS frame and transmit via UART blocking/IT/DMA.
  /// Uses blocking transmit for simplicity in superloop.
  /// isFailsafe: sets the SBUS failsafe flag if true
  void sendFrame(const RC::Channel::ChannelData &data, bool isFailsafe);

private:
  UART_HandleTypeDef &huart_;

  /// Converts 1000-2000µs range to SBUS 11-bit range
  static uint16_t usToSBUS(uint16_t us);
};

} // namespace RC::Output

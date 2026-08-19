// src/output/pwm/PWMOutput.hpp
#pragma once
#include "channel/ChannelProcessor.hpp"
#include <stdint.h>

namespace RC::Output {

/// Controls all 8 hardware PWM channels via TIM2, TIM3, and TIM4.
/// All channels run at 50 Hz (20ms period, 1MHz timer clock, ARR=19999).
///
/// Channel mapping:
///   CH1 (index 0) = PA0  — TIM2_CH1
///   CH2 (index 1) = PA1  — TIM2_CH2
///   CH3 (index 2) = PB6  — TIM4_CH1
///   CH4 (index 3) = PB7  — TIM4_CH2
///   CH5 (index 4) = PB8  — TIM4_CH3
///   CH6 (index 5) = PB9  — TIM4_CH4
///   CH7 (index 6) = PB4  — TIM3_CH1 (partial remap)
///   CH8 (index 7) = PB5  — TIM3_CH2 (partial remap)
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

  /// At 1 MHz timer clock: 1µs = 1 count → CCR = us directly.
  static uint32_t usToCCR(uint16_t us);
};

} // namespace RC::Output

// src/channel/ChannelProcessor.hpp
#pragma once
#include "config/FirmwareConfig.hpp"
#include <stdint.h>

namespace RC::Channel {

struct ChannelData {
  uint16_t us[RC::FW::PACKET_CHANNEL_COUNT]; ///< Processed µs values, 1000–2000
};

class ChannelProcessor {
public:
  ChannelProcessor();

  /// Process raw channel values from a decoded packet.
  /// Pipeline: range clamp → deadband → EMA filter.
  /// Returns the processed ChannelData.
  ChannelData process(const uint16_t raw[RC::FW::PACKET_CHANNEL_COUNT]);

  /// Return channel data with failsafe values applied.
  /// Channel index 2 (throttle) → CHANNEL_MIN_US. All others → center.
  ChannelData applyFailsafe();

  /// Reset EMA filter state to center (call on signal recovery from failsafe).
  void resetFilter();

private:
  float filtered_[RC::FW::PACKET_CHANNEL_COUNT];

  static uint16_t clamp(uint16_t val, uint16_t lo, uint16_t hi);
  static uint16_t applyDeadband(uint16_t val, uint16_t center, uint16_t band);
  float applyEMA(uint8_t ch, float newVal);
};

} // namespace RC::Channel

#pragma once

#include "config.hpp"
#include <stdint.h>

namespace RC::Channel {

struct ChannelData {
  uint16_t us[RC::FW::PACKET_CHANNEL_COUNT];
};

class ChannelProcessor {
public:
  ChannelProcessor();

  ChannelData process(const uint16_t raw[RC::FW::PACKET_CHANNEL_COUNT]);
  ChannelData applyFailsafe();
  void resetFilter();

private:
  float filtered_[RC::FW::PACKET_CHANNEL_COUNT];

  static uint16_t clamp(uint16_t val, uint16_t lo, uint16_t hi);
  static uint16_t applyDeadband(uint16_t val, uint16_t center, uint16_t band);
  float applyEMA(uint8_t ch, float newVal);
};

} // namespace RC::Channel

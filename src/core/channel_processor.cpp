#include "core/channel_processor.hpp"

namespace RC::Channel {

ChannelProcessor::ChannelProcessor() {
  for (auto &f : filtered_) {
    f = static_cast<float>(RC::FW::CHANNEL_CENTER_US);
  }
}

uint16_t ChannelProcessor::clamp(uint16_t val, uint16_t lo, uint16_t hi) {
  if (val < lo) return lo;
  if (val > hi) return hi;
  return val;
}

uint16_t ChannelProcessor::applyDeadband(uint16_t val, uint16_t center,
                                         uint16_t band) {
  if (val >= (center - band) && val <= (center + band)) {
    return center;
  }
  return val;
}

float ChannelProcessor::applyEMA(uint8_t ch, float newVal) {
  filtered_[ch] = RC::FW::CHANNEL_FILTER_ALPHA * newVal +
                  (1.0f - RC::FW::CHANNEL_FILTER_ALPHA) * filtered_[ch];
  return filtered_[ch];
}

ChannelData ChannelProcessor::process(
    const uint16_t raw[RC::FW::PACKET_CHANNEL_COUNT]) {
  ChannelData out{};
  for (uint8_t i = 0U; i < RC::FW::PACKET_CHANNEL_COUNT; ++i) {
    uint16_t v = clamp(raw[i], RC::FW::CHANNEL_MIN_US, RC::FW::CHANNEL_MAX_US);
    v = applyDeadband(v, RC::FW::CHANNEL_CENTER_US, RC::FW::DEADBAND_US);
    float filtered = applyEMA(i, static_cast<float>(v));
    out.us[i] = static_cast<uint16_t>(filtered + 0.5f);
  }
  return out;
}

ChannelData ChannelProcessor::applyFailsafe() {
  ChannelData out{};
  for (uint8_t i = 0U; i < RC::FW::PACKET_CHANNEL_COUNT; ++i) {
    out.us[i] = RC::FW::CHANNEL_CENTER_US;
  }
  out.us[2] = RC::FW::FAILSAFE_THROTTLE_US;
  return out;
}

void ChannelProcessor::resetFilter() {
  for (auto &f : filtered_) {
    f = static_cast<float>(RC::FW::CHANNEL_CENTER_US);
  }
}

} // namespace RC::Channel

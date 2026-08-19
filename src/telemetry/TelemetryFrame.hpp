// src/telemetry/TelemetryFrame.hpp
#pragma once
#include <stdint.h>

namespace RC::Telemetry {

struct __attribute__((packed)) TelemetryData {
  uint8_t version;     ///< Telemetry version (1)
  uint8_t rssi;        ///< RSSI metric (e.g. 0-100 or raw dBm mapping)
  uint8_t linkQuality; ///< LQI 0-100%
  uint8_t packetLoss;  ///< Packet loss %
  uint16_t voltage;    ///< Receiver battery voltage in mV
  uint8_t cpuLoad;     ///< Receiver CPU load %
  uint8_t reserved[3]; ///< Padding for 10-byte size
};

static_assert(sizeof(TelemetryData) == 10U, "TelemetryData must be 10 bytes");

} // namespace RC::Telemetry

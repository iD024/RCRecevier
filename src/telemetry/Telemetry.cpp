// src/telemetry/Telemetry.cpp
#include "telemetry/Telemetry.hpp"

namespace RC::Telemetry {

TelemetryManager::TelemetryManager(RC::Drivers::NRF24& radio) : radio_(radio) {
    data_.version = 1U;
    data_.rssi = 0U;
    data_.linkQuality = 0U;
    data_.packetLoss = 0U;
    data_.voltage = 0U;
    data_.cpuLoad = 0U;
    data_.reserved[0] = 0U;
    data_.reserved[1] = 0U;
    data_.reserved[2] = 0U;
}

void TelemetryManager::updateStats(uint8_t rssi, uint8_t lq, uint8_t loss, uint16_t vbat, uint8_t cpu) {
    data_.rssi = rssi;
    data_.linkQuality = lq;
    data_.packetLoss = loss;
    data_.voltage = vbat;
    data_.cpuLoad = cpu;
}

void TelemetryManager::writeAckPayload() {
    radio_.writeAckPayload(reinterpret_cast<const uint8_t*>(&data_), sizeof(data_));
}

} // namespace RC::Telemetry

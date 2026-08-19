// src/telemetry/Telemetry.hpp
#pragma once
#include "telemetry/TelemetryFrame.hpp"
#include "drivers/nrf24/NRF24.hpp"

namespace RC::Telemetry {

class TelemetryManager {
public:
    explicit TelemetryManager(RC::Drivers::NRF24& radio);

    /// Update internal stats to be sent on next ACK payload
    void updateStats(uint8_t rssi, uint8_t lq, uint8_t loss, uint16_t vbat, uint8_t cpu);

    /// Write the current telemetry data into NRF24 TX FIFO as an ACK payload
    void writeAckPayload();

private:
    RC::Drivers::NRF24& radio_;
    TelemetryData data_{};
};

} // namespace RC::Telemetry

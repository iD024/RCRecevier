// src/failsafe/Failsafe.hpp
#pragma once
#include "config/FirmwareConfig.hpp"
#include <cstdint>

namespace RC::Failsafe {

enum class FailsafeState : uint8_t {
    Active,     // Normal reception
    Failsafe    // Signal lost
};

class FailsafeController {
public:
    FailsafeController();

    /// Called every time a valid control packet is received.
    /// Resets the failsafe timer.
    void registerValidPacket();

    /// Call this periodically (e.g. in the main loop).
    /// Returns current state (Active or Failsafe).
    FailsafeState update(uint32_t currentTick);

    /// Checks if we just transitioned to failsafe on this exact update tick
    bool justEnteredFailsafe() const;

    /// Checks if we just recovered from failsafe on this exact update tick
    bool justRecovered() const;

private:
    uint32_t lastValidPacketTick_;
    FailsafeState currentState_;
    FailsafeState previousState_;
};

} // namespace RC::Failsafe

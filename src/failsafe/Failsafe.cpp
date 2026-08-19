// src/failsafe/Failsafe.cpp
#include "failsafe/Failsafe.hpp"
#include "stm32f1xx_hal.h"

namespace RC::Failsafe {

FailsafeController::FailsafeController()
    : lastValidPacketTick_(0U),
      currentState_(FailsafeState::Failsafe),
      previousState_(FailsafeState::Failsafe) {}

void FailsafeController::registerValidPacket() {
    lastValidPacketTick_ = HAL_GetTick();
}

FailsafeState FailsafeController::update(uint32_t currentTick) {
    previousState_ = currentState_;

    // If more than FAILSAFE_TIMEOUT_MS has passed since last valid packet
    if ((currentTick - lastValidPacketTick_) > RC::FW::FAILSAFE_TIMEOUT_MS) {
        currentState_ = FailsafeState::Failsafe;
    } else {
        currentState_ = FailsafeState::Active;
    }

    return currentState_;
}

bool FailsafeController::justEnteredFailsafe() const {
    return (currentState_ == FailsafeState::Failsafe &&
            previousState_ == FailsafeState::Active);
}

bool FailsafeController::justRecovered() const {
    return (currentState_ == FailsafeState::Active &&
            previousState_ == FailsafeState::Failsafe);
}

} // namespace RC::Failsafe

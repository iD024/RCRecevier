// src/output/ibus/IBUSOutput.hpp
#pragma once
#include "channel/ChannelProcessor.hpp"
#include "stm32f1xx_hal.h"
#include <cstdint>

namespace RC::Output {

/// IBUS Protocol Output
/// 115200 baud, 8N1, standard UART logic (not inverted)
/// 32-byte frame:
/// [0x20, 0x40, ch1_l, ch1_h, ..., ch14_l, ch14_h, chksum_l, chksum_h]
/// Supports 14 channels (each 16-bit little-endian).
/// Checksum is 0xFFFF - sum(bytes 0..29)
class IBUSOutput {
public:
    explicit IBUSOutput(UART_HandleTypeDef& huart);

    /// Convert channel data to IBUS frame and transmit via UART.
    void sendFrame(const RC::Channel::ChannelData& data);

private:
    UART_HandleTypeDef& huart_;
};

} // namespace RC::Output

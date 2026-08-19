// src/output/ibus/IBUSOutput.cpp
#include "output/ibus/IBUSOutput.hpp"

namespace RC::Output {

IBUSOutput::IBUSOutput(UART_HandleTypeDef& huart) : huart_(huart) {}

void IBUSOutput::sendFrame(const RC::Channel::ChannelData& data) {
    uint8_t frame[32];
    frame[0] = 0x20U;
    frame[1] = 0x40U;

    uint32_t checksum = 0xFFFFU;
    checksum -= frame[0];
    checksum -= frame[1];

    // IBUS supports 14 channels
    for (uint8_t i = 0U; i < 14U; ++i) {
        uint16_t us = RC::FW::CHANNEL_CENTER_US;
        if (i < RC::FW::PACKET_CHANNEL_COUNT) {
            us = data.us[i];
        }

        uint8_t lo = static_cast<uint8_t>(us & 0xFFU);
        uint8_t hi = static_cast<uint8_t>((us >> 8U) & 0xFFU);

        frame[2U + i * 2U] = lo;
        frame[3U + i * 2U] = hi;

        checksum -= lo;
        checksum -= hi;
    }

    frame[30] = static_cast<uint8_t>(checksum & 0xFFU);
    frame[31] = static_cast<uint8_t>((checksum >> 8U) & 0xFFU);

    HAL_UART_Transmit(&huart_, frame, sizeof(frame), 10U);
}

} // namespace RC::Output

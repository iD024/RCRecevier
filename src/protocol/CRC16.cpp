// src/protocol/CRC16.cpp
#include "protocol/CRC16.hpp"

namespace RC::Protocol {

uint16_t CRC16::compute(const uint8_t* data, uint16_t len) {
    uint16_t crc = INIT;
    for (uint16_t i = 0U; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8U;
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            if ((crc & 0x8000U) != 0U) {
                crc = static_cast<uint16_t>((crc << 1U) ^ POLY);
            } else {
                crc = static_cast<uint16_t>(crc << 1U);
            }
        }
    }
    return crc;
}

bool CRC16::verify(const uint8_t* data, uint16_t len, uint16_t expectedCrc) {
    return compute(data, len) == expectedCrc;
}

} // namespace RC::Protocol

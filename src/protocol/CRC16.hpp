// src/protocol/CRC16.hpp
#pragma once
#include <cstdint>

namespace RC::Protocol {

/// CRC-16/CCITT-FALSE implementation.
/// Polynomial: 0x1021, Initial value: 0xFFFF.
/// No input or output reflection.
/// Standard test vector: "123456789" → 0x29B1
class CRC16 {
public:
    /// Compute CRC-16 over data[0..len-1].
    /// Returns INIT (0xFFFF) for len==0 without dereferencing data.
    static uint16_t compute(const uint8_t* data, uint16_t len);

    /// Returns true if compute(data, len) == expectedCrc.
    static bool verify(const uint8_t* data, uint16_t len, uint16_t expectedCrc);

private:
    static constexpr uint16_t POLY = 0x1021U;
    static constexpr uint16_t INIT = 0xFFFFU;
};

} // namespace RC::Protocol

// src/protocol/Packet.hpp
#pragma once
#include "config/FirmwareConfig.hpp"
#include <cstdint>
#include <array>

namespace RC::Protocol {

enum class PacketType : uint8_t {
    Control      = 0x01U,
    Bind         = 0x02U,
    TelemetryReq = 0x03U,
};

enum class PacketFlags : uint8_t {
    None     = 0x00U,
    Failsafe = (1U << 0U),
    Bind     = (1U << 1U),
};

/// Wire-format packet — 32 bytes, matches NRF24 payload size exactly.
/// __attribute__((packed)) ensures no padding bytes are inserted by the compiler.
struct __attribute__((packed)) Packet {
    uint8_t  magic0;           ///< 0xAC
    uint8_t  magic1;           ///< 0x24
    uint8_t  version;          ///< PROTOCOL_VERSION = 0x01
    uint8_t  type;             ///< PacketType
    uint32_t transmitterId;    ///< Unique transmitter hardware ID
    uint32_t receiverId;       ///< 0xFFFFFFFF = broadcast (bind phase)
    uint8_t  sequenceNumber;   ///< Rolling 0–255 counter
    uint8_t  flags;            ///< PacketFlags bitmask
    uint16_t channels[RC::FW::PACKET_CHANNEL_COUNT]; ///< 1000–2000 µs
    uint16_t crc;              ///< CRC-16/CCITT-FALSE over bytes 0–29
};

static_assert(sizeof(Packet) == 32U, "Packet must be exactly 32 bytes");

} // namespace RC::Protocol

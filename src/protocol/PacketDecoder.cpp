// src/protocol/PacketDecoder.cpp
#include "protocol/PacketDecoder.hpp"
#include "protocol/CRC16.hpp"
#include "config/FirmwareConfig.hpp"

namespace RC::Protocol {

DecodeResult PacketDecoder::decode(const uint8_t* rawBuf,
                                   uint8_t        len,
                                   uint32_t       receiverId) {
    if (rawBuf == nullptr || len != sizeof(Packet)) {
        return { DecodeStatus::BadLength, nullptr };
    }

    const auto& pkt = *reinterpret_cast<const Packet*>(rawBuf);

    if (pkt.magic0 != RC::FW::MAGIC_BYTE_0 || pkt.magic1 != RC::FW::MAGIC_BYTE_1) {
        return { DecodeStatus::BadMagic, nullptr };
    }

    if (pkt.version != RC::FW::PROTOCOL_VERSION) {
        return { DecodeStatus::BadVersion, nullptr };
    }

    // Accept if receiver ID matches OR if packet targets broadcast (bind phase)
    if (pkt.receiverId != receiverId &&
        pkt.receiverId != RC::FW::RECEIVER_ID_BROADCAST) {
        return { DecodeStatus::ReceiverIdMismatch, nullptr };
    }

    // CRC covers bytes 0..29 (all bytes except the 2-byte CRC field itself)
    constexpr uint16_t CRC_OFFSET = sizeof(Packet) - sizeof(uint16_t);
    uint16_t computed = CRC16::compute(rawBuf, CRC_OFFSET);
    if (computed != pkt.crc) {
        return { DecodeStatus::BadCRC, nullptr };
    }

    if (!validateChannels(pkt)) {
        return { DecodeStatus::ChannelOutOfRange, nullptr };
    }

    return { DecodeStatus::Ok, &pkt };
}

bool PacketDecoder::validateChannels(const Packet& pkt) {
    for (uint8_t i = 0U; i < RC::FW::PACKET_CHANNEL_COUNT; ++i) {
        if (pkt.channels[i] < RC::FW::CHANNEL_MIN_US ||
            pkt.channels[i] > RC::FW::CHANNEL_MAX_US) {
            return false;
        }
    }
    return true;
}

} // namespace RC::Protocol

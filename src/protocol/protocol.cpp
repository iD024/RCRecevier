#include "protocol/protocol.hpp"

namespace RC::Protocol {

// ── CRC16 Implementation ────────────────────────────────────

uint16_t CRC16::compute(const uint8_t *data, uint16_t len) {
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

bool CRC16::verify(const uint8_t *data, uint16_t len, uint16_t expectedCrc) {
  return compute(data, len) == expectedCrc;
}

// ── PacketDecoder Implementation ────────────────────────────

DecodeResult PacketDecoder::decode(const uint8_t *rawBuf, uint8_t len,
                                   uint32_t receiverId) {
  if (rawBuf == nullptr || len != sizeof(Packet)) {
    return {DecodeStatus::BadLength, nullptr};
  }

  const auto &pkt = *reinterpret_cast<const Packet *>(rawBuf);

  if (pkt.magic0 != RC::FW::MAGIC_BYTE_0 || pkt.magic1 != RC::FW::MAGIC_BYTE_1) {
    return {DecodeStatus::BadMagic, nullptr};
  }

  if (pkt.version != RC::FW::PROTOCOL_VERSION) {
    return {DecodeStatus::BadVersion, nullptr};
  }

  if (pkt.receiverId != receiverId &&
      pkt.receiverId != RC::FW::RECEIVER_ID_BROADCAST) {
    return {DecodeStatus::ReceiverIdMismatch, nullptr};
  }

  constexpr uint16_t CRC_OFFSET = sizeof(Packet) - sizeof(uint16_t);
  uint16_t computed = CRC16::compute(rawBuf, CRC_OFFSET);
  if (computed != pkt.crc) {
    return {DecodeStatus::BadCRC, nullptr};
  }

  if (!validateChannels(pkt)) {
    return {DecodeStatus::ChannelOutOfRange, nullptr};
  }

  return {DecodeStatus::Ok, &pkt};
}

bool PacketDecoder::validateChannels(const Packet &pkt) {
  for (uint8_t i = 0U; i < RC::FW::PACKET_CHANNEL_COUNT; ++i) {
    if (pkt.channels[i] < RC::FW::CHANNEL_MIN_US ||
        pkt.channels[i] > RC::FW::CHANNEL_MAX_US) {
      return false;
    }
  }
  return true;
}

// ── BindingProtocol Implementation ─────────────────────────

bool BindingProtocol::isBindPacket(const Packet &pkt) {
  return pkt.type == static_cast<uint8_t>(PacketType::Bind);
}

void BindingProtocol::extractToConfig(const Packet &pkt,
                                      RC::Storage::ReceiverConfig &config) {
  config.boundTransmitterId = pkt.transmitterId;
  config.radioChannel = static_cast<uint8_t>((pkt.transmitterId % 80U) + 10U);

  config.radioAddr[0] = static_cast<uint8_t>(pkt.transmitterId & 0xFFU);
  config.radioAddr[1] = static_cast<uint8_t>((pkt.transmitterId >> 8U) & 0xFFU);
  config.radioAddr[2] = static_cast<uint8_t>((pkt.transmitterId >> 16U) & 0xFFU);
  config.radioAddr[3] = static_cast<uint8_t>((pkt.transmitterId >> 24U) & 0xFFU);
  config.radioAddr[4] = 0xE7U;
}

} // namespace RC::Protocol

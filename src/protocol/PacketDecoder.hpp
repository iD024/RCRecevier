// src/protocol/PacketDecoder.hpp
#pragma once
#include "protocol/Packet.hpp"
#include <stdint.h>

namespace RC::Protocol {

enum class DecodeStatus : uint8_t {
  Ok,
  BadLength,
  BadMagic,
  BadVersion,
  ReceiverIdMismatch,
  BadCRC,
  ChannelOutOfRange,
};

struct DecodeResult {
  DecodeStatus status;
  const Packet *packet; ///< Valid only when status == Ok. Points into rawBuf.
};

class PacketDecoder {
public:
  /// Decode and validate a raw 32-byte buffer received from NRF24.
  /// receiverId: the stored ID of this receiver (0xFFFFFFFF accepts broadcast).
  /// The returned packet pointer is valid only while rawBuf is unmodified.
  static DecodeResult decode(const uint8_t *rawBuf, uint8_t len,
                             uint32_t receiverId);

private:
  static bool validateChannels(const Packet &pkt);
};

} // namespace RC::Protocol

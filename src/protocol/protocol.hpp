#pragma once

#include "config.hpp"
#include "core/config_store.hpp"
#include <stdint.h>

namespace RC::Protocol {

// ── Packet Structure ─────────────────────────────────────────

enum class PacketType : uint8_t {
  Control = 0x01U,
  Bind = 0x02U,
  TelemetryReq = 0x03U,
};

enum class PacketFlags : uint8_t {
  None = 0x00U,
  Failsafe = (1U << 0U),
  Bind = (1U << 1U),
};

/// Wire-format packet — 32 bytes, matches NRF24 payload size.
struct __attribute__((packed)) Packet {
  uint8_t magic0;         ///< 0xAC
  uint8_t magic1;         ///< 0x24
  uint8_t version;        ///< PROTOCOL_VERSION = 0x01
  uint8_t type;           ///< PacketType
  uint32_t transmitterId; ///< Unique transmitter hardware ID
  uint32_t receiverId;    ///< 0xFFFFFFFF = broadcast (bind phase)
  uint8_t sequenceNumber; ///< Rolling 0–255 counter
  uint8_t flags;          ///< PacketFlags bitmask
  uint16_t channels[RC::FW::PACKET_CHANNEL_COUNT]; ///< 1000–2000 µs
  uint16_t crc; ///< CRC-16/CCITT-FALSE over bytes 0–29
};

static_assert(sizeof(Packet) == 32U, "Packet must be exactly 32 bytes");

// ── CRC16 Class ─────────────────────────────────────────────

class CRC16 {
public:
  static uint16_t compute(const uint8_t *data, uint16_t len);
  static bool verify(const uint8_t *data, uint16_t len, uint16_t expectedCrc);

private:
  static constexpr uint16_t POLY = 0x1021U;
  static constexpr uint16_t INIT = 0xFFFFU;
};

// ── Packet Decoder Class ─────────────────────────────────────

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
  const Packet *packet; ///< Valid only when status == Ok.
};

class PacketDecoder {
public:
  static DecodeResult decode(const uint8_t *rawBuf, uint8_t len,
                             uint32_t receiverId);

private:
  static bool validateChannels(const Packet &pkt);
};

// ── Binding Protocol Class ───────────────────────────────────

class BindingProtocol {
public:
  static bool isBindPacket(const Packet &pkt);
  static void extractToConfig(const Packet &pkt,
                              RC::Storage::ReceiverConfig &config);
};

} // namespace RC::Protocol

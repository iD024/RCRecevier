#pragma once
#include <stdint.h>

namespace RC::Storage {

struct __attribute__((packed)) ReceiverConfig {
  uint32_t magic;
  uint32_t version;
  uint32_t receiverId;
  uint32_t boundTransmitterId;
  uint8_t radioChannel;
  uint8_t radioAddr[5];
  uint16_t failsafeValues[8];
  uint8_t reserved[2];
  uint32_t checksum;
};

class ConfigStore {
public:
  static void load(ReceiverConfig &config);
  static void save(const ReceiverConfig &config);
  static void loadDefaults(ReceiverConfig &config);

private:
  static constexpr uint32_t FLASH_CONFIG_ADDR =
      0x0800FC00U; // Page 63 (last page of 64K flash)
  static constexpr uint32_t CONFIG_MAGIC = 0x43464731U; // "CFG1"

  static uint32_t calculateChecksum(const ReceiverConfig &config);
};

} // namespace RC::Storage

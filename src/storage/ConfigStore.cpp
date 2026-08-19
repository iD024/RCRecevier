// src/storage/ConfigStore.cpp
#include "storage/ConfigStore.hpp"
#include "stm32f1xx_hal.h"
#include <string.h>

namespace RC::Storage {

uint32_t ConfigStore::calculateChecksum(const ReceiverConfig &config) {
  uint32_t sum = 0;
  const uint8_t *ptr = reinterpret_cast<const uint8_t *>(&config);
  for (size_t i = 0; i < sizeof(ReceiverConfig) - sizeof(uint32_t); ++i) {
    sum += ptr[i];
  }
  return sum;
}

void ConfigStore::loadDefaults(ReceiverConfig &config) {
  config.magic = CONFIG_MAGIC;
  config.version = 1U;
  config.receiverId = 0x12345678U;         // Example default ID
  config.boundTransmitterId = 0xFFFFFFFFU; // Not bound
  config.radioChannel = 76U;
  config.radioAddr[0] = 0xE7U;
  config.radioAddr[1] = 0xE7U;
  config.radioAddr[2] = 0xE7U;
  config.radioAddr[3] = 0xE7U;
  config.radioAddr[4] = 0xE7U;
  for (uint8_t i = 0; i < 8U; ++i) {
    config.failsafeValues[i] = 1500U;
  }
  config.failsafeValues[2] = 1000U; // Throttle
  config.reserved[0] = 0U;
  config.reserved[1] = 0U;
  config.checksum = calculateChecksum(config);
}

void ConfigStore::load(ReceiverConfig &config) {
  const ReceiverConfig *flashConfig =
      reinterpret_cast<const ReceiverConfig *>(FLASH_CONFIG_ADDR);

  if (flashConfig->magic == CONFIG_MAGIC &&
      flashConfig->checksum == calculateChecksum(*flashConfig)) {
    memcpy(&config, flashConfig, sizeof(ReceiverConfig));
  } else {
    loadDefaults(config);
  }
}

void ConfigStore::save(const ReceiverConfig &config) {
  ReceiverConfig toSave;
  memcpy(&toSave, &config, sizeof(ReceiverConfig));
  toSave.checksum = calculateChecksum(toSave);

  HAL_FLASH_Unlock();

  FLASH_EraseInitTypeDef eraseInit;
  eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
  eraseInit.PageAddress = FLASH_CONFIG_ADDR;
  eraseInit.NbPages = 1U;

  uint32_t pageError = 0U;
  HAL_FLASHEx_Erase(&eraseInit, &pageError);

  uint32_t addr = FLASH_CONFIG_ADDR;
  const uint32_t *ptr = reinterpret_cast<const uint32_t *>(&toSave);
  size_t words = (sizeof(ReceiverConfig) + 3) / 4;

  for (size_t i = 0; i < words; ++i) {
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, ptr[i]);
    addr += 4;
  }

  HAL_FLASH_Lock();
}

} // namespace RC::Storage

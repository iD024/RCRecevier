#pragma once

#include "drivers/spi/SPIBus.hpp"
#include "stdint.h"

namespace RC::Drivers {

enum class Result : uint8_t { Ok, Error, Timeout, InvalidParam, NoData };

class NRF24 {
public:
  NRF24(SPIBus &spi, GPIO_TypeDef *cePort, uint16_t cePin);

  Result init(uint8_t channel, const uint8_t addr[5]);

  void startListening();
  void stopListening();

  uint8_t readRegister(uint8_t reg);

  bool isDataReady();

  Result readPayload(uint8_t *buf, uint8_t len);

  Result writeAckPayload(const uint8_t *buf, uint8_t len);

  uint8_t getRPD();

  void clearIRQ();

  uint8_t readStatus();

private:
  SPIBus &spi_;

  GPIO_TypeDef *cePort_;
  uint16_t cePin_;

  void ceHigh();
  void ceLow();

  void writeReg(uint8_t reg, uint8_t value);

  uint8_t readReg(uint8_t reg);

  void writeRegMulti(uint8_t reg, const uint8_t *buf, uint8_t len);

  void readRegMulti(uint8_t reg, uint8_t *buf, uint8_t len);

  void powerUp();
  void flushRx();
  void flushTx();
};

} // namespace RC::Drivers
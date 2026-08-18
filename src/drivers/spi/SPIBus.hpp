#pragma once

#include "stm32f1xx_hal.h"
#include <stdint.h>

namespace RC::Drivers {

class SPIBus {
public:
  SPIBus(SPI_HandleTypeDef &handle, GPIO_TypeDef *csPort, uint16_t csPin);

  void transfer(const uint8_t *txBuf, uint8_t *rxBuf, uint8_t len);

  void write(const uint8_t *txBuf, uint8_t len);

  void read(uint8_t *rxBuf, uint8_t len);

  void writeCommand(uint8_t command, const uint8_t *data, uint8_t len);

  void transferCommand(uint8_t command, uint8_t *rxBuf, uint8_t len);

private:
  SPI_HandleTypeDef &handle_;
  GPIO_TypeDef *csPort_;
  uint16_t csPin_;

  void csAssert();
  void csDeassert();
};

} // namespace RC::Drivers
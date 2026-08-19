#include "drivers/spi/SPIBus.hpp"

namespace RC::Drivers {

SPIBus::SPIBus(SPI_HandleTypeDef &handle, GPIO_TypeDef *csPort, uint16_t csPin)
    : handle_(handle), csPort_(csPort), csPin_(csPin) {}

void SPIBus::csAssert() { HAL_GPIO_WritePin(csPort_, csPin_, GPIO_PIN_RESET); }

void SPIBus::csDeassert() { HAL_GPIO_WritePin(csPort_, csPin_, GPIO_PIN_SET); }

void SPIBus::transfer(const uint8_t *txBuf, uint8_t *rxBuf, uint8_t len) {
  csAssert();

  HAL_SPI_TransmitReceive(&handle_, const_cast<uint8_t *>(txBuf), rxBuf, len,
                          10U);

  csDeassert();
}

void SPIBus::write(const uint8_t *txBuf, uint8_t len) {
  csAssert();

  HAL_SPI_Transmit(&handle_, const_cast<uint8_t *>(txBuf), len, 10U);

  csDeassert();
}

void SPIBus::read(uint8_t *rxBuf, uint8_t len) {
  if (rxBuf == nullptr || len == 0U) {
    return;
  }
  static uint8_t dummy[32] = {
      0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
      0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
      0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
      0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

  csAssert();
  if (len <= sizeof(dummy)) {
    HAL_SPI_TransmitReceive(&handle_, dummy, rxBuf, len, 10U);
  } else {
    for (uint8_t i = 0; i < len; ++i) {
      uint8_t d = 0xFFU;
      HAL_SPI_TransmitReceive(&handle_, &d, &rxBuf[i], 1U, 10U);
    }
  }
  csDeassert();
}

void SPIBus::writeCommand(uint8_t command, const uint8_t *data, uint8_t len) {
  csAssert();

  HAL_SPI_Transmit(&handle_, &command, 1U, 10U);

  if (data != nullptr && len > 0U) {
    HAL_SPI_Transmit(&handle_, const_cast<uint8_t *>(data), len, 10U);
  }

  csDeassert();
}

void SPIBus::transferCommand(uint8_t command, uint8_t *rxBuf, uint8_t len) {
  if (rxBuf == nullptr || len == 0U) {
    return;
  }
  static uint8_t dummy[32] = {
      0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
      0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
      0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
      0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

  csAssert();

  HAL_SPI_Transmit(&handle_, &command, 1U, 10U);

  if (len <= sizeof(dummy)) {
    HAL_SPI_TransmitReceive(&handle_, dummy, rxBuf, len, 10U);
  } else {
    for (uint8_t i = 0U; i < len; ++i) {
      uint8_t d = 0xFFU;
      HAL_SPI_TransmitReceive(&handle_, &d, &rxBuf[i], 1U, 10U);
    }
  }

  csDeassert();
}

} // namespace RC::Drivers
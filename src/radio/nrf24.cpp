#include "radio/nrf24.hpp"
#include "config.hpp"

namespace RC::Drivers {

// ── SPIBus Implementation ───────────────────────────────────

SPIBus::SPIBus(SPI_HandleTypeDef &handle, GPIO_TypeDef *csPort, uint16_t csPin)
    : handle_(handle), csPort_(csPort), csPin_(csPin) {}

void SPIBus::csAssert() { HAL_GPIO_WritePin(csPort_, csPin_, GPIO_PIN_RESET); }

void SPIBus::csDeassert() { HAL_GPIO_WritePin(csPort_, csPin_, GPIO_PIN_SET); }

void SPIBus::transfer(const uint8_t *txBuf, uint8_t *rxBuf, uint8_t len) {
  csAssert();
  HAL_SPI_TransmitReceive(&handle_, const_cast<uint8_t *>(txBuf), rxBuf, len, 10U);
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

// ── NRF24 Implementation ───────────────────────────────────

NRF24::NRF24(SPIBus &spi, GPIO_TypeDef *cePort, uint16_t cePin)
    : spi_(spi), cePort_(cePort), cePin_(cePin) {}

void NRF24::ceHigh() { HAL_GPIO_WritePin(cePort_, cePin_, GPIO_PIN_SET); }

void NRF24::ceLow() { HAL_GPIO_WritePin(cePort_, cePin_, GPIO_PIN_RESET); }

void NRF24::writeReg(uint8_t reg, uint8_t value) {
  uint8_t tx[2] = {
      static_cast<uint8_t>(NRF24Reg::CMD_W_REGISTER | (reg & 0x1FU)), value};
  spi_.write(tx, 2U);
}

uint8_t NRF24::readRegister(uint8_t reg) { return readReg(reg); }

void NRF24::writeRegMulti(uint8_t reg, const uint8_t *buf, uint8_t len) {
  if (buf == nullptr || len == 0U) {
    return;
  }
  uint8_t cmd = static_cast<uint8_t>(NRF24Reg::CMD_W_REGISTER | (reg & 0x1FU));
  spi_.writeCommand(cmd, buf, len);
}

void NRF24::readRegMulti(uint8_t reg, uint8_t *buf, uint8_t len) {
  if (buf == nullptr || len == 0U) {
    return;
  }
  uint8_t command =
      static_cast<uint8_t>(NRF24Reg::CMD_R_REGISTER | (reg & 0x1FU));
  spi_.transferCommand(command, buf, len);
}

uint8_t NRF24::readReg(uint8_t reg) {
  uint8_t tx[2] = {
      static_cast<uint8_t>(NRF24Reg::CMD_R_REGISTER | (reg & 0x1FU)),
      NRF24Reg::CMD_NOP};
  uint8_t rx[2] = {};
  spi_.transfer(tx, rx, 2U);
  return rx[1];
}

void NRF24::powerUp() {
  writeReg(NRF24Reg::REG_CONFIG,
           NRF24Reg::CFG_EN_CRC | NRF24Reg::CFG_CRC0 | NRF24Reg::CFG_PWR_UP |
               NRF24Reg::CFG_MASK_TX_DS | NRF24Reg::CFG_MASK_MAX_RT);
  HAL_Delay(5U);
}

void NRF24::flushRx() {
  uint8_t command = NRF24Reg::CMD_FLUSH_RX;
  spi_.write(&command, 1U);
}

void NRF24::flushTx() {
  uint8_t command = NRF24Reg::CMD_FLUSH_TX;
  spi_.write(&command, 1U);
}

uint8_t NRF24::readStatus() { return readReg(NRF24Reg::REG_STATUS); }

void NRF24::startListening() {
  uint8_t config = readReg(NRF24Reg::REG_CONFIG);
  config |= NRF24Reg::CFG_PRIM_RX;
  writeReg(NRF24Reg::REG_CONFIG, config);

  writeReg(NRF24Reg::REG_STATUS,
           NRF24Reg::ST_RX_DR | NRF24Reg::ST_TX_DS | NRF24Reg::ST_MAX_RT);
  ceHigh();
  HAL_Delay(1U);
}

void NRF24::stopListening() {
  ceLow();
  HAL_Delay(1U);
  uint8_t config = readReg(NRF24Reg::REG_CONFIG);
  config &= static_cast<uint8_t>(~NRF24Reg::CFG_PRIM_RX);
  writeReg(NRF24Reg::REG_CONFIG, config);
}

Result NRF24::init(uint8_t channel, const uint8_t addr[5]) {
  if (addr == nullptr) {
    return Result::InvalidParam;
  }

  HAL_Delay(100U);
  ceLow();
  powerUp();

  writeReg(NRF24Reg::REG_SETUP_AW, 0x03U);

  writeReg(NRF24Reg::REG_SETUP_RETR,
           static_cast<uint8_t>((RC::HW::NRF_AUTO_RETR_DELAY << 4U) |
                                RC::HW::NRF_AUTO_RETR_COUNT));

  writeReg(NRF24Reg::REG_RF_CH, channel & 0x7FU);
  writeReg(NRF24Reg::REG_RF_SETUP,
           NRF24Reg::RF_PWR_0DBM | NRF24Reg::RF_DR_1MBPS);

  writeReg(NRF24Reg::REG_EN_RXADDR, 0x03U);
  writeReg(NRF24Reg::REG_EN_AA, 0x03U);

  writeRegMulti(NRF24Reg::REG_RX_ADDR_P1, addr, 5U);
  writeRegMulti(NRF24Reg::REG_RX_ADDR_P0, addr, 5U);
  writeRegMulti(NRF24Reg::REG_TX_ADDR, addr, 5U);

  writeReg(NRF24Reg::REG_RX_PW_P1, RC::HW::NRF_PAYLOAD_SIZE);
  writeReg(NRF24Reg::REG_RX_PW_P0, RC::HW::NRF_PAYLOAD_SIZE);

  uint8_t activate[2] = {NRF24Reg::CMD_ACTIVATE, NRF24Reg::ACTIVATE_MAGIC};
  spi_.write(activate, 2U);
  writeReg(NRF24Reg::REG_FEATURE, NRF24Reg::FEAT_EN_ACK_PAY | NRF24Reg::FEAT_EN_DPL);
  writeReg(NRF24Reg::REG_DYNPD, 0x03U);

  writeReg(NRF24Reg::REG_STATUS,
           NRF24Reg::ST_RX_DR | NRF24Reg::ST_TX_DS | NRF24Reg::ST_MAX_RT);

  flushRx();
  flushTx();

  uint8_t readback = readReg(NRF24Reg::REG_RF_CH);
  if (readback != (channel & 0x7FU)) {
    return Result::Error;
  }

  return Result::Ok;
}

Result NRF24::writeAckPayload(const uint8_t *buf, uint8_t len) {
  if (buf == nullptr || len == 0U || len > 32U) {
    return Result::InvalidParam;
  }
  uint8_t cmd = NRF24Reg::CMD_W_ACK_PAYLOAD | 1U;
  spi_.writeCommand(cmd, buf, len);
  return Result::Ok;
}

bool NRF24::isDataReady() {
  uint8_t status = readStatus();
  return (status & NRF24Reg::ST_RX_DR) != 0U;
}

Result NRF24::readPayload(uint8_t *buf, uint8_t len) {
  if (buf == nullptr || len == 0U) {
    return Result::InvalidParam;
  }

  spi_.transferCommand(NRF24Reg::CMD_R_RX_PAYLOAD, buf, len);
  writeReg(NRF24Reg::REG_STATUS, NRF24Reg::ST_RX_DR);
  return Result::Ok;
}

} // namespace RC::Drivers

#include "drivers/nrf24/NRF24.hpp"
#include "config/HardwareConfig.hpp"
#include "drivers/nrf24/NRF24Registers.hpp"

namespace RC::Drivers {

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

  // Clear pending IRQ flags
  writeReg(NRF24Reg::REG_STATUS,
           NRF24Reg::ST_RX_DR | NRF24Reg::ST_TX_DS | NRF24Reg::ST_MAX_RT);

  ceHigh();

  // NRF24 requires at least 130 us before RX operation.
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

  // nRF24L01+ requires 100 ms from VDD stable before the crystal oscillator
  // and SPI interface are ready. Skipping this causes POR-state SPI transactions
  // to be silently ignored and MISO to float (reads return 0xFF / garbage).
  HAL_Delay(100U);

  ceLow();

  powerUp();

  // 5-byte address width
  writeReg(NRF24Reg::REG_SETUP_AW, 0x03U);

  // Auto retransmission:
  // ARD = 500 us
  // ARC = 3 retries
  writeReg(NRF24Reg::REG_SETUP_RETR,
           static_cast<uint8_t>((RC::HW::NRF_AUTO_RETR_DELAY << 4U) |
                                RC::HW::NRF_AUTO_RETR_COUNT));

  // RF channel
  writeReg(NRF24Reg::REG_RF_CH, channel & 0x7FU);

  // 0 dBm, 1 Mbps
  writeReg(NRF24Reg::REG_RF_SETUP,
           NRF24Reg::RF_PWR_0DBM | NRF24Reg::RF_DR_1MBPS);

  // Enable RX pipes 0 and 1
  writeReg(NRF24Reg::REG_EN_RXADDR, 0x03U);

  // Enable auto-ACK on pipes 0 and 1
  writeReg(NRF24Reg::REG_EN_AA, 0x03U);

  // Pipe 1 receives the main control packets
  writeRegMulti(NRF24Reg::REG_RX_ADDR_P1, addr, 5U);

  // Pipe 0 is used for ACK/telemetry
  writeRegMulti(NRF24Reg::REG_RX_ADDR_P0, addr, 5U);

  // TX address
  writeRegMulti(NRF24Reg::REG_TX_ADDR, addr, 5U);

  // Fixed 32-byte payloads
  writeReg(NRF24Reg::REG_RX_PW_P1, RC::HW::NRF_PAYLOAD_SIZE);

  writeReg(NRF24Reg::REG_RX_PW_P0, RC::HW::NRF_PAYLOAD_SIZE);

  // Enable ACK payload feature.
  // Original nRF24L01 (non-plus) requires an ACTIVATE 0x73 command before
  // REG_FEATURE is writable. Sending it is harmless on the nRF24L01+.
  uint8_t activate[2] = {NRF24Reg::CMD_ACTIVATE, NRF24Reg::ACTIVATE_MAGIC};
  spi_.write(activate, 2U);
  writeReg(NRF24Reg::REG_FEATURE, NRF24Reg::FEAT_EN_ACK_PAY);

  // Clear pending IRQ flags
  writeReg(NRF24Reg::REG_STATUS,
           NRF24Reg::ST_RX_DR | NRF24Reg::ST_TX_DS | NRF24Reg::ST_MAX_RT);

  flushRx();
  flushTx();

  return Result::Ok;
}

} // namespace RC::Drivers
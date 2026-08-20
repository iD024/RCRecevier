#pragma once

#include "stm32f1xx_hal.h"
#include <stdint.h>

namespace RC::Drivers {

// ── SPI Helper Bus ───────────────────────────────────────────
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

// ── NRF24 Register Definitions ──────────────────────────────
namespace NRF24Reg {

constexpr uint8_t CMD_R_REGISTER = 0x00U;
constexpr uint8_t CMD_W_REGISTER = 0x20U;
constexpr uint8_t CMD_R_RX_PAYLOAD = 0x61U;
constexpr uint8_t CMD_W_TX_PAYLOAD = 0xA0U;
constexpr uint8_t CMD_FLUSH_TX = 0xE1U;
constexpr uint8_t CMD_FLUSH_RX = 0xE2U;
constexpr uint8_t CMD_REUSE_TX_PL = 0xE3U;
constexpr uint8_t CMD_W_ACK_PAYLOAD = 0xA8U;
constexpr uint8_t CMD_NOP = 0xFFU;
constexpr uint8_t CMD_ACTIVATE = 0x50U;
constexpr uint8_t ACTIVATE_MAGIC = 0x73U;

constexpr uint8_t REG_CONFIG = 0x00U;
constexpr uint8_t REG_EN_AA = 0x01U;
constexpr uint8_t REG_EN_RXADDR = 0x02U;
constexpr uint8_t REG_SETUP_AW = 0x03U;
constexpr uint8_t REG_SETUP_RETR = 0x04U;
constexpr uint8_t REG_RF_CH = 0x05U;
constexpr uint8_t REG_RF_SETUP = 0x06U;
constexpr uint8_t REG_STATUS = 0x07U;
constexpr uint8_t REG_RPD = 0x09U;
constexpr uint8_t REG_RX_ADDR_P0 = 0x0AU;
constexpr uint8_t REG_RX_ADDR_P1 = 0x0BU;
constexpr uint8_t REG_TX_ADDR = 0x10U;
constexpr uint8_t REG_RX_PW_P0 = 0x11U;
constexpr uint8_t REG_RX_PW_P1 = 0x12U;
constexpr uint8_t REG_DYNPD = 0x1CU;
constexpr uint8_t REG_FEATURE = 0x1DU;

constexpr uint8_t CFG_PRIM_RX = (1U << 0U);
constexpr uint8_t CFG_PWR_UP = (1U << 1U);
constexpr uint8_t CFG_CRC0 = (1U << 2U);
constexpr uint8_t CFG_EN_CRC = (1U << 3U);
constexpr uint8_t CFG_MASK_MAX_RT = (1U << 4U);
constexpr uint8_t CFG_MASK_TX_DS = (1U << 5U);
constexpr uint8_t CFG_MASK_RX_DR = (1U << 6U);

constexpr uint8_t ST_RX_DR = (1U << 6U);
constexpr uint8_t ST_TX_DS = (1U << 5U);
constexpr uint8_t ST_MAX_RT = (1U << 4U);
constexpr uint8_t ST_RX_P_NO_MASK = (0x07U << 1U);

constexpr uint8_t RF_PWR_0DBM = 0x06U;
constexpr uint8_t RF_DR_250KBPS = (1U << 5U);
constexpr uint8_t RF_DR_1MBPS = 0x00U;
constexpr uint8_t RF_DR_2MBPS = (1U << 3U);

constexpr uint8_t FEAT_EN_DPL = (1U << 2U);
constexpr uint8_t FEAT_EN_ACK_PAY = (1U << 1U);
constexpr uint8_t FEAT_EN_DYN_ACK = (1U << 0U);

} // namespace NRF24Reg

// ── NRF24 Radio Driver Class ────────────────────────────────
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

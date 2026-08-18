#pragma once

#include "stdint.h"

namespace RC::Drivers::NRF24Reg {

// Commands ─────────────────────────────────────
constexpr uint8_t CMD_R_REGISTER = 0x00U;
constexpr uint8_t CMD_W_REGISTER = 0x20U;

constexpr uint8_t CMD_R_RX_PAYLOAD = 0x61U;
constexpr uint8_t CMD_W_TX_PAYLOAD = 0xA0U;

constexpr uint8_t CMD_FLUSH_TX = 0xE1U;
constexpr uint8_t CMD_FLUSH_RX = 0xE2U;

constexpr uint8_t CMD_REUSE_TX_PL = 0xE3U;
constexpr uint8_t CMD_W_ACK_PAYLOAD = 0xA8U;

constexpr uint8_t CMD_NOP = 0xFFU;

// Unlocks FEATURE register on original nRF24L01 (non-plus).
// Harmless on nRF24L01+. Send as: {CMD_ACTIVATE, ACTIVATE_MAGIC}.
constexpr uint8_t CMD_ACTIVATE    = 0x50U;
constexpr uint8_t ACTIVATE_MAGIC  = 0x73U;

// ── Register addresses ─────────────────────────────────────

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

// ── CONFIG register bits ───────────────────────────────────

constexpr uint8_t CFG_PRIM_RX = (1U << 0U);
constexpr uint8_t CFG_PWR_UP = (1U << 1U);
constexpr uint8_t CFG_CRC0 = (1U << 2U);
constexpr uint8_t CFG_EN_CRC = (1U << 3U);

constexpr uint8_t CFG_MASK_MAX_RT = (1U << 4U);
constexpr uint8_t CFG_MASK_TX_DS = (1U << 5U);
constexpr uint8_t CFG_MASK_RX_DR = (1U << 6U);

// ── STATUS register bits ───────────────────────────────────

constexpr uint8_t ST_RX_DR = (1U << 6U);
constexpr uint8_t ST_TX_DS = (1U << 5U);
constexpr uint8_t ST_MAX_RT = (1U << 4U);

constexpr uint8_t ST_RX_P_NO_MASK = (0x07U << 1U);

// ── RF_SETUP values ─────────────────────────────────────────

constexpr uint8_t RF_PWR_0DBM = 0x06U;

constexpr uint8_t RF_DR_250KBPS = (1U << 5U);
constexpr uint8_t RF_DR_1MBPS = 0x00U;
constexpr uint8_t RF_DR_2MBPS = (1U << 3U);

// ── FEATURE register bits ──────────────────────────────────

constexpr uint8_t FEAT_EN_DPL = (1U << 2U);
constexpr uint8_t FEAT_EN_ACK_PAY = (1U << 1U);
constexpr uint8_t FEAT_EN_DYN_ACK = (1U << 0U);

} // namespace RC::Drivers::NRF24Reg
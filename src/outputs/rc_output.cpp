#include "outputs/rc_output.hpp"
#include "config.hpp"

namespace RC::Output {

// ── PWMOutput Implementation ────────────────────────────────

PWMOutput::PWMOutput(TIM_HandleTypeDef &tim2, TIM_HandleTypeDef &tim3,
                     TIM_HandleTypeDef &tim4)
    : tim2_(tim2), tim3_(tim3), tim4_(tim4) {}

uint32_t PWMOutput::usToCCR(uint16_t us) {
  return static_cast<uint32_t>(us);
}

void PWMOutput::init() {
  HAL_TIM_PWM_Start(&tim2_, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&tim2_, TIM_CHANNEL_2);

  HAL_TIM_PWM_Start(&tim4_, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&tim4_, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&tim4_, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&tim4_, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&tim3_, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&tim3_, TIM_CHANNEL_2);

  for (uint8_t i = 0U; i < RC::HW::PWM_CHANNEL_COUNT; ++i) {
    setChannel(i, RC::FW::CHANNEL_CENTER_US);
  }
}

void PWMOutput::setChannel(uint8_t ch, uint16_t us) {
  const uint32_t ccr = usToCCR(us);
  switch (ch) {
  case 0U: __HAL_TIM_SET_COMPARE(&tim2_, TIM_CHANNEL_1, ccr); break;
  case 1U: __HAL_TIM_SET_COMPARE(&tim2_, TIM_CHANNEL_2, ccr); break;
  case 2U: __HAL_TIM_SET_COMPARE(&tim4_, TIM_CHANNEL_1, ccr); break;
  case 3U: __HAL_TIM_SET_COMPARE(&tim4_, TIM_CHANNEL_2, ccr); break;
  case 4U: __HAL_TIM_SET_COMPARE(&tim4_, TIM_CHANNEL_3, ccr); break;
  case 5U: __HAL_TIM_SET_COMPARE(&tim4_, TIM_CHANNEL_4, ccr); break;
  case 6U: __HAL_TIM_SET_COMPARE(&tim3_, TIM_CHANNEL_1, ccr); break;
  case 7U: __HAL_TIM_SET_COMPARE(&tim3_, TIM_CHANNEL_2, ccr); break;
  default: break;
  }
}

void PWMOutput::update(const RC::Channel::ChannelData &data) {
  constexpr uint8_t count = (RC::FW::PACKET_CHANNEL_COUNT < RC::HW::PWM_CHANNEL_COUNT)
                                ? RC::FW::PACKET_CHANNEL_COUNT
                                : RC::HW::PWM_CHANNEL_COUNT;
  for (uint8_t i = 0U; i < count; ++i) {
    setChannel(i, data.us[i]);
  }
}

// ── SBUSOutput Implementation ───────────────────────────────

SBUSOutput::SBUSOutput(UART_HandleTypeDef &huart) : huart_(huart) {}

uint16_t SBUSOutput::usToSBUS(uint16_t us) {
  if (us < 880U) {
    return 0U;
  }
  uint32_t sbus = (static_cast<uint32_t>(us) - 880U) * 8U / 5U;
  if (sbus > 2047U) {
    return 2047U;
  }
  return static_cast<uint16_t>(sbus);
}

void SBUSOutput::sendFrame(const RC::Channel::ChannelData &data,
                           bool isFailsafe) {
  uint8_t frame[25] = {0};
  frame[0] = 0x0FU;

  uint16_t sbusCh[16];
  for (uint8_t i = 0U; i < 16U; ++i) {
    if (i < RC::FW::PACKET_CHANNEL_COUNT) {
      sbusCh[i] = usToSBUS(data.us[i]);
    } else {
      sbusCh[i] = usToSBUS(RC::FW::CHANNEL_CENTER_US);
    }
  }

  frame[1] = static_cast<uint8_t>((sbusCh[0] & 0x07FF));
  frame[2] = static_cast<uint8_t>((sbusCh[0] & 0x07FF) >> 8 |
                                  (sbusCh[1] & 0x07FF) << 3);
  frame[3] = static_cast<uint8_t>((sbusCh[1] & 0x07FF) >> 5 |
                                  (sbusCh[2] & 0x07FF) << 6);
  frame[4] = static_cast<uint8_t>((sbusCh[2] & 0x07FF) >> 2);
  frame[5] = static_cast<uint8_t>((sbusCh[2] & 0x07FF) >> 10 |
                                  (sbusCh[3] & 0x07FF) << 1);
  frame[6] = static_cast<uint8_t>((sbusCh[3] & 0x07FF) >> 7 |
                                  (sbusCh[4] & 0x07FF) << 4);
  frame[7] = static_cast<uint8_t>((sbusCh[4] & 0x07FF) >> 4 |
                                  (sbusCh[5] & 0x07FF) << 7);
  frame[8] = static_cast<uint8_t>((sbusCh[5] & 0x07FF) >> 1);
  frame[9] = static_cast<uint8_t>((sbusCh[5] & 0x07FF) >> 9 |
                                  (sbusCh[6] & 0x07FF) << 2);
  frame[10] = static_cast<uint8_t>((sbusCh[6] & 0x07FF) >> 6 |
                                   (sbusCh[7] & 0x07FF) << 5);
  frame[11] = static_cast<uint8_t>((sbusCh[7] & 0x07FF) >> 3);
  frame[12] = static_cast<uint8_t>((sbusCh[8] & 0x07FF));
  frame[13] = static_cast<uint8_t>((sbusCh[8] & 0x07FF) >> 8 |
                                   (sbusCh[9] & 0x07FF) << 3);
  frame[14] = static_cast<uint8_t>((sbusCh[9] & 0x07FF) >> 5 |
                                   (sbusCh[10] & 0x07FF) << 6);
  frame[15] = static_cast<uint8_t>((sbusCh[10] & 0x07FF) >> 2);
  frame[16] = static_cast<uint8_t>((sbusCh[10] & 0x07FF) >> 10 |
                                   (sbusCh[11] & 0x07FF) << 1);
  frame[17] = static_cast<uint8_t>((sbusCh[11] & 0x07FF) >> 7 |
                                   (sbusCh[12] & 0x07FF) << 4);
  frame[18] = static_cast<uint8_t>((sbusCh[12] & 0x07FF) >> 4 |
                                   (sbusCh[13] & 0x07FF) << 7);
  frame[19] = static_cast<uint8_t>((sbusCh[13] & 0x07FF) >> 1);
  frame[20] = static_cast<uint8_t>((sbusCh[13] & 0x07FF) >> 9 |
                                   (sbusCh[14] & 0x07FF) << 2);
  frame[21] = static_cast<uint8_t>((sbusCh[14] & 0x07FF) >> 6 |
                                   (sbusCh[15] & 0x07FF) << 5);
  frame[22] = static_cast<uint8_t>((sbusCh[15] & 0x07FF) >> 3);

  frame[23] = 0x00U;
  if (isFailsafe) {
    frame[23] |= (1U << 3U);
  }
  frame[24] = 0x00U;

  HAL_UART_Transmit(&huart_, frame, sizeof(frame), 10U);
}

// ── IBUSOutput Implementation ───────────────────────────────

IBUSOutput::IBUSOutput(UART_HandleTypeDef &huart) : huart_(huart) {}

void IBUSOutput::sendFrame(const RC::Channel::ChannelData &data) {
  uint8_t frame[32];
  frame[0] = 0x20U;
  frame[1] = 0x40U;

  uint32_t checksum = 0xFFFFU;
  checksum -= frame[0];
  checksum -= frame[1];

  for (uint8_t i = 0U; i < 14U; ++i) {
    uint16_t us = RC::FW::CHANNEL_CENTER_US;
    if (i < RC::FW::PACKET_CHANNEL_COUNT) {
      us = data.us[i];
    }

    uint8_t lo = static_cast<uint8_t>(us & 0xFFU);
    uint8_t hi = static_cast<uint8_t>((us >> 8U) & 0xFFU);

    frame[2U + i * 2U] = lo;
    frame[3U + i * 2U] = hi;

    checksum -= lo;
    checksum -= hi;
  }

  frame[30] = static_cast<uint8_t>(checksum & 0xFFU);
  frame[31] = static_cast<uint8_t>((checksum >> 8U) & 0xFFU);

  HAL_UART_Transmit(&huart_, frame, sizeof(frame), 10U);
}

} // namespace RC::Output

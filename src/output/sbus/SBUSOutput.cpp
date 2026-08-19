// src/output/sbus/SBUSOutput.cpp
#include "output/sbus/SBUSOutput.hpp"

namespace RC::Output {

SBUSOutput::SBUSOutput(UART_HandleTypeDef &huart) : huart_(huart) {}

uint16_t SBUSOutput::usToSBUS(uint16_t us) {
  // 1000µs -> 192, 1500µs -> 992, 2000µs -> 1792
  // Math: sbus = (us - 880) * 1.6 = (us - 880) * 8 / 5
  if (us < 880U) {
    return 0U;
  }
  uint32_t sbus = (static_cast<uint32_t>(us) - 880U) * 8U / 5U;
  if (sbus > 2047U) { // 11-bit max
    return 2047U;
  }
  return static_cast<uint16_t>(sbus);
}

void SBUSOutput::sendFrame(const RC::Channel::ChannelData &data,
                           bool isFailsafe) {
  uint8_t frame[25] = {0};
  frame[0] = 0x0FU; // Header

  // 16 channels, 11 bits each
  uint16_t sbusCh[16];
  for (uint8_t i = 0U; i < 16U; ++i) {
    if (i < RC::FW::PACKET_CHANNEL_COUNT) {
      sbusCh[i] = usToSBUS(data.us[i]);
    } else {
      sbusCh[i] =
          usToSBUS(RC::FW::CHANNEL_CENTER_US); // Unused channels at center
    }
  }

  // Pack 16 11-bit values into 22 bytes
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

  // Flags
  frame[23] = 0x00U;
  if (isFailsafe) {
    frame[23] |= (1U << 3U); // Failsafe activated flag
  }

  // Footer
  frame[24] = 0x00U;

  HAL_UART_Transmit(&huart_, frame, sizeof(frame), 10U);
}

} // namespace RC::Output

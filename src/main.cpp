// src/main.cpp
#include "channel/ChannelProcessor.hpp"
#include "config/FirmwareConfig.hpp"
#include "config/HardwareConfig.hpp"
#include "cube_init.h"
#include "debug/DebugConsole.hpp"
#include "drivers/nrf24/NRF24.hpp"
#include "drivers/spi/SPIBus.hpp"
#include "failsafe/Failsafe.hpp"
#include "output/ibus/IBUSOutput.hpp"
#include "output/pwm/PWMOutput.hpp"
#include "output/sbus/SBUSOutput.hpp"
#include "protocol/BindingProtocol.hpp"
#include "protocol/PacketDecoder.hpp"
#include "stm32f1xx_hal.h"
#include "storage/ConfigStore.hpp"
#include "telemetry/Telemetry.hpp"

// Hardware handles defined in main.c
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart1; // Debug
extern UART_HandleTypeDef huart2; // SBUS
extern UART_HandleTypeDef huart3; // IBUS

// ── LED helpers (PC13 active-low) ─────────────────
static void ledOn() { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); }
static void ledOff() { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); }

static void printBootBanner(const RC::Storage::ReceiverConfig &config) {
  RC::Debug::DebugConsole::printf("\r\n");
  RC::Debug::DebugConsole::printf(
      "==================================================\r\n");
  RC::Debug::DebugConsole::printf(" [SYS] STM32 RC Receiver V2 Booting\r\n");
  RC::Debug::DebugConsole::printf(" [SYS] CPU Clock  : %lu MHz\r\n",
                                  SystemCoreClock / 1000000U);
  RC::Debug::DebugConsole::printf(
      " [SYS] Receiver ID: 0x%08lX\r\n",
      static_cast<unsigned long>(config.receiverId));
  if (config.boundTransmitterId == 0xFFFFFFFFU) {
    RC::Debug::DebugConsole::printf(
        " [SYS] Bound TX ID: NONE (Unbound / Broadcast Mode)\r\n");
  } else {
    RC::Debug::DebugConsole::printf(
        " [SYS] Bound TX ID: 0x%08lX (Bound)\r\n",
        static_cast<unsigned long>(config.boundTransmitterId));
  }
  RC::Debug::DebugConsole::printf(
      " [SYS] NRF24 CH   : %u (Addr: %02X:%02X:%02X:%02X:%02X)\r\n",
      config.radioChannel, config.radioAddr[0], config.radioAddr[1],
      config.radioAddr[2], config.radioAddr[3], config.radioAddr[4]);
  RC::Debug::DebugConsole::printf(
      " [SYS] Outputs    : 8x PWM (50Hz), SBUS (100k), IBUS (115.2k)\r\n");
  RC::Debug::DebugConsole::printf(
      "==================================================\r\n\r\n");
}

int main() {
  CubeMX_Init();

  // Initialize Debug Console (USART1)
  RC::Debug::DebugConsole::init(&huart1);

  // Load Configuration
  RC::Storage::ReceiverConfig config;
  RC::Storage::ConfigStore::load(config);

  printBootBanner(config);

  // Initialize Hardware Drivers
  RC::Drivers::SPIBus spiBus(hspi1, GPIOA, RC::HW::NRF_CSN_PIN);
  RC::Drivers::NRF24 radio(spiBus, GPIOB, RC::HW::NRF_CE_PIN);

  // Initialize Outputs
  RC::Output::PWMOutput pwmOut(htim2, htim3, htim4);
  pwmOut.init();
  RC::Output::SBUSOutput sbusOut(huart2);
  RC::Output::IBUSOutput ibusOut(huart3);

  // Initialize Processors
  RC::Channel::ChannelProcessor channelProc;
  RC::Failsafe::FailsafeController failsafe;
  RC::Telemetry::TelemetryManager telemetry(radio);

  // Initialize NRF24
  if (radio.init(config.radioChannel, config.radioAddr) !=
      RC::Drivers::Result::Ok) {
    RC::Debug::DebugConsole::printf("[ERROR] NRF24 hardware initialization "
                                    "failed! Check wiring/power.\r\n");
    while (1) {
      // 3 rapid blinks followed by 1s pause error pattern
      for (int i = 0; i < 3; i++) {
        ledOn();
        HAL_Delay(100);
        ledOff();
        HAL_Delay(150);
      }
      HAL_Delay(1000);
    }
  }
  radio.startListening();
  RC::Debug::DebugConsole::printf(
      "[SYS] Receiver Initialized & Ready! Listening for packets...\r\n");

  uint8_t payload[32];
  uint32_t lastDebugTick = HAL_GetTick();
  uint32_t lastLedPulseTick = HAL_GetTick();
  uint32_t totalPacketsReceived = 0U;
  bool pulseLedOff = false;

  while (true) {
    uint32_t currentTick = HAL_GetTick();

    // 1. Check for incoming RF data
    if (radio.isDataReady()) {
      if (radio.readPayload(payload, sizeof(payload)) ==
          RC::Drivers::Result::Ok) {
        // Decode packet
        auto res = RC::Protocol::PacketDecoder::decode(payload, sizeof(payload),
                                                       config.receiverId);

        if (res.status == RC::Protocol::DecodeStatus::Ok) {
          if (RC::Protocol::BindingProtocol::isBindPacket(*res.packet)) {
            RC::Debug::DebugConsole::printf(
                "[BIND] Bind packet received! Transmitter ID: 0x%08lX\r\n",
                static_cast<unsigned long>(res.packet->transmitterId));
            RC::Protocol::BindingProtocol::extractToConfig(*res.packet, config);
            RC::Storage::ConfigStore::save(config);
            RC::Debug::DebugConsole::printf(
                "[BIND] Configuration saved to flash. Rebooting system...\r\n");
            HAL_Delay(500);
            HAL_NVIC_SystemReset();
          } else if (res.packet->type ==
                     static_cast<uint8_t>(RC::Protocol::PacketType::Control)) {
            // Check if packet is from bound transmitter
            if (config.boundTransmitterId == res.packet->transmitterId) {
              failsafe.registerValidPacket();
              totalPacketsReceived++;

              // Process channels
              RC::Channel::ChannelData chData =
                  channelProc.process(res.packet->channels);

              // Output
              pwmOut.update(chData);
              sbusOut.sendFrame(chData, false);
              ibusOut.sendFrame(chData);

              // Telemetry ACK update
              telemetry.updateStats(100, 100, 0, 3300, 10);
              telemetry.writeAckPayload();

              // Trigger brief LED pulse off to signal active packet reception
              pulseLedOff = true;
              lastLedPulseTick = currentTick;
            }
          }
        }
      }
    }

    // 2. Failsafe Management & LED Diagnostics
    RC::Failsafe::FailsafeState state = failsafe.update(currentTick);

    if (failsafe.justEnteredFailsafe()) {
      RC::Debug::DebugConsole::printf(
          "[FAILSAFE] Signal lost! Applying failsafe values...\r\n");
    } else if (failsafe.justRecovered()) {
      RC::Debug::DebugConsole::printf(
          "[FAILSAFE] Signal recovered! Restoring control...\r\n");
      channelProc.resetFilter();
    }

    if (state == RC::Failsafe::FailsafeState::Failsafe) {
      RC::Channel::ChannelData fsData = channelProc.applyFailsafe();
      pwmOut.update(fsData);
      sbusOut.sendFrame(fsData, true);
      ibusOut.sendFrame(fsData);

      // Failsafe status LED: non-blocking 5Hz blink (100ms ON / 100ms OFF)
      if ((currentTick / 100U) % 2U == 0U) {
        ledOn();
      } else {
        ledOff();
      }
    } else {
      // Normal Active state LED: Solid ON, with 15ms pulse OFF on packet
      // reception
      if (pulseLedOff) {
        ledOff();
        if ((currentTick - lastLedPulseTick) >= 15U) {
          pulseLedOff = false;
        }
      } else {
        ledOn();
      }
    }

    // 3. Periodic Debug Telemetry & Heartbeat (1 Hz)
    if ((currentTick - lastDebugTick) >= 1000U) {
      lastDebugTick = currentTick;
      RC::Debug::DebugConsole::printf(
          "[STAT] Uptime: %lu s | Packets Rx: %lu | Status: %s\r\n",
          currentTick / 1000U, static_cast<unsigned long>(totalPacketsReceived),
          (state == RC::Failsafe::FailsafeState::Active)
              ? "READY / ACTIVE"
              : "FAILSAFE / SIGNAL LOST");
    }
  }
}
// src/main.cpp
#include "cube_init.h"
#include "config/HardwareConfig.hpp"
#include "config/FirmwareConfig.hpp"
#include "drivers/spi/SPIBus.hpp"
#include "drivers/nrf24/NRF24.hpp"
#include "storage/ConfigStore.hpp"
#include "protocol/PacketDecoder.hpp"
#include "protocol/BindingProtocol.hpp"
#include "channel/ChannelProcessor.hpp"
#include "output/pwm/PWMOutput.hpp"
#include "output/sbus/SBUSOutput.hpp"
#include "output/ibus/IBUSOutput.hpp"
#include "failsafe/Failsafe.hpp"
#include "telemetry/Telemetry.hpp"
#include "debug/DebugConsole.hpp"
#include "stm32f1xx_hal.h"

// Hardware handles defined in main.c
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart1; // Debug
extern UART_HandleTypeDef huart2; // SBUS
extern UART_HandleTypeDef huart3; // IBUS

// ── LED helpers (PC13 active-low) ─────────────────
static void ledOn()  { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); }
static void ledOff() { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); }

int main() {
    CubeMX_Init();

    // Initialize Debug Console (USART1)
    RC::Debug::DebugConsole::init(&huart1);
    RC::Debug::DebugConsole::printf("RC Receiver V2 Booting...\r\n");

    // Load Configuration
    RC::Storage::ReceiverConfig config;
    RC::Storage::ConfigStore::load(config);
    RC::Debug::DebugConsole::printf("Config Loaded. RX ID: 0x%08lX, Bound TX: 0x%08lX\r\n", 
                                    config.receiverId, config.boundTransmitterId);

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
    if (!radio.init(config.radioChannel, config.radioAddr)) {
        RC::Debug::DebugConsole::printf("NRF24 init failed! Check wiring.\r\n");
        while(1) {
            ledOn(); HAL_Delay(100); ledOff(); HAL_Delay(100);
        }
    }
    radio.startListening();
    RC::Debug::DebugConsole::printf("NRF24 Listening on CH %u\r\n", config.radioChannel);
    
    // Initial LED State
    ledOn(); // Solid LED = Ready

    uint8_t payload[32];
    uint32_t lastDebugTick = HAL_GetTick();

    while (true) {
        uint32_t currentTick = HAL_GetTick();

        // 1. Check for incoming RF data
        if (radio.hasData()) {
            if (radio.readPayload(payload, sizeof(payload)) == RC::Drivers::Result::Ok) {
                // Decode packet
                auto res = RC::Protocol::PacketDecoder::decode(payload, sizeof(payload), config.receiverId);
                
                if (res.status == RC::Protocol::DecodeStatus::Ok) {
                    if (RC::Protocol::BindingProtocol::isBindPacket(*res.packet)) {
                        RC::Debug::DebugConsole::printf("Bind packet received! TX ID: 0x%08lX\r\n", res.packet->transmitterId);
                        RC::Protocol::BindingProtocol::extractToConfig(*res.packet, config);
                        RC::Storage::ConfigStore::save(config);
                        RC::Debug::DebugConsole::printf("Bind config saved. Rebooting...\r\n");
                        HAL_Delay(500);
                        HAL_NVIC_SystemReset();
                    } else if (res.packet->type == static_cast<uint8_t>(RC::Protocol::PacketType::Control)) {
                        // Check if packet is from bound transmitter
                        if (config.boundTransmitterId == res.packet->transmitterId) {
                            failsafe.registerValidPacket();
                            
                            // Process channels
                            RC::Channel::ChannelData chData = channelProc.process(res.packet->channels);
                            
                            // Output
                            pwmOut.update(chData);
                            sbusOut.sendFrame(chData, false);
                            ibusOut.sendFrame(chData);

                            // Telemetry ACK update
                            // Mocking RSSI and Voltage for now
                            telemetry.updateStats(100, 100, 0, 3300, 10);
                            telemetry.writeAckPayload();
                            
                            // Flash LED briefly to indicate packet rx
                            ledOff(); 
                        }
                    }
                }
            }
        }

        // 2. Failsafe Management
        RC::Failsafe::FailsafeState state = failsafe.update(currentTick);
        
        if (failsafe.justEnteredFailsafe()) {
            RC::Debug::DebugConsole::printf("FAILSAFE ACTIVATED\r\n");
        } else if (failsafe.justRecovered()) {
            RC::Debug::DebugConsole::printf("RECOVERED FROM FAILSAFE\r\n");
            channelProc.resetFilter();
            ledOn(); // Ensure LED is on after recovery
        }

        if (state == RC::Failsafe::FailsafeState::Failsafe) {
            RC::Channel::ChannelData fsData = channelProc.applyFailsafe();
            pwmOut.update(fsData);
            sbusOut.sendFrame(fsData, true);
            ibusOut.sendFrame(fsData);
            
            // Blink LED rapidly in failsafe (10 Hz)
            if ((currentTick % 100U) < 50U) {
                ledOn();
            } else {
                ledOff();
            }
        } else {
            // Restore LED if it was flashed by a packet (extend flash time slightly)
            if ((currentTick % 20U) == 0U) {
                ledOn();
            }
        }

        // 3. Debug Output (1 Hz)
        if ((currentTick - lastDebugTick) >= 1000U) {
            lastDebugTick = currentTick;
            // Can print status here if needed
            // RC::Debug::DebugConsole::printf("Heartbeat: %lu\r\n", currentTick);
        }
    }
}
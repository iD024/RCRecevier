#include "output/pwm/PWMOutput.hpp"
#include "config/FirmwareConfig.hpp"
#include "config/HardwareConfig.hpp"

namespace RC::Output {

PWMOutput::PWMOutput(TIM_HandleTypeDef& tim2,
                     TIM_HandleTypeDef& tim3,
                     TIM_HandleTypeDef& tim4)
    : tim2_(tim2), tim3_(tim3), tim4_(tim4) {}

uint32_t PWMOutput::usToCCR(uint16_t us) {
    // Timer prescaler = 71, timer clock = 72MHz / (71+1) = 1MHz
    // 1 tick = 1µs → CCR = us directly
    return static_cast<uint32_t>(us);
}

void PWMOutput::init() {
    // TIM2: CH1 = PA0 (RC CH1), CH2 = PA1 (RC CH2)
    HAL_TIM_PWM_Start(&tim2_, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&tim2_, TIM_CHANNEL_2);

    // TIM4: CH1 = PB6 (RC CH3), CH2 = PB7 (RC CH4),
    //        CH3 = PB8 (RC CH5), CH4 = PB9 (RC CH6)
    HAL_TIM_PWM_Start(&tim4_, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&tim4_, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&tim4_, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&tim4_, TIM_CHANNEL_4);

    // TIM3 partial remap: CH1 = PB4 (RC CH7), CH2 = PB5 (RC CH8)
    HAL_TIM_PWM_Start(&tim3_, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&tim3_, TIM_CHANNEL_2);

    // Initialize all channels to center position
    for (uint8_t i = 0U; i < RC::HW::PWM_CHANNEL_COUNT; ++i) {
        setChannel(i, RC::FW::CHANNEL_CENTER_US);
    }
}

void PWMOutput::setChannel(uint8_t ch, uint16_t us) {
    const uint32_t ccr = usToCCR(us);
    switch (ch) {
        case 0U: __HAL_TIM_SET_COMPARE(&tim2_, TIM_CHANNEL_1, ccr); break; // PA0
        case 1U: __HAL_TIM_SET_COMPARE(&tim2_, TIM_CHANNEL_2, ccr); break; // PA1
        case 2U: __HAL_TIM_SET_COMPARE(&tim4_, TIM_CHANNEL_1, ccr); break; // PB6
        case 3U: __HAL_TIM_SET_COMPARE(&tim4_, TIM_CHANNEL_2, ccr); break; // PB7
        case 4U: __HAL_TIM_SET_COMPARE(&tim4_, TIM_CHANNEL_3, ccr); break; // PB8
        case 5U: __HAL_TIM_SET_COMPARE(&tim4_, TIM_CHANNEL_4, ccr); break; // PB9
        case 6U: __HAL_TIM_SET_COMPARE(&tim3_, TIM_CHANNEL_1, ccr); break; // PB4 (TIM3 partial remap)
        case 7U: __HAL_TIM_SET_COMPARE(&tim3_, TIM_CHANNEL_2, ccr); break; // PB5 (TIM3 partial remap)
        default: break;
    }
}

void PWMOutput::update(const RC::Channel::ChannelData& data) {
    constexpr uint8_t count = (RC::FW::PACKET_CHANNEL_COUNT < RC::HW::PWM_CHANNEL_COUNT) 
                              ? RC::FW::PACKET_CHANNEL_COUNT 
                              : RC::HW::PWM_CHANNEL_COUNT;
    for (uint8_t i = 0U; i < count; ++i) {
        setChannel(i, data.us[i]);
    }
}

} // namespace RC::Output

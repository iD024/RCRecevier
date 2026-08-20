// test/test_channel_processor/test_channel_processor.cpp
#include <unity.h>
#include "core/channel_processor.hpp"
#include <cstdlib>

using namespace RC::Channel;

ChannelProcessor* proc;

void setUp()    { proc = new ChannelProcessor(); }
void tearDown() { delete proc; proc = nullptr; }

void test_normal_values_pass_through() {
    uint16_t raw[8] = {1000, 1200, 1500, 1700, 2000, 1500, 1500, 1500};
    ChannelData d = proc->process(raw);
    TEST_ASSERT_EQUAL_UINT16(1100U, d.us[0]);
}

void test_out_of_range_clamped() {
    uint16_t raw[8] = {500, 500, 500, 500, 500, 500, 500, 500};
    ChannelData d = proc->process(raw);
    for (uint8_t i = 0; i < 8; ++i) {
        TEST_ASSERT_EQUAL_UINT16(1100U, d.us[i]);
    }
}

void test_deadband_at_center() {
    uint16_t raw[8] = {1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502};
    ChannelData d = proc->process(raw);
    for (uint8_t i = 0; i < 8; ++i) {
        TEST_ASSERT_EQUAL_UINT16(1500U, d.us[i]);
    }
}

void test_failsafe_cuts_throttle() {
    ChannelData d = proc->applyFailsafe();
    TEST_ASSERT_EQUAL_UINT16(1000U, d.us[2]); // throttle cut
    TEST_ASSERT_EQUAL_UINT16(1500U, d.us[0]); // other channels at center
    TEST_ASSERT_EQUAL_UINT16(1500U, d.us[7]); // last channel at center
}

void test_reset_filter_restores_center() {
    uint16_t raw[8] = {2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000};
    proc->process(raw);
    proc->resetFilter();
    ChannelData d = proc->applyFailsafe();
    TEST_ASSERT_EQUAL_UINT16(1000U, d.us[2]);
    TEST_ASSERT_EQUAL_UINT16(1500U, d.us[0]);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_normal_values_pass_through);
    RUN_TEST(test_out_of_range_clamped);
    RUN_TEST(test_deadband_at_center);
    RUN_TEST(test_failsafe_cuts_throttle);
    RUN_TEST(test_reset_filter_restores_center);
    return UNITY_END();
}

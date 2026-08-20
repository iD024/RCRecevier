// test/test_crc16/test_crc16.cpp
#include <unity.h>
#include "protocol/protocol.hpp"

using namespace RC::Protocol;

void setUp()    {}
void tearDown() {}

void test_crc16_known_vector() {
    // "123456789" → CRC-16/CCITT-FALSE = 0x29B1 (standard test vector)
    const uint8_t data[] = {'1','2','3','4','5','6','7','8','9'};
    TEST_ASSERT_EQUAL_HEX16(0x29B1U, CRC16::compute(data, sizeof(data)));
}

void test_crc16_empty_data() {
    // Zero length returns INIT without dereferencing data
    TEST_ASSERT_EQUAL_HEX16(0xFFFFU, CRC16::compute(nullptr, 0U));
}

void test_crc16_verify_valid() {
    const uint8_t data[] = {'1','2','3','4','5','6','7','8','9'};
    TEST_ASSERT_TRUE(CRC16::verify(data, sizeof(data), 0x29B1U));
}

void test_crc16_verify_corrupted() {
    uint8_t data[] = {'1','2','3','4','5','6','7','8','9'};
    data[4] ^= 0xFFU; // corrupt one byte
    TEST_ASSERT_FALSE(CRC16::verify(data, sizeof(data), 0x29B1U));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_crc16_known_vector);
    RUN_TEST(test_crc16_empty_data);
    RUN_TEST(test_crc16_verify_valid);
    RUN_TEST(test_crc16_verify_corrupted);
    return UNITY_END();
}

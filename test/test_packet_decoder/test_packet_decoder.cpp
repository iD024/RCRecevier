// test/test_packet_decoder/test_packet_decoder.cpp
#include <unity.h>
#include "protocol/PacketDecoder.hpp"
#include "protocol/CRC16.hpp"
#include <cstring>

using namespace RC::Protocol;

static Packet makeValidPacket(uint32_t rxId = 0xDEADBEEFU) {
    Packet p{};
    p.magic0         = 0xACU;
    p.magic1         = 0x24U;
    p.version        = 0x01U;
    p.type           = static_cast<uint8_t>(PacketType::Control);
    p.transmitterId  = 0x12345678U;
    p.receiverId     = rxId;
    p.sequenceNumber = 42U;
    p.flags          = 0x00U;
    for (uint8_t i = 0U; i < 8U; ++i) p.channels[i] = 1500U;
    constexpr uint16_t CRC_LEN = sizeof(Packet) - sizeof(uint16_t);
    p.crc = CRC16::compute(reinterpret_cast<uint8_t*>(&p), CRC_LEN);
    return p;
}

void setUp()    {}
void tearDown() {}

void test_valid_packet_accepted() {
    Packet p = makeValidPacket();
    auto r = PacketDecoder::decode(reinterpret_cast<uint8_t*>(&p), sizeof(p), 0xDEADBEEFU);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DecodeStatus::Ok),
                            static_cast<uint8_t>(r.status));
    TEST_ASSERT_NOT_NULL(r.packet);
}

void test_bad_magic_rejected() {
    Packet p = makeValidPacket();
    p.magic0 = 0x00U;
    auto r = PacketDecoder::decode(reinterpret_cast<uint8_t*>(&p), sizeof(p), 0xDEADBEEFU);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DecodeStatus::BadMagic),
                            static_cast<uint8_t>(r.status));
}

void test_bad_crc_rejected() {
    Packet p = makeValidPacket();
    p.crc ^= 0xFFFFU; // corrupt CRC
    auto r = PacketDecoder::decode(reinterpret_cast<uint8_t*>(&p), sizeof(p), 0xDEADBEEFU);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DecodeStatus::BadCRC),
                            static_cast<uint8_t>(r.status));
}

void test_wrong_receiver_id_rejected() {
    Packet p = makeValidPacket(0x11111111U);
    auto r = PacketDecoder::decode(reinterpret_cast<uint8_t*>(&p), sizeof(p), 0xDEADBEEFU);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DecodeStatus::ReceiverIdMismatch),
                            static_cast<uint8_t>(r.status));
}

void test_broadcast_id_accepted_always() {
    Packet p = makeValidPacket(0xFFFFFFFFU); // broadcast
    auto r = PacketDecoder::decode(reinterpret_cast<uint8_t*>(&p), sizeof(p), 0xDEADBEEFU);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DecodeStatus::Ok),
                            static_cast<uint8_t>(r.status));
}

void test_channel_out_of_range_rejected() {
    Packet p = makeValidPacket();
    p.channels[3] = 2500U; // out of range
    // Recompute valid CRC so CRC check passes but channel check fails
    constexpr uint16_t CRC_LEN = sizeof(Packet) - sizeof(uint16_t);
    p.crc = CRC16::compute(reinterpret_cast<uint8_t*>(&p), CRC_LEN);
    auto r = PacketDecoder::decode(reinterpret_cast<uint8_t*>(&p), sizeof(p), 0xDEADBEEFU);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DecodeStatus::ChannelOutOfRange),
                            static_cast<uint8_t>(r.status));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_valid_packet_accepted);
    RUN_TEST(test_bad_magic_rejected);
    RUN_TEST(test_bad_crc_rejected);
    RUN_TEST(test_wrong_receiver_id_rejected);
    RUN_TEST(test_broadcast_id_accepted_always);
    RUN_TEST(test_channel_out_of_range_rejected);
    return UNITY_END();
}

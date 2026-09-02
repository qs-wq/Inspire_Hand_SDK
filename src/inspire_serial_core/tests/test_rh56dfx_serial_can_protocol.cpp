#include <gtest/gtest.h>

#include <vector>

#include "RH56DFX_serial_can_protocol.hpp"

namespace {

class TestableRH56DFXSerialCanProtocol final : public RH56DFX_serial_can_Protocol {
public:
    using RH56DFX_serial_can_Protocol::decodeValuesByRule;
    using RH56DFX_serial_can_Protocol::getDefaultReadLength;
};

TestableRH56DFXSerialCanProtocol makeProto() {
    TestableRH56DFXSerialCanProtocol p;
    p.setDeviceId(1);
    return p;
}

uint32_t decodeCanId(const std::vector<uint8_t>& frame) {
    return static_cast<uint32_t>(frame[2]) |
           (static_cast<uint32_t>(frame[3]) << 8) |
           (static_cast<uint32_t>(frame[4]) << 16) |
           (static_cast<uint32_t>(frame[5]) << 24);
}

}  // namespace

// 寄存器名 -> 地址映射
TEST(RH56DFXSerialCanProtocol, RegisterAddressLookup) {
    auto p = makeProto();
    EXPECT_EQ(p.getRegisterAddress("angleSet"), 1486);
    EXPECT_EQ(p.getRegisterAddress("temp"), 1618);
    EXPECT_EQ(p.getRegisterAddress("not_exists"), -1);
}

// temp/errorCode/status 默认读取长度应为 6 字节（每指 1 字节）
TEST(RH56DFXSerialCanProtocol, ReadLengthForSingleByteFingerRegisters) {
    auto p = makeProto();
    EXPECT_EQ(p.getDefaultReadLength("temp"), 6u);
    EXPECT_EQ(p.getDefaultReadLength("errorCode"), 6u);
    EXPECT_EQ(p.getDefaultReadLength("status"), 6u);
}

// 读命令帧结构正确（帧头帧尾、CAN ID、长度字段、校验和）
TEST(RH56DFXSerialCanProtocol, BuildReadCommandFrameFormat) {
    auto p = makeProto();
    auto frame = p.buildReadCommand(1618, 6);  // temp

    ASSERT_EQ(frame.size(), 21u);
    EXPECT_EQ(frame[0], 0xAA);
    EXPECT_EQ(frame[1], 0xAA);
    EXPECT_EQ(frame[19], 0x55);
    EXPECT_EQ(frame[20], 0x55);

    const uint32_t can_id = decodeCanId(frame);
    const uint8_t rw_flag = static_cast<uint8_t>((can_id >> 26) & 0x07);
    const int address = static_cast<int>((can_id >> 14) & 0x0FFF);
    const int hand_id = static_cast<int>(can_id & 0x3FFF);
    EXPECT_EQ(rw_flag, 0u);
    EXPECT_EQ(address, 1618);
    EXPECT_EQ(hand_id, 1);

    EXPECT_EQ(frame[6], 6u);  // 读取长度
    for (size_t i = 7; i < 14; ++i) {
        EXPECT_EQ(frame[i], 0u);
    }
    EXPECT_EQ(frame[14], 0x01);
    EXPECT_TRUE(p.validateChecksum(frame));
}

// 写命令帧结构正确（数据区小端序，写长度字段=有效数据字节数）
TEST(RH56DFXSerialCanProtocol, BuildWriteCommandFrameFormat) {
    auto p = makeProto();
    auto frame = p.buildWriteCommand(1486, {100, -1});  // angleSet

    ASSERT_EQ(frame.size(), 21u);
    EXPECT_EQ(frame[0], 0xAA);
    EXPECT_EQ(frame[1], 0xAA);
    EXPECT_EQ(frame[19], 0x55);
    EXPECT_EQ(frame[20], 0x55);

    const uint32_t can_id = decodeCanId(frame);
    const uint8_t rw_flag = static_cast<uint8_t>((can_id >> 26) & 0x07);
    const int address = static_cast<int>((can_id >> 14) & 0x0FFF);
    const int hand_id = static_cast<int>(can_id & 0x3FFF);
    EXPECT_EQ(rw_flag, 1u);
    EXPECT_EQ(address, 1486);
    EXPECT_EQ(hand_id, 1);

    // payload: 100 -> 0x64 0x00, -1 -> 0xFF 0xFF
    EXPECT_EQ(frame[6], 0x64);
    EXPECT_EQ(frame[7], 0x00);
    EXPECT_EQ(frame[8], 0xFF);
    EXPECT_EQ(frame[9], 0xFF);
    EXPECT_EQ(frame[14], 0x04);
    EXPECT_TRUE(p.validateChecksum(frame));
}

// temp/errorCode/status 按 1 字节解码，避免被拼成 16 位值（如 514）
TEST(RH56DFXSerialCanProtocol, DecodeSingleByteFingerRegisters) {
    auto p = makeProto();

    const auto temp_values = p.decodeValuesByRule("temp", {0x26, 0x28, 0x28, 0x26, 0x26, 0x26});
    EXPECT_EQ(temp_values, (std::vector<int>{38, 40, 40, 38, 38, 38}));

    const auto error_values = p.decodeValuesByRule("errorCode", {0x00, 0x00, 0x00, 0x02, 0x02, 0x02});
    EXPECT_EQ(error_values, (std::vector<int>{0, 0, 0, 2, 2, 2}));

    const auto status_values = p.decodeValuesByRule("status", {0x01, 0x01, 0x02, 0x02, 0x03, 0x03});
    EXPECT_EQ(status_values, (std::vector<int>{1, 1, 2, 2, 3, 3}));
}

// 其它寄存器继续按 2 字节有符号值解码
TEST(RH56DFXSerialCanProtocol, DecodeTwoByteSignedForRegularRegisters) {
    auto p = makeProto();
    auto values = p.decodeValuesByRule("angleAct", {0x64, 0x00, 0xFF, 0xFF});
    EXPECT_EQ(values, (std::vector<int>{100, -1}));
}

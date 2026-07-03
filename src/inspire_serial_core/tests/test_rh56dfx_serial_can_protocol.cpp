#include <gtest/gtest.h>

#include <cstdint>
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

uint32_t buildCanIdForTest(uint8_t rw_flag, int address, int hand_id) {
    const uint32_t rw = static_cast<uint32_t>(rw_flag & 0x07) << 26;
    const uint32_t reg = static_cast<uint32_t>(address & 0x0FFF) << 14;
    const uint32_t hid = static_cast<uint32_t>(hand_id) & 0x3FFF;
    return rw | reg | hid;
}

std::vector<uint8_t> buildExpectedSerialCanFrame(
    uint32_t can_id,
    const std::vector<uint8_t>& payload8,
    uint8_t payload_len_field) {
    std::vector<uint8_t> frame = {
        0xAA,
        0xAA,
        static_cast<uint8_t>(can_id & 0xFF),
        static_cast<uint8_t>((can_id >> 8) & 0xFF),
        static_cast<uint8_t>((can_id >> 16) & 0xFF),
        static_cast<uint8_t>((can_id >> 24) & 0xFF),
    };
    frame.insert(frame.end(), payload8.begin(), payload8.end());
    frame.push_back(payload_len_field);
    frame.push_back(0x00);
    frame.push_back(0x01);
    frame.push_back(0x00);

    uint8_t checksum = 0;
    for (size_t i = 2; i < frame.size(); ++i) {
        checksum = static_cast<uint8_t>(checksum + frame[i]);
    }
    frame.push_back(checksum);
    frame.push_back(0x55);
    frame.push_back(0x55);
    return frame;
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

// 读命令字节序列正确（完整帧比对）
TEST(RH56DFXSerialCanProtocol, BuildReadCommand) {
    auto p = makeProto();
    auto cmd = p.buildReadCommand(1618, 6);  // temp

    const uint32_t can_id = buildCanIdForTest(0, 1618, 1);
    std::vector<uint8_t> expected_payload = {0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    auto expected = buildExpectedSerialCanFrame(can_id, expected_payload, 0x01);
    EXPECT_EQ(cmd, expected);
}

// 写命令字节序列正确（多值小端序 + 完整帧比对）
TEST(RH56DFXSerialCanProtocol, BuildWriteCommand) {
    auto p = makeProto();
    auto cmd = p.buildWriteCommand(1486, {100, -1});  // angleSet

    const uint32_t can_id = buildCanIdForTest(1, 1486, 1);
    std::vector<uint8_t> expected_payload = {0x64, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    auto expected = buildExpectedSerialCanFrame(can_id, expected_payload, 0x04);
    EXPECT_EQ(cmd, expected);
}

// 校验和验证：正确帧通过，篡改后失败
TEST(RH56DFXSerialCanProtocol, ValidateChecksum) {
    auto p = makeProto();
    const uint32_t can_id = buildCanIdForTest(0, 1618, 1);
    auto frame = buildExpectedSerialCanFrame(can_id, {0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 0x01);
    EXPECT_TRUE(p.validateChecksum(frame));

    frame[18] ^= 0xFF;  // 破坏校验和
    EXPECT_FALSE(p.validateChecksum(frame));
}

// 帧头/帧尾错误应拒绝
TEST(RH56DFXSerialCanProtocol, ValidateChecksumRejectsWrongHeaderOrTail) {
    auto p = makeProto();
    const uint32_t can_id = buildCanIdForTest(1, 1486, 1);
    auto frame = buildExpectedSerialCanFrame(can_id, {0x64, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, 0x04);

    frame[0] = 0xEB;
    EXPECT_FALSE(p.validateChecksum(frame));

    frame = buildExpectedSerialCanFrame(can_id, {0x64, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, 0x04);
    frame[20] = 0xAA;
    EXPECT_FALSE(p.validateChecksum(frame));
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

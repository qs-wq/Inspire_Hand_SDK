#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "EG2_4C2_serial_can_protocol.hpp"

namespace {

class TestableEG2_4C2SerialCanProtocol final : public EG2_4C2_serial_can_Protocol {
public:
    using EG2_4C2_serial_can_Protocol::buildCanId;
    using EG2_4C2_serial_can_Protocol::buildSerialCanFrame;
    using EG2_4C2_serial_can_Protocol::removeA5Escape;
    using EG2_4C2_serial_can_Protocol::parseAndValidateFrame;
    using EG2_4C2_serial_can_Protocol::getDefaultReadLength;
};

TestableEG2_4C2SerialCanProtocol makeProto() {
    TestableEG2_4C2SerialCanProtocol p;
    p.setDeviceId(1);
    return p;
}

// 构造一个完整的 Serial-CAN 帧（21 字节），不含转义字节
std::vector<uint8_t> buildExpectedSerialCanFrame(
    uint32_t can_id,
    const std::vector<uint8_t>& payload8,
    uint8_t payload_len_field,
    bool is_read) {
    (void)is_read;
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

// ============================================================================
// 寄存器名 -> 地址映射
// ============================================================================
TEST(EG2_4C2SerialCanProtocol, RegisterAddressLookup) {
    auto p = makeProto();
    EXPECT_EQ(p.getRegisterAddress("openLenSet"), 1010);
    EXPECT_EQ(p.getRegisterAddress("speedSet"), 1011);
    EXPECT_EQ(p.getRegisterAddress("forceSet"), 1012);
    EXPECT_EQ(p.getRegisterAddress("gripperStatusBlock"), 1060);
    EXPECT_EQ(p.getRegisterAddress("not_exists"), -1);
}

// 默认读取长度（gripperStatusBlock=12B 跨双帧，单值=2B）
TEST(EG2_4C2SerialCanProtocol, ReadLengthDefaults) {
    auto p = makeProto();
    EXPECT_EQ(p.getDefaultReadLength("openLenSet"), 2u);
    EXPECT_EQ(p.getDefaultReadLength("forceAct"), 2u);
    EXPECT_EQ(p.getDefaultReadLength("gripperStatusBlock"), 12u);
}

// 29 位 ExtId 编码（bit27~26 = op_type, bit25~14 = addr, bit13~0 = hid）
// buildCanId(op, addr) = (op<<26) | (addr<<14) | 1
// openLenSet=1010(0x3F2): 1010<<14 = 0xFC8000
TEST(EG2_4C2SerialCanProtocol, BuildCanId) {
    auto p = makeProto();
    // 读 openLenSet (1010): 0<<26 | 0xFC8000 | 1 = 0x00FC8001
    EXPECT_EQ(p.buildCanId(0, 1010), 0x00FC8001u);
    // 写 openLenSet (1010): 1<<26 | 0xFC8000 | 1 = 0x04FC8001
    EXPECT_EQ(p.buildCanId(1, 1010), 0x04FC8001u);
    // 写 speedSet (1011): 1<<26 | (1011<<14) | 1 = 0x04FCC001
    EXPECT_EQ(p.buildCanId(1, 1011), 0x04FCC001u);
    // 定位 openLenSet: 2<<26 | 0xFC8000 | 1 = 0x08FC8001
    EXPECT_EQ(p.buildCanId(2, 1010), 0x08FC8001u);
    // 随动 openLenSet: 3<<26 | 0xFC8000 | 1 = 0x0CFC8001
    EXPECT_EQ(p.buildCanId(3, 1010), 0x0CFC8001u);
}

// 读命令字节序列正确（读 openLenSet 2B）
// 与 gripper_demo_can_serial.py build_read_frame 对齐：ExtId=0x00FC8001, Meta[0]=0x01
TEST(EG2_4C2SerialCanProtocol, BuildReadCommand) {
    auto p = makeProto();
    auto cmd = p.buildReadCommand(1010, 2);  // openLenSet, 读 2 字节

    const uint32_t can_id = 0x00FC8001u;
    std::vector<uint8_t> expected_payload = {0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    auto expected = buildExpectedSerialCanFrame(can_id, expected_payload, 0x01, true);
    EXPECT_EQ(cmd, expected);
}

// 写命令字节序列正确（写 openLenSet=800, 数据低字节在前）
// 单寄存器有效数据为 2 字节，因此 Meta[0]（CAN DLC）必须为 0x02。
TEST(EG2_4C2SerialCanProtocol, BuildWriteCommand) {
    auto p = makeProto();
    auto cmd = p.buildWriteCommand(1010, {800});  // openLenSet=800 (0x0320)

    const uint32_t can_id = 0x04FC8001u;
    std::vector<uint8_t> expected_payload = {0x20, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    auto expected = buildExpectedSerialCanFrame(can_id, expected_payload, 0x02, false);
    EXPECT_EQ(cmd, expected);
}

// 写三连寄存器（position + speed + force，从 openLenSet 起始地址连写 3 个寄存器）
// 三寄存器有效数据为 6 字节，因此 Meta[0]（CAN DLC）必须为 0x06。
TEST(EG2_4C2SerialCanProtocol, BuildWriteCommandMultiValues) {
    auto p = makeProto();
    // position=800 (0x0320), speed=800 (0x0320), force=500 (0x01F4)
    auto cmd = p.buildWriteCommand(1010, {800, 800, 500});

    const uint32_t can_id = 0x04FC8001u;
    std::vector<uint8_t> expected_payload = {
        0x20, 0x03,    // 800
        0x20, 0x03,    // 800
        0xF4, 0x01,    // 500
        0xFF, 0xFF,    // 填充
    };
    auto expected = buildExpectedSerialCanFrame(can_id, expected_payload, 0x06, false);
    EXPECT_EQ(cmd, expected);
}

// 校验和验证：正确帧通过，篡改后失败
TEST(EG2_4C2SerialCanProtocol, ValidateChecksum) {
    auto p = makeProto();
    const uint32_t can_id = 0x00FC8001u;
    std::vector<uint8_t> expected_payload = {0x02, 0, 0, 0, 0, 0, 0, 0};
    auto frame = buildExpectedSerialCanFrame(can_id, expected_payload, 0x01, true);
    EXPECT_TRUE(p.validateChecksum(frame));

    // 篡改校验位
    frame[18] ^= 0xFF;
    EXPECT_FALSE(p.validateChecksum(frame));
}

// 校验和验证：长度非法
TEST(EG2_4C2SerialCanProtocol, ValidateChecksumRejectsWrongLength) {
    auto p = makeProto();
    std::vector<uint8_t> frame(10, 0xAA);
    EXPECT_FALSE(p.validateChecksum(frame));
}

// 校验和验证：错误的帧头/帧尾
TEST(EG2_4C2SerialCanProtocol, ValidateChecksumRejectsWrongHeader) {
    auto p = makeProto();
    auto frame = buildExpectedSerialCanFrame(0x03F28001u, {0x02}, 0x01, true);
    frame[0] = 0x55;  // 篡改帧头
    EXPECT_FALSE(p.validateChecksum(frame));
}

// parseAndValidateFrame: 应答帧合法（同 read_can_id + 同 address + 同 hid）
// ExtId=0x00FC8001, payload=[0x20,0x03,0,0,0,0,0,0] (openLenSet=800)
TEST(EG2_4C2SerialCanProtocol, ParseAndValidateFrameOk) {
    auto p = makeProto();
    const uint32_t can_id = 0x00FC8001u;
    std::vector<uint8_t> expected_payload = {0x20, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    auto frame = buildExpectedSerialCanFrame(can_id, expected_payload, 0x02, true);

    std::vector<uint8_t> out_payload;
    uint8_t out_len = 0;
    ASSERT_TRUE(p.parseAndValidateFrame(frame, /*op=*/0, /*addr=*/1010, &out_payload, &out_len));
    EXPECT_EQ(out_len, 0x02);
    ASSERT_EQ(out_payload.size(), 8u);
    EXPECT_EQ(out_payload[0], 0x20);
    EXPECT_EQ(out_payload[1], 0x03);
}

// parseAndValidateFrame: 设备 ID 不匹配应拒绝
TEST(EG2_4C2SerialCanProtocol, ParseAndValidateFrameRejectsHandIdMismatch) {
    auto p = makeProto();
    // 构造一个 hand_id=2 的应答
    const uint32_t can_id = (0u << 26) | (1010u << 14) | 2u;
    auto frame = buildExpectedSerialCanFrame(can_id, {0x02}, 0x01, true);

    std::vector<uint8_t> out_payload;
    uint8_t out_len = 0;
    EXPECT_FALSE(p.parseAndValidateFrame(frame, 0, 1010, &out_payload, &out_len));
}

// parseAndValidateFrame: op_type 不匹配应拒绝
// 帧 ExtId=0x08FC8001 (op=2 定位) 但期望读帧 (op=0)
TEST(EG2_4C2SerialCanProtocol, ParseAndValidateFrameRejectsOpTypeMismatch) {
    auto p = makeProto();
    const uint32_t can_id = 0x08FC8001u;
    auto frame = buildExpectedSerialCanFrame(can_id, {0x02}, 0x01, true);

    std::vector<uint8_t> out_payload;
    uint8_t out_len = 0;
    EXPECT_FALSE(p.parseAndValidateFrame(frame, 0, 1010, &out_payload, &out_len));
}

// A5 转义还原：发送时若 ExtId/数据区出现 0xAA/0x55/0xA5（与帧头/帧尾/标识符冲突），
// 被转义为 A5 + 原字节；接收时需还原。与 gripper_demo_can_serial.py unescape_a5()
// (行 128-139) 行为对齐：仅当 A5 后跟 55/AA/A5 时才视为转义。
// 测试 ExtId=0x00AA8001 小端 [01,80,AA,00]，byte[2]=0xAA -> 转义为 A5 AA。
TEST(EG2_4C2SerialCanProtocol, RemoveA5Escape) {
    auto p = makeProto();
    // 转义帧（不含帧头 AA AA，从 ExtId 起始）
    // ExtId=0x00AA8001 小端: [01,80,AA,00] -> 发送时 [01,80,A5,AA,00]
    // data=[02,00,...], meta=[01,00,01,00], chk=0xD4, tail=[55,55]
    std::vector<uint8_t> raw = {
        0x01, 0x80, 0xA5, 0xAA, 0x00,  // ExtId 小端，A5 AA 替代原 0xAA（转义标识符+被转义字节）
        0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // data
        0x01, 0x00, 0x01, 0x00,        // meta
        0xD4,                           // checksum
        0x55, 0x55                      // frame tail
    };
    auto unescaped = p.removeA5Escape(raw);
    // 期望还原：ExtId byte[2]=0xAA，数据完整 19 字节
    std::vector<uint8_t> expected = {
        0x01, 0x80, 0xAA, 0x00,
        0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x01, 0x00, 0x01, 0x00,
        0xD4,
        0x55, 0x55
    };
    EXPECT_EQ(unescaped, expected);
}

// A5 转义还原：非转义字节保持不变
TEST(EG2_4C2SerialCanProtocol, RemoveA5EscapeKeepsNonEscaped) {
    auto p = makeProto();
    std::vector<uint8_t> raw = {0xAA, 0xAA, 0x55, 0x55};
    auto unescaped = p.removeA5Escape(raw);
    EXPECT_EQ(unescaped, raw);
}

// 触觉数据应返回 NotSupported
TEST(EG2_4C2SerialCanProtocol, ParseTouchDataNotSupported) {
    auto p = makeProto();
    RingBuffer rb(64);
    auto [ok, _] = p.parseTouchData(rb, 1);
    EXPECT_FALSE(ok);
}

// parseResponse 默认 no-op（4C2 协议走 readOneFrameRaw + parseAndValidateFrame）
TEST(EG2_4C2SerialCanProtocol, ParseResponseDefaultNoOp) {
    auto p = makeProto();
    RingBuffer rb(64);
    auto [ok, values] = p.parseResponse(rb);
    EXPECT_FALSE(ok);
    EXPECT_TRUE(values.empty());
}

#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

#include "RH524J1_485_protocol.hpp"
#include "ring_buffer.hpp"
#include "test_helpers.hpp"

using test_helpers::pushLE16;
using test_helpers::withChecksum;

namespace {

RH524J1_485_Protocol makeProto() {
    RH524J1_485_Protocol p;
    p.setDeviceId(1);
    return p;
}

} // namespace

TEST(RH524J1Protocol, RegisterAddressLookup) {
    auto p = makeProto();
    EXPECT_EQ(p.getRegisterAddress("id"), 100);
    EXPECT_EQ(p.getRegisterAddress("baudRate"), 101);
    EXPECT_EQ(p.getRegisterAddress("clearError"), 103);
    EXPECT_EQ(p.getRegisterAddress("save"), 104);
    EXPECT_EQ(p.getRegisterAddress("resetPara"), 105);
    EXPECT_EQ(p.getRegisterAddress("gestureForceClb"), 107);
    EXPECT_EQ(p.getRegisterAddress("pause"), 110);
    EXPECT_EQ(p.getRegisterAddress("stop"), 111);
    EXPECT_EQ(p.getRegisterAddress("angleSet"), 320);
    EXPECT_EQ(p.getRegisterAddress("forceSet"), 344);
    EXPECT_EQ(p.getRegisterAddress("speedSet"), 368);
    EXPECT_EQ(p.getRegisterAddress("angleAct"), 416);
    EXPECT_EQ(p.getRegisterAddress("forceAct"), 440);
    EXPECT_EQ(p.getRegisterAddress("errorCode"), 488);
    EXPECT_EQ(p.getRegisterAddress("status"), 512);
    EXPECT_EQ(p.getRegisterAddress("temp"), 536);
    EXPECT_EQ(p.getRegisterAddress("currentSet"), 224);
    EXPECT_EQ(p.getRegisterAddress("posSet"), 296);
    EXPECT_EQ(p.getRegisterAddress("不存在"), -1);
}

TEST(RH524J1Protocol, BuildReadCommand) {
    auto p = makeProto();
    auto cmd = p.buildReadCommand(416, 48); // angleAct, 24*2 字节
    std::vector<uint8_t> expected = withChecksum({0xEB, 0x90, 0x01, 0x04, 0x11, 0xA0, 0x01, 0x30});
    EXPECT_EQ(cmd, expected);
}

TEST(RH524J1Protocol, BuildWriteCommandPartial) {
    auto p = makeProto();
    auto cmd = p.buildWriteCommand(320, {100, 200, 300});
    std::vector<uint8_t> body = {0xEB, 0x90, 0x01, static_cast<uint8_t>(3 * 2 + 3), 0x12, 0x40, 0x01};
    pushLE16(body, 100);
    pushLE16(body, 200);
    pushLE16(body, 300);
    EXPECT_EQ(cmd, withChecksum(body));
}

TEST(RH524J1Protocol, BuildWriteCommandSingleJointNeg150) {
    auto p = makeProto();
    auto cmd = p.buildWriteCommand(320, {-150});
    ASSERT_EQ(cmd.size(), static_cast<size_t>(10));
    EXPECT_EQ(cmd[0], 0xEB);
    EXPECT_EQ(cmd[1], 0x90);
    EXPECT_EQ(cmd[2], 0x01);
    EXPECT_EQ(cmd[3], 0x05);
    EXPECT_EQ(cmd[4], 0x12);
    EXPECT_EQ(cmd[5], 0x40);
    EXPECT_EQ(cmd[6], 0x01);
    EXPECT_EQ(cmd[7], 0x6A);
    EXPECT_EQ(cmd[8], 0xFF);
    EXPECT_EQ(cmd[9], 0xC2);
}

TEST(RH524J1Protocol, BuildWriteCommand24Values) {
    auto p = makeProto();
    std::vector<int> vals(24, 0);
    vals[0] = -100; // ID1 小指翻折 -150~0
    vals[1] = -200; // ID2 侧摆，有符号
    vals[17] = 1000;
    auto cmd = p.buildWriteCommand(320, vals);
    ASSERT_EQ(cmd.size(), static_cast<size_t>(2 + 1 + 1 + 1 + 2 + 48 + 1));
    EXPECT_EQ(cmd[0], 0xEB);
    EXPECT_EQ(cmd[1], 0x90);
    EXPECT_EQ(cmd[3], static_cast<uint8_t>(24 * 2 + 3));
    EXPECT_EQ(cmd[4], 0x12);
    EXPECT_EQ(cmd[5], 0x40);
    EXPECT_EQ(cmd[6], 0x01);
    EXPECT_EQ(cmd[7], 0x9C); // -100 = 0xFF9C LE
    EXPECT_EQ(cmd[8], 0xFF);
    EXPECT_EQ(cmd[9], 0x38); // -200 = 0xFF38 LE
    EXPECT_EQ(cmd[10], 0xFF);
}

TEST(RH524J1Protocol, BuildWriteCommandTooManyValuesThrows) {
    auto p = makeProto();
    std::vector<int> twenty_five(25, 0);
    EXPECT_THROW(p.buildWriteCommand(320, twenty_five), std::runtime_error);
}

TEST(RH524J1Protocol, ParseReadResponse24Joints) {
    auto p = makeProto();
    std::vector<int> expected_vals;
    std::vector<uint8_t> body = {0x90, 0xEB, 0x01, static_cast<uint8_t>(48 + 3), 0x11, 0xA0, 0x01};
    for (int i = 0; i < 24; ++i) {
        int v = (i == 1) ? -200 : i * 10;
        expected_vals.push_back(v);
        pushLE16(body, v);
    }
    auto frame = withChecksum(body);

    RingBuffer rb(256);
    rb.push(frame.data(), frame.size());
    auto [ok, values] = p.parseResponse(rb);
    ASSERT_TRUE(ok);
    EXPECT_EQ(values, expected_vals);
}

TEST(RH524J1Protocol, ParseResponseRejectsBadChecksum) {
    auto p = makeProto();
    std::vector<uint8_t> body = {0x90, 0xEB, 0x01, 0x05, 0x11, 0xA0, 0x01};
    pushLE16(body, 55);
    auto frame = withChecksum(body);
    frame.back() ^= 0xFF;

    RingBuffer rb(64);
    rb.push(frame.data(), frame.size());
    auto [ok, values] = p.parseResponse(rb);
    EXPECT_FALSE(ok);
}

TEST(RH524J1Protocol, ValidateChecksumRejectsWrongHeader) {
    auto p = makeProto();
    std::vector<uint8_t> frame = withChecksum({0xEB, 0x90, 0x01, 0x04, 0x11, 0xA0, 0x01, 0x30});
    EXPECT_FALSE(p.validateChecksum(frame));
}

TEST(RH524J1Protocol, ParseTouchDataNotSupported) {
    auto p = makeProto();
    RingBuffer rb(32);
    auto [ok, data] = p.parseTouchData(rb, 1);
    EXPECT_FALSE(ok);
    EXPECT_TRUE(data.fingerResults.empty());
}

TEST(RH524J1Protocol, ParseWriteResponseOk) {
    auto p = makeProto();
    std::vector<uint8_t> body = {0x90, 0xEB, 0x01, 0x04, 0x12, 0x40, 0x01, 0x01};
    auto frame = withChecksum(body);
    RingBuffer rb(64);
    rb.push(frame.data(), frame.size());
    auto [ok, values] = p.parseResponse(rb);
    EXPECT_TRUE(ok);
    EXPECT_TRUE(values.empty());
}

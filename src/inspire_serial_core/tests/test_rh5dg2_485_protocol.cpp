#include <gtest/gtest.h>

#include <vector>

#include "RH5DG2_485_protocol.hpp"
#include "ring_buffer.hpp"
#include "test_helpers.hpp"

using test_helpers::pushLE16;
using test_helpers::withChecksum;

namespace {

RH5DG2_485_Protocol makeProto() {
    RH5DG2_485_Protocol p;
    p.setDeviceId(1);
    return p;
}

} // namespace

// 寄存器名 -> 地址 映射（13 自由度机型）
TEST(RH5DG2Protocol, RegisterAddressLookup) {
    auto p = makeProto();
    EXPECT_EQ(p.getRegisterAddress("angleSet"), 1080);
    EXPECT_EQ(p.getRegisterAddress("angleAct"), 1136);
    EXPECT_EQ(p.getRegisterAddress("不存在"), -1);
}

// 读命令字节序列正确
TEST(RH5DG2Protocol, BuildReadCommand) {
    auto p = makeProto();
    auto cmd = p.buildReadCommand(1136, 26); // angleAct, 13*2 字节
    std::vector<uint8_t> expected = withChecksum({0xEB, 0x90, 0x01, 0x04, 0x11, 0x70, 0x04, 0x1A});
    EXPECT_EQ(cmd, expected);
}

// 写命令字节序列正确（多值小端序）
TEST(RH5DG2Protocol, BuildWriteCommand) {
    auto p = makeProto();
    auto cmd = p.buildWriteCommand(1080, {100, 200, 300}); // angleSet 部分值
    std::vector<uint8_t> body = {0xEB, 0x90, 0x01, static_cast<uint8_t>(3 * 2 + 3), 0x12, 0x38, 0x04};
    pushLE16(body, 100);
    pushLE16(body, 200);
    pushLE16(body, 300);
    EXPECT_EQ(cmd, withChecksum(body));
}

// 写命令值个数超过 13 应抛异常
TEST(RH5DG2Protocol, BuildWriteCommandTooManyValuesThrows) {
    auto p = makeProto();
    std::vector<int> fourteen(14, 0);
    EXPECT_THROW(p.buildWriteCommand(1080, fourteen), std::runtime_error);
}

// 解析读回复：13 自由度数据全部还原
TEST(RH5DG2Protocol, ParseReadResponse) {
    auto p = makeProto();
    std::vector<int> expected_vals;
    std::vector<uint8_t> body = {0x90, 0xEB, 0x01, static_cast<uint8_t>(26 + 3), 0x11, 0x70, 0x04};
    for (int i = 0; i < 13; ++i) {
        int v = i * 10 - 30; // 含负数，验证有符号还原
        expected_vals.push_back(v);
        pushLE16(body, v);
    }
    auto frame = withChecksum(body);

    RingBuffer rb(128);
    rb.push(frame.data(), frame.size());
    auto [ok, values] = p.parseResponse(rb);
    ASSERT_TRUE(ok);
    EXPECT_EQ(values, expected_vals);
}

// 校验和错误应拒绝
TEST(RH5DG2Protocol, ParseResponseRejectsBadChecksum) {
    auto p = makeProto();
    std::vector<uint8_t> body = {0x90, 0xEB, 0x01, 0x05, 0x11, 0x70, 0x04};
    pushLE16(body, 55);
    auto frame = withChecksum(body);
    frame.back() ^= 0xFF;

    RingBuffer rb(64);
    rb.push(frame.data(), frame.size());
    auto [ok, values] = p.parseResponse(rb);
    EXPECT_FALSE(ok);
}

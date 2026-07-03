#include <gtest/gtest.h>

#include <vector>

#include "RH56F1_485_protocol.hpp"
#include "ring_buffer.hpp"
#include "test_helpers.hpp"

using test_helpers::pushLE16;
using test_helpers::withChecksum;

namespace {

RH56F1_485_Protocol makeProto() {
    RH56F1_485_Protocol p;
    p.setDeviceId(1);
    return p;
}

} // namespace

// 寄存器名 -> 地址 映射
TEST(RH56F1Protocol, RegisterAddressLookup) {
    auto p = makeProto();
    EXPECT_EQ(p.getRegisterAddress("angleSet"), 1040);
    EXPECT_EQ(p.getRegisterAddress("angleAct"), 1064);
    EXPECT_EQ(p.getRegisterAddress("不存在"), -1);
}

// 读命令字节序列正确（含帧头、ID、命令、地址小端、长度、校验和）
TEST(RH56F1Protocol, BuildReadCommand) {
    auto p = makeProto();
    auto cmd = p.buildReadCommand(1064, 12); // angleAct
    std::vector<uint8_t> expected = withChecksum({0xEB, 0x90, 0x01, 0x04, 0x11, 0x28, 0x04, 0x0C});
    EXPECT_EQ(cmd, expected);
}

// 写命令字节序列正确（数据长度字段 = 值个数*2+3，数据小端序）
TEST(RH56F1Protocol, BuildWriteCommand) {
    auto p = makeProto();
    auto cmd = p.buildWriteCommand(1040, {100, -1}); // angleSet
    std::vector<uint8_t> body = {0xEB, 0x90, 0x01, 0x07, 0x12, 0x10, 0x04};
    pushLE16(body, 100); // 0x64 0x00
    pushLE16(body, -1);  // 0xFF 0xFF
    EXPECT_EQ(cmd, withChecksum(body));
}

// 写命令值个数超过 6 应抛异常
TEST(RH56F1Protocol, BuildWriteCommandTooManyValuesThrows) {
    auto p = makeProto();
    std::vector<int> seven(7, 0);
    EXPECT_THROW(p.buildWriteCommand(1040, seven), std::runtime_error);
}

// 校验和验证：正确帧通过，篡改后失败
TEST(RH56F1Protocol, ValidateChecksum) {
    auto p = makeProto();
    auto frame = withChecksum({0x90, 0xEB, 0x01, 0x07, 0x11, 0x28, 0x04, 0x64, 0x00, 0xFF, 0xFF});
    EXPECT_TRUE(p.validateChecksum(frame));

    frame.back() ^= 0xFF; // 篡改校验和
    EXPECT_FALSE(p.validateChecksum(frame));
}

// 解析读回复：还原小端序值并正确处理有符号数
TEST(RH56F1Protocol, ParseReadResponse) {
    auto p = makeProto();
    std::vector<uint8_t> body = {0x90, 0xEB, 0x01, 0x07, 0x11, 0x28, 0x04};
    pushLE16(body, 100);
    pushLE16(body, -1);
    auto frame = withChecksum(body);

    RingBuffer rb(64);
    rb.push(frame.data(), frame.size());
    auto [ok, values] = p.parseResponse(rb);
    ASSERT_TRUE(ok);
    ASSERT_EQ(values.size(), 2u);
    EXPECT_EQ(values[0], 100);
    EXPECT_EQ(values[1], -1);
}

// 帧头前混入噪声字节，仍应能定位帧头并解析
TEST(RH56F1Protocol, ParseResponseSkipsLeadingGarbage) {
    auto p = makeProto();
    std::vector<uint8_t> body = {0x90, 0xEB, 0x01, 0x05, 0x11, 0x28, 0x04};
    pushLE16(body, 777);
    auto frame = withChecksum(body);

    std::vector<uint8_t> stream = {0xAA, 0xBB, 0x00}; // 噪声前缀
    stream.insert(stream.end(), frame.begin(), frame.end());

    RingBuffer rb(64);
    rb.push(stream.data(), stream.size());
    auto [ok, values] = p.parseResponse(rb);
    ASSERT_TRUE(ok);
    ASSERT_EQ(values.size(), 1u);
    EXPECT_EQ(values[0], 777);
}

// 解析写回复（命令 0x12）成功且不返回数据值
TEST(RH56F1Protocol, ParseWriteResponse) {
    auto p = makeProto();
    auto frame = withChecksum({0x90, 0xEB, 0x01, 0x03, 0x12, 0x28, 0x04, 0x01});
    RingBuffer rb(64);
    rb.push(frame.data(), frame.size());
    auto [ok, values] = p.parseResponse(rb);
    EXPECT_TRUE(ok);
    EXPECT_TRUE(values.empty());
}

// 校验和错误的帧不应被解析为成功
TEST(RH56F1Protocol, ParseResponseRejectsBadChecksum) {
    auto p = makeProto();
    std::vector<uint8_t> body = {0x90, 0xEB, 0x01, 0x05, 0x11, 0x28, 0x04};
    pushLE16(body, 123);
    auto frame = withChecksum(body);
    frame.back() ^= 0xFF; // 破坏校验和

    RingBuffer rb(64);
    rb.push(frame.data(), frame.size());
    auto [ok, values] = p.parseResponse(rb);
    EXPECT_FALSE(ok);
}

// ID 不匹配时拒绝解析
TEST(RH56F1Protocol, ParseResponseRejectsIdMismatch) {
    auto p = makeProto();                                                   // device id = 1
    std::vector<uint8_t> body = {0x90, 0xEB, 0x09, 0x05, 0x11, 0x28, 0x04}; // ID=9
    pushLE16(body, 123);
    auto frame = withChecksum(body);

    RingBuffer rb(64);
    rb.push(frame.data(), frame.size());
    auto [ok, values] = p.parseResponse(rb);
    EXPECT_FALSE(ok);
}

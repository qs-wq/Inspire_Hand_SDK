#include <gtest/gtest.h>

#include <vector>

#include "ring_buffer.hpp"

namespace {

std::vector<uint8_t> popAll(RingBuffer& rb) {
    std::vector<uint8_t> out(rb.size());
    if (!out.empty()) {
        size_t n = rb.pop(out.data(), out.size());
        out.resize(n);
    }
    return out;
}

} // namespace

// 新建缓冲区应为空
TEST(RingBufferTest, NewBufferIsEmpty) {
    RingBuffer rb(8);
    EXPECT_EQ(rb.size(), 0u);
}

// push 后 size 正确，pop 能取回相同数据
TEST(RingBufferTest, PushPopRoundTrip) {
    RingBuffer rb(8);
    const uint8_t data[] = {0x11, 0x22, 0x33, 0x44};
    rb.push(data, 4);
    EXPECT_EQ(rb.size(), 4u);

    uint8_t out[4] = {0};
    size_t n = rb.pop(out, 4);
    EXPECT_EQ(n, 4u);
    EXPECT_EQ(out[0], 0x11);
    EXPECT_EQ(out[1], 0x22);
    EXPECT_EQ(out[2], 0x33);
    EXPECT_EQ(out[3], 0x44);
    EXPECT_EQ(rb.size(), 0u);
}

// pop 请求长度大于现有数据时，只返回现有数据
TEST(RingBufferTest, PopMoreThanAvailable) {
    RingBuffer rb(8);
    const uint8_t data[] = {0xAA, 0xBB};
    rb.push(data, 2);

    uint8_t out[8] = {0};
    size_t n = rb.pop(out, 8);
    EXPECT_EQ(n, 2u);
    EXPECT_EQ(out[0], 0xAA);
    EXPECT_EQ(out[1], 0xBB);
}

// advance 应丢弃指定数量的旧数据
TEST(RingBufferTest, AdvanceDropsData) {
    RingBuffer rb(8);
    const uint8_t data[] = {1, 2, 3, 4, 5};
    rb.push(data, 5);
    rb.advance(2);
    EXPECT_EQ(rb.size(), 3u);

    auto rest = popAll(rb);
    ASSERT_EQ(rest.size(), 3u);
    EXPECT_EQ(rest[0], 3);
    EXPECT_EQ(rest[1], 4);
    EXPECT_EQ(rest[2], 5);
}

// advance 超过现有数据量应抛异常
TEST(RingBufferTest, AdvanceBeyondSizeThrows) {
    RingBuffer rb(8);
    const uint8_t data[] = {1, 2, 3};
    rb.push(data, 3);
    EXPECT_THROW(rb.advance(4), std::runtime_error);
}

// clear 应清空缓冲区
TEST(RingBufferTest, ClearEmptiesBuffer) {
    RingBuffer rb(8);
    const uint8_t data[] = {1, 2, 3};
    rb.push(data, 3);
    rb.clear();
    EXPECT_EQ(rb.size(), 0u);
}

// 写满后继续写入会覆盖最旧数据（自动覆盖语义）
TEST(RingBufferTest, OverwriteOldestWhenFull) {
    RingBuffer rb(4);
    const uint8_t first[] = {1, 2, 3, 4};
    rb.push(first, 4);
    EXPECT_EQ(rb.size(), 4u);

    const uint8_t more[] = {5};
    rb.push(more, 1); // 覆盖最旧的 1
    EXPECT_EQ(rb.size(), 4u);

    auto content = popAll(rb);
    ASSERT_EQ(content.size(), 4u);
    EXPECT_EQ(content[0], 2);
    EXPECT_EQ(content[1], 3);
    EXPECT_EQ(content[2], 4);
    EXPECT_EQ(content[3], 5);
}

// 跨环回绕后 contiguousDataSize 只反映到物理末尾的连续段
TEST(RingBufferTest, ContiguousSizeAfterWrap) {
    RingBuffer rb(4);
    const uint8_t data[] = {1, 2, 3};
    rb.push(data, 3); // tail=0, head=3
    rb.advance(2);    // tail=2, 剩余 {3}
    const uint8_t more[] = {4, 5};
    rb.push(more, 2); // 写入回绕：head 2->3->0
    EXPECT_EQ(rb.size(), 3u);
    // 从 tail=2 起到物理末尾(索引3)为连续段，长度 2
    EXPECT_EQ(rb.contiguousDataSize(), 2u);

    auto content = popAll(rb);
    ASSERT_EQ(content.size(), 3u);
    EXPECT_EQ(content[0], 3);
    EXPECT_EQ(content[1], 4);
    EXPECT_EQ(content[2], 5);
}

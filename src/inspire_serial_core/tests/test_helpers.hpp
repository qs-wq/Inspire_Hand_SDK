#pragma once

#include <cstdint>
#include <vector>

namespace test_helpers {

// 给定“除校验和外”的所有字节，按 485 系列协议算法（byte[2] 起累加）追加校验和，返回完整帧。
// RH56F1 / RH5DG2 / EG5CD1 三个 485 协议的校验和算法一致。
inline std::vector<uint8_t> withChecksum(std::vector<uint8_t> body) {
    uint8_t sum = 0;
    for (size_t i = 2; i < body.size(); ++i) {
        sum = static_cast<uint8_t>(sum + body[i]);
    }
    body.push_back(sum);
    return body;
}

// 低字节在前的 16 位小端序拆分并追加到向量末尾。
inline void pushLE16(std::vector<uint8_t>& v, int value) {
    v.push_back(static_cast<uint8_t>(value & 0xFF));
    v.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
}

}  // namespace test_helpers

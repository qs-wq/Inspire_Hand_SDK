#pragma once

#include "RH5DG2_485_protocol.hpp"

/**
 * @brief RH5DG2 CAN-FD 协议实现类
 *
 * 与 RH5DG2_485 协议在帧格式、寄存器映射、解析规则完全一致，
 * 唯一差异在于「读数据段长度」必须符合 CAN-FD 合法字节列表：
 *
 *   [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64]
 *
 * 规则同 RH56F1_canfd_Protocol，支持：
 *  - 子 64 字节长度向上补齐到最近合法值（如 26 → 32）
 *  - 超过 64 字节自动拆分为多帧，最后一帧按规则补齐
 *  - 多帧读取结果自动拼接，仅向上层返回逻辑请求长度
 */
class RH5DG2_canfd_Protocol : public RH5DG2_485_Protocol {
public:
    // 命令构建（仅写命令需要放宽长度限制）
    std::vector<uint8_t> buildWriteCommand(int address, const std::vector<int>& values) override;

    // 读/写寄存器（实现 CAN-FD 下的数据长度补齐与拆帧逻辑）
    IoError writeRegister(Device device, const std::string& reg_name, const std::vector<int>& values) override;
    RegisterReadResult readRegister(
        Device device,
        RingBuffer& ringBuffer,
        const std::string& reg_name,
        size_t length) override;

    // 触觉数据读取（内部按 68 字节逻辑长度拼接两帧：64B + 4B）
    TouchReadResult readTouchData(
        Device device,
        RingBuffer& ringBuffer,
        int version) override;

protected:
    /**
     * @brief 将请求字节长度补齐到最近的合法 CAN-FD 字节长度
     * @param requested_bytes 请求的字节长度
     * @return 补齐后的合法字节长度（如果已经是合法长度则返回原值）
     * 
     * 合法字节列表：[0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64]
     * 补齐规则：向上补齐到最近的合法字节（例如：10 → 12, 26 → 32）
     */
    size_t adjustToValidCanfdLength(size_t requested_bytes) const;
};


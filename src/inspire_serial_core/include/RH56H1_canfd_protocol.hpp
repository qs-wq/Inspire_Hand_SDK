#pragma once

#include "RH56H1_485_protocol.hpp"

/**
 * @brief RH56H1 CAN-FD 协议实现类
 *
 * 与 RH56H1_485 协议在帧格式、寄存器映射、解析规则完全一致，
 * 唯一差异在于「读数据段长度」必须符合 CAN-FD 合法字节列表：
 *
 *   [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64]
 *
 * 适配规则：
 *  1. 当请求数据长度（字节数）<= 64 且不在合法列表中时：
 *     - 向上补齐到最近的合法字节数（例如：10 → 12）
 *     - 实际发送帧的数据长度为补齐后的值
 *     - 解析后仅返回调用层真正请求的长度（多余部分丢弃）
 *
 *  2. 当请求数据长度（字节数）> 64 时：
 *     - 自动拆分为多帧请求，单帧数据段最大 64 字节
 *     - 除最后一帧外，其余帧长度固定 64 字节
 *     - 最后一帧长度按规则1补齐到合法字节数
 *     - 多帧读取到的数据按顺序拼接，只返回调用层真正请求的长度
 */
class RH56H1_canfd_Protocol : public RH56H1_485_Protocol {
public:
    std::vector<uint8_t> buildWriteCommand(int address, const std::vector<int>& values) override;

    IoError writeRegister(Device device, const std::string& reg_name, const std::vector<int>& values) override;
    RegisterReadResult readRegister(Device device, RingBuffer& ringBuffer, const std::string& reg_name,
                                    size_t length) override;

    TouchReadResult readTouchData(Device device, RingBuffer& ringBuffer, int version) override;

protected:
    size_t adjustToValidCanfdLength(size_t requested_bytes) const;
};

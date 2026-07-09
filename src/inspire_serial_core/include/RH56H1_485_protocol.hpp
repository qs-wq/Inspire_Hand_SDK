#pragma once

#include "protocol.hpp"
#include <chrono>
#include <iomanip>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

/**
 * @brief RH56H1 485 协议：寄存器映射、帧格式、寄存器读写与 RH56F1_485 完全一致。
 *
 * 与 F1 唯一的区别在于触觉传感器为 version2（压阻式），因此 readTouchData /
 * parseTouchData 两个触觉相关函数实现 version2 逻辑；其余全部与 F1 相同。
 */
class RH56H1_485_Protocol : public Protocol {
public:
    int getRegisterAddress(const std::string& register_name) const override;
    std::vector<uint8_t> buildReadCommand(int address, size_t length) override;
    std::vector<uint8_t> buildWriteCommand(int address, const std::vector<int>& values) override;
    std::pair<bool, std::vector<int>> parseResponse(RingBuffer& ringBuffer) override;
    bool validateChecksum(const std::vector<uint8_t>& response) const override;
    std::pair<bool, TouchDataResult> parseTouchData(RingBuffer& ringBuffer, int version) override;

    IoError writeRegister(Device device, const std::string& reg_name, const std::vector<int>& values) override;
    RegisterReadResult readRegister(Device device, RingBuffer& ringBuffer, const std::string& reg_name,
                                    size_t length) override;
    TouchReadResult readTouchData(Device device, RingBuffer& ringBuffer, int version) override;

protected:
    static const std::map<std::string, int> REGISTER_MAP;
    static const std::map<std::string, size_t> REGISTER_READ_LENGTH_MAP;

    size_t getDefaultReadLength(const std::string& reg_name) const;

    /**
     * @brief 按寄存器语义修正解码结果，与 docs/Hand_control.cpp 及协议手册一致
     */
    void applyRegisterDecodeRule(const std::string& reg_name, std::vector<int>& values) const;

    /** @brief 将请求字节长度补齐到手册规定的合法单帧长度 */
    size_t adjustToValidFrameLength(size_t requested_bytes) const;

    /** @brief 从 start_addr 起连续读取 byte_len 字节（>64 分帧，非法长度补齐，只取逻辑字节） */
    std::vector<uint8_t> readRegisterBytes(Device device, int start_addr, size_t byte_len,
                                           const char* log_context = nullptr);

    uint8_t readByteAtOffset(const RingBuffer& ringBuffer, size_t offset) const;
    std::vector<uint8_t> extractFromRingBuffer(const RingBuffer& ringBuffer, size_t startOffset, size_t length) const;
    std::string formatBytesToHex(const uint8_t* data, size_t length) const;
    std::vector<uint8_t> readResponseWithLoop(Device device, int timeout_ms, size_t min_bytes = 9,
                                              bool is_read_response = false) const;
};

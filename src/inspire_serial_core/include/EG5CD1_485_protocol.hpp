#pragma once

#include "protocol.hpp"
#include <map>
#include <string>
#include <sstream>
#include <iomanip>
#include <thread>
#include <chrono>

/**
 * 因时 EG-5CD1 电动夹爪 RS485 自定义协议（见 src/document/夹爪485寄存器规则.md）
 * 指令帧头 EB 90，应答帧头 EE 16；读命令 0x00，写命令 0x01。
 */
class EG5CD1_485_Protocol : public Protocol {
public:
    int getRegisterAddress(const std::string& register_name) const override;
    std::vector<uint8_t> buildReadCommand(int address, size_t length) override;
    std::vector<uint8_t> buildWriteCommand(int address, const std::vector<int>& values) override;
    std::pair<bool, std::vector<int>> parseResponse(RingBuffer& ringBuffer) override;
    bool validateChecksum(const std::vector<uint8_t>& response) const override;
    std::pair<bool, TouchDataResult> parseTouchData(RingBuffer& ringBuffer, int version) override;

    IoError writeRegister(Device device, const std::string& reg_name, const std::vector<int>& values) override;
    RegisterReadResult readRegister(
        Device device, RingBuffer& ringBuffer, const std::string& reg_name, size_t length) override;
    TouchReadResult readTouchData(Device device, RingBuffer& ringBuffer, int version) override;

protected:
    static const std::map<std::string, int> REGISTER_MAP;
    static const std::map<std::string, size_t> REGISTER_READ_LENGTH_MAP;

    size_t getDefaultReadLength(const std::string& reg_name) const;

    uint8_t readByteAtOffset(const RingBuffer& ringBuffer, size_t offset) const;
    std::vector<uint8_t> extractFromRingBuffer(const RingBuffer& ringBuffer, size_t startOffset, size_t length) const;
    std::string formatBytesToHex(const uint8_t* data, size_t length) const;

    std::vector<uint8_t> readResponseWithLoop(
        Device device, int timeout_ms, size_t min_bytes = 8, bool is_read_response = false) const;
};

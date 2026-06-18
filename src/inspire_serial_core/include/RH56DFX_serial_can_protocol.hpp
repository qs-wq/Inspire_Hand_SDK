#pragma once

#include "protocol.hpp"

#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>

/**
 * @brief RH56DFX Serial-CAN 协议实现。
 *
 * 对上层保持与 RH56F1 兼容的寄存器名称（angleSet / angleAct / forceSet ...），
 * 仅在底层实现 Serial-CAN 封装 + 29 位扩展 ID 的读写规则，隔离硬件通信差异。
 */
class RH56DFX_serial_can_Protocol : public Protocol {
public:
    int getRegisterAddress(const std::string& register_name) const override;
    std::vector<uint8_t> buildReadCommand(int address, size_t length) override;
    std::vector<uint8_t> buildWriteCommand(int address, const std::vector<int>& values) override;
    std::pair<bool, std::vector<int>> parseResponse(RingBuffer& ringBuffer) override;
    bool validateChecksum(const std::vector<uint8_t>& response) const override;
    std::pair<bool, TouchDataResult> parseTouchData(RingBuffer& ringBuffer, int version) override;

    IoError writeRegister(Device device, const std::string& reg_name, const std::vector<int>& values) override;
    RegisterReadResult readRegister(
        Device device,
        RingBuffer& ringBuffer,
        const std::string& reg_name,
        size_t length) override;
    TouchReadResult readTouchData(Device device, RingBuffer& ringBuffer, int version) override;

protected:
    struct RegisterWriteRule {
        size_t value_width_bytes = 2;
        size_t max_value_count = 6;
    };

    static const std::map<std::string, int> REGISTER_MAP;
    static const std::map<std::string, size_t> REGISTER_READ_LENGTH_MAP;
    static const std::map<std::string, RegisterWriteRule> REGISTER_WRITE_RULE_MAP;
    static const std::set<std::string> NOT_SUPPORTED_REGISTERS;

    static constexpr uint8_t kRwReadHand = 0;
    static constexpr uint8_t kRwWriteHand = 1;
    static constexpr size_t kSerialFrameLength = 21;
    static constexpr size_t kCanPayloadLength = 8;

    size_t getDefaultReadLength(const std::string& reg_name) const;
    bool isNotSupportedRegister(const std::string& reg_name) const;

    uint32_t buildCanId(uint8_t rw_flag, int address) const;
    std::vector<uint8_t> buildSerialCanFrame(uint32_t can_id, const std::vector<uint8_t>& payload, bool is_read) const;
    std::vector<uint8_t> readOneFrameRaw(Device device, int timeout_ms) const;
    std::vector<uint8_t> removeA5Escape(const std::vector<uint8_t>& raw) const;

    bool parseAndValidateFrame(
        const std::vector<uint8_t>& frame,
        uint8_t expected_rw,
        int expected_address,
        std::vector<uint8_t>* out_payload,
        uint8_t* out_valid_len) const;

    std::vector<uint8_t> encodeValuesByRule(const std::string& reg_name, const std::vector<int>& values, IoError* err) const;
    std::vector<int> decodeValuesByRule(const std::string& reg_name, const std::vector<uint8_t>& payload) const;
};


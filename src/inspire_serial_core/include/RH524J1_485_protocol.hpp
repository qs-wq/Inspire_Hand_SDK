#pragma once

#include "RH5DG2_485_protocol.hpp"

#include <map>
#include <string>
#include <vector>

/**
 * @brief RH524J1（065 腱绳驱动 24 自由度灵巧手）RS485 协议。
 *
 * 帧格式与 RH5DG2_485 完全一致（请求 EB 90、应答 90 EB、小端、校验和
 * sum(byte[2..n-2]) & 0xFF）。寄存器地址取自 065demo/hand_param.h，
 * 与 RH5DG2 的 1000+ 地址表不同：ZONE1 从 100 起，ZONE2 从 200 起，
 * HAND_NUM_DRV=24。angleSet 等关节寄存器一次读写 24×int16（48 字节）。
 *
 * 手册与 065demo 不一致时以 demo_485_cable_driven_hand.py 为准。
 */
class RH524J1_485_Protocol : public RH5DG2_485_Protocol {
public:
    int getRegisterAddress(const std::string& register_name) const override;
    std::vector<uint8_t> buildWriteCommand(int address, const std::vector<int>& values) override;
    std::pair<bool, TouchDataResult> parseTouchData(RingBuffer& ringBuffer, int version) override;

    IoError writeRegister(Device device, const std::string& reg_name, const std::vector<int>& values) override;
    RegisterReadResult readRegister(Device device, RingBuffer& ringBuffer, const std::string& reg_name,
                                    size_t length) override;
    TouchReadResult readTouchData(Device device, RingBuffer& ringBuffer, int version) override;

    static constexpr size_t kJointCount = 24;
    static constexpr size_t kJointBytes = kJointCount * 2; // 48

protected:
    static const std::map<std::string, int> REGISTER_MAP;
    static const std::map<std::string, size_t> REGISTER_READ_LENGTH_MAP;

    size_t getDefaultReadLength(const std::string& reg_name) const;
    int clampAngle(int drive_id, int angle) const;
    void clampAngleSetValues(std::vector<int>& values) const;

    /** 用户 API 角度 → 硬件 angleSet（0=张开，数值越大越弯曲） */
    int userAngleToHardware(int drive_id, int user_angle) const;
    /** 硬件 angleAct → 用户 API 角度 */
    int hardwareAngleToUser(int drive_id, int hardware_angle) const;
    bool isAngleJointRegister(const std::string& reg_name) const;
    bool isPackedJointRegister(const std::string& reg_name) const;
    /** 实机拇指指尖/掌指与电缸 ID20/ID22 对调；数组下标仍等于 驱动ID-1 */
    void swapThumbDipMcp(std::vector<int>& values) const;
    std::vector<int> permuteUserToHardware(const std::vector<int>& user_values) const;
    std::vector<int> permuteHardwareToUser(const std::vector<int>& hardware_values) const;
    std::vector<int> hardwareAnglesToUser(const std::vector<int>& hardware_values) const;
    std::vector<int> userAnglesToHardware(const std::vector<int>& user_values) const;
    RegisterReadResult readRegisterHardware(Device device, RingBuffer& ringBuffer, const std::string& reg_name,
                                            size_t length);
    std::vector<int> prepareAngleSetHardwareValues(Device device, const std::vector<int>& user_values);
};

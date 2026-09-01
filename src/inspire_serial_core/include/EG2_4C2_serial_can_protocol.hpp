#pragma once

#include "protocol.hpp"

#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>

/**
 * @brief 因时 EG2-4C2 电动夹爪 Serial-CAN 协议实现
 *
 * 继承自 Protocol 基类，实现 Serial-CAN 通信协议。
 * 串口帧格式：AA AA | ExtId(4B 小端) | Data[8] | Meta[4] | Checksum | 55 55
 *
 * ExtId 编码（29位）：
 *   bit27~26: op_type（0=读 1=写 2=定位 3=随动）
 *   bit25~14: 寄存器起始地址（Modbus 地址 12 位）
 *   bit13~0 : 设备 ID（14 位）
 */
class EG2_4C2_serial_can_Protocol : public Protocol {
public:
    // 重写父类纯虚函数
    int getRegisterAddress(const std::string& register_name) const override;
    std::vector<uint8_t> buildReadCommand(int address, size_t length) override;
    std::vector<uint8_t> buildWriteCommand(int address, const std::vector<int>& values) override;
    std::pair<bool, std::vector<int>> parseResponse(RingBuffer& ringBuffer) override;
    bool validateChecksum(const std::vector<uint8_t>& response) const override;
    std::pair<bool, TouchDataResult> parseTouchData(RingBuffer& ringBuffer, int version) override;

    // 重写父类虚函数
    IoError writeRegister(Device device, const std::string& reg_name, const std::vector<int>& values) override;
    RegisterReadResult readRegister(
        Device device,
        RingBuffer& ringBuffer,
        const std::string& reg_name,
        size_t length) override;
    TouchReadResult readTouchData(Device device, RingBuffer& ringBuffer, int version) override;

protected:
    /** 寄存器名称 → Modbus 地址 */
    static const std::map<std::string, int> REGISTER_MAP;
    /** 寄存器默认读取长度（字节数） */
    static const std::map<std::string, size_t> REGISTER_READ_LENGTH_MAP;
    /** 寄存器写入规则：单帧最大写入数量 */
    static const std::map<std::string, size_t> REGISTER_WRITE_RULE_MAX_COUNT;
    /** 当前机型不支持的寄存器（调用即返回 NotSupported） */
    static const std::set<std::string> NOT_SUPPORTED_REGISTERS;

    // CAN 操作类型（bit27~26）
    static constexpr uint8_t kOpRead = 0;
    static constexpr uint8_t kOpWrite = 1;
    // 帧结构常量
    static constexpr size_t kSerialFrameLength = 21;
    static constexpr size_t kCanPayloadLength = 8;
    static constexpr size_t kMetaLength = 4;

    /**
     * @brief 根据寄存器名称获取默认读取长度
     * @param reg_name 寄存器名称
     * @return 默认读取长度（字节数）
     */
    size_t getDefaultReadLength(const std::string& reg_name) const;

    /**
     * @brief 检查是否为不支持的寄存器
     * @param reg_name 寄存器名称
     * @return true=不支持，false=支持
     */
    bool isNotSupportedRegister(const std::string& reg_name) const;

    /**
     * @brief 构造 29 位 CAN 扩展标识符
     * @param op_type 操作类型（0=读 1=写）
     * @param address Modbus 地址
     * @return 32 位 ExtId
     */
    uint32_t buildCanId(uint8_t op_type, int address) const;

    /**
     * @brief 构造 Serial-CAN 帧（21 字节）
     * @param can_id 29 位扩展 ID
     * @param payload 8 字节 CAN 数据区
     * @param is_read true=读帧 false=写帧
     * @return 完整 Serial-CAN 帧
     */
    std::vector<uint8_t> buildSerialCanFrame(
        uint32_t can_id,
        const std::vector<uint8_t>& payload,
        bool is_read) const;

    /**
     * @brief 循环读取直到收到一帧
     * @param device 设备对象
     * @param timeout_ms 超时时间（毫秒）
     * @return 读取到的帧数据
     */
    std::vector<uint8_t> readOneFrameRaw(Device device, int timeout_ms) const;

    /**
     * @brief 去除 0xA5 转义字节
     * @param raw 原始数据
     * @return 转义处理后的数据
     */
    std::vector<uint8_t> removeA5Escape(const std::vector<uint8_t>& raw) const;

    /**
     * @brief 解析并校验响应帧
     * @param frame 帧数据
     * @param expected_op_type 期望的操作类型
     * @param expected_address 期望的寄存器地址
     * @param out_payload 输出：8字节 CAN 数据区
     * @param out_valid_len 输出：有效字节数
     * @return true=校验通过
     */
    bool parseAndValidateFrame(
        const std::vector<uint8_t>& frame,
        uint8_t expected_op_type,
        int expected_address,
        std::vector<uint8_t>* out_payload,
        uint8_t* out_valid_len) const;
};

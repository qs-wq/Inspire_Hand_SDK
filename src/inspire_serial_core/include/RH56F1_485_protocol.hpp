#pragma once

#include "protocol.hpp"
#include <map>
#include <string>
#include <stdexcept>
#include <sstream>
#include <iomanip>
#include <thread>
#include <chrono>

class RH56F1_485_Protocol : public Protocol {
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
    RegisterReadResult readRegister(Device device, RingBuffer& ringBuffer, const std::string& reg_name, size_t length) override;
    TouchReadResult readTouchData(Device device, RingBuffer& ringBuffer, int version) override;
    
protected:
    // 寄存器地址映射表
    static const std::map<std::string, int> REGISTER_MAP;
    
    // 寄存器默认读取长度映射表（字节数，0表示使用通用默认值12字节）
    static const std::map<std::string, size_t> REGISTER_READ_LENGTH_MAP;
    
    /**
     * @brief 根据寄存器名称获取默认读取长度
     * @param reg_name 寄存器名称
     * @return 默认读取长度（字节数），如果未找到则返回12（通用默认值）
     */
    size_t getDefaultReadLength(const std::string& reg_name) const;
    
    /**
     * @brief 从环形缓冲区读取指定偏移位置的字节
     * @param ringBuffer 环形缓冲区
     * @param offset 从tail开始的偏移量
     * @return 读取到的字节，如果超出范围返回0xFF
     */
    uint8_t readByteAtOffset(const RingBuffer& ringBuffer, size_t offset) const;
    
    /**
     * @brief 从环形缓冲区提取指定范围的数据到vector（仅在需要校验时才调用）
     * @param ringBuffer 环形缓冲区
     * @param startOffset 起始偏移（从tail开始）
     * @param length 提取长度
     * @return 提取的数据
     */
    std::vector<uint8_t> extractFromRingBuffer(const RingBuffer& ringBuffer, size_t startOffset, size_t length) const;
    
    /**
     * @brief 将字节数组格式化为十六进制字符串（用于debug日志）
     * @param data 字节数组
     * @param length 数据长度
     * @return 十六进制字符串（格式：0xXX 0xXX ...）
     */
    std::string formatBytesToHex(const uint8_t* data, size_t length) const;
    
    /**
     * @brief 循环读取响应数据，直到接收到足够字节或超时
     * @param device 设备对象
     * @param timeout_ms 总超时时间（毫秒）
     * @param min_bytes 最小字节数（写回复固定9字节，读回复需要动态判断）
     * @param is_read_response 是否为读回复（true=读回复需要动态计算长度，false=写回复固定9字节）
     * @return 读取到的数据
     */
    std::vector<uint8_t> readResponseWithLoop(Device device, int timeout_ms, size_t min_bytes = 9, bool is_read_response = false) const;
};


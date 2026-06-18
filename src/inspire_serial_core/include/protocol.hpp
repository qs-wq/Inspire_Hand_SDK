#pragma once

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <iostream>

#include "serial_port.hpp"
#include "ring_buffer.hpp"
#include "logger_manager.hpp"
#include "io_error.hpp"

// 触觉数据结构体
struct TouchDataResult {
    using FingerDataMap = std::map<std::string, std::vector<uint16_t>>;
    using PalmDataMap = std::map<std::string, uint16_t>;

    FingerDataMap fingerResults;
    PalmDataMap palmResults;
};

// 读触觉数据结果：错误码 + 触觉数据
struct TouchReadResult {
    IoError error = IoError::Ok;
    TouchDataResult data;

    bool ok() const { return error == IoError::Ok; }
};

using Device = std::shared_ptr<SerialPortBase>;

/**
 * @brief 协议抽象基类
 * 
 * 定义所有协议实现必须遵循的接口规范。
 * 具体的协议实现（如RH56F1_485_Protocol）继承此类并实现所有纯虚函数。
 */
class Protocol {
protected:
    // 当前协议实例所对应的设备ID（Hand_ID），默认1
    uint8_t device_id_ = 0x01;
    // 当前协议实例所对应的设备名称
    std::string device_name_;

public:
    virtual ~Protocol() = default;

    /**
     * @brief 设置当前协议实例所使用的设备ID（Hand_ID）
     * @param id 设备ID（1~254）
     */
    void setDeviceId(uint8_t id) { device_id_ = id; }

    /**
     * @brief 获取当前协议实例所使用的设备ID（Hand_ID）
     * @return 设备ID
     */
    uint8_t getDeviceId() const { return device_id_; }

    /**
     * @brief 设置当前协议实例所对应的设备名称（用于日志前缀）
     * @param name 设备名称
     */
    void setDeviceName(const std::string& name) { device_name_ = name; }

    /**
     * @brief 获取当前协议实例所对应的设备名称
     * @return 设备名称
     */
    const std::string& getDeviceName() const { return device_name_; }

    /**
     * @brief 获取日志对象（自动设置设备名前缀）
     * @return 日志对象指针
     */
    std::shared_ptr<spdlog::logger> getLogger() const {
        // 设置当前线程的设备名（用于日志前缀）
        if (!device_name_.empty()) {
            LoggerManager::setThreadDeviceName(device_name_);
        }
        return ::getLogger();
    }

    /**
     * @brief 根据寄存器名称获取寄存器地址
     * @param register_name 寄存器名称（如"angleSet"）
     * @return 寄存器地址，如果不存在返回-1
     */
    virtual int getRegisterAddress(const std::string& register_name) const = 0;
    
    /**
     * @brief 构建读取命令
     * @param address 寄存器地址
     * @param length 读取长度
     * @return 命令字节序列
     */
    virtual std::vector<uint8_t> buildReadCommand(int address, size_t length) = 0;
    
    /**
     * @brief 构建写入命令
     * @param address 寄存器地址
     * @param values 要写入的值列表
     * @return 命令字节序列
     */
    virtual std::vector<uint8_t> buildWriteCommand(int address, const std::vector<int>& values) = 0;
    
    /**
     * @brief 解析响应数据
     * @param ringBuffer 环形缓冲区，包含响应数据
     * @return 解析结果（成功标志，解析出的值列表）
     */
    virtual std::pair<bool, std::vector<int>> parseResponse(RingBuffer& ringBuffer) = 0;
    
    /**
     * @brief 验证响应数据的校验和
     * @param response 响应数据
     * @return true表示校验和正确，false表示错误
     */
    virtual bool validateChecksum(const std::vector<uint8_t>& response) const = 0;

    /**
     * @brief 解析触觉数据
     * @param ringBuffer 环形缓冲区，包含触觉数据响应
     * @param version 触觉数据版本号
     * @return 解析结果（成功标志，触觉数据结构）
     */
    virtual std::pair<bool, TouchDataResult> parseTouchData(RingBuffer& ringBuffer, int version) = 0;

    /**
     * @brief 写入寄存器值
     * @param device 设备对象
     * @param reg_name 寄存器名称
     * @param values 要写入的值列表
     * @return IoError::Ok 表示成功，否则为对应错误码
     */
    virtual IoError writeRegister(Device device, const std::string& reg_name, const std::vector<int>& values) = 0;
    
    /**
     * @brief 读取寄存器值
     * @param device 设备对象
     * @param ringBuffer 环形缓冲区，用于缓存响应数据
     * @param reg_name 寄存器名称
     * @param length 读取长度
     * @return 读取结果（错误码 + 读取到的值列表）
     */
    virtual RegisterReadResult readRegister(Device device, RingBuffer& ringBuffer, const std::string& reg_name, size_t length) = 0;
    
    /**
     * @brief 读取触觉数据
     * @param device 设备对象
     * @param ringBuffer 环形缓冲区，用于缓存响应数据
     * @param version 触觉数据版本号
     * @return 读取结果（错误码 + 触觉数据结构）
     */
    virtual TouchReadResult readTouchData(Device device, RingBuffer& ringBuffer, int version) = 0;
};


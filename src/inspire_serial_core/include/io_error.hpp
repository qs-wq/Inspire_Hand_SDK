#pragma once

#include <cstdint>
#include <vector>

/**
 * @brief 寄存器读写统一错误码。
 *
 * 贯穿 Protocol（协议层）→ IRegisterIoBackend（节点后端）→ InterfaceAdapter（ROS 适配器）→ Service 响应，
 * 让上层能区分超时 / 校验失败 / 设备离线 / 参数非法等不同失败原因，而非仅一个 bool。
 */
enum class IoError : std::uint8_t {
    Ok = 0,          ///< 成功
    Timeout,         ///< 无应答 / 读超时
    ChecksumError,   ///< 校验和不匹配
    BadResponse,     ///< 帧格式非法 / 解析失败
    UnknownRegister, ///< 寄存器名未注册
    InvalidArgument, ///< 参数非法（如值越界、个数超限）
    NotSupported,    ///< 该机型 / 协议不支持此操作
    DeviceError,     ///< 串口 / 设备异常
};

/** @brief 是否成功 */
inline bool isOk(IoError e) { return e == IoError::Ok; }

/** @brief 错误码转可读字符串（日志 / Service message 用） */
inline const char* toString(IoError e) {
    switch (e) {
    case IoError::Ok:
        return "ok";
    case IoError::Timeout:
        return "timeout";
    case IoError::ChecksumError:
        return "checksum_error";
    case IoError::BadResponse:
        return "bad_response";
    case IoError::UnknownRegister:
        return "unknown_register";
    case IoError::InvalidArgument:
        return "invalid_argument";
    case IoError::NotSupported:
        return "not_supported";
    case IoError::DeviceError:
        return "device_error";
    }
    return "unknown";
}

/**
 * @brief 读寄存器结果：错误码 + 解析出的整型值列表。
 */
struct RegisterReadResult {
    IoError error = IoError::Ok;
    std::vector<int> values;

    bool ok() const { return error == IoError::Ok; }
};

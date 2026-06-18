#include "RH56F1_canfd_protocol.hpp"
#include "logger_manager.hpp"
#include "protocol_factory.hpp"

#include <sstream>
#include <algorithm>

// CAN-FD 合法字节长度列表
static const std::vector<size_t> VALID_CANFD_LENGTHS = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

// 将请求字节长度补齐到最近的合法 CAN-FD 字节长度
size_t RH56F1_canfd_Protocol::adjustToValidCanfdLength(size_t requested_bytes) const {
    // 如果请求长度已经是合法长度，直接返回
    if (std::find(VALID_CANFD_LENGTHS.begin(), VALID_CANFD_LENGTHS.end(), requested_bytes) != VALID_CANFD_LENGTHS.end()) {
        return requested_bytes;
    }
    
    // 如果请求长度超过最大值64，返回64（分帧逻辑会在上层处理）
    if (requested_bytes > 64) {
        return 64;
    }
    
    // 向上补齐到最近的合法字节长度
    for (size_t valid_len : VALID_CANFD_LENGTHS) {
        if (valid_len >= requested_bytes) {
            return valid_len;
        }
    }

    return 64;
}

// 写命令构建：与 485 版本格式一致，只是允许更大的数据长度（最多 64 字节）
std::vector<uint8_t> RH56F1_canfd_Protocol::buildWriteCommand(int address, const std::vector<int>& values) {
    // 这里的 values 数量由上层拆帧逻辑控制，保证字节数不超过 64
    const size_t bytes = values.size() * 2;
    if (bytes > 64) {
        throw std::runtime_error("CAN-FD 单帧写入数据长度超过 64 字节");
    }

    std::vector<uint8_t> cmd = {
        0xEB, 0x90,           // 帧头
        device_id_,           // 设备ID（来自配置 Hand_ID）
        static_cast<uint8_t>(bytes + 3), // 数据长度 = Register_Length + 3
        0x12,                 // 写入命令
        static_cast<uint8_t>(address & 0xFF),         // 地址低字节
        static_cast<uint8_t>((address >> 8) & 0xFF)   // 地址高字节
    };

    // 写入数据（每个值两个字节，小端序）
    for (int value : values) {
        cmd.push_back(static_cast<uint8_t>(value & 0xFF));        // 低字节
        cmd.push_back(static_cast<uint8_t>((value >> 8) & 0xFF)); // 高字节
    }

    // 计算校验和（从第2个字节开始到最后一个数据字节）
    uint8_t checksum = 0;
    for (size_t i = 2; i < cmd.size(); ++i) {
        checksum += cmd[i];
    }
    cmd.push_back(checksum);
    return cmd;
}

IoError RH56F1_canfd_Protocol::writeRegister(Device device, const std::string& reg_name, const std::vector<int>& values) {
    auto logger = getLogger();
    std::ostringstream oss;

    auto it = REGISTER_MAP.find(reg_name);
    if (it == REGISTER_MAP.end()) {
        oss << "未知寄存器名: " << reg_name;
        logger->error(oss.str());
        return IoError::UnknownRegister;
    }
    int base_address = it->second;

    if (values.empty()) {
        logger->warn("写入寄存器 {} 的值为空，跳过写入", reg_name);
        return IoError::Ok;
    }

    // 总逻辑字节长度
    const size_t total_bytes = values.size() * 2;
    size_t remaining_values = values.size();
    size_t value_index = 0;
    int current_address = base_address;

    bool all_ok = true;

    while (remaining_values > 0) {
        // 当前剩余逻辑字节数
        size_t remaining_bytes = remaining_values * 2;

        // 计算本帧 CAN-FD 数据段字节数（<=64，不需要补全）
        size_t frame_bytes;
        if (remaining_bytes > 64) {
            frame_bytes = 64;
        } else {
            frame_bytes = remaining_bytes;  // 直接使用剩余字节数，不补全
        }

        size_t frame_value_count = frame_bytes / 2;  // 本帧实际需要的值数量

        // 构造本帧写入数据（不需要填充）
        std::vector<int> frame_values;
        frame_values.reserve(frame_value_count);
        for (size_t i = 0; i < frame_value_count; ++i) {
            frame_values.push_back(values[value_index + i]);
        }

        try {
            auto cmd = buildWriteCommand(current_address, frame_values);

            logger->debug("[CANFD-写入命令] 寄存器: {}, 地址: 0x{:04X}, 帧数据字节: {}, 命令: {}",
                           reg_name, current_address, frame_bytes,
                           formatBytesToHex(cmd.data(), cmd.size()));

            device->write(cmd);

            // 每帧均需要确认写入应答
            auto writeResponse = readResponseWithLoop(device, 25, 9, false);
            if (!writeResponse.empty()) {
                logger->debug("[CANFD-写响应] 寄存器: {}, 地址: 0x{:04X}, 响应: {}",
                               reg_name, current_address,
                               formatBytesToHex(writeResponse.data(), writeResponse.size()));
            } else {
                logger->error("[CANFD-写响应] 寄存器: {}, 地址: 0x{:04X}, 响应为空", reg_name, current_address);
                return IoError::Timeout;
            }

            // 使用 parseResponse 校验帧合法性
            RingBuffer tempBuffer(128);
            tempBuffer.push(writeResponse.data(), writeResponse.size());
            auto [parseSuccess, _] = parseResponse(tempBuffer);
            if (!parseSuccess) {
                logger->error("[CANFD] 写入寄存器响应解析失败：{}, 地址: 0x{:04X}", reg_name, current_address);
                return IoError::BadResponse;
            }
        } catch (const std::exception& e) {
            logger->error("[CANFD] 写入寄存器失败：{}, 地址: 0x{:04X}, 异常: {}", reg_name, current_address, e.what());
            return IoError::DeviceError;
        } catch (...) {
            logger->error("[CANFD] 写入寄存器失败：{}, 地址: 0x{:04X}", reg_name, current_address);
            return IoError::DeviceError;
        }

        // 更新索引 / 地址
        remaining_values -= frame_value_count;
        value_index += frame_value_count;
        current_address += static_cast<int>(frame_value_count);
    }

    // 只有当所有帧写入都成功时，才认为整体成功，并打印一次汇总日志
    if (all_ok) {
        oss << "[CANFD] 写入" << reg_name << ":(";
        for (size_t i = 0; i < values.size(); ++i) {
            oss << values[i];
            if (i != values.size() - 1) oss << " ";
        }
        oss << ")";
        logger->info(oss.str());
    }

    // 特殊处理逻辑：
    // - defaultSpeedSet / defaultForceSet 写完后写 save=1
    // - actionSeqIndex 写完后写 actionSeqRun=1
    try {
        if (reg_name == "defaultSpeedSet" || reg_name == "defaultForceSet") {
            logger->debug("[CANFD-特殊处理] 检测到 {} 写入完成，准备写入 save 寄存器", reg_name);

            std::this_thread::sleep_for(std::chrono::milliseconds(5));

            auto saveCmd = buildWriteCommand(REGISTER_MAP.at("save"), {1});
            logger->debug("[CANFD-写入命令] 寄存器: save, 地址: 0x{:04X}, 命令: {}",
                           REGISTER_MAP.at("save"), formatBytesToHex(saveCmd.data(), saveCmd.size()));

            device->write(saveCmd);
            auto saveResponse = readResponseWithLoop(device, 25, 9, false);

            if (!saveResponse.empty()) {
                RingBuffer tempBuffer(128);
                tempBuffer.push(saveResponse.data(), saveResponse.size());
                auto [parseSuccess, _] = parseResponse(tempBuffer);
                if (parseSuccess) {
                    logger->debug("[CANFD-写响应] 寄存器: save, 响应: {}",
                                   formatBytesToHex(saveResponse.data(), saveResponse.size()));
                    logger->info("[CANFD] 写入save:(1)");
                } else {
                    logger->warn("[CANFD-写响应] 寄存器: save, 响应格式验证失败");
                    return IoError::BadResponse;
                }
            } else {
                logger->warn("[CANFD-写响应] 寄存器: save, 响应为空");
                return IoError::Timeout;
            }
        } else if (reg_name == "actionSeqIndex") {
            logger->debug("[CANFD-特殊处理] 检测到 actionSeqIndex 写入完成，准备写入 actionSeqRun 寄存器");

            std::this_thread::sleep_for(std::chrono::milliseconds(5));

            auto actionSeqRunCmd = buildWriteCommand(REGISTER_MAP.at("actionSeqRun"), {1});
            logger->debug("[CANFD-写入命令] 寄存器: actionSeqRun, 地址: 0x{:04X}, 命令: {}",
                           REGISTER_MAP.at("actionSeqRun"),
                           formatBytesToHex(actionSeqRunCmd.data(), actionSeqRunCmd.size()));

            device->write(actionSeqRunCmd);
            auto actionSeqRunResponse = readResponseWithLoop(device, 25, 9, false);

            if (!actionSeqRunResponse.empty()) {
                RingBuffer tempBuffer(128);
                tempBuffer.push(actionSeqRunResponse.data(), actionSeqRunResponse.size());
                auto [parseSuccess, _] = parseResponse(tempBuffer);
                if (parseSuccess) {
                    logger->debug("[CANFD-写响应] 寄存器: actionSeqRun, 响应: {}",
                                   formatBytesToHex(actionSeqRunResponse.data(), actionSeqRunResponse.size()));
                    logger->info("[CANFD] 写入actionSeqRun:(1)");
                } else {
                    logger->warn("[CANFD-写响应] 寄存器: actionSeqRun, 响应格式验证失败");
                    return IoError::BadResponse;
                }
            } else {
                logger->warn("[CANFD-写响应] 寄存器: actionSeqRun, 响应为空");
                return IoError::Timeout;
            }
        }
    } catch (const std::exception& e) {
        logger->error("[CANFD] 写入寄存器后续处理失败：{}, 异常: {}", reg_name, e.what());
        return IoError::DeviceError;
    }

    return all_ok ? IoError::Ok : IoError::BadResponse;
}

RegisterReadResult RH56F1_canfd_Protocol::readRegister(
    Device device,
    RingBuffer& ringBuffer,
    const std::string& reg_name,
    size_t length) {

    auto logger = getLogger();
    std::ostringstream oss;

    // 每次读取前清空环形缓冲，避免历史数据干扰
    ringBuffer.clear();

    auto it = REGISTER_MAP.find(reg_name);
    if (it == REGISTER_MAP.end()) {
        oss << "未知寄存器名: " << reg_name;
        logger->error(oss.str());
        return {IoError::UnknownRegister, {}};
    }
    int base_address = it->second;

    // 逻辑请求长度（字节），0 表示使用默认长度
    size_t target_bytes = length;
    if (target_bytes == 0) {
        target_bytes = getDefaultReadLength(reg_name);
        if (target_bytes == 0) {
            logger->warn("[CANFD-动态长度] 寄存器 {} 未配置默认读取长度，使用通用默认值12字节", reg_name);
            target_bytes = 12;
        }
        logger->debug("[CANFD-动态长度] 寄存器: {}, 自动确定读取长度: {} 字节", reg_name, target_bytes);
    }

    if (target_bytes == 0) {
        logger->warn("[CANFD] 读取寄存器 {} 请求长度为0，直接返回空结果", reg_name);
        return {IoError::Ok, {}};
    }

    size_t remaining_bytes = target_bytes;
    int current_address = base_address;
    std::vector<int> all_values;

    while (remaining_bytes > 0) {
        // 计算本帧实际请求的字节数（逻辑剩余字节数，<=64）
        size_t logical_frame_bytes;
        if (remaining_bytes > 64) {
            logical_frame_bytes = 64;
        } else {
            logical_frame_bytes = remaining_bytes;
        }
        
        // 将请求字节数补齐到最近的合法 CAN-FD 字节长度
        size_t frame_bytes = adjustToValidCanfdLength(logical_frame_bytes);
        
        if (frame_bytes != logical_frame_bytes) {
            logger->debug("[CANFD-长度补齐] 寄存器: {}, 逻辑请求: {} 字节, 补齐到: {} 字节",
                          reg_name, logical_frame_bytes, frame_bytes);
        }

        auto cmd = buildReadCommand(current_address, frame_bytes);
        logger->debug("[CANFD-读取命令] 寄存器: {}, 地址: 0x{:04X}, 逻辑剩余: {} 字节, 实际发送: {} 字节, 命令: {}",
                      reg_name, current_address, remaining_bytes, frame_bytes,
                      formatBytesToHex(cmd.data(), cmd.size()));

        try {
            device->write(cmd);

            auto response = readResponseWithLoop(device, 25, 8, true);
            if (!response.empty()) {
                logger->debug("[CANFD-原始响应] 寄存器: {}, 地址: 0x{:04X}, 响应: {}",
                               reg_name, current_address,
                               formatBytesToHex(response.data(), response.size()));
            } else {
                logger->error("[CANFD] 读取寄存器失败：{} (地址 0x{:04X}, 无响应)", reg_name, current_address);
                return {IoError::Timeout, {}};
            }

            // 使用独立的 ringBuffer 解析当前帧
            ringBuffer.clear();
            ringBuffer.push(response.data(), response.size());
            auto [ok, frame_values] = parseResponse(ringBuffer);
            if (!ok) {
                logger->error("[CANFD] 解析寄存器回复失败：{} (地址 0x{:04X})", reg_name, current_address);
                return {IoError::BadResponse, {}};
            }

            // 每个值占 2 字节
            // 注意：frame_values 包含补齐后的所有数据，但只使用逻辑请求的长度
            size_t available_bytes = frame_values.size() * 2;
            // 只使用逻辑请求的字节数（logical_frame_bytes），丢弃补齐的部分
            size_t used_bytes = std::min(available_bytes, logical_frame_bytes);
            size_t used_values = used_bytes / 2;

            if (used_values > frame_values.size()) {
                logger->error("[CANFD] 解析得到的数据长度不足：{} (需要 {} 个值)", reg_name, used_values);
                return {IoError::BadResponse, {}};
            }

            // 打印实际使用的值个数（而不是补齐后的个数）
            if (frame_values.size() != used_values) {
                logger->debug("[CANFD] 解析读寄存器回复: 补齐后 {} 个值, 实际使用 {} 个值", 
                              frame_values.size(), used_values);
            } else {
                logger->debug("[CANFD] 成功解析读寄存器回复: {} 个值", used_values);
            }

            all_values.insert(all_values.end(), frame_values.begin(), frame_values.begin() + used_values);

            remaining_bytes -= used_bytes;
            current_address += static_cast<int>(used_values);

        } catch (const std::exception& e) {
            logger->error("[CANFD] 读取寄存器异常：{}, 地址 0x{:04X}, 异常: {}", reg_name, current_address, e.what());
            return {IoError::DeviceError, {}};
        } catch (...) {
            logger->error("[CANFD] 读取寄存器异常：{}, 地址 0x{:04X}", reg_name, current_address);
            return {IoError::DeviceError, {}};
        }
    }

    // 打印汇总日志（只打印逻辑请求的数据）
    oss << "[CANFD] 读取" << reg_name << ":(";
    for (size_t i = 0; i < all_values.size(); ++i) {
        oss << all_values[i];
        if (i != all_values.size() - 1) oss << " ";
    }
    oss << ")";
    logger->info(oss.str());

    return {IoError::Ok, std::move(all_values)};
}

// 触觉数据读取（逻辑长度固定 68 字节），内部按照 64B + 4B 拆两帧
TouchReadResult RH56F1_canfd_Protocol::readTouchData(
    Device device,
    RingBuffer& ringBuffer,
    int version) {

    auto logger = getLogger();
    std::ostringstream oss;

    if (version != 1) {
        logger->warn("[CANFD-触觉] 暂只实现版本1解析，version={}, 返回失败", version);
        return {IoError::NotSupported, {}};
    }

    try {
        ringBuffer.clear();

        const int touchAddress = getRegisterAddress("touchAct");
        const size_t logical_bytes = 68;   // 逻辑触觉数据总长度

        // 使用与 readRegister 相同的拆帧规则读取原始字节
        size_t remaining_bytes = logical_bytes;
        int current_address = touchAddress;
        std::vector<uint8_t> raw_bytes;
        raw_bytes.reserve(logical_bytes);

        while (remaining_bytes > 0) {
            // 计算本帧实际请求的字节数（逻辑剩余字节数，<=64）
            size_t logical_frame_bytes;
            if (remaining_bytes > 64) {
                logical_frame_bytes = 64;
            } else {
                logical_frame_bytes = remaining_bytes;
            }
            
            // 将请求字节数补齐到最近的合法 CAN-FD 字节长度
            size_t frame_bytes = adjustToValidCanfdLength(logical_frame_bytes);
            
            if (frame_bytes != logical_frame_bytes) {
                logger->debug("[CANFD-长度补齐-触觉] 逻辑请求: {} 字节, 补齐到: {} 字节",
                              logical_frame_bytes, frame_bytes);
            }

            auto readTouchCmd = buildReadCommand(current_address, frame_bytes);
            logger->debug("[CANFD-读取命令-触觉] 地址: 0x{:04X}, 逻辑剩余: {} 字节, 实际发送: {} 字节, 命令: {}",
                           current_address, remaining_bytes, frame_bytes,
                           formatBytesToHex(readTouchCmd.data(), readTouchCmd.size()));

            device->write(readTouchCmd);
            auto resp = readResponseWithLoop(device, 25, 8, true);

            if (!resp.empty()) {
                logger->debug("[CANFD-原始响应-触觉] 地址: 0x{:04X}, 响应: {}",
                               current_address, formatBytesToHex(resp.data(), resp.size()));
            } else {
                logger->error("[CANFD-触觉] 读取失败：地址 0x{:04X} 无响应", current_address);
                return {IoError::Timeout, {}};
            }

            // 校验并提取本帧数据段
            if (!validateChecksum(resp)) {
                logger->error("[CANFD-触觉] 校验和错误，地址 0x{:04X}", current_address);
                return {IoError::ChecksumError, {}};
            }
            if (resp.size() < 8) {
                logger->error("[CANFD-触觉] 响应长度过短，地址 0x{:04X}", current_address);
                return {IoError::BadResponse, {}};
            }

            const uint8_t hands_id = resp[2];
            const uint8_t data_length = resp[3];
            const uint8_t command = resp[4];

            if (device_id_ != 0 && hands_id != device_id_) {
                logger->error("[CANFD-触觉] Hands_ID 不匹配: 期望 {}, 实际 {}, 地址 0x{:04X}",
                              device_id_, hands_id, current_address);
                return {IoError::BadResponse, {}};
            }
            if (command != 0x11) {
                logger->error("[CANFD-触觉] 命令类型错误: 0x{:02X}, 期望 0x11, 地址 0x{:04X}",
                              command, current_address);
                return {IoError::BadResponse, {}};
            }
            if (data_length < 3) {
                logger->error("[CANFD-触觉] data_length 无效: {}, 地址 0x{:04X}", data_length, current_address);
                return {IoError::BadResponse, {}};
            }

            const size_t register_length = static_cast<size_t>(data_length) - 3;
            const size_t expected_frame_size = 8 + register_length;
            if (resp.size() < expected_frame_size) {
                logger->error("[CANFD-触觉] 响应长度不足: 实际 {}, 期望 >= {}, 地址 0x{:04X}",
                              resp.size(), expected_frame_size, current_address);
                return {IoError::BadResponse, {}};
            }

            // 提取数据段（不包含头/地址/校验）
            // 注意：register_length 可能包含补齐后的数据，但只使用逻辑请求的长度
            const size_t bytes_to_use = std::min(register_length, logical_frame_bytes);
            const size_t data_start = 7;
            for (size_t i = 0; i < bytes_to_use; ++i) {
                raw_bytes.push_back(resp[data_start + i]);
            }

            remaining_bytes -= bytes_to_use;
            current_address += static_cast<int>(bytes_to_use / 2); // 每 2 字节一个寄存器
        }

        if (raw_bytes.size() < logical_bytes) {
            logger->error("[CANFD-触觉] 合并后的数据长度不足: 实际 {} 字节, 期望 {} 字节",
                          raw_bytes.size(), logical_bytes);
            return {IoError::BadResponse, {}};
        }

        // 下面的解析逻辑参考 RH56F1_485_Protocol::parseTouchData(version=1)
        TouchDataResult result;
        const std::string fingers[] = {"little", "ring", "middle", "index", "thumb"};

        // 每个手指 10 字节：3 个 16 位值(6字节) + 1 个 24 位值(3字节) + 1 字节保留
        const size_t finger_data_length = 5 * 10; // 5 个手指，共 50 字节

        // 解析手指触觉数据
        for (int i = 0; i < 5; ++i) {
            size_t base_idx = i * 10;
            if (base_idx + 10 > raw_bytes.size()) {
                logger->warn("[CANFD-触觉] 数据不足，无法解析第 {} 个手指数据", i + 1);
                break;
            }

            std::vector<uint16_t> finger_vals;
            finger_vals.reserve(4);

            // 3 个 16 位值
            for (int j = 0; j < 3; ++j) {
                uint8_t low = raw_bytes[base_idx + j * 2];
                uint8_t high = raw_bytes[base_idx + j * 2 + 1];
                uint16_t val = static_cast<uint16_t>(low | (high << 8));
                finger_vals.push_back(val);
            }

            // 1 个 24 位值
            uint8_t b0 = raw_bytes[base_idx + 6];
            uint8_t b1 = raw_bytes[base_idx + 7];
            uint8_t b2 = raw_bytes[base_idx + 8];
            uint32_t combined = static_cast<uint32_t>(b0 | (b1 << 8) | (b2 << 16));
            finger_vals.push_back(static_cast<uint16_t>(combined)); // 保留低 16 位（与原实现一致）

            result.fingerResults[fingers[i]] = std::move(finger_vals);
        }

        // 解析掌心触觉数据（9 个 16 位值，共 18 字节）
        size_t palm_start_idx = finger_data_length;
        const size_t palm_data_length = 9 * 2;
        if (palm_start_idx + palm_data_length <= raw_bytes.size()) {
            for (int j = 0; j < 9; ++j) {
                size_t idx = palm_start_idx + j * 2;
                uint16_t val = static_cast<uint16_t>(raw_bytes[idx] | (raw_bytes[idx + 1] << 8));
                result.palmResults["palm_data_" + std::to_string(j + 1)] = val;
            }
        } else {
            logger->warn("[CANFD-触觉] 掌心数据长度不足 (需要 {} 字节，实际 {} 字节)",
                         palm_data_length, raw_bytes.size() > palm_start_idx
                             ? (raw_bytes.size() - palm_start_idx)
                             : 0);
        }

        logger->debug("[CANFD-触觉] 成功解析触觉数据: {} 个手指, {} 个掌心数据点",
                      result.fingerResults.size(), result.palmResults.size());

        // 打印概览日志
        oss << "[CANFD] 读取touchAct:(";
        for (const auto& finger_pair : result.fingerResults) {
            oss << finger_pair.first << ":";
            for (size_t i = 0; i < finger_pair.second.size(); ++i) {
                oss << finger_pair.second[i];
                if (i != finger_pair.second.size() - 1) oss << " ";
            }
            oss << " ";
        }
        for (const auto& palm_pair : result.palmResults) {
            oss << palm_pair.first << ":" << palm_pair.second << " ";
        }
        oss << ")";
        logger->info(oss.str());

        return {IoError::Ok, std::move(result)};

    } catch (const std::exception& e) {
        logger->error("[CANFD-触觉] 读取触觉寄存器异常: {}", e.what());
        return {IoError::DeviceError, {}};
    } catch (...) {
        logger->error("[CANFD-触觉] 读取触觉寄存器异常");
        return {IoError::DeviceError, {}};
    }
}

// 自动注册 RH56F1 CAN-FD 协议
// 在设备配置文件 device_protocol_config.yaml 中，可通过：
//   protocol:
//     type: RH56F1_canfd
// 来选择使用本协议。
REGISTER_PROTOCOL("RH56F1_canfd", RH56F1_canfd_Protocol);


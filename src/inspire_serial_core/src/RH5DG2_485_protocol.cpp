#include "RH5DG2_485_protocol.hpp"
#include "logger_manager.hpp"  // 提供全局getLogger()
#include "protocol_factory.hpp"  // 用于协议注册

// 寄存器字典
const std::map<std::string, int> RH5DG2_485_Protocol::REGISTER_MAP = {
    {"id", 1000},
    {"baudRate", 1001},
    {"clearError", 1003},
    {"save", 1004},
    {"resetPara", 1005},
    {"gestureForceClb", 1007},
    {"currentSet", 1024},
    {"defaultSpeedSet", 1038},
    {"defaultForceSet", 1052},
    {"posSet", 1066},
    {"angleSet", 1080},
    {"forceSet", 1094},
    {"speedSet", 1108},
    {"posAct", 1122},
    {"angleAct", 1136},
    {"forceAct", 1150},
    {"currentAct", 1164},
    {"errorCode", 1178}, 
    {"status", 1192},
    {"temp", 1206},  
    {"mode", 1220},
    {"pause", 1300},
    {"stop", 1301},
    {"actionLibraryIndex", 1431},
    {"actionLibraryRun", 1432},    
    {"actionSeqIndex", 2160},
    {"actionSeqRun", 2162}, 
    {"touchAct", 3000} 
};

// 寄存器默认读取长度映射表（字节数）
const std::map<std::string, size_t> RH5DG2_485_Protocol::REGISTER_READ_LENGTH_MAP = {
    // 多值寄存器（13个自由度，26字节）
    {"angleSet", 26},
    {"angleAct", 26},
    {"forceSet", 26},
    {"forceAct", 26},
    {"currentSet", 26},
    {"currentAct", 26},
    {"posSet", 26},
    {"posAct", 26},
    {"speedSet", 26},
    {"defaultSpeedSet", 26},
    {"defaultForceSet", 26},
    {"errorCode", 26},
    {"status", 26},
    {"temp", 26},
    {"mode", 26},
    // 单值寄存器（2字节）
    {"id", 2},
    {"baudRate", 2},
    {"clearError", 2},
    {"save", 2},
    {"resetPara", 2},
    {"gestureForceClb", 2},
    {"pause", 2},
    {"stop", 2},
    {"actionSeqIndex", 2},
    {"actionSeqRun", 2},
    {"actionLibraryIndex", 2},
    {"actionLibraryRun", 2},
    // 触觉数据（特殊处理，不使用此映射）
    // {"touchAct", 68},  // 触觉数据使用专门的readTouchData函数
};

size_t RH5DG2_485_Protocol::getDefaultReadLength(const std::string& reg_name) const {
    auto it = REGISTER_READ_LENGTH_MAP.find(reg_name);
    if (it != REGISTER_READ_LENGTH_MAP.end()) {
        return it->second;
    }
    // 如果未找到，返回通用默认值26字节（13个值）
    return 26;
}

int RH5DG2_485_Protocol::getRegisterAddress(const std::string& register_name) const {
    auto it = REGISTER_MAP.find(register_name);
    return (it != REGISTER_MAP.end()) ? it->second : -1;
}

std::vector<uint8_t> RH5DG2_485_Protocol::buildWriteCommand(int address, const std::vector<int>& values) {
    if (values.size() > 13) {
        throw std::runtime_error("超过允许的值数量，最多只能写入13个值");
    }

    // 根据协议规范：RS485写寄存器指令帧格式
    // byte[0]: 0xEB (包头)
    // byte[1]: 0x90 (包头)
    // byte[2]: Hands_ID (灵巧手ID号)
    // byte[3]: Register_Length + 3 (该帧数据部分长度)
    // byte[4]: 0x12 (写寄存器命令标志)
    // byte[5]: Address_L (寄存器起始地址低八位)
    // byte[6]: Address_H (寄存器起始地址高八位)
    // byte[7] ~ byte[7+Register_Length-1]: Data[0] ~ Data[Register_Length-1] (写入数据)
    // byte[7+Register_Length]: Checksum (校验和)
    std::vector<uint8_t> cmd = {
        0xEB, 0x90,           // 帧头
        device_id_,           // byte[2]: Hands_ID
        static_cast<uint8_t>(values.size() * 2 + 3), // byte[3]: Register_Length + 3
        0x12,                 // byte[4]: 写寄存器命令标志
        static_cast<uint8_t>(address & 0xFF),     // byte[5]: Address_L
        static_cast<uint8_t>((address >> 8) & 0xFF) // byte[6]: Address_H
    };

    // 写入数据（每个值两个字节，小端序：低字节在前，高字节在后）
    // byte[7] ~ byte[7+Register_Length-1]: Data[0] ~ Data[Register_Length-1]
    for (int value : values) {
        cmd.push_back(static_cast<uint8_t>(value & 0xFF));        // 低字节
        cmd.push_back(static_cast<uint8_t>((value >> 8) & 0xFF)); // 高字节
    }

    // 计算校验和：从byte[2]开始累加到最后一个数据字节（不包括校验和本身）
    uint8_t checksum = 0;
    for (size_t i = 2; i < cmd.size(); ++i) {
        checksum += cmd[i];
    }
    cmd.push_back(checksum); // byte[7+Register_Length]: Checksum

    return cmd;
}

std::vector<uint8_t> RH5DG2_485_Protocol::buildReadCommand(int address, size_t length) {
    // 根据协议规范：RS485读寄存器指令帧格式
    // byte[0]: 0xEB (包头)
    // byte[1]: 0x90 (包头)
    // byte[2]: Hands_ID (灵巧手ID号)
    // byte[3]: 0x04 (该帧数据部分长度，固定4)
    // byte[4]: 0x11 (读寄存器)
    // byte[5]: Address_L (寄存器起始地址低八位)
    // byte[6]: Address_H (寄存器起始地址高八位)
    // byte[7]: Register_Length (读取数据的字节长度)
    // byte[8]: Checksum (校验和)
    std::vector<uint8_t> cmd = {
        0xEB, 0x90,           // 帧头
        device_id_,           // byte[2]: Hands_ID
        0x04,                 // byte[3]: 该帧数据部分长度（固定4）
        0x11,                 // byte[4]: 读寄存器
        static_cast<uint8_t>(address & 0xFF),        // byte[5]: Address_L
        static_cast<uint8_t>((address >> 8) & 0xFF), // byte[6]: Address_H
        static_cast<uint8_t>(length & 0xFF)          // byte[7]: Register_Length (字节长度)
    };

    // 计算校验和：从byte[2]开始累加到最后一个数据字节（不包括校验和本身）
    uint8_t checksum = 0;
    for (size_t i = 2; i < cmd.size(); ++i) {
        checksum += cmd[i];
    }
    cmd.push_back(checksum); // byte[8]: Checksum
    return cmd;
}

// 辅助函数：从环形缓冲区读取指定偏移位置的字节
uint8_t RH5DG2_485_Protocol::readByteAtOffset(const RingBuffer& ringBuffer, size_t offset) const {
    size_t bufSize = ringBuffer.size();
    if (offset >= bufSize) {
        return 0xFF;
    }
    const std::vector<uint8_t>& buf = ringBuffer.getBuffer();
    size_t tailIndex = ringBuffer.getTail();
    size_t bufferSize = buf.size();
    return buf[(tailIndex + offset) % bufferSize];
}

// 辅助函数：从环形缓冲区提取指定范围的数据（仅在需要校验时才调用）
std::vector<uint8_t> RH5DG2_485_Protocol::extractFromRingBuffer(const RingBuffer& ringBuffer, size_t startOffset, size_t length) const {
    std::vector<uint8_t> result(length);
    const std::vector<uint8_t>& buf = ringBuffer.getBuffer();
    size_t tailIndex = ringBuffer.getTail();
    size_t bufferSize = buf.size();

    for (size_t i = 0; i < length; ++i) {
        result[i] = buf[(tailIndex + startOffset + i) % bufferSize];
    }
    return result;
}

std::string RH5DG2_485_Protocol::formatBytesToHex(const uint8_t* data, size_t length) const {
    std::ostringstream oss;
    oss << std::hex << std::uppercase;
    for (size_t i = 0; i < length; ++i) {
        if (i > 0) oss << " ";
        oss << "0x" << std::setfill('0') << std::setw(2) << static_cast<int>(data[i]);
    }
    return oss.str();
}

// 循环读取响应数据，直到接收到足够字节或超时
std::vector<uint8_t> RH5DG2_485_Protocol::readResponseWithLoop(Device device, int timeout_ms, size_t min_bytes, bool is_read_response) const {
    auto logger = getLogger();
    std::vector<uint8_t> response;
    auto start_time = std::chrono::steady_clock::now();
    auto timeout = std::chrono::milliseconds(timeout_ms);
    
    // 循环读取，直到接收到足够字节或超时
    while (true) {
        // 检查总超时
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);
        if (elapsed_ms >= timeout) {
            logger->debug("读取响应超时: 已等待 {} ms, 当前数据长度: {}", timeout_ms, response.size());
            break;
        }
        
        // 使用剩余超时时间读取数据（每次最多等待剩余时间，但不超过5ms）
        auto remaining_ms = timeout - elapsed_ms;
        auto max_wait = std::chrono::milliseconds(5);
        auto read_timeout = (remaining_ms < max_wait) ? remaining_ms : max_wait;
        auto new_data = device->read(read_timeout);
        
        if (!new_data.empty()) {
            response.insert(response.end(), new_data.begin(), new_data.end());
            logger->debug("循环读取: 本次读取 {} 字节, 累计 {} 字节", new_data.size(), response.size());
        }
        
        // 在缓冲区中搜索帧头 0x90 0xEB（回复帧帧头）
        size_t header_pos = SIZE_MAX;
        for (size_t i = 0; i + 1 < response.size(); ++i) {
            if (response[i] == 0x90 && response[i + 1] == 0xEB) {
                header_pos = i;
                break;
            }
        }
        
        // 如果找到帧头，从帧头位置开始验证和计算长度
        if (header_pos != SIZE_MAX) {
            size_t frame_start = header_pos;
            
            // 检查是否有足够的数据来判断长度（至少需要8字节：帧头2 + ID1 + 长度1 + 命令1 + 地址2 + 至少1字节数据）
            if (response.size() >= frame_start + 8) {
                uint8_t hands_id = response[frame_start + 2];
                uint8_t data_length = response[frame_start + 3];
                uint8_t command = response[frame_start + 4];
                
                // 验证命令类型
                bool valid_command = (command == 0x11 || command == 0x12);
                
                // 验证Hands_ID（如果已设置device_id_）
                bool valid_id = (device_id_ == 0 || hands_id == device_id_);
                
                if (valid_command && valid_id) {
                    // 计算期望的总长度
                    size_t expected_len = 0;
                    if (command == 0x11) {
                        // 读回复：byte[3] = Register_Length + 3
                        if (data_length >= 3) {
                            size_t register_length = data_length - 3;
                            expected_len = frame_start + 8 + register_length; // 从帧头开始的总长度
                        } else {
                            // 数据长度无效，继续读取
                            logger->debug("数据长度无效 ({}), 继续读取...", data_length);
                            if (response.empty()) {
                                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                            }
                            continue;
                        }
                    } else {
                        // 写回复：固定9字节
                        expected_len = frame_start + 9;
                    }
                    
                    // 检查是否已接收到完整帧
                    if (response.size() >= expected_len) {
                        // 提取完整帧（从帧头开始到帧结束）
                        std::vector<uint8_t> complete_frame(response.begin() + frame_start, 
                                                           response.begin() + expected_len);
                        logger->debug("{}回复接收完成: 期望 {} 字节(从位置{}开始), 实际 {} 字节", 
                                     (command == 0x11 ? "读" : "写"), 
                                     expected_len - frame_start, frame_start, response.size());
                        return complete_frame;
                    } else {
                        // 数据还不够，继续读取
                        logger->debug("帧头已找到(位置{}), 需要 {} 字节, 当前 {} 字节, 继续读取...", 
                                     frame_start, expected_len, response.size());
                    }
                } else {
                    // 命令类型或ID不匹配，可能是错误的帧头，继续搜索
                    if (response.size() > frame_start + 1) {
                        // 从帧头后继续搜索
                        logger->debug("帧头位置{}的命令/ID验证失败 (cmd=0x{:02X}, id={}), 继续搜索...", 
                                     frame_start, command, hands_id);
                    }
                }
            } else {
                // 找到帧头但数据还不够，继续读取
                logger->debug("帧头已找到(位置{}), 但数据不足8字节, 继续读取...", frame_start);
            }
        } else {
            // 未找到帧头，继续读取
            if (response.size() >= 2) {
                logger->debug("未找到帧头, 当前数据: 0x{:02X} 0x{:02X}..., 继续读取...", 
                             response[0], response.size() > 1 ? response[1] : 0);
            }
        }
        
        // 如果已经接收到一些数据但还不够，短暂休眠后继续
        if (response.empty()) {
            // 没有数据时，稍微等待一下
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    
    // 超时退出，如果找到了帧头但数据不完整，返回从帧头开始的部分数据
    if (response.size() >= 2) {
        size_t header_pos = SIZE_MAX;
        for (size_t i = 0; i + 1 < response.size(); ++i) {
            if (response[i] == 0x90 && response[i + 1] == 0xEB) {
                header_pos = i;
                break;
            }
        }
        if (header_pos != SIZE_MAX) {
            // 返回从帧头开始的数据
            std::vector<uint8_t> partial_frame(response.begin() + header_pos, response.end());
            logger->debug("超时退出，返回从帧头位置{}开始的部分数据: {} 字节", header_pos, partial_frame.size());
            return partial_frame;
        }
    }
    
    return response;
}

// 常规解析函数（按协议规范动态计算帧长度，支持读/写回复）
// 读回复帧格式：byte[0]=0x90, byte[1]=0xEB, byte[2]=Hands_ID, byte[3]=Register_Length+3,
//              byte[4]=0x11, byte[5-6]=Address, byte[7]~byte[7+Register_Length-1]=Data,
//              byte[7+Register_Length]=Checksum
// 写回复帧格式：byte[0]=0x90, byte[1]=0xEB, byte[2]=Hands_ID, byte[3]=4,
//              byte[4]=0x12, byte[5-6]=Address, byte[7]=1, byte[8]=Checksum
std::pair<bool, std::vector<int>> RH5DG2_485_Protocol::parseResponse(RingBuffer& ringBuffer) {
    auto logger = getLogger();

    try {
        size_t bufSize = ringBuffer.size();
        if (bufSize < 8) {
            return {false, {}};
        }

        // 搜索回复帧头 0x90 0xEB（回复帧与命令帧帧头顺序相反）
        size_t startIdx = 0;
        bool found = false;
        for (size_t i = 0; i + 1 < bufSize; ++i) {
            if (readByteAtOffset(ringBuffer, i) == 0x90 &&
                readByteAtOffset(ringBuffer, i + 1) == 0xEB) {
                startIdx = i;
                found = true;
                break;
            }
        }
        if (!found) {
            return {false, {}};
        }

        if (bufSize < startIdx + 8) {
            return {false, {}};
        }

        // 读取回复帧关键字段
        uint8_t hands_id = readByteAtOffset(ringBuffer, startIdx + 2);      // byte[2]: Hands_ID
        uint8_t data_length = readByteAtOffset(ringBuffer, startIdx + 3);    // byte[3]: 数据部分长度
        uint8_t command = readByteAtOffset(ringBuffer, startIdx + 4);        // byte[4]: 命令类型

        // 验证命令类型：0x11=读寄存器回复，0x12=写寄存器回复
        if (command != 0x11 && command != 0x12) {
            logger->debug("无效的命令类型: 0x{:02X}，跳过帧头", command);
            ringBuffer.advance(startIdx + 1);
            return {false, {}};
        }

        // 验证Hands_ID是否匹配
        if (device_id_ != 0 && hands_id != device_id_) {
            logger->debug("Hands_ID不匹配: 期望 {}, 实际 {}", device_id_, hands_id);
            ringBuffer.advance(startIdx + 1);
            return {false, {}};
        }

        // 根据协议规范计算帧总长度
        size_t response_len = 0;
        if (command == 0x11) {
            // 读回复：byte[3] = Register_Length + 3
            // 总长度 = 8 + Register_Length = 8 + (data_length - 3) = 5 + data_length
            if (data_length < 3) {
                logger->error("无效的数据长度: {} (应 >= 3)", data_length);
                ringBuffer.advance(startIdx + 1);
                return {false, {}};
            }
            size_t register_length = data_length - 3;
            response_len = 8 + register_length; // 帧头(2) + ID(1) + 长度(1) + 命令(1) + 地址(2) + 数据(register_length) + 校验和(1)
        } else {
            // 写回复：固定9字节（byte[3] = 4，总长度 = 9）
            response_len = 9;
        }

        if (bufSize < startIdx + response_len) {
            return {false, {}};
        }

        std::vector<uint8_t> response = extractFromRingBuffer(ringBuffer, startIdx, response_len);

        // 校验和验证
        if (!validateChecksum(response)) {
            logger->debug("校验和验证失败，跳过当前帧头");
            ringBuffer.advance(startIdx + 1);
            return {false, {}};
        }

        // 写回复：不需要校验
        if (command == 0x12) {
            // 验证byte[7]是否为1（写寄存器回复的固定值）
            //if (response.size() >= 8 && response[7] != 1) {
            //    logger->warn("写寄存器回复格式异常: byte[7] = {} (期望 1)", response[7]);
            //}
            ringBuffer.advance(startIdx + response_len);
            return {true, {}};
        }

        // 读回复：解析数据区（小端序：低字节在前，高字节在后）
        // byte[7] ~ byte[7+Register_Length-1]: Data[0] ~ Data[Register_Length-1]
        std::vector<int> parsed_values;
        size_t register_length = data_length - 3;
        parsed_values.reserve(register_length / 2); // 每个值2字节

        const size_t data_start = 7; // 数据从byte[7]开始
        const size_t data_end = data_start + register_length;
        for (size_t j = data_start; j + 1 < data_end && j + 1 < response.size(); j += 2) {
            // 小端序解析：response[j]是低字节，response[j+1]是高字节
            int value = response[j] | (response[j + 1] << 8);
            // 处理有符号数（如果值 > 32767，则为负数）
            if (value > 32767) {
                value -= 65536;
            }
            parsed_values.push_back(value);
        }

        ringBuffer.advance(startIdx + response_len);
        logger->debug("成功解析读寄存器回复: {} 个值", parsed_values.size());
        return {true, std::move(parsed_values)};
    } catch (const std::exception& e) {
        logger->error("解析异常: {}", e.what());
        return {false, {}};
    }
}

bool RH5DG2_485_Protocol::validateChecksum(const std::vector<uint8_t>& response) const {
    // 根据协议规范：校验和是从byte[2]开始到最后一个数据字节的累加和的低字节
    // 校验和应该等于响应的最后一个字节
    if (response.size() < 9) {
        return false;
    }
    
    // 验证回复帧头
    if (response[0] != 0x90 || response[1] != 0xEB) {
        return false;
    }
    
    // 计算校验和：从byte[2]（Hands_ID）开始到倒数第二个字节（不包括校验和本身）
    uint8_t checksum = 0;
    for (size_t i = 2; i < response.size() - 1; ++i) {
        checksum += response[i];
    }
    
    // 校验和应该等于响应的最后一个字节
    bool valid = (checksum == response.back());
    if (!valid) {
        auto logger = getLogger();
        logger->debug("校验和验证失败: 计算值 = 0x{:02X}, 期望值 = 0x{:02X}", checksum, response.back());
    }
    return valid;
}

// 触觉数据解析
std::pair<bool, TouchDataResult> RH5DG2_485_Protocol::parseTouchData(RingBuffer& ringBuffer, int version) {
    auto logger = getLogger();

    try {
        size_t bufSize = ringBuffer.size();
        if (bufSize < 8) {
            return {false, {}};
        }

        if (version != 1) {
            if (version == 2) {
                logger->warn("触觉数据版本2暂未实现");
            } else {
                logger->error("未知的触觉数据版本: {}", version);
            }
            return {false, {}};
        }

        size_t startIdx = 0;
        bool found = false;
        for (size_t i = 0; i + 1 < bufSize; ++i) {
            if (readByteAtOffset(ringBuffer, i) == 0x90 &&
                readByteAtOffset(ringBuffer, i + 1) == 0xEB) {
                startIdx = i;
                found = true;
                break;
            }
        }
        if (!found) {
            return {false, {}};
        }

        if (bufSize < startIdx + 8) {
            return {false, {}};
        }

        uint8_t hands_id = readByteAtOffset(ringBuffer, startIdx + 2);
        uint8_t data_length = readByteAtOffset(ringBuffer, startIdx + 3);
        uint8_t command = readByteAtOffset(ringBuffer, startIdx + 4);

        if (command != 0x11) {
            logger->debug("触觉数据无效的命令类型: 0x{:02X}，跳过帧头", command);
            ringBuffer.advance(startIdx + 1);
            return {false, {}};
        }

        if (device_id_ != 0 && hands_id != device_id_) {
            logger->debug("触觉数据Hands_ID不匹配: 期望 {}, 实际 {}", device_id_, hands_id);
            ringBuffer.advance(startIdx + 1);
            return {false, {}};
        }

        if (data_length < 3) {
            logger->error("触觉数据无效的数据长度: {} (应 >= 3)", data_length);
            ringBuffer.advance(startIdx + 1);
            return {false, {}};
        }

        size_t register_length = data_length - 3;
        size_t response_len = 8 + register_length;

        if (bufSize < startIdx + response_len) {
            return {false, {}};
        }

        std::vector<uint8_t> response = extractFromRingBuffer(ringBuffer, startIdx, response_len);
        if (!validateChecksum(response)) {
            logger->debug("触觉数据校验和验证失败，跳过当前帧头");
            ringBuffer.advance(startIdx + 1);
            return {false, {}};
        }

        TouchDataResult result;
        const size_t data_start = 7;
        const std::string fingers[] = {"little", "ring", "middle", "index", "thumb"};

        size_t available_data_length = register_length;
        size_t finger_data_length = 5 * 10;
        size_t palm_start_idx = data_start + finger_data_length;
        size_t palm_data_length = 9 * 2;

        // 手指数据
        for (int i = 0; i < 5; ++i) {
            size_t base_idx = data_start + i * 10;
            if (base_idx + 10 > data_start + available_data_length || base_idx + 10 > response.size()) {
                logger->warn("触觉数据不足，无法解析第{}个手指数据", i + 1);
                break;
            }

            std::vector<uint16_t> finger_vals;
            finger_vals.reserve(4);
            for (int j = 0; j < 3; ++j) {
                uint8_t low_byte = response[base_idx + j * 2];
                uint8_t high_byte = response[base_idx + j * 2 + 1];
                uint16_t val = low_byte | (high_byte << 8);
                finger_vals.push_back(val);
            }

            if (base_idx + 9 <= response.size()) {
                uint8_t b0 = response[base_idx + 6];
                uint8_t b1 = response[base_idx + 7];
                uint8_t b2 = response[base_idx + 8];
                uint32_t combined = b0 | (b1 << 8) | (b2 << 16);
                finger_vals.push_back(combined);
            } else {
                logger->warn("触觉数据不足，无法解析第{}个手指的24位值", i + 1);
            }

            result.fingerResults[fingers[i]] = std::move(finger_vals);
        }

        // 掌心数据
        if (palm_start_idx + palm_data_length <= data_start + available_data_length &&
            palm_start_idx + palm_data_length <= response.size()) {
            for (int j = 0; j < 9; ++j) {
                size_t idx = palm_start_idx + j * 2;
                if (idx + 1 < response.size()) {
                    uint16_t val = response[idx] | (response[idx + 1] << 8);
                    result.palmResults["palm_data_" + std::to_string(j + 1)] = val;
                } else {
                    logger->warn("触觉数据不足，无法解析第{}个掌心数据", j + 1);
                    break;
                }
            }
        } else {
            logger->warn("触觉数据不足，无法解析掌心数据 (需要 {} 字节，实际可用 {} 字节)",
                         palm_data_length,
                         (data_start + available_data_length > palm_start_idx) ?
                             (data_start + available_data_length - palm_start_idx) : 0);
        }

        ringBuffer.advance(startIdx + response_len);
        return {true, std::move(result)};
    } catch (const std::exception& e) {
        logger->error("触觉解析异常: {}", e.what());
        return {false, {}};
    }
}

IoError RH5DG2_485_Protocol::writeRegister(Device device, const std::string& reg_name, const std::vector<int>& values) {
    std::ostringstream oss;
    auto logger = getLogger();

    auto it = REGISTER_MAP.find(reg_name);
    if (it == REGISTER_MAP.end()) {
        logger->error("未知寄存器名: {}", reg_name);
        return IoError::UnknownRegister;
    }
    int address = it->second;

    std::vector<uint8_t> cmd;
    try {
        cmd = buildWriteCommand(address, values);
    } catch (const std::exception& e) {
        logger->error("写入寄存器参数非法：{}, 异常: {}", reg_name, e.what());
        return IoError::InvalidArgument;
    }

    try {
        logger->debug("[写入命令] 寄存器: {}, 地址: 0x{:04X}, 命令: {}", reg_name, address, formatBytesToHex(cmd.data(), cmd.size()));

        device->write(cmd);

        // 循环读取写回复（固定9字节），超时25ms
        auto writeResponse = readResponseWithLoop(device, 25, 9, false);
        if (writeResponse.empty()) {
            logger->debug("[写响应] 寄存器: {}, 响应为空", reg_name);
            logger->error("写入寄存器失败：{} (无响应)", reg_name);
            return IoError::Timeout;
        }
        logger->debug("[写响应] 寄存器: {}, 响应: {}", reg_name, formatBytesToHex(writeResponse.data(), writeResponse.size()));

        {
            RingBuffer tempBuffer(128);
            tempBuffer.push(writeResponse.data(), writeResponse.size());
            auto [parseSuccess, _] = parseResponse(tempBuffer);
            if (!parseSuccess) {
                logger->error("写入寄存器响应解析失败：{}", reg_name);
                return IoError::BadResponse;
            }
            oss << "写入" << reg_name << ":(";
            for (size_t i = 0; i < values.size(); ++i) {
                oss << values[i];
                if (i != values.size() - 1) oss << " ";
            }
            oss << ")";
            logger->info(oss.str());
        }

        // 特殊处理：写入 defaultSpeedSet 或 defaultForceSet 后，间隔5ms写入 save=1
        if (reg_name == "defaultSpeedSet" || reg_name == "defaultForceSet") {
            logger->debug("[特殊处理] 检测到 {} 写入完成，准备写入 save 寄存器", reg_name);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));

            auto saveCmd = buildWriteCommand(REGISTER_MAP.at("save"), {1});
            logger->debug("[写入命令] 寄存器: save, 地址: 0x{:04X}, 命令: {}",
                         REGISTER_MAP.at("save"), formatBytesToHex(saveCmd.data(), saveCmd.size()));
            device->write(saveCmd);
            auto saveResponse = readResponseWithLoop(device, 25, 9, false);

            if (!saveResponse.empty()) {
                RingBuffer tempBuffer(128);
                tempBuffer.push(saveResponse.data(), saveResponse.size());
                auto [parseSuccess, _] = parseResponse(tempBuffer);
                if (parseSuccess) {
                    logger->debug("[写响应] 寄存器: save, 响应: {}", formatBytesToHex(saveResponse.data(), saveResponse.size()));
                    logger->info("写入save:(1)");
                } else {
                    logger->warn("[写响应] 寄存器: save, 响应格式验证失败");
                    return IoError::BadResponse;
                }
            } else {
                logger->warn("[写响应] 寄存器: save, 响应为空");
                return IoError::Timeout;
            }
        }
        // 特殊处理：写入 actionSeqIndex 后，间隔5ms写入 actionSeqRun=1
        else if (reg_name == "actionSeqIndex") {
            logger->debug("[特殊处理] 检测到 actionSeqIndex 写入完成，准备写入 actionSeqRun 寄存器");
            std::this_thread::sleep_for(std::chrono::milliseconds(5));

            auto actionSeqRunCmd = buildWriteCommand(REGISTER_MAP.at("actionSeqRun"), {1});
            logger->debug("[写入命令] 寄存器: actionSeqRun, 地址: 0x{:04X}, 命令: {}",
                         REGISTER_MAP.at("actionSeqRun"), formatBytesToHex(actionSeqRunCmd.data(), actionSeqRunCmd.size()));
            device->write(actionSeqRunCmd);
            auto actionSeqRunResponse = readResponseWithLoop(device, 25, 9, false);

            if (!actionSeqRunResponse.empty()) {
                RingBuffer tempBuffer(128);
                tempBuffer.push(actionSeqRunResponse.data(), actionSeqRunResponse.size());
                auto [parseSuccess, _] = parseResponse(tempBuffer);
                if (parseSuccess) {
                    logger->debug("[写响应] 寄存器: actionSeqRun, 响应: {}",
                                 formatBytesToHex(actionSeqRunResponse.data(), actionSeqRunResponse.size()));
                    logger->info("写入actionSeqRun:(1)");
                } else {
                    logger->warn("[写响应] 寄存器: actionSeqRun, 响应格式验证失败");
                    return IoError::BadResponse;
                }
            } else {
                logger->warn("[写响应] 寄存器: actionSeqRun, 响应为空");
                return IoError::Timeout;
            }
        }
        // 特殊处理：写入 actionLibraryIndex 后，间隔5ms写入 actionLibraryRun=1
        else if (reg_name == "actionLibraryIndex") {
            logger->debug("[特殊处理] 检测到 actionLibraryIndex 写入完成，准备写入 actionLibraryRun 寄存器");
            std::this_thread::sleep_for(std::chrono::milliseconds(5));

            auto actionLibraryRunCmd = buildWriteCommand(REGISTER_MAP.at("actionLibraryRun"), {1});
            logger->debug("[写入命令] 寄存器: actionLibraryRun, 地址: 0x{:04X}, 命令: {}",
                         REGISTER_MAP.at("actionLibraryRun"), formatBytesToHex(actionLibraryRunCmd.data(), actionLibraryRunCmd.size()));
            device->write(actionLibraryRunCmd);
            auto actionLibraryRunResponse = readResponseWithLoop(device, 25, 9, false);

            if (!actionLibraryRunResponse.empty()) {
                RingBuffer tempBuffer(128);
                tempBuffer.push(actionLibraryRunResponse.data(), actionLibraryRunResponse.size());
                auto [parseSuccess, _] = parseResponse(tempBuffer);
                if (parseSuccess) {
                    logger->debug("[写响应] 寄存器: actionLibraryRun, 响应: {}",
                                 formatBytesToHex(actionLibraryRunResponse.data(), actionLibraryRunResponse.size()));
                    logger->info("写入actionLibraryRun:(1)");
                } else {
                    logger->warn("[写响应] 寄存器: actionLibraryRun, 响应格式验证失败");
                    return IoError::BadResponse;
                }
            } else {
                logger->warn("[写响应] 寄存器: actionLibraryRun, 响应为空");
                return IoError::Timeout;
            }
        }

        return IoError::Ok;
    } catch (const std::exception& e) {
        logger->error("写入寄存器失败：{}, 异常: {}", reg_name, e.what());
        return IoError::DeviceError;
    } catch (...) {
        logger->error("写入寄存器失败：{}", reg_name);
        return IoError::DeviceError;
    }
}

RegisterReadResult RH5DG2_485_Protocol::readRegister(Device device, RingBuffer& ringBuffer, const std::string& reg_name, size_t length) {
    std::ostringstream oss;
    auto logger = getLogger();

    ringBuffer.clear();

    auto it = REGISTER_MAP.find(reg_name);
    if (it == REGISTER_MAP.end()) {
        logger->error("未知寄存器名: {}", reg_name);
        return {IoError::UnknownRegister, {}};
    }
    int address = it->second;

    // 如果length为0，则根据寄存器名称自动确定默认读取长度
    if (length == 0) {
        length = getDefaultReadLength(reg_name);
        logger->debug("[动态长度] 寄存器: {}, 自动确定读取长度: {} 字节", reg_name, length);
    }

    try {
        auto cmd = buildReadCommand(address, length);
        logger->debug("[读取命令] 寄存器: {}, 地址: 0x{:04X}, 长度: {}, 命令: {}", reg_name, address, length, formatBytesToHex(cmd.data(), cmd.size()));

        device->write(cmd);
        // 循环读取读回复（动态长度），超时20ms
        auto response = readResponseWithLoop(device, 20, 8, true);

        if (response.empty()) {
            logger->debug("[原始响应] 寄存器: {}, 响应为空", reg_name);
            logger->error("读取寄存器失败：{} (无响应)", reg_name);
            return {IoError::Timeout, {}};
        }
        logger->debug("[原始响应] 寄存器: {}, 响应: {}", reg_name, formatBytesToHex(response.data(), response.size()));

        ringBuffer.push(response.data(), response.size());
        auto result = parseResponse(ringBuffer);
        if (result.first) {
            oss << "读取" << reg_name << ":(";
            for (size_t i = 0; i < result.second.size(); ++i) {
                oss << result.second[i];
                if (i != result.second.size() - 1) oss << " ";
            }
            oss << ")";
            logger->info(oss.str());
            return {IoError::Ok, std::move(result.second)};
        }

        logger->error("读取寄存器失败：{}", reg_name);
        return {IoError::BadResponse, {}};
    } catch (...) {
        logger->error("读取寄存器异常：{}", reg_name);
        return {IoError::DeviceError, {}};
    }
}

TouchReadResult RH5DG2_485_Protocol::readTouchData(Device device, RingBuffer& ringBuffer, int version) {
    std::ostringstream oss;
    auto logger = getLogger();

    try {
        ringBuffer.clear();

        int touchAddress = getRegisterAddress("touchAct");
        auto readTouchCmd = buildReadCommand(touchAddress, 68);
        logger->debug("[读取命令-触觉] 地址: 0x{:04X}, 长度: 68, 命令: {}", touchAddress, formatBytesToHex(readTouchCmd.data(), readTouchCmd.size()));

        device->write(readTouchCmd);
        // 循环读取触觉数据回复（动态长度），超时25ms
        auto resp = readResponseWithLoop(device, 25, 8, true);

        if (!resp.empty()) {
            logger->debug("[原始响应-触觉] 响应: {}", formatBytesToHex(resp.data(), resp.size()));
        } else {
            logger->debug("[原始响应-触觉] 响应为空");
        }

        ringBuffer.push(resp.data(), resp.size());
        auto result = parseTouchData(ringBuffer, version);

        if (result.first) {
            const TouchDataResult& touchData = result.second;
            oss << "读取touchAct:(";
            for (const auto& finger_pair : touchData.fingerResults) {
                oss << finger_pair.first << ":";
                for (size_t i = 0; i < finger_pair.second.size(); ++i) {
                    oss << finger_pair.second[i];
                    if (i != finger_pair.second.size() - 1) oss << " ";
                }
                oss << " ";
            }
            for (const auto& palm_pair : touchData.palmResults) {
                oss << palm_pair.first << ":" << palm_pair.second << " ";
            }
            oss << ")";
            logger->info(oss.str());
        } else {
            logger->error("读取触觉寄存器失败");
            return {IoError::BadResponse, {}};
        }

        return {IoError::Ok, std::move(result.second)};
    } catch (...) {
        logger->error("读取触觉寄存器异常");
        return {IoError::DeviceError, {}};
    }
}

// 自动注册RH5DG2_485协议
REGISTER_PROTOCOL("RH5DG2_485", RH5DG2_485_Protocol);

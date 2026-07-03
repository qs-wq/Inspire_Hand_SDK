#include "EG5CD1_485_protocol.hpp"
#include "logger_manager.hpp"
#include "protocol_factory.hpp"

#include <algorithm>

namespace {

constexpr uint8_t kHdr0 = 0xEB;
constexpr uint8_t kHdr1 = 0x90;
constexpr uint8_t kRespHdr0 = 0xEE;
constexpr uint8_t kRespHdr1 = 0x16;
constexpr uint8_t kCmdRead = 0x00;
constexpr uint8_t kCmdWrite = 0x01;

} // namespace

const std::map<std::string, int> EG5CD1_485_Protocol::REGISTER_MAP = {
    {"save", 1002},
    {"defaultPar", 1004},
    {"id", 1006},
    {"reduRatio", 1008},
    {"catchModeSet", 1010},
    {"stop", 1012},
    {"clearError", 1014},
    {"openLenSet", 1020},
    {"speedSet", 1022},
    {"forceSet", 1024},
    {"forceAct", 1120},
    {"openLenAct", 1122},
    {"currentAct", 1124},
    {"temp", 1126},
    {"errorCode", 1128},
    {"status", 1130},
    {"speedAct", 1132},
    {"catchModeClose", 1136},
    {"catchModeOpen", 1138},
    {"gripperStatusBlock", 1120},
};

const std::map<std::string, size_t> EG5CD1_485_Protocol::REGISTER_READ_LENGTH_MAP = {
    {"gripperStatusBlock", 14},
    {"save", 2},
    {"defaultPar", 2},
    {"id", 2},
    {"reduRatio", 2},
    {"catchModeSet", 2},
    {"stop", 2},
    {"clearError", 2},
    {"openLenSet", 2},
    {"speedSet", 2},
    {"forceSet", 2},
    {"forceAct", 2},
    {"openLenAct", 2},
    {"currentAct", 2},
    {"temp", 2},
    {"errorCode", 2},
    {"status", 2},
    {"speedAct", 2},
    {"catchModeClose", 2},
    {"catchModeOpen", 2},
};

size_t EG5CD1_485_Protocol::getDefaultReadLength(const std::string& reg_name) const {
    auto it = REGISTER_READ_LENGTH_MAP.find(reg_name);
    if (it != REGISTER_READ_LENGTH_MAP.end()) {
        return it->second;
    }
    return 2;
}

int EG5CD1_485_Protocol::getRegisterAddress(const std::string& register_name) const {
    auto it = REGISTER_MAP.find(register_name);
    return (it != REGISTER_MAP.end()) ? it->second : -1;
}

std::vector<uint8_t> EG5CD1_485_Protocol::buildReadCommand(int address, size_t length) {
    std::vector<uint8_t> cmd = {
        kHdr0,
        kHdr1,
        device_id_,
        0x04,
        kCmdRead,
        static_cast<uint8_t>(address & 0xFF),
        static_cast<uint8_t>((address >> 8) & 0xFF),
        static_cast<uint8_t>(length & 0xFF),
    };
    uint8_t checksum = 0;
    for (size_t i = 2; i < cmd.size(); ++i) {
        checksum = static_cast<uint8_t>(checksum + cmd[i]);
    }
    cmd.push_back(checksum);
    return cmd;
}

std::vector<uint8_t> EG5CD1_485_Protocol::buildWriteCommand(int address, const std::vector<int>& values) {
    if (values.empty()) {
        throw std::runtime_error("EG5CD1 写寄存器至少需要 1 个值");
    }
    if (values.size() > 2) {
        throw std::runtime_error("EG5CD1 写寄存器最多 2 个值");
    }

    const uint8_t payload_len = static_cast<uint8_t>(3 + values.size() * 2);
    std::vector<uint8_t> cmd = {
        kHdr0,
        kHdr1,
        device_id_,
        payload_len,
        kCmdWrite,
        static_cast<uint8_t>(address & 0xFF),
        static_cast<uint8_t>((address >> 8) & 0xFF),
    };
    for (int v : values) {
        cmd.push_back(static_cast<uint8_t>(v & 0xFF));
        cmd.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
    }
    uint8_t checksum = 0;
    for (size_t i = 2; i < cmd.size(); ++i) {
        checksum = static_cast<uint8_t>(checksum + cmd[i]);
    }
    cmd.push_back(checksum);
    return cmd;
}

uint8_t EG5CD1_485_Protocol::readByteAtOffset(const RingBuffer& ringBuffer, size_t offset) const {
    size_t bufSize = ringBuffer.size();
    if (offset >= bufSize) {
        return 0xFF;
    }
    const std::vector<uint8_t>& buf = ringBuffer.getBuffer();
    size_t tailIndex = ringBuffer.getTail();
    size_t bufferSize = buf.size();
    return buf[(tailIndex + offset) % bufferSize];
}

std::vector<uint8_t> EG5CD1_485_Protocol::extractFromRingBuffer(const RingBuffer& ringBuffer, size_t startOffset,
                                                                size_t length) const {
    std::vector<uint8_t> result(length);
    const std::vector<uint8_t>& buf = ringBuffer.getBuffer();
    size_t tailIndex = ringBuffer.getTail();
    size_t bufferSize = buf.size();
    for (size_t i = 0; i < length; ++i) {
        result[i] = buf[(tailIndex + startOffset + i) % bufferSize];
    }
    return result;
}

std::string EG5CD1_485_Protocol::formatBytesToHex(const uint8_t* data, size_t length) const {
    std::ostringstream oss;
    oss << std::hex << std::uppercase;
    for (size_t i = 0; i < length; ++i) {
        if (i > 0) {
            oss << " ";
        }
        oss << "0x" << std::setfill('0') << std::setw(2) << static_cast<int>(data[i]);
    }
    return oss.str();
}

std::vector<uint8_t> EG5CD1_485_Protocol::readResponseWithLoop(Device device, int timeout_ms, size_t /*min_bytes*/,
                                                               bool /*is_read_response*/) const {
    auto logger = getLogger();
    std::vector<uint8_t> response;
    const auto start_time = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::milliseconds(timeout_ms);

    while (true) {
        const auto elapsed = std::chrono::steady_clock::now() - start_time;
        const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);
        if (elapsed_ms >= timeout) {
            logger->debug("EG5CD1 读取响应超时: {} ms, 当前 {} 字节", timeout_ms, response.size());
            if (response.size() >= 2) {
                for (size_t i = 0; i + 1 < response.size(); ++i) {
                    if (response[i] == kRespHdr0 && response[i + 1] == kRespHdr1) {
                        return std::vector<uint8_t>(response.begin() + static_cast<std::ptrdiff_t>(i), response.end());
                    }
                }
            }
            return response;
        }

        const auto remaining_ms = timeout - elapsed_ms;
        const auto max_wait = std::chrono::milliseconds(5);
        const auto read_timeout = (remaining_ms < max_wait) ? remaining_ms : max_wait;
        const auto new_data = device->read(read_timeout);
        if (!new_data.empty()) {
            response.insert(response.end(), new_data.begin(), new_data.end());
        }

        for (size_t frame_start = 0; frame_start + 5 < response.size(); ++frame_start) {
            if (response[frame_start] != kRespHdr0 || response[frame_start + 1] != kRespHdr1) {
                continue;
            }
            const uint8_t hands_id = response[frame_start + 2];
            const uint8_t data_length = response[frame_start + 3];
            const uint8_t command = response[frame_start + 4];

            if (device_id_ != 0 && hands_id != device_id_) {
                continue;
            }
            if (command != kCmdRead && command != kCmdWrite) {
                continue;
            }

            size_t expected_len = 0;
            if (command == kCmdRead) {
                if (data_length < 3) {
                    continue;
                }
                expected_len = frame_start + 4 + static_cast<size_t>(data_length) + 1;
            } else {
                expected_len = frame_start + 9;
            }

            if (response.size() >= expected_len) {
                return std::vector<uint8_t>(response.begin() + static_cast<std::ptrdiff_t>(frame_start),
                                            response.begin() + static_cast<std::ptrdiff_t>(expected_len));
            }
        }

        if (response.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

std::pair<bool, std::vector<int>> EG5CD1_485_Protocol::parseResponse(RingBuffer& ringBuffer) {
    auto logger = getLogger();
    try {
        const size_t bufSize = ringBuffer.size();
        if (bufSize < 8) {
            return {false, {}};
        }

        size_t startIdx = 0;
        bool found = false;
        for (size_t i = 0; i + 1 < bufSize; ++i) {
            if (readByteAtOffset(ringBuffer, i) == kRespHdr0 && readByteAtOffset(ringBuffer, i + 1) == kRespHdr1) {
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

        const uint8_t hands_id = readByteAtOffset(ringBuffer, startIdx + 2);
        const uint8_t data_length = readByteAtOffset(ringBuffer, startIdx + 3);
        const uint8_t command = readByteAtOffset(ringBuffer, startIdx + 4);

        if (command != kCmdRead && command != kCmdWrite) {
            ringBuffer.advance(startIdx + 1);
            return {false, {}};
        }
        if (device_id_ != 0 && hands_id != device_id_) {
            logger->debug("EG5CD1 应答 ID 不匹配: 期望 {}, 实际 {}", device_id_, hands_id);
            ringBuffer.advance(startIdx + 1);
            return {false, {}};
        }

        size_t response_len = 0;
        if (command == kCmdRead) {
            if (data_length < 3) {
                ringBuffer.advance(startIdx + 1);
                return {false, {}};
            }
            response_len = 8 + (data_length - 3);
        } else {
            response_len = 9;
        }

        if (bufSize < startIdx + response_len) {
            return {false, {}};
        }

        std::vector<uint8_t> response = extractFromRingBuffer(ringBuffer, startIdx, response_len);
        if (!validateChecksum(response)) {
            logger->debug("EG5CD1 校验和失败，跳过帧头");
            ringBuffer.advance(startIdx + 1);
            return {false, {}};
        }

        if (command == kCmdWrite) {
            ringBuffer.advance(startIdx + response_len);
            return {true, {}};
        }

        std::vector<int> parsed_values;
        const size_t register_length = data_length - 3;
        parsed_values.reserve(register_length / 2);
        const size_t data_start = 7;
        const size_t data_end = data_start + register_length;
        for (size_t j = data_start; j + 1 < data_end && j + 1 < response.size(); j += 2) {
            int value = response[j] | (response[j + 1] << 8);
            if (value > 32767) {
                value -= 65536;
            }
            parsed_values.push_back(value);
        }
        ringBuffer.advance(startIdx + response_len);
        return {true, std::move(parsed_values)};
    } catch (const std::exception& e) {
        getLogger()->error("EG5CD1 parseResponse 异常: {}", e.what());
        return {false, {}};
    }
}

bool EG5CD1_485_Protocol::validateChecksum(const std::vector<uint8_t>& response) const {
    if (response.size() < 9) {
        return false;
    }
    if (response[0] != kRespHdr0 || response[1] != kRespHdr1) {
        return false;
    }
    uint8_t checksum = 0;
    for (size_t i = 2; i < response.size() - 1; ++i) {
        checksum = static_cast<uint8_t>(checksum + response[i]);
    }
    return checksum == response.back();
}

std::pair<bool, TouchDataResult> EG5CD1_485_Protocol::parseTouchData(RingBuffer&, int) { return {false, {}}; }

IoError EG5CD1_485_Protocol::writeRegister(Device device, const std::string& reg_name, const std::vector<int>& values) {
    auto logger = getLogger();
    if (reg_name == "gripperStatusBlock") {
        logger->error("EG5CD1 寄存器 gripperStatusBlock 只读，不可写");
        return IoError::NotSupported;
    }

    auto it = REGISTER_MAP.find(reg_name);
    if (it == REGISTER_MAP.end()) {
        logger->error("EG5CD1 未知寄存器名: {}", reg_name);
        return IoError::UnknownRegister;
    }
    const int address = it->second;

    std::vector<uint8_t> cmd;
    try {
        cmd = buildWriteCommand(address, values);
    } catch (const std::exception& e) {
        logger->error("EG5CD1 写寄存器参数非法 {}: {}", reg_name, e.what());
        return IoError::InvalidArgument;
    }

    try {
        logger->debug("[EG5CD1 写] {} 地址=0x{:04X} 帧={}", reg_name, address,
                      formatBytesToHex(cmd.data(), cmd.size()));
        device->write(cmd);
        auto writeResponse = readResponseWithLoop(device, 25, 9, false);
        if (writeResponse.empty()) {
            logger->error("EG5CD1 写寄存器无应答: {}", reg_name);
            return IoError::Timeout;
        }
        RingBuffer tempBuffer(128);
        tempBuffer.push(writeResponse.data(), writeResponse.size());
        auto [parseSuccess, _] = parseResponse(tempBuffer);
        if (parseSuccess) {
            std::ostringstream oss;
            oss << "写入" << reg_name << ":(";
            for (size_t i = 0; i < values.size(); ++i) {
                oss << values[i];
                if (i + 1 < values.size()) {
                    oss << " ";
                }
            }
            oss << ")";
            logger->info(oss.str());
            return IoError::Ok;
        }
        logger->error("EG5CD1 写应答解析失败: {}", reg_name);
        return IoError::BadResponse;
    } catch (const std::exception& e) {
        logger->error("EG5CD1 写寄存器异常 {}: {}", reg_name, e.what());
        return IoError::DeviceError;
    }
}

RegisterReadResult EG5CD1_485_Protocol::readRegister(Device device, RingBuffer& ringBuffer, const std::string& reg_name,
                                                     size_t length) {
    auto logger = getLogger();
    ringBuffer.clear();

    auto it = REGISTER_MAP.find(reg_name);
    if (it == REGISTER_MAP.end()) {
        logger->error("EG5CD1 未知寄存器名: {}", reg_name);
        return {IoError::UnknownRegister, {}};
    }
    const int address = it->second;

    if (length == 0) {
        length = getDefaultReadLength(reg_name);
    }

    try {
        const auto cmd = buildReadCommand(address, length);
        logger->debug("[EG5CD1 读] {} 地址=0x{:04X} len={} 帧={}", reg_name, address, length,
                      formatBytesToHex(cmd.data(), cmd.size()));
        device->write(cmd);
        auto response = readResponseWithLoop(device, 25, 8, true);
        if (response.empty()) {
            logger->error("EG5CD1 读寄存器无应答: {}", reg_name);
            return {IoError::Timeout, {}};
        }
        ringBuffer.push(response.data(), response.size());
        auto result = parseResponse(ringBuffer);
        if (result.first) {
            std::ostringstream oss;
            oss << "读取" << reg_name << ":(";
            for (size_t i = 0; i < result.second.size(); ++i) {
                oss << result.second[i];
                if (i + 1 < result.second.size()) {
                    oss << " ";
                }
            }
            oss << ")";
            logger->info(oss.str());
            return {IoError::Ok, std::move(result.second)};
        }
        logger->error("EG5CD1 读应答解析失败: {}", reg_name);
        return {IoError::BadResponse, {}};
    } catch (...) {
        logger->error("EG5CD1 读寄存器异常: {}", reg_name);
        return {IoError::DeviceError, {}};
    }
}

TouchReadResult EG5CD1_485_Protocol::readTouchData(Device, RingBuffer&, int) { return {IoError::NotSupported, {}}; }

REGISTER_PROTOCOL("EG5CD1_485", EG5CD1_485_Protocol);

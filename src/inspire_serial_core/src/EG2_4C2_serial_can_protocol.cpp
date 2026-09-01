#include "EG2_4C2_serial_can_protocol.hpp"
#include "protocol_factory.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <thread>

namespace {

constexpr uint8_t kFrameHead0 = 0xAA;
constexpr uint8_t kFrameHead1 = 0xAA;
constexpr uint8_t kFrameTail0 = 0x55;
constexpr uint8_t kFrameTail1 = 0x55;
constexpr uint8_t kEscapeByte = 0xA5;

std::string bytesToHex(const std::vector<uint8_t>& data) {
    std::ostringstream oss;
    oss << std::hex << std::uppercase;
    for (size_t i = 0; i < data.size(); ++i) {
        if (i > 0) {
            oss << " ";
        }
        oss << "0x" << std::setw(2) << std::setfill('0') << static_cast<int>(data[i]);
    }
    return oss.str();
}

std::string valuesToText(const std::vector<int>& values) {
    std::ostringstream oss;
    for (size_t i = 0; i < values.size(); ++i) {
        oss << values[i];
        if (i + 1 < values.size()) {
            oss << " ";
        }
    }
    return oss.str();
}

}  // namespace

// 寄存器地址映射表（与手册一致）
const std::map<std::string, int> EG2_4C2_serial_can_Protocol::REGISTER_MAP = {
    {"save", 1001},
    {"defaultPar", 1002},
    {"id", 1003},
    {"baud", 1004},
    {"catchMode", 1005},
    {"stop", 1006},
    {"clearError", 1007},
    {"openLenSet", 1010},
    {"speedSet", 1011},
    {"forceSet", 1012},
    {"maxOpenLen", 1016},
    {"minOpenLen", 1017},
    {"forceAct", 1060},
    {"openLenAct", 1061},
    {"currentAct", 1062},
    {"temp", 1063},
    {"errorCode", 1064},
    {"status", 1065},
    {"gripperStatusBlock", 1060},
};

// 寄存器默认读取长度（字节数）
const std::map<std::string, size_t> EG2_4C2_serial_can_Protocol::REGISTER_READ_LENGTH_MAP = {
    // 单值寄存器（2 字节）
    {"save", 2},
    {"defaultPar", 2},
    {"id", 2},
    {"baud", 2},
    {"catchMode", 2},
    {"stop", 2},
    {"clearError", 2},
    {"openLenSet", 2},
    {"speedSet", 2},
    {"forceSet", 2},
    {"maxOpenLen", 2},
    {"minOpenLen", 2},
    {"forceAct", 2},
    {"openLenAct", 2},
    {"currentAct", 2},
    {"temp", 2},
    {"errorCode", 2},
    {"status", 2},
    // gripperStatusBlock：连续读取状态寄存器，共 12 字节
    {"gripperStatusBlock", 12},
};

// 寄存器写入规则：单帧最大写入数量
const std::map<std::string, size_t> EG2_4C2_serial_can_Protocol::REGISTER_WRITE_RULE_MAX_COUNT = {
    {"id", 1},
    {"baud", 1},
    {"catchMode", 1},
    {"clearError", 1},
    {"stop", 1},
    {"save", 1},
    {"defaultPar", 1},
    {"maxOpenLen", 1},
    {"minOpenLen", 1},
    {"openLenSet", 4},  // 允许 position + speed + force 三寄存器连写
    {"speedSet", 1},
    {"forceSet", 1},
};

// 不支持的寄存器
const std::set<std::string> EG2_4C2_serial_can_Protocol::NOT_SUPPORTED_REGISTERS = {
    "touchAct",
};

int EG2_4C2_serial_can_Protocol::getRegisterAddress(const std::string& register_name) const {
    const auto it = REGISTER_MAP.find(register_name);
    if (it == REGISTER_MAP.end()) {
        return -1;
    }
    return it->second;
}

size_t EG2_4C2_serial_can_Protocol::getDefaultReadLength(const std::string& reg_name) const {
    const auto it = REGISTER_READ_LENGTH_MAP.find(reg_name);
    if (it == REGISTER_READ_LENGTH_MAP.end()) {
        return 2;
    }
    return it->second;
}

bool EG2_4C2_serial_can_Protocol::isNotSupportedRegister(const std::string& reg_name) const {
    return NOT_SUPPORTED_REGISTERS.find(reg_name) != NOT_SUPPORTED_REGISTERS.end();
}

uint32_t EG2_4C2_serial_can_Protocol::buildCanId(uint8_t op_type, int address) const {
    const uint32_t op = static_cast<uint32_t>(op_type & 0x03) << 26;
    const uint32_t reg = static_cast<uint32_t>(address & 0x0FFF) << 14;
    const uint32_t hid = static_cast<uint32_t>(device_id_) & 0x3FFF;
    return op | reg | hid;
}

std::vector<uint8_t> EG2_4C2_serial_can_Protocol::buildSerialCanFrame(
    uint32_t can_id,
    const std::vector<uint8_t>& payload,
    bool is_read) const {
    std::vector<uint8_t> frame;
    frame.reserve(kSerialFrameLength);
    frame.push_back(kFrameHead0);
    frame.push_back(kFrameHead1);

    frame.push_back(static_cast<uint8_t>(can_id & 0xFF));
    frame.push_back(static_cast<uint8_t>((can_id >> 8) & 0xFF));
    frame.push_back(static_cast<uint8_t>((can_id >> 16) & 0xFF));
    frame.push_back(static_cast<uint8_t>((can_id >> 24) & 0xFF));

    // 8 字节 CAN 数据区
    std::vector<uint8_t> data(kCanPayloadLength, 0xFF);
    if (is_read) {
        std::fill(data.begin(), data.end(), 0x00);
        data[0] = payload.empty() ? 0 : payload[0];
    } else {
        const size_t n = std::min(payload.size(), kCanPayloadLength);
        for (size_t i = 0; i < n; ++i) {
            data[i] = payload[i];
        }
    }
    frame.insert(frame.end(), data.begin(), data.end());

    // Meta[0] 是 CAN 帧的有效数据长度（DLC）。
    // 读命令的数据区只有 1 字节 read_len；写命令必须使用实际 payload 长度。
    // 不能把写 DLC 固定为 8，否则单寄存器 Topic 写入的 0xFF 填充会被设备
    // 当作后续寄存器数据，导致 openLenSet 等运动命令不执行。
    const uint8_t valid_len = is_read
                                  ? static_cast<uint8_t>(0x01)
                                  : static_cast<uint8_t>(std::min(payload.size(), kCanPayloadLength));
    frame.push_back(valid_len);
    frame.push_back(0x00);
    frame.push_back(0x01);
    frame.push_back(0x00);

    // 校验和：sum(ExtId..Meta) & 0xFF
    uint8_t checksum = 0;
    for (size_t i = 2; i < frame.size(); ++i) {
        checksum = static_cast<uint8_t>(checksum + frame[i]);
    }
    frame.push_back(checksum);
    frame.push_back(kFrameTail0);
    frame.push_back(kFrameTail1);

    return frame;
}

std::vector<uint8_t> EG2_4C2_serial_can_Protocol::removeA5Escape(const std::vector<uint8_t>& raw) const {
    std::vector<uint8_t> out;
    out.reserve(raw.size());

    for (size_t i = 0; i < raw.size(); ++i) {
        if (raw[i] == 0xA5 && (i + 1) < raw.size()) {
            const uint8_t next = raw[i + 1];
            if (next == 0x55 || next == 0xAA || next == 0xA5) {
                out.push_back(next);
                ++i;
                continue;
            }
        }
        out.push_back(raw[i]);
    }
    return out;
}

std::vector<uint8_t> EG2_4C2_serial_can_Protocol::readOneFrameRaw(Device device, int timeout_ms) const {
    auto logger = getLogger();
    std::vector<uint8_t> raw_stream;
    raw_stream.reserve(256);

    const auto start = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::milliseconds(timeout_ms);

    while (std::chrono::steady_clock::now() - start < timeout) {
        const auto chunk = device->read(std::chrono::milliseconds(5));
        if (!chunk.empty()) {
            raw_stream.insert(raw_stream.end(), chunk.begin(), chunk.end());
        }

        if (raw_stream.size() < 4) {
            continue;
        }

        for (size_t s = 0; s + 1 < raw_stream.size(); ++s) {
            if (raw_stream[s] != kFrameHead0 || raw_stream[s + 1] != kFrameHead1) {
                continue;
            }
            for (size_t e = s + 3; e < raw_stream.size(); ++e) {
                if (raw_stream[e - 1] == kFrameTail0 && raw_stream[e] == kFrameTail1) {
                    std::vector<uint8_t> candidate(raw_stream.begin() + static_cast<std::ptrdiff_t>(s),
                                                   raw_stream.begin() + static_cast<std::ptrdiff_t>(e + 1));
                    auto unescaped = removeA5Escape(candidate);
                    if (unescaped.size() == kSerialFrameLength) {
                        return unescaped;
                    }
                }
            }
        }

        if (raw_stream.size() > 1024) {
            raw_stream.erase(raw_stream.begin(), raw_stream.begin() + 512);
        }
    }

    logger->debug("[EG2-4C2] 响应读取超时，原始累计字节={}", raw_stream.size());
    return {};
}

bool EG2_4C2_serial_can_Protocol::validateChecksum(const std::vector<uint8_t>& response) const {
    if (response.size() != kSerialFrameLength) {
        return false;
    }
    if (response[0] != kFrameHead0 || response[1] != kFrameHead1 ||
        response[19] != kFrameTail0 || response[20] != kFrameTail1) {
        return false;
    }

    uint8_t checksum = 0;
    for (size_t i = 2; i <= 17; ++i) {
        checksum = static_cast<uint8_t>(checksum + response[i]);
    }
    return checksum == response[18];
}

bool EG2_4C2_serial_can_Protocol::parseAndValidateFrame(
    const std::vector<uint8_t>& frame,
    uint8_t expected_op_type,
    int expected_address,
    std::vector<uint8_t>* out_payload,
    uint8_t* out_valid_len) const {
    if (!validateChecksum(frame)) {
        return false;
    }

    const uint32_t can_id = static_cast<uint32_t>(frame[2]) |
                            (static_cast<uint32_t>(frame[3]) << 8) |
                            (static_cast<uint32_t>(frame[4]) << 16) |
                            (static_cast<uint32_t>(frame[5]) << 24);

    const uint8_t op_type = static_cast<uint8_t>((can_id >> 26) & 0x03);
    const int address = static_cast<int>((can_id >> 14) & 0x0FFF);
    const int hand_id = static_cast<int>(can_id & 0x3FFF);

    if (op_type != (expected_op_type & 0x03) || address != (expected_address & 0x0FFF)) {
        return false;
    }
    if (hand_id != (device_id_ & 0x3FFF)) {
        return false;
    }

    if (out_payload != nullptr) {
        out_payload->assign(frame.begin() + 6, frame.begin() + 14);
    }
    if (out_valid_len != nullptr) {
        *out_valid_len = frame[14];
    }
    return true;
}

std::vector<uint8_t> EG2_4C2_serial_can_Protocol::buildReadCommand(int address, size_t length) {
    const uint32_t can_id = buildCanId(kOpRead, address);
    std::vector<uint8_t> read_req = {static_cast<uint8_t>(length & 0xFF)};
    return buildSerialCanFrame(can_id, read_req, true);
}

std::vector<uint8_t> EG2_4C2_serial_can_Protocol::buildWriteCommand(
    int address,
    const std::vector<int>& values) {
    std::vector<uint8_t> payload;
    payload.reserve(values.size() * 2);
    for (int v : values) {
        const uint16_t u = static_cast<uint16_t>(v & 0xFFFF);
        payload.push_back(static_cast<uint8_t>(u & 0xFF));
        payload.push_back(static_cast<uint8_t>((u >> 8) & 0xFF));
    }
    const uint32_t can_id = buildCanId(kOpWrite, address);
    return buildSerialCanFrame(can_id, payload, false);
}

std::pair<bool, std::vector<int>> EG2_4C2_serial_can_Protocol::parseResponse(RingBuffer& ringBuffer) {
    (void)ringBuffer;
    return {false, {}};
}

IoError EG2_4C2_serial_can_Protocol::writeRegister(
    Device device,
    const std::string& reg_name,
    const std::vector<int>& values) {
    auto logger = getLogger();

    if (isNotSupportedRegister(reg_name)) {
        return IoError::NotSupported;
    }

    const auto it_addr = REGISTER_MAP.find(reg_name);
    if (it_addr == REGISTER_MAP.end()) {
        logger->error("[EG2-4C2] 未知寄存器名: {}", reg_name);
        return IoError::UnknownRegister;
    }
    const int address = it_addr->second;

    if (values.empty()) {
        logger->error("[EG2-4C2] 写寄存器 {} 至少需要 1 个值", reg_name);
        return IoError::InvalidArgument;
    }

    size_t max_count = 1;
    const auto it_rule = REGISTER_WRITE_RULE_MAX_COUNT.find(reg_name);
    if (it_rule != REGISTER_WRITE_RULE_MAX_COUNT.end()) {
        max_count = it_rule->second;
    }
    if (values.size() > max_count) {
        logger->error(
            "[EG2-4C2] 写寄存器 {} 超出单帧上限: 输入 {} 个, 上限 {} 个",
            reg_name, values.size(), max_count);
        return IoError::InvalidArgument;
    }

    std::vector<uint8_t> cmd;
    try {
        cmd = buildWriteCommand(address, values);
    } catch (const std::exception& e) {
        logger->error("[EG2-4C2] 写寄存器参数非法 {}: {}", reg_name, e.what());
        return IoError::InvalidArgument;
    }

    try {
        logger->info(
            "[EG2-4C2] 写寄存器 {} addr=0x{:04X} can_id=0x{:08X} values=({}) tx={}",
            reg_name, address, buildCanId(kOpWrite, address), valuesToText(values), bytesToHex(cmd));

        device->clearBuffer();
        device->write(cmd);
    } catch (...) {
        logger->error("[EG2-4C2] 写寄存器 {} 发送失败（device_error）", reg_name);
        return IoError::DeviceError;
    }

    const auto response = readOneFrameRaw(device, 30);
    if (response.empty()) {
        logger->error("[EG2-4C2] 写寄存器 {} 超时无回包（timeout）", reg_name);
        return IoError::Timeout;
    }
    logger->info("[EG2-4C2] 写寄存器 {} rx={}", reg_name, bytesToHex(response));

    std::vector<uint8_t> payload;
    uint8_t valid_len = 0;
    if (!parseAndValidateFrame(response, kOpWrite, address, &payload, &valid_len)) {
        logger->error("[EG2-4C2] 写寄存器 {} 回包校验失败（bad_response）", reg_name);
        return IoError::BadResponse;
    }

    logger->info("写入{}:({})", reg_name, valuesToText(values));
    return IoError::Ok;
}

RegisterReadResult EG2_4C2_serial_can_Protocol::readRegister(
    Device device,
    RingBuffer& ringBuffer,
    const std::string& reg_name,
    size_t length) {
    (void)ringBuffer;
    auto logger = getLogger();

    if (isNotSupportedRegister(reg_name)) {
        return {IoError::NotSupported, {}};
    }

    const auto it_addr = REGISTER_MAP.find(reg_name);
    if (it_addr == REGISTER_MAP.end()) {
        logger->error("[EG2-4C2] 未知寄存器名: {}", reg_name);
        return {IoError::UnknownRegister, {}};
    }

    size_t target_len = length;
    if (target_len == 0) {
        target_len = getDefaultReadLength(reg_name);
    }
    if (target_len == 0) {
        return {IoError::InvalidArgument, {}};
    }
    if (target_len > kCanPayloadLength) {
        if (target_len != 12) {
            logger->error("[EG2-4C2] 读寄存器 {} 长度 {} 超过单帧 8 字节上限", reg_name, target_len);
            return {IoError::InvalidArgument, {}};
        }
    }

    std::vector<uint8_t> merged_payload;
    merged_payload.reserve(target_len);

    size_t byte_offset = 0;
    while (byte_offset < target_len) {
        size_t chunk_len = std::min(kCanPayloadLength, target_len - byte_offset);
        const int frame_addr = it_addr->second + static_cast<int>(byte_offset);
        const uint32_t can_id = buildCanId(kOpRead, frame_addr);

        std::vector<uint8_t> read_req = {static_cast<uint8_t>(chunk_len & 0xFF)};
        const auto cmd = buildSerialCanFrame(can_id, read_req, true);
        logger->debug(
            "[EG2-4C2] 读寄存器 {} addr=0x{:04X} can_id=0x{:08X} len={} tx={}",
            reg_name, frame_addr, can_id, chunk_len, bytesToHex(cmd));

        try {
            device->clearBuffer();
            device->write(cmd);
        } catch (...) {
            logger->error("[EG2-4C2] 读寄存器 {} 发送失败（device_error）", reg_name);
            return {IoError::DeviceError, {}};
        }

        const auto response = readOneFrameRaw(device, 30);
        if (response.empty()) {
            logger->error(
                "[EG2-4C2] 读寄存器 {} 超时无回包（timeout），addr=0x{:04X} len={}",
                reg_name, frame_addr, chunk_len);
            return {IoError::Timeout, {}};
        }
        logger->debug("[EG2-4C2] 读寄存器 {} rx={}", reg_name, bytesToHex(response));

        std::vector<uint8_t> payload;
        uint8_t valid_len = 0;
        if (!parseAndValidateFrame(response, kOpRead, frame_addr, &payload, &valid_len)) {
            logger->error("[EG2-4C2] 读寄存器 {} 回包校验失败（bad_response），addr=0x{:04X}", reg_name, frame_addr);
            return {IoError::BadResponse, {}};
        }

        const size_t used = std::min<size_t>({payload.size(), static_cast<size_t>(valid_len), chunk_len});
        if (used == 0) {
            logger->error("[EG2-4C2] 读寄存器 {} 回包长度非法（bad_response）", reg_name);
            return {IoError::BadResponse, {}};
        }

        merged_payload.insert(merged_payload.end(),
                              payload.begin(),
                              payload.begin() + static_cast<std::ptrdiff_t>(used));
        byte_offset += used;

        if (byte_offset < target_len) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    // 按 int16 小端解码
    std::vector<int> decoded;
    decoded.reserve(merged_payload.size() / 2);
    for (size_t i = 0; i + 1 < merged_payload.size(); i += 2) {
        int v = static_cast<int>(merged_payload[i]) | (static_cast<int>(merged_payload[i + 1]) << 8);
        if (v > 32767) {
            v -= 65536;
        }
        decoded.push_back(v);
    }

    if (decoded.empty() && !merged_payload.empty()) {
        logger->error("[EG2-4C2] 读寄存器 {} 解码失败（bad_response）", reg_name);
        return {IoError::BadResponse, {}};
    }

    logger->info("读取{}:({})", reg_name, valuesToText(decoded));
    return {IoError::Ok, std::move(decoded)};
}

std::pair<bool, TouchDataResult> EG2_4C2_serial_can_Protocol::parseTouchData(RingBuffer&, int) {
    return {false, {}};
}

TouchReadResult EG2_4C2_serial_can_Protocol::readTouchData(Device, RingBuffer&, int) {
    return {IoError::NotSupported, {}};
}

REGISTER_PROTOCOL("EG2_4C2_serial_can", EG2_4C2_serial_can_Protocol);

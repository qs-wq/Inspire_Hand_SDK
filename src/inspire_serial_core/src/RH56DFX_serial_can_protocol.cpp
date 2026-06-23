#include "RH56DFX_serial_can_protocol.hpp"

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

bool isFingerSeriesRegister(const std::string& reg_name) {
    return reg_name == "posSet" ||
           reg_name == "angleSet" ||
           reg_name == "forceSet" ||
           reg_name == "speedSet" ||
           reg_name == "angleAct" ||
           reg_name == "forceAct" ||
           reg_name == "errorCode" ||
           reg_name == "status" ||
           reg_name == "temp";
}

bool isMotionWriteRegister(const std::string& reg_name) {
    return reg_name == "posSet" ||
           reg_name == "angleSet" ||
           reg_name == "forceSet" ||
           reg_name == "speedSet";
}

std::string resolveWriteRegisterName(const std::string& reg_name) {
    if (reg_name == "defaultSpeedSet") {
        return "speedSet";
    }
    if (reg_name == "defaultForceSet") {
        return "forceSet";
    }
    return reg_name;
}

}  // namespace

const std::map<std::string, int> RH56DFX_serial_can_Protocol::REGISTER_MAP = {
    {"id", 250},                 // HAND_ID
    {"baudRate", 4008},          // REDU_RATIO
    {"clearError", 4016},        // CLEAR_ERROR
    {"save", 4020},              // SAVE
    {"posSet", 1474},
    {"angleSet", 1486},
    {"forceSet", 1498},
    {"speedSet", 1522},
    {"angleAct", 1546},
    {"forceAct", 1582},
    {"errorCode", 1606},
    {"status", 1612},
    {"temp", 1618},
    {"actionSeqIndex", 2320},
    {"actionSeqRun", 2322},
    {"touchAct", 3000},
};

const std::map<std::string, size_t> RH56DFX_serial_can_Protocol::REGISTER_READ_LENGTH_MAP = {
    {"id", 2},
    {"baudRate", 1},
    {"clearError", 1},
    {"save", 1},
    {"posSet", 12},
    {"angleSet", 12},
    {"forceSet", 12},
    {"speedSet", 12},
    {"angleAct", 12},
    {"forceAct", 12},
    {"errorCode", 12},
    {"status", 12},
    {"temp", 12},
    {"actionSeqIndex", 2},
    {"actionSeqRun", 2},
};

const std::map<std::string, RH56DFX_serial_can_Protocol::RegisterWriteRule>
RH56DFX_serial_can_Protocol::REGISTER_WRITE_RULE_MAP = {
    {"id", {2, 1}},
    {"baudRate", {1, 1}},
    {"clearError", {1, 1}},
    {"save", {1, 1}},
    {"posSet", {2, 6}},
    {"angleSet", {2, 6}},
    {"forceSet", {2, 6}},
    {"speedSet", {2, 6}},
    {"actionSeqIndex", {2, 1}},
    {"actionSeqRun", {2, 1}},
};

const std::set<std::string> RH56DFX_serial_can_Protocol::NOT_SUPPORTED_REGISTERS = {
    "resetPara",
    "gestureForceClb",
    "currentSet",
    "defaultSpeedSet",
    "defaultForceSet",
    "posAct",
    "currentAct",
    "mode",
    "pause",
    "stop",
    "actionLibraryIndex",
    "touchAct",
};

int RH56DFX_serial_can_Protocol::getRegisterAddress(const std::string& register_name) const {
    const auto it = REGISTER_MAP.find(register_name);
    if (it == REGISTER_MAP.end()) {
        return -1;
    }
    return it->second;
}

size_t RH56DFX_serial_can_Protocol::getDefaultReadLength(const std::string& reg_name) const {
    const auto it = REGISTER_READ_LENGTH_MAP.find(reg_name);
    if (it == REGISTER_READ_LENGTH_MAP.end()) {
        return 12;
    }
    return it->second;
}

bool RH56DFX_serial_can_Protocol::isNotSupportedRegister(const std::string& reg_name) const {
    return NOT_SUPPORTED_REGISTERS.find(reg_name) != NOT_SUPPORTED_REGISTERS.end();
}

uint32_t RH56DFX_serial_can_Protocol::buildCanId(uint8_t rw_flag, int address) const {
    const uint32_t rw = static_cast<uint32_t>(rw_flag & 0x07) << 26;
    const uint32_t reg = static_cast<uint32_t>(address & 0x0FFF) << 14;
    const uint32_t hid = static_cast<uint32_t>(device_id_) & 0x3FFF;
    return rw | reg | hid;
}

std::vector<uint8_t> RH56DFX_serial_can_Protocol::buildSerialCanFrame(
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

    std::vector<uint8_t> data(kCanPayloadLength, 0xFF);
    if (is_read) {
        data.assign(kCanPayloadLength, 0x00);
        data[0] = payload.empty() ? 0 : payload[0];
    } else {
        const size_t n = std::min(payload.size(), kCanPayloadLength);
        for (size_t i = 0; i < n; ++i) {
            data[i] = payload[i];
        }
    }
    frame.insert(frame.end(), data.begin(), data.end());

    // 固定字段
    if (is_read) {
        frame.push_back(0x01);
    } else {
        // 按 2.py 兼容：写帧长度字段使用实际数据长度（如 2/4/8）
        frame.push_back(static_cast<uint8_t>(std::min(payload.size(), kCanPayloadLength)));
    }
    frame.push_back(0x00);
    frame.push_back(0x01);
    frame.push_back(0x00);

    uint8_t checksum = 0;
    for (size_t i = 2; i < frame.size(); ++i) {
        checksum = static_cast<uint8_t>(checksum + frame[i]);
    }
    frame.push_back(checksum);
    frame.push_back(kFrameTail0);
    frame.push_back(kFrameTail1);

    return frame;
}

std::vector<uint8_t> RH56DFX_serial_can_Protocol::removeA5Escape(const std::vector<uint8_t>& raw) const {
    std::vector<uint8_t> out;
    out.reserve(raw.size());

    for (size_t i = 0; i < raw.size(); ++i) {
        if (raw[i] == kEscapeByte && (i + 1) < raw.size()) {
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

std::vector<uint8_t> RH56DFX_serial_can_Protocol::readOneFrameRaw(Device device, int timeout_ms) const {
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

    logger->debug("[RH56DFX] 响应读取超时，原始累计字节={}", raw_stream.size());
    return {};
}

bool RH56DFX_serial_can_Protocol::validateChecksum(const std::vector<uint8_t>& response) const {
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

bool RH56DFX_serial_can_Protocol::parseAndValidateFrame(
    const std::vector<uint8_t>& frame,
    uint8_t expected_rw,
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

    const uint8_t rw_flag = static_cast<uint8_t>((can_id >> 26) & 0x07);
    const int address = static_cast<int>((can_id >> 14) & 0x0FFF);
    const int hand_id = static_cast<int>(can_id & 0x3FFF);

    if (rw_flag != expected_rw || address != (expected_address & 0x0FFF)) {
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

std::vector<uint8_t> RH56DFX_serial_can_Protocol::encodeValuesByRule(
    const std::string& reg_name,
    const std::vector<int>& values,
    IoError* err) const {
    if (values.empty()) {
        if (err != nullptr) {
            *err = IoError::InvalidArgument;
        }
        return {};
    }

    RegisterWriteRule rule{};
    const auto it_rule = REGISTER_WRITE_RULE_MAP.find(reg_name);
    if (it_rule != REGISTER_WRITE_RULE_MAP.end()) {
        rule = it_rule->second;
    }

    if (values.size() > rule.max_value_count) {
        if (err != nullptr) {
            *err = IoError::InvalidArgument;
        }
        return {};
    }

    std::vector<uint8_t> bytes;
    bytes.reserve(values.size() * rule.value_width_bytes);

    for (const int value : values) {
        if (rule.value_width_bytes == 1) {
            if (value < 0 || value > 255) {
                if (err != nullptr) {
                    *err = IoError::InvalidArgument;
                }
                return {};
            }
            bytes.push_back(static_cast<uint8_t>(value));
            continue;
        }

        const uint16_t u = static_cast<uint16_t>(value & 0xFFFF);
        bytes.push_back(static_cast<uint8_t>(u & 0xFF));
        bytes.push_back(static_cast<uint8_t>((u >> 8) & 0xFF));
    }

    if (err != nullptr) {
        *err = IoError::Ok;
    }
    return bytes;
}

std::vector<int> RH56DFX_serial_can_Protocol::decodeValuesByRule(
    const std::string& reg_name,
    const std::vector<uint8_t>& payload) const {
    RegisterWriteRule rule{};
    const auto it_rule = REGISTER_WRITE_RULE_MAP.find(reg_name);
    if (it_rule != REGISTER_WRITE_RULE_MAP.end()) {
        rule = it_rule->second;
    }

    std::vector<int> values;
    if (rule.value_width_bytes == 1) {
        values.reserve(payload.size());
        for (uint8_t b : payload) {
            values.push_back(static_cast<int>(b));
        }
        return values;
    }

    values.reserve(payload.size() / 2);
    for (size_t i = 0; i + 1 < payload.size(); i += 2) {
        int v = static_cast<int>(payload[i]) | (static_cast<int>(payload[i + 1]) << 8);
        if (v > 32767) {
            v -= 65536;
        }
        values.push_back(v);
    }
    return values;
}

IoError RH56DFX_serial_can_Protocol::writeRegister(
    Device device,
    const std::string& reg_name,
    const std::vector<int>& values) {
    auto logger = getLogger();

    const std::string effective_reg = resolveWriteRegisterName(reg_name);
    if (isNotSupportedRegister(reg_name) && effective_reg == reg_name) {
        return IoError::NotSupported;
    }
    if (effective_reg != reg_name) {
        logger->debug("[RH56DFX] {} 映射写入 {}（无持久化 default 寄存器）", reg_name, effective_reg);
    }

    const auto it_addr = REGISTER_MAP.find(effective_reg);
    if (it_addr == REGISTER_MAP.end()) {
        return IoError::UnknownRegister;
    }

    IoError precheck_err = IoError::Ok;
    (void)encodeValuesByRule(effective_reg, values, &precheck_err);
    if (!isOk(precheck_err)) {
        return precheck_err;
    }

    RegisterWriteRule rule{};
    const auto it_rule = REGISTER_WRITE_RULE_MAP.find(effective_reg);
    if (it_rule != REGISTER_WRITE_RULE_MAP.end()) {
        rule = it_rule->second;
    }
    const size_t bytes_per_value = rule.value_width_bytes;

    int base_address = it_addr->second;
    size_t value_offset = 0;
    while (value_offset < values.size()) {
        const size_t remain = values.size() - value_offset;
        size_t one_frame_values = 1;
        if (isFingerSeriesRegister(effective_reg) && bytes_per_value == 2) {
            // 对齐 2.py：6 自由度寄存器优先 4+2 分包
            if (value_offset == 0 && remain >= 4) {
                one_frame_values = 4;
            } else {
                one_frame_values = std::min<size_t>(2, remain);
            }
        } else {
            one_frame_values = std::min<size_t>(remain, std::max<size_t>(1, kCanPayloadLength / bytes_per_value));
        }

        std::vector<int> frame_values;
        frame_values.reserve(one_frame_values);
        for (size_t i = 0; i < one_frame_values; ++i) {
            frame_values.push_back(values[value_offset + i]);
        }

        IoError frame_encode_err = IoError::Ok;
        const auto frame_payload = encodeValuesByRule(effective_reg, frame_values, &frame_encode_err);
        if (!isOk(frame_encode_err)) {
            return frame_encode_err;
        }

        const int frame_addr = base_address + static_cast<int>(value_offset * bytes_per_value);
        const uint32_t can_id = buildCanId(kRwWriteHand, frame_addr);
        const auto cmd = buildSerialCanFrame(can_id, frame_payload, false);
        logger->info(
            "[RH56DFX] 写寄存器 {} addr={} can_id=0x{:08X} values={} tx={}",
            reg_name,
            frame_addr,
            can_id,
            frame_values.size(),
            bytesToHex(cmd));

        try {
            // 对齐 2.py 的行为：每次发送前清空输入缓冲，避免历史数据干扰当前收包
            device->clearBuffer();
            device->write(cmd);
        } catch (...) {
            logger->error("[RH56DFX] 写寄存器 {} 发送失败（device_error）", reg_name);
            return IoError::DeviceError;
        }

        // 完全对齐 2.py：运动命令只发送，不依赖回包判定成功
        if (isMotionWriteRegister(effective_reg)) {
            value_offset += one_frame_values;
            // 协议文档建议帧间 10~50ms，这里取 20ms 兼顾稳定性
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }

        const auto response = readOneFrameRaw(device, 30);
        if (response.empty()) {
            logger->error("[RH56DFX] 写寄存器 {} 超时无回包（timeout）", reg_name);
            return IoError::Timeout;
        }
        logger->info("[RH56DFX] 写寄存器 {} rx={}", reg_name, bytesToHex(response));

        std::vector<uint8_t> payload;
        uint8_t valid_len = 0;
        if (!parseAndValidateFrame(response, kRwWriteHand, frame_addr, &payload, &valid_len)) {
            logger->error(
                "[RH56DFX] 写寄存器 {} 回包校验失败（bad_response），addr={} expected_rw={}",
                reg_name,
                frame_addr,
                static_cast<int>(kRwWriteHand));
            return IoError::BadResponse;
        }

        value_offset += one_frame_values;
        std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }

    if (reg_name == "actionSeqIndex") {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        const IoError run_err = writeRegister(device, "actionSeqRun", {1});
        if (!isOk(run_err)) {
            logger->warn("[RH56DFX] actionSeqIndex 写入后自动触发 actionSeqRun 失败: {}", toString(run_err));
            return run_err;
        }
    }

    return IoError::Ok;
}

RegisterReadResult RH56DFX_serial_can_Protocol::readRegister(
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
        return {IoError::UnknownRegister, {}};
    }

    size_t target_len = length;
    if (target_len == 0) {
        target_len = getDefaultReadLength(reg_name);
    }
    if (target_len == 0) {
        return {IoError::InvalidArgument, {}};
    }

    std::vector<uint8_t> merged_payload;
    merged_payload.reserve(target_len);

    size_t byte_offset = 0;
    while (byte_offset < target_len) {
        size_t chunk_len = std::min(kCanPayloadLength, target_len - byte_offset);
        const int frame_addr = it_addr->second + static_cast<int>(byte_offset);
        const uint32_t can_id = buildCanId(kRwReadHand, frame_addr);

        std::vector<uint8_t> read_req = {static_cast<uint8_t>(chunk_len)};
        const auto cmd = buildSerialCanFrame(can_id, read_req, true);
        logger->debug(
            "[RH56DFX] 读寄存器 {} addr={} can_id=0x{:08X} len={} tx={}",
            reg_name,
            frame_addr,
            can_id,
            chunk_len,
            bytesToHex(cmd));

        try {
            // 对齐 2.py 的行为：每次发送前清空输入缓冲，避免历史数据干扰当前收包
            device->clearBuffer();
            device->write(cmd);
        } catch (...) {
            logger->error("[RH56DFX] 读寄存器 {} 发送失败（device_error）", reg_name);
            return {IoError::DeviceError, {}};
        }

        const auto response = readOneFrameRaw(device, 30);
        if (response.empty()) {
            logger->error(
                "[RH56DFX] 读寄存器 {} 超时无回包（timeout），addr={} len={}",
                reg_name,
                frame_addr,
                chunk_len);
            return {IoError::Timeout, {}};
        }
        logger->debug("[RH56DFX] 读寄存器 {} rx={}", reg_name, bytesToHex(response));

        std::vector<uint8_t> payload;
        uint8_t valid_len = 0;
        if (!parseAndValidateFrame(response, kRwReadHand, frame_addr, &payload, &valid_len)) {
            logger->error(
                "[RH56DFX] 读寄存器 {} 回包校验失败（bad_response），addr={}",
                reg_name,
                frame_addr);
            return {IoError::BadResponse, {}};
        }

        const size_t used = std::min<size_t>({payload.size(), static_cast<size_t>(valid_len), chunk_len});
        if (used == 0) {
            logger->error(
                "[RH56DFX] 读寄存器 {} 回包长度非法（bad_response），addr={} payload={} valid_len={}",
                reg_name,
                frame_addr,
                payload.size(),
                static_cast<int>(valid_len));
            return {IoError::BadResponse, {}};
        }

        merged_payload.insert(merged_payload.end(), payload.begin(), payload.begin() + static_cast<std::ptrdiff_t>(used));
        byte_offset += used;
    }

    auto decoded = decodeValuesByRule(reg_name, merged_payload);
    if (decoded.empty() && !merged_payload.empty()) {
        logger->error(
            "[RH56DFX] 读寄存器 {} 解码失败（bad_response），payload_len={}",
            reg_name,
            merged_payload.size());
        return {IoError::BadResponse, {}};
    }

    // 与 RH5DG2/RH56F1 读流程对齐：读成功时在 INFO 级别输出值，
    // 链路恢复后会再次出现成功日志，便于现场判断通信恢复。
    logger->info("读取{}:({})", reg_name, valuesToText(decoded));

    return {IoError::Ok, std::move(decoded)};
}

uint8_t RH56DFX_serial_can_Protocol::readByteAtOffset(const RingBuffer& ringBuffer, size_t offset) const {
    const size_t bufSize = ringBuffer.size();
    if (offset >= bufSize) {
        return 0xFF;
    }
    const std::vector<uint8_t>& buf = ringBuffer.getBuffer();
    const size_t tailIndex = ringBuffer.getTail();
    const size_t bufferSize = buf.size();
    return buf[(tailIndex + offset) % bufferSize];
}

std::vector<uint8_t> RH56DFX_serial_can_Protocol::extractFromRingBuffer(
    const RingBuffer& ringBuffer,
    size_t startOffset,
    size_t length) const {
    std::vector<uint8_t> result(length);
    const std::vector<uint8_t>& buf = ringBuffer.getBuffer();
    const size_t tailIndex = ringBuffer.getTail();
    const size_t bufferSize = buf.size();

    for (size_t i = 0; i < length; ++i) {
        result[i] = buf[(tailIndex + startOffset + i) % bufferSize];
    }
    return result;
}

bool RH56DFX_serial_can_Protocol::validate485FrameChecksum(const std::vector<uint8_t>& response) const {
    if (response.size() < 9) {
        return false;
    }
    if (response[0] != 0x90 || response[1] != 0xEB) {
        return false;
    }

    uint8_t checksum = 0;
    for (size_t i = 2; i < response.size() - 1; ++i) {
        checksum += response[i];
    }

    const bool valid = (checksum == response.back());
    if (!valid) {
        auto logger = getLogger();
        logger->debug("触觉数据校验和验证失败: 计算值 = 0x{:02X}, 期望值 = 0x{:02X}", checksum, response.back());
    }
    return valid;
}

std::pair<bool, TouchDataResult> RH56DFX_serial_can_Protocol::parseTouchData(RingBuffer& ringBuffer, int version) {
    auto logger = getLogger();

    try {
        const size_t bufSize = ringBuffer.size();
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
            if (readByteAtOffset(ringBuffer, i) == 0x90 && readByteAtOffset(ringBuffer, i + 1) == 0xEB) {
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

        const size_t register_length = data_length - 3;
        const size_t response_len = 8 + register_length;

        if (bufSize < startIdx + response_len) {
            return {false, {}};
        }

        const std::vector<uint8_t> response = extractFromRingBuffer(ringBuffer, startIdx, response_len);
        if (!validate485FrameChecksum(response)) {
            logger->debug("触觉数据校验和验证失败，跳过当前帧头");
            ringBuffer.advance(startIdx + 1);
            return {false, {}};
        }

        TouchDataResult result;
        const size_t data_start = 7;
        const std::string fingers[] = {"little", "ring", "middle", "index", "thumb"};

        const size_t available_data_length = register_length;
        const size_t finger_data_length = 5 * 10;
        const size_t palm_start_idx = data_start + finger_data_length;
        const size_t palm_data_length = 9 * 2;

        for (int i = 0; i < 5; ++i) {
            const size_t base_idx = data_start + static_cast<size_t>(i) * 10;
            if (base_idx + 10 > data_start + available_data_length || base_idx + 10 > response.size()) {
                logger->warn("触觉数据不足，无法解析第{}个手指数据", i + 1);
                break;
            }

            std::vector<uint16_t> finger_vals;
            finger_vals.reserve(4);
            for (int j = 0; j < 3; ++j) {
                const uint8_t low_byte = response[base_idx + static_cast<size_t>(j) * 2];
                const uint8_t high_byte = response[base_idx + static_cast<size_t>(j) * 2 + 1];
                const uint16_t val = low_byte | (high_byte << 8);
                finger_vals.push_back(val);
            }

            if (base_idx + 9 <= response.size()) {
                const uint8_t b0 = response[base_idx + 6];
                const uint8_t b1 = response[base_idx + 7];
                const uint8_t b2 = response[base_idx + 8];
                const uint32_t combined = b0 | (b1 << 8) | (b2 << 16);
                finger_vals.push_back(combined);
            } else {
                logger->warn("触觉数据不足，无法解析第{}个手指的24位值", i + 1);
            }

            result.fingerResults[fingers[i]] = std::move(finger_vals);
        }

        if (palm_start_idx + palm_data_length <= data_start + available_data_length &&
            palm_start_idx + palm_data_length <= response.size()) {
            for (int j = 0; j < 9; ++j) {
                const size_t idx = palm_start_idx + static_cast<size_t>(j) * 2;
                if (idx + 1 < response.size()) {
                    const uint16_t val = response[idx] | (response[idx + 1] << 8);
                    result.palmResults["palm_data_" + std::to_string(j + 1)] = val;
                } else {
                    logger->warn("触觉数据不足，无法解析第{}个掌心数据", j + 1);
                    break;
                }
            }
        } else {
            logger->warn(
                "触觉数据不足，无法解析掌心数据 (需要 {} 字节，实际可用 {} 字节)",
                palm_data_length,
                (data_start + available_data_length > palm_start_idx)
                    ? (data_start + available_data_length - palm_start_idx)
                    : 0);
        }

        ringBuffer.advance(startIdx + response_len);
        return {true, std::move(result)};
    } catch (const std::exception& e) {
        logger->error("触觉解析异常: {}", e.what());
        return {false, {}};
    }
}

TouchReadResult RH56DFX_serial_can_Protocol::readTouchData(Device device, RingBuffer& ringBuffer, int version) {
    std::ostringstream oss;
    auto logger = getLogger();

    try {
        if (isNotSupportedRegister("touchAct")) {
            logger->debug("RH56DFX 机型无触觉传感器，touchAct 不可用");
            return {IoError::NotSupported, {}};
        }

        ringBuffer.clear();

        const int touchAddress = getRegisterAddress("touchAct");
        if (touchAddress < 0) {
            logger->error("未知触觉寄存器 touchAct");
            return {IoError::UnknownRegister, {}};
        }

        const auto readTouchCmd = buildReadCommand(touchAddress, 68);
        logger->debug(
            "[读取命令-触觉] 地址: 0x{:04X}, 长度: 68, 命令: {}",
            touchAddress,
            bytesToHex(readTouchCmd));

        try {
            device->clearBuffer();
            device->write(readTouchCmd);
        } catch (...) {
            logger->error("读取触觉寄存器发送失败（device_error）");
            return {IoError::DeviceError, {}};
        }

        const auto resp = readOneFrameRaw(device, 30);
        if (!resp.empty()) {
            logger->debug("[原始响应-触觉] 响应: {}", bytesToHex(resp));
        } else {
            logger->debug("[原始响应-触觉] 响应为空");
        }

        if (resp.empty()) {
            logger->error("读取触觉寄存器失败");
            return {IoError::Timeout, {}};
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
                    if (i != finger_pair.second.size() - 1) {
                        oss << " ";
                    }
                }
                oss << " ";
            }
            for (const auto& palm_pair : touchData.palmResults) {
                oss << palm_pair.first << ":" << palm_pair.second << " ";
            }
            oss << ")";
            logger->info(oss.str());
            return {IoError::Ok, std::move(result.second)};
        }

        logger->error("读取触觉寄存器失败");
        return {IoError::BadResponse, {}};
    } catch (...) {
        logger->error("读取触觉寄存器异常");
        return {IoError::DeviceError, {}};
    }
}

std::vector<uint8_t> RH56DFX_serial_can_Protocol::buildReadCommand(int address, size_t length) {
    const uint32_t can_id = buildCanId(kRwReadHand, address);
    std::vector<uint8_t> read_req = {static_cast<uint8_t>(length & 0xFF)};
    return buildSerialCanFrame(can_id, read_req, true);
}

std::vector<uint8_t> RH56DFX_serial_can_Protocol::buildWriteCommand(
    int address,
    const std::vector<int>& values) {
    std::vector<uint8_t> payload;
    payload.reserve(values.size() * 2);
    for (int v : values) {
        const uint16_t u = static_cast<uint16_t>(v & 0xFFFF);
        payload.push_back(static_cast<uint8_t>(u & 0xFF));
        payload.push_back(static_cast<uint8_t>((u >> 8) & 0xFF));
    }
    const uint32_t can_id = buildCanId(kRwWriteHand, address);
    return buildSerialCanFrame(can_id, payload, false);
}

std::pair<bool, std::vector<int>> RH56DFX_serial_can_Protocol::parseResponse(RingBuffer& ringBuffer) {
    (void)ringBuffer;
    return {false, {}};
}

REGISTER_PROTOCOL("RH56DFX_serial_can", RH56DFX_serial_can_Protocol);


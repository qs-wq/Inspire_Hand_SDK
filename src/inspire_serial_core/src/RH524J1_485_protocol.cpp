#include "RH524J1_485_protocol.hpp"
#include "logger_manager.hpp"
#include "protocol_factory.hpp"

#include <algorithm>
#include <chrono>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace {
constexpr int kZone2Offset = 200;
constexpr int kNumDrv = static_cast<int>(RH524J1_485_Protocol::kJointCount);
constexpr int kFingerBendMax = 900;
constexpr int kPinkyFoldLo = -150; // ID1 小指翻折，实机标定
constexpr int kPinkyFoldHi = 0;
// 数组下标 = 电缸 ID-1；仅拇指指尖(下标19)与掌指(下标21)与实机对调
constexpr size_t kThumbDipUserIndex = 19;
constexpr size_t kThumbMcpUserIndex = 21;

size_t userIndexToAngleSetRegisterIndex(size_t user_index) {
    if (user_index == kThumbDipUserIndex) {
        return kThumbMcpUserIndex;
    }
    if (user_index == kThumbMcpUserIndex) {
        return kThumbDipUserIndex;
    }
    return user_index;
}

constexpr int kAngleKeepSentinel = -1;

bool isAngleExplicitValue(int v) { return v != kAngleKeepSentinel; }
} // namespace

// 寄存器地址：065demo/hand_param.h + demo_485_cable_driven_hand.py
const std::map<std::string, int> RH524J1_485_Protocol::REGISTER_MAP = {
    {"id", 100},
    {"baudRate", 101},
    {"clearError", 103},
    {"save", 104},
    {"resetPara", 105},
    {"gestureForceClb", 107},
    {"pause", 110},
    {"stop", 111},
    {"currentSet", kZone2Offset + kNumDrv * 1},      // 224
    {"defaultSpeedSet", kZone2Offset + kNumDrv * 2}, // 248
    {"defaultForceSet", kZone2Offset + kNumDrv * 3}, // 272
    {"posSet", kZone2Offset + kNumDrv * 4},          // 296
    {"angleSet", kZone2Offset + kNumDrv * 5},        // 320
    {"forceSet", kZone2Offset + kNumDrv * 6},        // 344
    {"speedSet", kZone2Offset + kNumDrv * 7},        // 368
    {"posAct", kZone2Offset + kNumDrv * 8},          // 392
    {"angleAct", kZone2Offset + kNumDrv * 9},        // 416
    {"forceAct", kZone2Offset + kNumDrv * 10},       // 440
    {"currentAct", kZone2Offset + kNumDrv * 11},     // 464
    {"errorCode", kZone2Offset + kNumDrv * 12},      // 488
    {"status", kZone2Offset + kNumDrv * 13},         // 512
    {"temp", kZone2Offset + kNumDrv * 14},           // 536
    {"mode", kZone2Offset + kNumDrv * 15},           // 560
};

const std::map<std::string, size_t> RH524J1_485_Protocol::REGISTER_READ_LENGTH_MAP = {
    {"angleSet", RH524J1_485_Protocol::kJointBytes},
    {"angleAct", RH524J1_485_Protocol::kJointBytes},
    {"forceSet", RH524J1_485_Protocol::kJointBytes},
    {"forceAct", RH524J1_485_Protocol::kJointBytes},
    {"currentSet", RH524J1_485_Protocol::kJointBytes},
    {"currentAct", RH524J1_485_Protocol::kJointBytes},
    {"posSet", RH524J1_485_Protocol::kJointBytes},
    {"posAct", RH524J1_485_Protocol::kJointBytes},
    {"speedSet", RH524J1_485_Protocol::kJointBytes},
    {"defaultSpeedSet", RH524J1_485_Protocol::kJointBytes},
    {"defaultForceSet", RH524J1_485_Protocol::kJointBytes},
    {"errorCode", RH524J1_485_Protocol::kJointBytes},
    {"status", RH524J1_485_Protocol::kJointBytes},
    {"temp", RH524J1_485_Protocol::kJointBytes},
    {"mode", RH524J1_485_Protocol::kJointBytes},
    {"id", 2},
    {"baudRate", 2},
    {"clearError", 2},
    {"save", 2},
    {"resetPara", 2},
    {"gestureForceClb", 2},
    {"pause", 2},
    {"stop", 2},
};

size_t RH524J1_485_Protocol::getDefaultReadLength(const std::string& reg_name) const {
    auto it = REGISTER_READ_LENGTH_MAP.find(reg_name);
    if (it != REGISTER_READ_LENGTH_MAP.end()) {
        return it->second;
    }
    return kJointBytes;
}

int RH524J1_485_Protocol::getRegisterAddress(const std::string& register_name) const {
    auto it = REGISTER_MAP.find(register_name);
    return (it != REGISTER_MAP.end()) ? it->second : -1;
}

int RH524J1_485_Protocol::clampAngle(int drive_id, int angle) const {
    // ID1 小指翻折实机 -150~0（0 张开，-150 弯曲）；ID2~5 侧摆 -200~200；
    // ID18/19/23/24 拇指旋转/侧摆与手腕 0~1000；其余弯曲关节 0~900
    int lo = 0;
    int hi = kFingerBendMax;
    if (drive_id == 1) {
        lo = kPinkyFoldLo;
        hi = kPinkyFoldHi;
    } else if (drive_id >= 2 && drive_id <= 5) {
        lo = -200;
        hi = 200;
    } else if (drive_id == 18 || drive_id == 19 || drive_id == 23 || drive_id == 24) {
        lo = 0;
        hi = 1000;
    }
    return std::clamp(angle, lo, hi);
}

bool RH524J1_485_Protocol::isAngleJointRegister(const std::string& reg_name) const {
    return reg_name == "angleAct" || reg_name == "angleSet";
}

bool RH524J1_485_Protocol::isPackedJointRegister(const std::string& reg_name) const {
    return getDefaultReadLength(reg_name) == kJointBytes;
}

std::vector<int> RH524J1_485_Protocol::permuteUserToHardware(const std::vector<int>& user_values) const {
    std::vector<int> hardware_values = user_values;
    if (hardware_values.size() < kJointCount) {
        hardware_values.resize(kJointCount, 0);
    }
    swapThumbDipMcp(hardware_values);
    return hardware_values;
}

std::vector<int> RH524J1_485_Protocol::permuteHardwareToUser(const std::vector<int>& hardware_values) const {
    std::vector<int> user_values = hardware_values;
    if (user_values.size() < kJointCount) {
        user_values.resize(kJointCount, 0);
    }
    swapThumbDipMcp(user_values);
    return user_values;
}

void RH524J1_485_Protocol::swapThumbDipMcp(std::vector<int>& values) const {
    if (values.size() > kThumbMcpUserIndex) {
        std::swap(values[kThumbDipUserIndex], values[kThumbMcpUserIndex]);
    }
}

int RH524J1_485_Protocol::userAngleToHardware(int drive_id, int user_angle) const {
    return clampAngle(drive_id, user_angle);
}

int RH524J1_485_Protocol::hardwareAngleToUser(int drive_id, int hardware_angle) const {
    (void)drive_id;
    return hardware_angle;
}

void RH524J1_485_Protocol::clampAngleSetValues(std::vector<int>& values) const {
    values = userAnglesToHardware(values);
}

std::vector<int> RH524J1_485_Protocol::hardwareAnglesToUser(const std::vector<int>& hardware_values) const {
    std::vector<int> converted = hardware_values;
    for (size_t i = 0; i < converted.size(); ++i) {
        converted[i] = hardwareAngleToUser(static_cast<int>(i + 1), converted[i]);
    }
    return permuteHardwareToUser(converted);
}

std::vector<int> RH524J1_485_Protocol::userAnglesToHardware(const std::vector<int>& user_values) const {
    std::vector<int> hardware_values = permuteUserToHardware(user_values);
    for (size_t i = 0; i < hardware_values.size(); ++i) {
        hardware_values[i] = userAngleToHardware(static_cast<int>(i + 1), hardware_values[i]);
    }
    return hardware_values;
}

std::vector<int> RH524J1_485_Protocol::prepareAngleSetHardwareValues(Device device,
                                                                     const std::vector<int>& user_values) {
    std::vector<int> user_padded = user_values;
    if (user_padded.size() > kJointCount) {
        throw std::runtime_error("angleSet 最多只能写入 24 个关节值");
    }
    if (user_padded.size() < kJointCount) {
        user_padded.resize(kJointCount, -1);
    }

    const bool need_merge =
        std::any_of(user_padded.begin(), user_padded.end(), [](int v) { return v == kAngleKeepSentinel; });

    std::vector<int> merged_user(kJointCount, 0);
    if (need_merge) {
        RingBuffer rb(256);
        RegisterReadResult current = readRegisterHardware(device, rb, "angleSet", kJointBytes);
        if (current.error != IoError::Ok) {
            rb.clear();
            current = readRegisterHardware(device, rb, "angleAct", kJointBytes);
        }
        if (current.error != IoError::Ok || current.values.size() < kJointCount) {
            throw std::runtime_error("angleSet 含 -1 占位但读取当前角度失败，已中止写入");
        }
        merged_user = hardwareAnglesToUser(current.values);
        if (merged_user.size() < kJointCount) {
            merged_user.resize(kJointCount, 0);
        }
    }

    for (size_t i = 0; i < kJointCount; ++i) {
        if (isAngleExplicitValue(user_padded[i])) {
            merged_user[i] = user_padded[i];
        }
    }
    return userAnglesToHardware(merged_user);
}

std::vector<uint8_t> RH524J1_485_Protocol::buildWriteCommand(int address, const std::vector<int>& values) {
    if (values.size() > kJointCount) {
        throw std::runtime_error("超过允许的值数量，最多只能写入24个值");
    }

    std::vector<uint8_t> cmd = {
        0xEB,
        0x90,
        device_id_,
        static_cast<uint8_t>(values.size() * 2 + 3),
        0x12,
        static_cast<uint8_t>(address & 0xFF),
        static_cast<uint8_t>((address >> 8) & 0xFF),
    };

    for (int value : values) {
        cmd.push_back(static_cast<uint8_t>(value & 0xFF));
        cmd.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
    }

    uint8_t checksum = 0;
    for (size_t i = 2; i < cmd.size(); ++i) {
        checksum += cmd[i];
    }
    cmd.push_back(checksum);
    return cmd;
}

std::pair<bool, TouchDataResult> RH524J1_485_Protocol::parseTouchData(RingBuffer& ringBuffer, int version) {
    (void)ringBuffer;
    (void)version;
    auto logger = getLogger();
    logger->warn("[RH524J1] 当前机型无触觉寄存器，parseTouchData 返回 NotSupported");
    return {false, {}};
}

IoError RH524J1_485_Protocol::writeRegister(Device device, const std::string& reg_name, const std::vector<int>& values) {
    std::ostringstream oss;
    auto logger = getLogger();

    auto it = REGISTER_MAP.find(reg_name);
    if (it == REGISTER_MAP.end()) {
        logger->error("[RH524J1] 未知寄存器名: {}", reg_name);
        return IoError::UnknownRegister;
    }
    int address = it->second;

    std::vector<int> write_values = values;
    if (reg_name == "angleSet") {
        try {
            std::vector<int> user_padded = values;
            if (user_padded.size() > kJointCount) {
                throw std::runtime_error("angleSet 最多只能写入 24 个关节值");
            }
            if (user_padded.size() < kJointCount) {
                user_padded.resize(kJointCount, -1);
            }

            size_t explicit_count = 0;
            size_t explicit_index = 0;
            for (size_t i = 0; i < kJointCount; ++i) {
                if (isAngleExplicitValue(user_padded[i])) {
                    ++explicit_count;
                    explicit_index = i;
                }
            }

            // 实机验证：仅写 1 路 angleSet（短帧 len=0x05）可动；整包 24 路易与腱绳冲突
            if (explicit_count == 1) {
                const size_t reg_index = userIndexToAngleSetRegisterIndex(explicit_index);
                const int drive_id = static_cast<int>(explicit_index + 1);
                write_values = {userAngleToHardware(drive_id, user_padded[explicit_index])};
                address += static_cast<int>(reg_index * 2);
                logger->info("[RH524J1] angleSet 单关节写入 ID{} 下标{} 地址0x{:04X} 值={}",
                             drive_id, explicit_index, address, write_values[0]);
            } else {
                write_values = prepareAngleSetHardwareValues(device, values);
            }
        } catch (const std::exception& e) {
            logger->error("[RH524J1] angleSet 参数非法: {}", e.what());
            return IoError::InvalidArgument;
        }
    } else if (isPackedJointRegister(reg_name) && values.size() == kJointCount) {
        write_values = permuteUserToHardware(values);
    }

    std::vector<uint8_t> cmd;
    try {
        cmd = buildWriteCommand(address, write_values);
    } catch (const std::exception& e) {
        logger->error("[RH524J1] 写入寄存器参数非法：{}, 异常: {}", reg_name, e.what());
        return IoError::InvalidArgument;
    }

    try {
        logger->debug("[RH524J1][写入命令] 寄存器: {}, 地址: 0x{:04X}, 命令: {}", reg_name, address,
                      formatBytesToHex(cmd.data(), cmd.size()));

        device->write(cmd);

        auto writeResponse = readResponseWithLoop(device, 50, 9, false);
        if (writeResponse.empty()) {
            logger->error("[RH524J1] 写入寄存器失败：{} (无响应)", reg_name);
            return IoError::Timeout;
        }
        logger->debug("[RH524J1][写响应] 寄存器: {}, 响应: {}", reg_name,
                      formatBytesToHex(writeResponse.data(), writeResponse.size()));

        RingBuffer tempBuffer(256);
        tempBuffer.push(writeResponse.data(), writeResponse.size());
        auto [parseSuccess, _] = parseResponse(tempBuffer);
        if (!parseSuccess) {
            logger->error("[RH524J1] 写入寄存器响应解析失败：{}", reg_name);
            return IoError::BadResponse;
        }

        oss << "写入" << reg_name << ":(";
        for (size_t i = 0; i < write_values.size(); ++i) {
            oss << write_values[i];
            if (i != write_values.size() - 1)
                oss << " ";
        }
        oss << ")";
        logger->info(oss.str());

        if (reg_name == "defaultSpeedSet" || reg_name == "defaultForceSet") {
            logger->debug("[RH524J1][特殊处理] {} 写入完成，接着写 save=1", reg_name);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));

            auto saveCmd = buildWriteCommand(REGISTER_MAP.at("save"), {1});
            device->write(saveCmd);
            auto saveResponse = readResponseWithLoop(device, 50, 9, false);
            if (saveResponse.empty()) {
                logger->warn("[RH524J1][写响应] 寄存器: save, 响应为空");
                return IoError::Timeout;
            }
            RingBuffer saveBuf(256);
            saveBuf.push(saveResponse.data(), saveResponse.size());
            auto [saveOk, unused] = parseResponse(saveBuf);
            (void)unused;
            if (!saveOk) {
                logger->warn("[RH524J1][写响应] 寄存器: save, 响应格式验证失败");
                return IoError::BadResponse;
            }
            logger->info("写入save:(1)");
        }

        return IoError::Ok;
    } catch (const std::exception& e) {
        logger->error("[RH524J1] 写入寄存器失败：{}, 异常: {}", reg_name, e.what());
        return IoError::DeviceError;
    } catch (...) {
        logger->error("[RH524J1] 写入寄存器失败：{}", reg_name);
        return IoError::DeviceError;
    }
}

RegisterReadResult RH524J1_485_Protocol::readRegisterHardware(Device device, RingBuffer& ringBuffer,
                                                              const std::string& reg_name, size_t length) {
    std::ostringstream oss;
    auto logger = getLogger();

    ringBuffer.clear();

    auto it = REGISTER_MAP.find(reg_name);
    if (it == REGISTER_MAP.end()) {
        logger->error("[RH524J1] 未知寄存器名: {}", reg_name);
        return {IoError::UnknownRegister, {}};
    }
    int address = it->second;

    if (length == 0) {
        length = getDefaultReadLength(reg_name);
        logger->debug("[RH524J1][动态长度] 寄存器: {}, 自动确定读取长度: {} 字节", reg_name, length);
    }

    try {
        auto cmd = buildReadCommand(address, length);
        logger->debug("[RH524J1][读取命令] 寄存器: {}, 地址: 0x{:04X}, 长度: {}, 命令: {}", reg_name, address, length,
                      formatBytesToHex(cmd.data(), cmd.size()));

        device->write(cmd);
        auto response = readResponseWithLoop(device, 50, 8, true);

        if (response.empty()) {
            logger->error("[RH524J1] 读取寄存器失败：{} (无响应)", reg_name);
            return {IoError::Timeout, {}};
        }
        logger->debug("[RH524J1][原始响应] 寄存器: {}, 响应: {}", reg_name,
                      formatBytesToHex(response.data(), response.size()));

        ringBuffer.push(response.data(), response.size());
        auto result = parseResponse(ringBuffer);
        if (result.first) {
            oss << "读取" << reg_name << ":(";
            for (size_t i = 0; i < result.second.size(); ++i) {
                oss << result.second[i];
                if (i != result.second.size() - 1)
                    oss << " ";
            }
            oss << ")";
            logger->info(oss.str());
            return {IoError::Ok, std::move(result.second)};
        }

        logger->error("[RH524J1] 读取寄存器失败：{}", reg_name);
        return {IoError::BadResponse, {}};
    } catch (...) {
        logger->error("[RH524J1] 读取寄存器异常：{}", reg_name);
        return {IoError::DeviceError, {}};
    }
}

RegisterReadResult RH524J1_485_Protocol::readRegister(Device device, RingBuffer& ringBuffer, const std::string& reg_name,
                                                      size_t length) {
    auto result = readRegisterHardware(device, ringBuffer, reg_name, length);
    if (result.error == IoError::Ok && isPackedJointRegister(reg_name) && result.values.size() == kJointCount) {
        if (isAngleJointRegister(reg_name)) {
            result.values = hardwareAnglesToUser(result.values);
        } else {
            result.values = permuteHardwareToUser(result.values);
        }
    }
    return result;
}

TouchReadResult RH524J1_485_Protocol::readTouchData(Device device, RingBuffer& ringBuffer, int version) {
    (void)device;
    (void)ringBuffer;
    (void)version;
    auto logger = getLogger();
    logger->warn("[RH524J1] 当前机型无触觉硬件，readTouchData 返回 NotSupported");
    return {IoError::NotSupported, {}};
}

REGISTER_PROTOCOL("RH524J1_485", RH524J1_485_Protocol);

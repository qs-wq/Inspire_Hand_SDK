#include "config_loader.hpp"
#include "device_manager.hpp"
#include "logger_manager.hpp"
#include "protocol.hpp"
#include "serial_port.hpp"
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

// 全局运行标志，用于控制所有线程退出
std::atomic<bool> g_running(true);

/**
 * @brief 信号处理函数，用于优雅退出
 * @param signal 信号编号
 */
void signalHandler(int signal) {
    auto logger = getLogger();
    logger->info("接收到退出信号({})，正在停止所有设备控制线程...", signal);
    g_running = false;
}

/**
 * @brief 设备控制线程函数
 * @param deviceName 设备名称
 * @param port 串口端口
 * @param device 设备对象
 * @param protocol 协议对象
 *
 * 每个设备在独立线程中运行，执行以下操作：
 * 1. 写入角度设定值（angleSet）
 * 2. 读取当前角度值（angleAct）
 * 3. 尝试读取触觉数据（touchAct，RH56DFX 预期不支持）
 * 控制频率：40Hz（25ms周期）
 */
void deviceControlThread(const std::string& deviceName, const std::string& port,
                         std::shared_ptr<SerialPortBase> device, std::shared_ptr<Protocol> protocol) {
    // 设置当前线程设备名（用于日志前缀）
    LoggerManager::setThreadDeviceName(deviceName);

    auto logger = getLogger();
    logger->info("设备控制线程启动: {} ({})", deviceName, port);

    // 每个设备使用独立 RingBuffer，避免数据竞争
    RingBuffer ringBuffer(1024);

    // 初始化角度数组（6关节）
    std::vector<int> angles = {1700, 1700, 1700, 1700, 1350, 1700};
    const int min_angle = 900;
    const int max_angle = 1700;
    const int step = -10;

    // 统计计数器
    int cycle_count = 0;

    // 控制循环：40Hz，25ms周期
    while (g_running.load()) {
        try {
            // 更新角度数组（前4个手指循环变化）
            for (int i = 0; i < static_cast<int>(angles.size()) - 2; ++i) {
                angles[i] += step;
                if (angles[i] < min_angle) {
                    angles[i] = max_angle;
                }
            }

            // 1. 写入角度设定值（angleSet）
            IoError write_err = protocol->writeRegister(device, "angleSet", angles);
            if (!isOk(write_err) && cycle_count % 100 == 0) {
                logger->warn("[{}] angleSet 写入失败: {}", deviceName, toString(write_err));
            }

            // 2. 读取当前角度值（angleAct）
            auto readResult = protocol->readRegister(device, ringBuffer, "angleAct", 0);
            if (readResult.ok() && cycle_count % 100 == 0) {
                logger->info("[{}] angleAct 读取成功，{} 个值", deviceName, readResult.values.size());
            }

            // 3. 读取触觉数据（touchAct）；RH56DFX 无触觉硬件，预期返回 NotSupported
            auto touchResult = protocol->readTouchData(device, ringBuffer, 1);
            if (touchResult.error == IoError::NotSupported && cycle_count % 100 == 0) {
                logger->debug("[{}] touchAct 不可用（机型无触觉传感器）", deviceName);
            } else if (!touchResult.ok() && cycle_count % 100 == 0) {
                logger->warn("[{}] touchAct 读取失败: {}", deviceName, toString(touchResult.error));
            }

            // 每100个周期（约2.5秒）输出一次状态信息
            cycle_count++;
            if (cycle_count % 100 == 0) {
                logger->info("[{}] 运行正常，已完成 {} 个控制周期", deviceName, cycle_count);
            }

            // 控制循环频率：40Hz，25ms周期
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        } catch (const std::exception& e) {
            logger->error("[{}] 控制循环异常: {}", deviceName, e.what());
            // 发生异常时稍作延迟后继续
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    logger->info("设备控制线程退出: {} ({})", deviceName, port);
}

int main() {
    constexpr const char* kConfigPath = "config/device_protocol_rh56dfx_example.yaml";

    try {
        // 注册信号处理函数（支持Ctrl+C优雅退出）
        std::signal(SIGINT, signalHandler);
        std::signal(SIGTERM, signalHandler);

        // 配置日志
        ConfigLoader::configureLogging(kConfigPath);
        auto logger = getLogger();
        logger->info("=== RH56DFX 多设备并行控制系统启动 ===");

        // 创建设备管理器
        DeviceManager device_manager;

        // 加载设备配置，包含设备名称、端口、波特率和Hand_ID
        auto deviceConfig = ConfigLoader::loadDeviceConfig(kConfigPath);
        logger->info("已加载 {} 个设备配置", deviceConfig.size());

        // 为每个设备创建独立协议实例，并设置对应 Hand_ID
        std::unordered_map<std::string, std::shared_ptr<Protocol>> deviceProtocols;

        // 添加所有设备（端口、波特率和协议）
        for (const auto& [port, deviceInfo] : deviceConfig) {
            int baudRate = deviceInfo.baudrate;

            // 为每个设备创建独立协议对象
            auto protocol = ConfigLoader::createProtocolFromConfig(kConfigPath);
            protocol->setDeviceId(static_cast<uint8_t>(deviceInfo.hand_id));
            protocol->setDeviceName(deviceInfo.name);
            logger->info("为设备 {} 设置 Hand_ID = {}", deviceInfo.name, deviceInfo.hand_id);

            device_manager.addDevice(port, protocol, baudRate);
            deviceProtocols[port] = protocol;

            logger->info("设备已添加: {} ({}, 波特率: {}, Hand_ID: {})", deviceInfo.name, port, baudRate,
                         deviceInfo.hand_id);
        }

        // 存储所有线程对象
        std::vector<std::thread> deviceThreads;
        std::vector<std::string> threadDeviceNames;

        // 为每个设备创建独立控制线程
        for (const auto& [port, deviceInfo] : deviceConfig) {
            auto device = device_manager.getDevice(port);
            if (!device) {
                logger->error("无法获取设备对象: {} ({})", deviceInfo.name, port);
                continue;
            }

            auto protocol = deviceProtocols[port];

            // 创建设备控制线程
            deviceThreads.emplace_back(deviceControlThread, deviceInfo.name, port, device, protocol);
            threadDeviceNames.push_back(deviceInfo.name);
            logger->info("设备控制线程已创建: {} ({})", deviceInfo.name, port);

            // 稍微延迟，避免多个设备同时初始化串口造成冲突
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        logger->info("所有设备控制线程已启动，系统运行中... (按Ctrl+C退出)");

        // 等待所有线程结束
        for (size_t i = 0; i < deviceThreads.size(); ++i) {
            if (deviceThreads[i].joinable()) {
                deviceThreads[i].join();
                logger->info("设备控制线程已结束: {}", threadDeviceNames[i]);
            }
        }

        logger->info("=== RH56DFX 多设备并行控制系统退出 ===");
        return 0;
    } catch (const std::exception& e) {
        auto logger = getLogger();
        logger->error("程序异常退出: {}", e.what());
        g_running = false; // 确保所有线程退出
        return 1;
    }
}

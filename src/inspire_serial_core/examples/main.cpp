#include <iostream>
#include <thread>
#include <chrono>
#include <stdexcept>
#include <unordered_map>
#include <memory>
#include <vector>
#include <string>
#include <atomic>
#include <csignal>
#include <sstream>
#include "protocol.hpp"
#include "serial_port.hpp"
#include "device_manager.hpp"
#include "config_loader.hpp"
#include "logger_manager.hpp"

// 全局运行标志，用于控制所有线程的退出
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
 * 3. 读取触觉数据（touchAct）
 * 控制频率：50Hz（25ms周期）
 */
void deviceControlThread(
    const std::string& deviceName,
    const std::string& port,
    std::shared_ptr<SerialPortBase> device,
    std::shared_ptr<Protocol> protocol
) {
    // 设置当前线程的设备名（用于日志前缀）
    LoggerManager::setThreadDeviceName(deviceName);
    
    auto logger = getLogger();
    logger->info("设备控制线程启动: {} ({})", deviceName, port);
    
    // 每个设备使用独立的RingBuffer，避免数据竞争
    RingBuffer ringBuffer(1024);
    
    // 初始化角度数组
    std::vector<int> angles = {1800, 1800, 1800, 0, 1800, 0, 1900, 1900, 1900, 1900, 1750, 1600, 2080};
    
    // 定义角度上下限和步进
    const int min_angle = 965;
    const int max_angle = 1800;
    const int step = -10;
    
    // 统计计数器
    int cycle_count = 0;
    
    // 控制循环：40Hz，25ms周期
    while (g_running.load()) {
        try {
            // 更新角度数组（前4个手指循环变化）
            for (int i = 0; i < static_cast<int>(angles.size()) - 10; ++i) {
                angles[i] += step;
                if (angles[i] < min_angle) {
                    angles[i] = max_angle;
                }
            }
            
            IoError writeSuccess = protocol->writeRegister(device, "angleSet", angles);
            (void)writeSuccess;
            
            // 2. 读取当前角度值（angleAct）
            auto readResult = protocol->readRegister(device, ringBuffer, "angleAct", 26);
            (void)readResult;
            
            // 3. 读取触觉数据（touchAct）
            auto touchResult = protocol->readTouchData(device, ringBuffer, 1);
            (void)touchResult;
            
            // 每100个周期（2秒）输出一次状态信息
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
    try {
        // 注册信号处理函数（支持Ctrl+C优雅退出）
        std::signal(SIGINT, signalHandler);
        std::signal(SIGTERM, signalHandler);
        
        // 配置日志
        ConfigLoader::configureLogging("config/device_protocol_config.yaml");
        auto logger = getLogger();
        logger->info("=== 多设备并行控制系统启动 ===");
    
        // 创建设备管理器
        DeviceManager device_manager;
        
        // 加载设备配置，包含设备名称、端口、波特率和Hand_ID
        auto deviceConfig = ConfigLoader::loadDeviceConfig("config/device_protocol_config.yaml");
        logger->info("已加载 {} 个设备配置", deviceConfig.size());

        // 为每个设备创建独立的协议实例，并设置对应的Hand_ID
        std::unordered_map<std::string, std::shared_ptr<Protocol>> deviceProtocols;

        // 添加所有设备（端口、波特率和协议）
        for (const auto& [port, deviceInfo] : deviceConfig) {
            int baudRate = deviceInfo.baudrate;

            // 为每个设备创建独立协议对象
            std::shared_ptr<Protocol> protocol = ConfigLoader::createProtocolFromConfig("config/device_protocol_config.yaml");

            // 设置设备ID和设备名称（用于日志前缀）
            protocol->setDeviceId(static_cast<uint8_t>(deviceInfo.hand_id));
            protocol->setDeviceName(deviceInfo.name);
            logger->info("为设备 {} 设置 Hand_ID = {}", deviceInfo.name, deviceInfo.hand_id);

            device_manager.addDevice(port, protocol, baudRate);
            deviceProtocols[port] = protocol;

            logger->info("设备已添加: {} ({}, 波特率: {}, Hand_ID: {})",
                         deviceInfo.name, port, baudRate, deviceInfo.hand_id);
        }

        // 存储所有线程对象
        std::vector<std::thread> deviceThreads;
        std::vector<std::string> threadDeviceNames;
        
        // 为每个设备创建独立的控制线程
        for (const auto& [port, deviceInfo] : deviceConfig) {
            auto device = device_manager.getDevice(port);
            if (!device) {
                logger->error("无法获取设备对象: {} ({})", deviceInfo.name, port);
                continue;
            }
            auto protocol = deviceProtocols[port];
            
            // 创建设备控制线程
            deviceThreads.emplace_back(
                deviceControlThread,
                deviceInfo.name,
                port,
                device,
                protocol
            );
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
        
        logger->info("=== 多设备并行控制系统退出 ===");
        return 0;
        
    } catch (const std::exception& e) {
        auto logger = getLogger();
        logger->error("程序异常退出: {}", e.what());
        g_running = false;  // 确保所有线程退出
        return 1;
    }
}

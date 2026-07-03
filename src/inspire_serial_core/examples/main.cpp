#include "config_loader.hpp"
#include "device_manager.hpp"
#include "logger_manager.hpp"
#include "protocol.hpp"
#include "serial_port.hpp"
#include <atomic>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <csignal>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

// 全局运行标志，用于控制所有线程的退出
std::atomic<bool> g_running(true);

/** 控制模式：演示动画 / 固定角度 / 只读状态 */
enum class ControlMode {
    Demo,       // 自动开合演示（默认）
    Fixed,      // 保持 --angles 或 --angles-file 指定的角度
    ReadOnly,   // 只读 angleAct，不写 angleSet
};

struct ControlOptions {
    ControlMode mode = ControlMode::Demo;
    std::vector<int> fixed_angles;
};

namespace {

std::string anglesToText(const std::vector<int>& values) {
    std::ostringstream oss;
    for (size_t i = 0; i < values.size(); ++i) {
        oss << values[i];
        if (i + 1 < values.size()) {
            oss << ' ';
        }
    }
    return oss.str();
}

std::vector<int> parseAngleList(const std::string& text) {
    std::vector<int> values;
    std::string token;
    std::stringstream ss(text);
    while (std::getline(ss, token, ',')) {
        if (token.empty()) {
            continue;
        }
        try {
            values.push_back(std::stoi(token));
        } catch (...) {
            throw std::invalid_argument("角度值非法: " + token);
        }
    }
    if (values.empty()) {
        throw std::invalid_argument("角度列表为空");
    }
    return values;
}

std::vector<int> loadAnglesFromFile(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::invalid_argument("无法打开角度文件: " + path);
    }
    std::string line;
    while (std::getline(file, line)) {
        if (line.find_first_not_of(" \t\r\n") == std::string::npos) {
            continue;
        }
        for (char& ch : line) {
            if (ch == ' ' || ch == '\t') {
                ch = ',';
            }
        }
        return parseAngleList(line);
    }
    throw std::invalid_argument("角度文件为空: " + path);
}

}  // namespace

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
void deviceControlThread(const std::string& deviceName, const std::string& port, std::shared_ptr<SerialPortBase> device,
                         std::shared_ptr<Protocol> protocol, ControlOptions options) {
    // 设置当前线程的设备名（用于日志前缀）
    LoggerManager::setThreadDeviceName(deviceName);

    auto logger = getLogger();
    logger->info("设备控制线程启动: {} ({})", deviceName, port);

    // 每个设备使用独立的RingBuffer，避免数据竞争
    RingBuffer ringBuffer(1024);

    std::vector<int> angles = options.fixed_angles.empty()
                                  ? std::vector<int>{1800, 1800, 1800, 1800, 1350, 1800}
                                  : options.fixed_angles;
    bool joint_count_initialized = !options.fixed_angles.empty();

    const int min_angle = 965;
    const int max_angle = 1800;
    const int step = -10;

    if (options.mode == ControlMode::Fixed) {
        logger->info("[{}] 固定角度模式: ({})", deviceName, anglesToText(angles));
    } else if (options.mode == ControlMode::ReadOnly) {
        logger->info("[{}] 只读模式：仅读取 angleAct，不写入 angleSet", deviceName);
    } else {
        logger->info("[{}] 演示模式：前几个关节自动开合", deviceName);
    }

    int cycle_count = 0;

    while (g_running.load()) {
        try {
            auto readResult = protocol->readRegister(device, ringBuffer, "angleAct", 0);
            if (!joint_count_initialized && readResult.ok() && !readResult.values.empty()) {
                const size_t joint_count = readResult.values.size();
                if (options.mode == ControlMode::Fixed &&
                    options.fixed_angles.size() != joint_count) {
                    throw std::runtime_error(
                        "角度数量(" + std::to_string(options.fixed_angles.size()) +
                        ")与设备关节数(" + std::to_string(joint_count) + ")不一致");
                }
                if (options.mode == ControlMode::Demo) {
                    angles.assign(joint_count, 1800);
                    if (joint_count >= 4) {
                        angles[3] = 0;
                    }
                }
                joint_count_initialized = true;
                logger->info("[{}] 检测到 {} 个关节", deviceName, joint_count);
            }

            if (readResult.ok() && cycle_count % 20 == 0) {
                logger->info("[{}] angleAct:({})", deviceName, anglesToText(readResult.values));
            }

            if (options.mode == ControlMode::Demo) {
                const int animate_count = std::max(0, static_cast<int>(angles.size()) - 2);
                for (int i = 0; i < animate_count; ++i) {
                    angles[i] += step;
                    if (angles[i] < min_angle) {
                        angles[i] = max_angle;
                    }
                }
            }

            if (options.mode != ControlMode::ReadOnly) {
                IoError write_err = protocol->writeRegister(device, "angleSet", angles);
                if (!isOk(write_err) && cycle_count % 20 == 0) {
                    logger->warn("[{}] angleSet 写入失败: {}", deviceName, toString(write_err));
                }
            }

            if (options.mode != ControlMode::ReadOnly) {
                auto touchResult = protocol->readTouchData(device, ringBuffer, 1);
                (void)touchResult;
            }

            cycle_count++;
            if (cycle_count % 100 == 0) {
                logger->info("[{}] 运行正常，已完成 {} 个控制周期", deviceName, cycle_count);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(25));

        } catch (const std::exception& e) {
            logger->error("[{}] 控制循环异常: {}", deviceName, e.what());
            // 发生异常时稍作延迟后继续
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    logger->info("设备控制线程退出: {} ({})", deviceName, port);
}

namespace {

constexpr const char* kDefaultConfigPath = "config/device_protocol_config.yaml";

struct RunOptions {
    std::string config_path = kDefaultConfigPath;
    ControlOptions control;
};

void printUsage(const char* program) {
    std::cerr << "用法: " << program << " [选项]\n"
              << "\n"
              << "选项:\n"
              << "  --config|-c <yaml>       设备协议配置（默认: " << kDefaultConfigPath << ")\n"
              << "  --angles <v1,v2,...>   固定角度，逗号分隔（RH56DFX/RH56F1=6个，RH5DG2=13个）\n"
              << "  --angles-file <path>     从文件读取角度（一行，逗号或空格分隔）\n"
              << "  --demo                   自动开合演示（未指定 --angles 时默认）\n"
              << "  --read-only              只读 angleAct，不写 angleSet\n"
              << "  --help|-h                显示帮助\n"
              << "\n"
              << "示例:\n"
              << "  # 自动演示\n"
              << "  " << program << " --config config/device_protocol_rh56dfx_example.yaml\n"
              << "\n"
              << "  # 握拳（RH56DFX 六个关节）\n"
              << "  " << program << " -c config/device_protocol_rh56dfx_example.yaml \\\n"
              << "    --angles 1000,1000,1000,1000,1200,1800\n"
              << "\n"
              << "  # 只读当前角度\n"
              << "  " << program << " -c config/device_protocol_rh56dfx_example.yaml --read-only\n";
}

RunOptions parseRunOptions(int argc, char* argv[]) {
    RunOptions opts;
    bool demo_requested = false;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            std::exit(0);
        }
        if (arg == "--config" || arg == "-c") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("缺少 --config 参数值");
            }
            opts.config_path = argv[++i];
            continue;
        }
        if (arg == "--angles") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("缺少 --angles 参数值");
            }
            opts.control.fixed_angles = parseAngleList(argv[++i]);
            opts.control.mode = ControlMode::Fixed;
            continue;
        }
        if (arg == "--angles-file") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("缺少 --angles-file 参数值");
            }
            opts.control.fixed_angles = loadAnglesFromFile(argv[++i]);
            opts.control.mode = ControlMode::Fixed;
            continue;
        }
        if (arg == "--demo") {
            demo_requested = true;
            opts.control.mode = ControlMode::Demo;
            opts.control.fixed_angles.clear();
            continue;
        }
        if (arg == "--read-only") {
            opts.control.mode = ControlMode::ReadOnly;
            opts.control.fixed_angles.clear();
            continue;
        }
        if (!arg.empty() && arg[0] != '-') {
            opts.config_path = arg;
            continue;
        }
        throw std::invalid_argument("未知参数: " + arg);
    }

    if (demo_requested && opts.control.mode == ControlMode::Fixed) {
        throw std::invalid_argument("不能同时使用 --demo 与 --angles/--angles-file");
    }
    return opts;
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        const RunOptions run_opts = parseRunOptions(argc, argv);
        const std::string& config_path = run_opts.config_path;
        const ControlOptions control_opts = run_opts.control;

        // 注册信号处理函数（支持Ctrl+C优雅退出）
        std::signal(SIGINT, signalHandler);
        std::signal(SIGTERM, signalHandler);

        // 配置日志
        ConfigLoader::configureLogging(config_path);
        auto logger = getLogger();
        logger->info("=== 多设备并行控制系统启动 ===");
        logger->info("配置文件: {}", config_path);

        // 创建设备管理器
        DeviceManager device_manager;

        // 加载设备配置，包含设备名称、端口、波特率和Hand_ID
        auto deviceConfig = ConfigLoader::loadDeviceConfig(config_path);
        logger->info("已加载 {} 个设备配置", deviceConfig.size());

        // 为每个设备创建独立的协议实例，并设置对应的Hand_ID
        std::unordered_map<std::string, std::shared_ptr<Protocol>> deviceProtocols;

        // 添加所有设备（端口、波特率和协议）
        for (const auto& [port, deviceInfo] : deviceConfig) {
            int baudRate = deviceInfo.baudrate;

            // 为每个设备创建独立协议对象
            std::shared_ptr<Protocol> protocol = ConfigLoader::createProtocolFromConfig(config_path);

            // 设置设备ID和设备名称（用于日志前缀）
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

        // 为每个设备创建独立的控制线程
        for (const auto& [port, deviceInfo] : deviceConfig) {
            auto device = device_manager.getDevice(port);
            if (!device) {
                logger->error("无法获取设备对象: {} ({})", deviceInfo.name, port);
                continue;
            }
            auto protocol = deviceProtocols[port];

            // 创建设备控制线程
            deviceThreads.emplace_back(deviceControlThread, deviceInfo.name, port, device, protocol,
                                       control_opts);
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
        g_running = false; // 确保所有线程退出
        return 1;
    }
}

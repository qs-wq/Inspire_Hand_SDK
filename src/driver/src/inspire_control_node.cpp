#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <vector>
#include <csignal>
#include <atomic>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "register_controller.hpp"
#include "inspire_controller_factory.hpp"
#include "config_loader.hpp"
#include "device_manager.hpp"
#include "logger_manager.hpp"

std::atomic<bool> g_running(true);

void signalHandler(int signal) {
    auto logger = getLogger();
    logger->info("接收到退出信号({})，正在停止ROS2节点...", signal);
    g_running = false;
    rclcpp::shutdown();
}

int main(int argc, char** argv) {
    try {
        rclcpp::init(argc, argv);

        std::signal(SIGINT, signalHandler);
        std::signal(SIGTERM, signalHandler);

        std::string device_config_path;
        std::string controller_config_path;
        std::string device_name;

        device_config_path =
            ament_index_cpp::get_package_share_directory("inspire_control_ros2") +
            "/config/device_protocol_config.yaml";
        controller_config_path =
            ament_index_cpp::get_package_share_directory("inspire_control_ros2") +
            "/config/ros2_controller_config.yaml";

        for (int i = 1; i < argc; ++i) {
            if (i + 1 < argc) {
                if (std::string(argv[i]) == "--device-config") {
                    device_config_path = argv[++i];
                } else if (std::string(argv[i]) == "--controller-config") {
                    controller_config_path = argv[++i];
                } else if (std::string(argv[i]) == "--device") {
                    device_name = argv[++i];
                }
            }
        }

        ConfigLoader::configureLogging(device_config_path);
        auto logger = getLogger();
        logger->info("=== Inspire 灵巧手 ROS2 节点启动 ===");
        logger->info("设备配置: {}", device_config_path);
        logger->info("控制器配置: {}", controller_config_path);
        if (!device_name.empty()) {
            logger->info("单设备模式: {}", device_name);
        }

        DeviceManager device_manager;

        auto deviceConfig = ConfigLoader::loadDeviceConfig(device_config_path);
        logger->info("已加载 {} 个设备配置", deviceConfig.size());

        std::unordered_map<std::string, std::shared_ptr<Protocol>> device_protocols;

        for (const auto& [port, deviceInfo] : deviceConfig) {
            if (!device_name.empty() && deviceInfo.name != device_name) {
                continue;
            }

            int baudRate = deviceInfo.baudrate;

            std::shared_ptr<Protocol> protocol = ConfigLoader::createProtocolFromConfig(device_config_path);

            protocol->setDeviceId(static_cast<uint8_t>(deviceInfo.hand_id));
            protocol->setDeviceName(deviceInfo.name);
            logger->info("为设备 {} 设置 Hand_ID = {}", deviceInfo.name, deviceInfo.hand_id);

            device_manager.addDevice(port, protocol, baudRate);
            device_protocols[port] = protocol;

            logger->info("设备已添加: {} ({}, 波特率: {}, Hand_ID: {})",
                         deviceInfo.name, port, baudRate, deviceInfo.hand_id);
        }

        std::vector<std::shared_ptr<RegisterController>> controllers;

        if (!device_name.empty()) {
            logger->info("单设备模式：启动设备节点 {}", device_name);

            auto all_configs = InspireControllerFactory::loadDeviceNodeConfigs(
                controller_config_path, device_config_path);
            DeviceNodeConfig target_config;
            bool found = false;

            for (const auto& config : all_configs) {
                if (config.device_name == device_name) {
                    target_config = config;
                    found = true;
                    break;
                }
            }

            if (!found) {
                throw std::runtime_error("未找到设备配置: " + device_name);
            }

            auto controller = InspireControllerFactory::createDeviceNode(
                target_config,
                device_manager,
                device_protocols,
                device_config_path);
            controllers.push_back(controller);
        } else {
            logger->info("多设备模式：启动所有设备节点");
            controllers = InspireControllerFactory::createAllDeviceNodes(
                controller_config_path,
                device_manager,
                device_protocols,
                device_config_path);
        }

        logger->info("共创建 {} 个设备节点", controllers.size());

        for (auto& controller : controllers) {
            controller->start();
        }

        logger->info("所有设备节点已启动，ROS2节点运行中... (按Ctrl+C退出)");

        rclcpp::executors::MultiThreadedExecutor executor;
        for (auto& controller : controllers) {
            executor.add_node(controller);
        }

        while (rclcpp::ok() && g_running.load()) {
            executor.spin_some(std::chrono::milliseconds(100));
        }

        logger->info("正在停止所有设备节点...");
        for (auto& controller : controllers) {
            controller->stop();
        }

        logger->info("=== Inspire 灵巧手 ROS2 节点退出 ===");

        rclcpp::shutdown();
        return 0;

    } catch (const std::exception& e) {
        auto logger = getLogger();
        logger->error("程序异常退出: {}", e.what());
        rclcpp::shutdown();
        return 1;
    }
}

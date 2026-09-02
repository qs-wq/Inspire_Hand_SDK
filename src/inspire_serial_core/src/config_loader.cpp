#include "config_loader.hpp"
#include "logger_manager.hpp" // 使用日志管理器
#include "protocol.hpp"
#include "protocol_factory.hpp" // 使用协议工厂
#include <stdexcept>
#include <yaml-cpp/yaml.h>

YAML::Node ConfigLoader::loadYAMLConfig(const std::string& path) { return YAML::LoadFile(path); }

std::unordered_map<std::string, DeviceInfo> ConfigLoader::loadDeviceConfig(const std::string& config_path) {
    std::unordered_map<std::string, DeviceInfo> config;
    YAML::Node node = loadYAMLConfig(config_path);

    // 加载配置
    if (node["devices"]) {
        for (const auto& device : node["devices"]) {
            std::string name = device["name"].as<std::string>();
            std::string port = device["port"].as<std::string>();
            int baudrate = device["baudrate"].as<int>();

            // 读取设备ID（Hand_ID），可选字段，默认值为1
            int hand_id = 1;
            if (device["Hand_ID"]) {
                hand_id = device["Hand_ID"].as<int>();
            }

            DeviceInfo info;
            info.name = name;
            info.baudrate = baudrate;
            info.hand_id = hand_id;

            // 将设备信息存入 map，键为port
            config[port] = info;
        }
    }

    return config;
}

std::shared_ptr<Protocol> ConfigLoader::createProtocolFromConfig(const std::string& config_path) {
    YAML::Node node = loadYAMLConfig(config_path);

    // 根据配置创建协议对象
    if (node["protocol"] && node["protocol"]["type"]) {
        std::string protocol_type = node["protocol"]["type"].as<std::string>();

        // 使用工厂模式创建协议对象，支持动态扩展
        // 新协议只需注册即可，无需修改此处代码
        try {
            return ProtocolFactory::create(protocol_type);
        } catch (const std::exception& e) {
            // 错误信息：无法创建协议
            throw std::runtime_error("Failed to create protocol '" + protocol_type + "': " + e.what() +
                                     ". Available types: " + [&protocol_type]() {
                                         auto types = ProtocolFactory::getRegisteredTypes();
                                         if (types.empty())
                                             return std::string("(none registered)");
                                         std::string result;
                                         for (const auto& t : types) {
                                             if (!result.empty())
                                                 result += ", ";
                                             result += t;
                                         }
                                         return result;
                                     }());
        }
    }

    throw std::runtime_error("Invalid protocol configuration: missing protocol type");
}

std::string ConfigLoader::getProtocolTypeString(const std::string& config_path) {
    YAML::Node node = loadYAMLConfig(config_path);
    if (node["protocol"] && node["protocol"]["type"]) {
        return node["protocol"]["type"].as<std::string>();
    }
    throw std::runtime_error("设备协议配置缺少 protocol.type: " + config_path);
}

std::string ConfigLoader::interfacesProfileFromProtocolType(const std::string& protocol_type) {
    if (protocol_type.rfind("RH5DG2", 0) == 0) {
        return "RH5DG2";
    }
    if (protocol_type.rfind("RH56H1", 0) == 0) {
        return "RH56H1";
    }
    if (protocol_type.rfind("RH56F1", 0) == 0) {
        return "RH56F1";
    }
    if (protocol_type.rfind("RH56DFX", 0) == 0) {
        return "RH56DFX";
    }
    if (protocol_type.rfind("EG5CD1", 0) == 0) {
        return "EG5CD1";
    }
    throw std::runtime_error(
        "无法从协议类型 \"" + protocol_type +
        "\" 推导 ROS 接口机型，请使用以 RH5DG2、RH56F1、RH56H1、RH56DFX 或 EG5CD1 为前缀的 protocol.type（如 RH56H1_485、RH56DFX_serial_can、RH5DG2_485）");
}

// 日志配置方法
void ConfigLoader::configureLogging(const std::string& config_path) {
    YAML::Node node = loadYAMLConfig(config_path);

    if (node["logging"]) {
        // 使用日志管理器初始化
        LoggerManager::initialize(node["logging"]);
    } else {
        // 如果没有配置，使用默认配置
        LoggerManager::LogConfig default_config;
        LoggerManager::initialize(default_config);
    }
}

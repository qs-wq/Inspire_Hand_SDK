#pragma once
#include <string>
#include <unordered_map> 
#include <memory>
#include <yaml-cpp/yaml.h>

#include "spdlog/spdlog.h"

class Protocol;  // 前向声明

// 设备信息（从YAML配置文件加载）
struct DeviceInfo {
    std::string name;
    int baudrate;
    int hand_id = 1;   // 设备ID（Hand_ID），默认为1，支持多设备区分
};

class ConfigLoader {
public:
    static std::unordered_map<std::string, DeviceInfo> loadDeviceConfig(const std::string& config_path);
    static std::shared_ptr<Protocol> createProtocolFromConfig(const std::string& config_path);
    /** 读取 device_protocol_config.yaml 中的 protocol.type（不实例化协议） */
    static std::string getProtocolTypeString(const std::string& config_path);
    /** 将 protocol.type（如 RH5DG2_485）映射为 ROS 接口机型 RH5DG2 / RH56F1 / EG5CD1 */
    static std::string interfacesProfileFromProtocolType(const std::string& protocol_type);
    static void configureLogging(const std::string& config_path);

private:
    static YAML::Node loadYAMLConfig(const std::string& path);
};

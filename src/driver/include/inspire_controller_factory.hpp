#pragma once

#include <string>
#include <vector>
#include <memory>
#include <yaml-cpp/yaml.h>
#include "register_controller.hpp"
#include "protocol.hpp"
#include "device_manager.hpp"

/**
 * @brief Inspire 灵巧手 ROS2 控制器工厂
 *
 * 根据配置文件创建和管理设备节点（一个设备对应一个节点）
 */
class InspireControllerFactory {
public:
    /**
     * @brief 从YAML配置加载设备节点配置
     * @param config_path ros2_controller_config.yaml 路径
     * @param device_protocol_config_path device_protocol_config.yaml 路径（用于从 protocol.type 推导 interfaces_profile）
     * @return 设备节点配置列表（按设备分组）
     */
    static std::vector<DeviceNodeConfig> loadDeviceNodeConfigs(
        const std::string& config_path,
        const std::string& device_protocol_config_path);

    /**
     * @brief 创建设备节点实例
     * @param config 设备节点配置（interfaces_profile 应由 loadDeviceNodeConfigs 根据协议配置填写）
     * @param device_manager 设备管理器
     * @param device_protocols 设备协议映射（port -> protocol）
     * @param device_protocol_config_path 与启动时一致的设备协议 YAML，用于按设备名解析串口
     */
    static std::shared_ptr<RegisterController> createDeviceNode(
        const DeviceNodeConfig& config,
        DeviceManager& device_manager,
        const std::unordered_map<std::string, std::shared_ptr<Protocol>>& device_protocols,
        const std::string& device_protocol_config_path);

    /**
     * @brief 批量创建所有设备节点
     * @param config_path ros2_controller_config.yaml
     * @param device_manager 设备管理器
     * @param device_protocols 设备协议映射
     * @param device_protocol_config_path device_protocol_config.yaml
     */
    static std::vector<std::shared_ptr<RegisterController>> createAllDeviceNodes(
        const std::string& config_path,
        DeviceManager& device_manager,
        const std::unordered_map<std::string, std::shared_ptr<Protocol>>& device_protocols,
        const std::string& device_protocol_config_path);

private:
    static TopicConfig parseTopicConfig(const YAML::Node& node);
    static ServiceConfig parseServiceConfig(const YAML::Node& node);
    static DeviceNodeConfig parseDeviceNodeConfig(const YAML::Node& node);
};

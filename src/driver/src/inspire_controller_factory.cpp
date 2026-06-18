#include "inspire_controller_factory.hpp"
#include "config_loader.hpp"
#include "logger_manager.hpp"
#include <yaml-cpp/yaml.h>
#include <stdexcept>
#include <fstream>

std::vector<DeviceNodeConfig> InspireControllerFactory::loadDeviceNodeConfigs(
    const std::string& config_path,
    const std::string& device_protocol_config_path
) {
    auto logger = getLogger();
    logger->info("加载设备节点配置: {}", config_path);

    const std::string protocol_type =
        ConfigLoader::getProtocolTypeString(device_protocol_config_path);
    const std::string profile =
        ConfigLoader::interfacesProfileFromProtocolType(protocol_type);
    logger->info("根据设备协议配置推导 ROS 接口机型: protocol.type={} -> interfaces_profile={}",
                 protocol_type, profile);

    YAML::Node node = YAML::LoadFile(config_path);
    std::vector<DeviceNodeConfig> configs;

    if (!node["device_nodes"]) {
        throw std::runtime_error("配置文件中未找到 'device_nodes' 节点");
    }

    for (const auto& device_node : node["device_nodes"]) {
        DeviceNodeConfig config = parseDeviceNodeConfig(device_node);
        config.interfaces_profile = profile;
        configs.push_back(config);
        logger->info("加载设备节点配置: {} profile={} (topics: {}, services: {})",
                     config.device_name, config.interfaces_profile,
                     config.topics.size(), config.services.size());
    }

    logger->info("共加载 {} 个设备节点配置", configs.size());
    return configs;
}

TopicConfig InspireControllerFactory::parseTopicConfig(const YAML::Node& node) {
    TopicConfig config;

    if (node["name"]) {
        config.name = node["name"].as<std::string>();
    }

    if (node["registers"]) {
        if (node["registers"]["write"]) {
            for (const auto& reg : node["registers"]["write"]) {
                config.write_registers.push_back(reg.as<std::string>());
            }
        }
        if (node["registers"]["read"]) {
            for (const auto& reg : node["registers"]["read"]) {
                config.read_registers.push_back(reg.as<std::string>());
            }
        }
    }

    if (node["command_topic"]) {
        config.command_topic = node["command_topic"].as<std::string>();
    }

    if (node["state_topic"]) {
        config.state_topic = node["state_topic"].as<std::string>();
    }

    if (node["touch_version"]) {
        config.touch_version = node["touch_version"].as<int>();
    }

    return config;
}

ServiceConfig InspireControllerFactory::parseServiceConfig(const YAML::Node& node) {
    ServiceConfig config;

    if (!node["register_name"]) {
        throw std::runtime_error("服务配置缺少 'register_name' 字段");
    }
    config.register_name = node["register_name"].as<std::string>();

    if (node["set_service_name"]) {
        config.set_service_name = node["set_service_name"].as<std::string>();
    }

    if (node["get_service_name"]) {
        config.get_service_name = node["get_service_name"].as<std::string>();
    }

    if (node["is_write_register"]) {
        config.is_write_register = node["is_write_register"].as<bool>();
    } else {
        config.is_write_register = !config.set_service_name.empty();
    }

    return config;
}

DeviceNodeConfig InspireControllerFactory::parseDeviceNodeConfig(const YAML::Node& node) {
    DeviceNodeConfig config;

    if (!node["device"]) {
        throw std::runtime_error("设备节点配置缺少 'device' 字段");
    }
    config.device_name = node["device"].as<std::string>();

    if (node["topics"]) {
        for (const auto& topic_node : node["topics"]) {
            TopicConfig topic_config = parseTopicConfig(topic_node);
            config.topics.push_back(topic_config);
        }
    }

    if (node["services"]) {
        for (const auto& service_node : node["services"]) {
            ServiceConfig service_config = parseServiceConfig(service_node);
            config.services.push_back(service_config);
        }
    }

    if (node["update_rate"]) {
        config.update_rate = node["update_rate"].as<double>();
    }

    if (node["publish_header"] && node["publish_header"]["frame_id"]) {
        config.publish_frame_id = node["publish_header"]["frame_id"].as<std::string>();
    } else if (node["frame_id"]) {
        config.publish_frame_id = node["frame_id"].as<std::string>();
    }

    if (node["joint_names"]) {
        for (const auto& jn : node["joint_names"]) {
            config.joint_names.push_back(jn.as<std::string>());
        }
    }

    if (node["initial_registers"]) {
        for (const auto& reg_node : node["initial_registers"]) {
            if (!reg_node["register_name"]) {
                throw std::runtime_error(
                    "initial_registers 项缺少 'register_name' 字段 (device: " + config.device_name + ")");
            }
            InitialRegisterConfig initial;
            initial.register_name = reg_node["register_name"].as<std::string>();
            if (!reg_node["values"]) {
                throw std::runtime_error(
                    "initial_registers 项缺少 'values' 字段 (register: " + initial.register_name + ")");
            }
            for (const auto& v : reg_node["values"]) {
                initial.values.push_back(v.as<int>());
            }
            config.initial_registers.push_back(std::move(initial));
        }
    }

    return config;
}

std::shared_ptr<RegisterController> InspireControllerFactory::createDeviceNode(
    const DeviceNodeConfig& config,
    DeviceManager& device_manager,
    const std::unordered_map<std::string, std::shared_ptr<Protocol>>& device_protocols,
    const std::string& device_protocol_config_path
) {
    auto logger = getLogger();

    auto deviceConfig = ConfigLoader::loadDeviceConfig(device_protocol_config_path);
    std::string device_port;

    for (const auto& [port, deviceInfo] : deviceConfig) {
        if (deviceInfo.name == config.device_name) {
            device_port = port;
            break;
        }
    }

    if (device_port.empty()) {
        throw std::runtime_error("未找到设备: " + config.device_name);
    }

    auto device = device_manager.getDevice(device_port);
    if (!device) {
        throw std::runtime_error("无法获取设备对象: " + config.device_name + " (" + device_port + ")");
    }

    auto it = device_protocols.find(device_port);
    if (it == device_protocols.end()) {
        throw std::runtime_error("未找到设备协议: " + config.device_name);
    }
    auto protocol = it->second;

    std::string node_name = config.device_name + "_node";
    auto controller = std::make_shared<RegisterController>(
        node_name,
        config,
        device,
        protocol);

    controller->initialize();

    logger->info("创建设备节点: {} (device: {}, port: {}, topics: {}, services: {})",
                 node_name, config.device_name, device_port,
                 config.topics.size(), config.services.size());

    return controller;
}

std::vector<std::shared_ptr<RegisterController>> InspireControllerFactory::createAllDeviceNodes(
    const std::string& config_path,
    DeviceManager& device_manager,
    const std::unordered_map<std::string, std::shared_ptr<Protocol>>& device_protocols,
    const std::string& device_protocol_config_path
) {
    auto configs = loadDeviceNodeConfigs(config_path, device_protocol_config_path);
    std::vector<std::shared_ptr<RegisterController>> controllers;

    for (const auto& config : configs) {
        try {
            auto controller = createDeviceNode(config, device_manager, device_protocols, device_protocol_config_path);
            controllers.push_back(controller);
        } catch (const std::exception& e) {
            auto logger = getLogger();
            logger->error("创建设备节点失败 {}: {}", config.device_name, e.what());
        }
    }

    return controllers;
}

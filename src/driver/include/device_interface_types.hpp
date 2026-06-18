#pragma once

#include <string>
#include <vector>

/** @brief 话题配置（YAML topics 项） */
struct TopicConfig {
    std::string name;
    std::vector<std::string> write_registers;
    std::vector<std::string> read_registers;
    std::string command_topic;
    std::string state_topic;
    int touch_version = 1;
};

/** @brief 启动时写入寄存器（YAML initial_registers 项，对齐 RH5DG2 默认速度/力意图） */
struct InitialRegisterConfig {
    std::string register_name;
    std::vector<int> values;
};

/** @brief 服务配置（YAML services 项） */
struct ServiceConfig {
    std::string register_name;
    std::string set_service_name;
    std::string get_service_name;
    bool is_write_register = false;
};

/** @brief 设备节点配置（一个设备对应一个 ROS 节点） */
struct DeviceNodeConfig {
    std::string device_name;
    std::string interfaces_profile;  // 由 InspireControllerFactory 根据 device_protocol_config.yaml 的 protocol.type 填写
    /** 发布状态类消息时 std_msgs/Header::frame_id，对应 YAML publish_header.frame_id（或简写 frame_id） */
    std::string publish_frame_id;
    /** 与机型一致：RH5DG2 为 13、RH56F1 为 6；EG5CD1 可不填；不足补空串，超出忽略 */
    std::vector<std::string> joint_names;
    std::vector<TopicConfig> topics;
    std::vector<ServiceConfig> services;
    /** 节点 start() 时按顺序写入（如 speedSet / forceSet），再开始定时发布 */
    std::vector<InitialRegisterConfig> initial_registers;
    double update_rate = 50.0;
};

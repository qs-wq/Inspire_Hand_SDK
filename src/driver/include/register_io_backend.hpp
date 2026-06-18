#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "protocol.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief 寄存器与定时器能力抽象，供 InterfaceAdapter 调用，与具体 ROS 节点解耦。
 */
class IRegisterIoBackend {
public:
    virtual ~IRegisterIoBackend() = default;

    virtual rclcpp::Node* ioNode() = 0;

    virtual RegisterReadResult ioReadRegister(
        const std::string& register_name, size_t length = 0) = 0;

    virtual IoError ioWriteRegister(
        const std::string& register_name, const std::vector<int>& values) = 0;

    virtual TouchReadResult ioReadTouchData(int version) = 0;

    virtual void ioPauseTimer(int duration_ms) = 0;

    virtual int32_t ioHandId() const = 0;

    virtual std::string ioNodeName() const = 0;
};

/**
 * @brief 发布/订阅/服务实体存放处（由 RegisterController 持有，适配器仅引用）。
 */
struct RosEntityMaps {
    std::map<std::string, rclcpp::PublisherBase::SharedPtr>& publishers;
    std::map<std::string, rclcpp::SubscriptionBase::SharedPtr>& subscribers;
    std::map<std::string, rclcpp::ServiceBase::SharedPtr>& services;
};

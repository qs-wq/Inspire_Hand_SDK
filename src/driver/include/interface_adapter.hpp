#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "device_interface_types.hpp"
#include "register_io_backend.hpp"

/**
 * 入站 Topic/Service 的 hand_id 是否针对本节点（与 device_protocol_config.yaml 的 Hand_ID 一致）。
 * hand_id==0 视为未显式指定，仍受理，兼容旧客户端；其它值必须与 ioHandId() 相等。
 */
inline bool rosIncomingHandIdTargetsThisNode(const IRegisterIoBackend& backend, std::int32_t msg_hand_id)
{
    const std::int32_t mine = backend.ioHandId();
    return msg_hand_id == mine || msg_hand_id == 0;
}

/** 从设备节点配置的 joint_names 取第 index 个关节名，未配置时为空串 */
inline std::string configuredJointName(const std::vector<std::string>& names, std::size_t index)
{
    return index < names.size() ? names[index] : std::string();
}

/**
 * @brief 产品线 ROS 接口适配器：负责 Topic/Service 类型选择与消息填充。
 */
class InterfaceAdapter {
public:
    InterfaceAdapter(
        IRegisterIoBackend& backend,
        const DeviceNodeConfig& config,
        RosEntityMaps maps)
        : backend_(backend),
          config_(config),
          maps_(maps) {}

    virtual ~InterfaceAdapter() = default;

    virtual void wireTopics() = 0;
    virtual void wireServices() = 0;

    virtual void publishRegisterData(
        const TopicConfig& topic_config,
        const std::vector<int>& values) = 0;

    virtual void publishTouchData(
        const TopicConfig& topic_config,
        const TouchDataResult& touchData,
        int version) = 0;

protected:
    /**
     * @brief 创建归入「服务回调组」的 Service（与定时器分组以实现并发）。
     *
     * 用法与 node->create_service<T>(name, callback) 完全一致，仅自动补上 QoS 与回调组，
     * 因此适配器内可把 `node->create_service` 直接替换为 `this->makeGroupedService`。
     */
    template <typename ServiceT, typename CallbackT>
    typename rclcpp::Service<ServiceT>::SharedPtr makeGroupedService(
        const std::string& name, CallbackT&& callback) {
        return backend_.ioNode()->template create_service<ServiceT>(
            name,
            std::forward<CallbackT>(callback),
            rmw_qos_profile_services_default,
            backend_.ioServiceCallbackGroup());
    }

    /**
     * @brief 创建归入「服务回调组」的 Subscription（写入类回调，与定时读分组并发）。
     *
     * 用法与 node->create_subscription<Msg>(name, qos, callback) 一致，自动补上回调组。
     */
    template <typename MessageT, typename CallbackT>
    typename rclcpp::Subscription<MessageT>::SharedPtr makeGroupedSubscription(
        const std::string& name, const rclcpp::QoS& qos, CallbackT&& callback) {
        rclcpp::SubscriptionOptions options;
        options.callback_group = backend_.ioServiceCallbackGroup();
        return backend_.ioNode()->template create_subscription<MessageT>(
            name, qos, std::forward<CallbackT>(callback), options);
    }

    IRegisterIoBackend& backend_;
    DeviceNodeConfig config_;
    RosEntityMaps maps_;
};

std::unique_ptr<InterfaceAdapter> makeInterfaceAdapter(
    const std::string& interfaces_profile,
    IRegisterIoBackend& backend,
    const DeviceNodeConfig& config,
    RosEntityMaps maps);

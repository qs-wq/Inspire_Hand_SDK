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
    IRegisterIoBackend& backend_;
    DeviceNodeConfig config_;
    RosEntityMaps maps_;
};

std::unique_ptr<InterfaceAdapter> makeInterfaceAdapter(
    const std::string& interfaces_profile,
    IRegisterIoBackend& backend,
    const DeviceNodeConfig& config,
    RosEntityMaps maps);

#pragma once

#include "interface_adapter.hpp"

/** EG-5CD1 夹爪：eg5cd1_interfaces（GripperState / SetInt32 / Trigger / 标量查询） */
class EG5CD1InterfaceAdapter : public InterfaceAdapter {
public:
    using InterfaceAdapter::InterfaceAdapter;

    void wireTopics() override;
    void wireServices() override;

    void publishRegisterData(
        const TopicConfig& topic_config,
        const std::vector<int>& values) override;

    void publishTouchData(
        const TopicConfig& topic_config,
        const TouchDataResult& touchData,
        int version) override;
};

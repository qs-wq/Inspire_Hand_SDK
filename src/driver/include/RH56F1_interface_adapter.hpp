#pragma once

#include "interface_adapter.hpp"

/** RH56F1 产品线：6 关节 + rh56f1_interfaces */
class RH56F1InterfaceAdapter : public InterfaceAdapter {
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

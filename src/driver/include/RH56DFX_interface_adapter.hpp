#pragma once

#include "interface_adapter.hpp"

/** RH56DFX 产品线：6 关节 + rh56dfx_interfaces（最小联调接口） */
class RH56DFXInterfaceAdapter : public InterfaceAdapter {
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


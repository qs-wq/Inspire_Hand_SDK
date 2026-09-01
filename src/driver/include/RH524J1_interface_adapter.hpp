#pragma once

#include "interface_adapter.hpp"

/** RH524J1 产品线：24 关节腱绳手 + rh524j1_interfaces */
class RH524J1InterfaceAdapter : public InterfaceAdapter {
public:
    using InterfaceAdapter::InterfaceAdapter;

    void wireTopics() override;
    void wireServices() override;

    void publishRegisterData(const TopicConfig& topic_config, const std::vector<int>& values) override;

    void publishTouchData(const TopicConfig& topic_config, const TouchDataResult& touchData, int version) override;
};

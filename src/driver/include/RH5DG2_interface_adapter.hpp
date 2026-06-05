#pragma once

#include "interface_adapter.hpp"

/** RH5DG2 产品线：13 关节 + rh5dg2_interfaces */
class RH5DG2InterfaceAdapter : public InterfaceAdapter {
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

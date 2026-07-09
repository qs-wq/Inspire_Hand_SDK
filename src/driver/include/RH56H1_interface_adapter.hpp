#pragma once

#include "interface_adapter.hpp"

/** RH56H1 产品线：6 关节 + rh56h1_interfaces（寄存器/帧与 F1 相同，触觉为 version2 压阻式） */
class RH56H1InterfaceAdapter : public InterfaceAdapter {
public:
    using InterfaceAdapter::InterfaceAdapter;

    void wireTopics() override;
    void wireServices() override;

    void publishRegisterData(const TopicConfig& topic_config, const std::vector<int>& values) override;

    void publishTouchData(const TopicConfig& topic_config, const TouchDataResult& touchData, int version) override;
};

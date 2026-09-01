#pragma once

#include "interface_adapter.hpp"

#include <atomic>

/**
 * @brief EG2-4C2 电动夹爪 ROS 接口适配器
 *
 * 继承自 InterfaceAdapter 基类，实现 EG2-4C2 夹爪的 ROS Topic/Service 映射。
 */
class EG2_4C2InterfaceAdapter : public InterfaceAdapter {
public:
    using InterfaceAdapter::InterfaceAdapter;

    void wireTopics() override;
    void wireServices() override;

    void publishRegisterData(const TopicConfig& topic_config, const std::vector<int>& values) override;

    void publishTouchData(const TopicConfig& topic_config, const TouchDataResult& touchData, int version) override;

private:
    /**
     * EG2-4C2 的运动帧需要同时携带 position、speed、force。
     * speed_set/force_set Topic 更新缓存，open_len_set Topic 使用三者组成同一 CAN 帧。
     */
    std::atomic<int> topic_speed_{600};
    std::atomic<int> topic_force_{300};
};

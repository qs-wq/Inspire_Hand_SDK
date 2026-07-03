#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

/**
 * @brief ROS2消息类型定义
 *
 * 结构体对应ROS2消息类型，用于在Protocol数据和ROS2消息之间转换
 */

/**
 * @brief 角度命令消息（逻辑示意；Topic 请使用 rh*_interfaces/msg/SetAngle1 等）
 */
struct AngleCommand {
    std::vector<int32_t> values; // 手指的角度值
};

/**
 * @brief 角度状态消息
 */
struct AngleState {
    std::vector<int32_t> values;
};

/**
 * @brief 触觉数据消息
 */
struct TouchData {
    std::map<std::string, std::vector<uint16_t>> finger_data;
    std::map<std::string, uint16_t> palm_data;
};

/**
 * @brief 通用寄存器数据消息
 */
struct RegisterData {
    std::string register_name;
    std::vector<int32_t> values;
};

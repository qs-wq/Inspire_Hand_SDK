#include "RH56H1_interface_adapter.hpp"
#include "logger_manager.hpp"

#include <algorithm>
#include <cmath>
#include <std_msgs/msg/header.hpp>
#include <stdexcept>

#include <rh56h1_interfaces/msg/get_angle_act1.hpp>
#include <rh56h1_interfaces/msg/get_current_act1.hpp>
#include <rh56h1_interfaces/msg/get_force_act1.hpp>
#include <rh56h1_interfaces/msg/get_percent_act1.hpp>
#include <rh56h1_interfaces/msg/set_angle1.hpp>
#include <rh56h1_interfaces/msg/set_current1.hpp>
#include <rh56h1_interfaces/msg/set_force1.hpp>
#include <rh56h1_interfaces/msg/set_percent1.hpp>
#include <rh56h1_interfaces/msg/set_speed1.hpp>
#include <rh56h1_interfaces/msg/touch_data1.hpp>
#include <rh56h1_interfaces/msg/touch_data2.hpp>

#include <rh56h1_interfaces/srv/geterror.hpp>
#include <rh56h1_interfaces/srv/getpercent.hpp>
#include <rh56h1_interfaces/srv/setpercent.hpp>
#include <rh56h1_interfaces/srv/getstatus.hpp>
#include <rh56h1_interfaces/srv/gettemp.hpp>
#include <rh56h1_interfaces/srv/setactionlibraryindex.hpp>
#include <rh56h1_interfaces/srv/setactionseqindex.hpp>
#include <rh56h1_interfaces/srv/setangle.hpp>
#include <rh56h1_interfaces/srv/setbaudrate.hpp>
#include <rh56h1_interfaces/srv/setclearerror.hpp>
#include <rh56h1_interfaces/srv/setdefaultforceset.hpp>
#include <rh56h1_interfaces/srv/setdefaultspeed.hpp>
#include <rh56h1_interfaces/srv/setforce.hpp>
#include <rh56h1_interfaces/srv/setgestureforceclb.hpp>
#include <rh56h1_interfaces/srv/setid.hpp>
#include <rh56h1_interfaces/srv/setmode.hpp>
#include <rh56h1_interfaces/srv/setpause.hpp>
#include <rh56h1_interfaces/srv/setresetpara.hpp>
#include <rh56h1_interfaces/srv/setsave.hpp>
#include <rh56h1_interfaces/srv/setspeed.hpp>
#include <rh56h1_interfaces/srv/setstop.hpp>

namespace {

constexpr size_t kRH56H1Joints = 6;

// ---- 百分比接口量程（依据 RH56H1 用户手册 V1.2；下限均为 0）----
// 速度(speedSet) 0-3000、力(forceSet) 0-900、电流(currentSet) 0-1500，前 5 指统一量程。
constexpr double kSpeedRawMax = 3000.0;   // speedSet（前 5 指；大拇指旋转手册为 0-20，暂沿用统一量程）
constexpr double kForceRawMax = 900.0;    // forceSet
constexpr double kCurrentRawMax = 1500.0; // currentSet

// ---- 位置百分比：改用「角度 angleSet/angleAct」换算 ----
// 手册 2.5.9 明确不建议用电缸位置 posSet(0-2000) 设定手指角度，推荐用 angleSet。
// 各自由度角度范围不同（手册 表35/表40，单位 0.1°）：角度越大越张开、越小越握紧。
// 索引：0 小拇指 / 1 无名指 / 2 中指 / 3 食指 / 4 大拇指弯曲 / 5 大拇指旋转。
constexpr int kAngleRawMin[6] = {870, 870, 870, 870, 950, 700};
constexpr int kAngleRawMax[6] = {1690, 1690, 1690, 1690, 1350, 1700};

// 百分比(0~100，超范围自动裁剪) → 寄存器原始值(0~raw_max)
int percentToRaw(double percent, double raw_max) {
    percent = std::clamp(percent, 0.0, 100.0);
    return static_cast<int>(std::lround(percent / 100.0 * raw_max));
}

// 寄存器原始值 → 百分比(0~100，超范围自动裁剪)
float rawToPercent(int raw, double raw_max) {
    double p = (raw_max > 0.0) ? (static_cast<double>(raw) / raw_max * 100.0) : 0.0;
    return static_cast<float>(std::clamp(p, 0.0, 100.0));
}

// 位置百分比 → angleSet 原始值：按该自由度角度范围换算，0%=握紧(最小角度)、100%=张开(最大角度)
int anglePercentToRaw(double percent, size_t joint) {
    if (joint >= 6) {
        joint = 5;
    }
    percent = std::clamp(percent, 0.0, 100.0);
    const int mn = kAngleRawMin[joint];
    const int mx = kAngleRawMax[joint];
    return static_cast<int>(std::lround(mn + percent / 100.0 * (mx - mn)));
}

// angleSet 写入裁剪（手册表35）：四指/拇指弯曲允许 -1（不动作）；超范围裁剪到合法区间。
// 注意：2000 是 posSet（电缸位置）上限，不是 angleSet；四指 angleSet 上限为 1690。
int clampAngleSetRaw(int raw, size_t joint) {
    if (joint >= kRH56H1Joints) {
        joint = kRH56H1Joints - 1;
    }
    if (joint <= 4 && raw == -1) {
        return -1;
    }
    return static_cast<int>(std::clamp(raw, kAngleRawMin[joint], kAngleRawMax[joint]));
}

std::vector<int> clampAngleSetValues(const std::vector<int>& vals) {
    std::vector<int> out;
    out.reserve(vals.size());
    for (size_t i = 0; i < vals.size(); ++i) {
        out.push_back(clampAngleSetRaw(vals[i], i));
    }
    return out;
}

void logAngleSetClamped(const std::shared_ptr<spdlog::logger>& logger, const std::string& node_name,
                        const std::vector<int>& before, const std::vector<int>& after) {
    for (size_t i = 0; i < before.size() && i < after.size(); ++i) {
        if (before[i] != after[i]) {
            logger->warn(
                "[{}] angleSet joint[{}] 超手册范围 {}，已裁剪为 {}（四指870~1690/拇指弯950~1350/拇指旋700~1700；"
                "2000 为 posSet 量程非 angleSet）",
                node_name, i, before[i], after[i]);
        }
    }
}

// angleAct 原始值 → 位置百分比：0%=握紧(最小角度)、100%=张开(最大角度)
float angleRawToPercent(int raw, size_t joint) {
    if (joint >= 6) {
        joint = 5;
    }
    const int mn = kAngleRawMin[joint];
    const int mx = kAngleRawMax[joint];
    double p = (mx > mn) ? (static_cast<double>(raw - mn) / (mx - mn) * 100.0) : 0.0;
    return static_cast<float>(std::clamp(p, 0.0, 100.0));
}

// 按百分比话题/服务名（本文件约定的 topic name 或 register_name）返回对应量程上限。
// 找不到返回 0（不做换算，视为未映射）。
double percentRawMaxForRegister(const std::string& reg) {
    if (reg == "speedSet" || reg == "speedSetPercent") {
        return kSpeedRawMax;
    }
    if (reg == "forceSet" || reg == "forceSetPercent") {
        return kForceRawMax;
    }
    if (reg == "currentSet" || reg == "currentSetPercent") {
        return kCurrentRawMax;
    }
    return 0.0;
}

void stamp_header(std_msgs::msg::Header& h, rclcpp::Node* node, const std::string& frame_id) {
    h.stamp = node->now();
    h.frame_id = frame_id;
}

void touch_to_msg(const TouchDataResult& touchData, rh56h1_interfaces::msg::TouchData1& msg) {
    static const char* kFingerOrder[] = {"little", "ring", "middle", "index", "thumb"};
    for (size_t i = 0; i < 5; ++i) {
        auto it = touchData.fingerResults.find(kFingerOrder[i]);
        if (it != touchData.fingerResults.end() && it->second.size() >= 4) {
            msg.finger_forces[i] = static_cast<int32_t>(it->second[0]);
            msg.finger_tangentials[i] = static_cast<int32_t>(it->second[1]);
            msg.finger_angles[i] = static_cast<int32_t>(it->second[2]);
            msg.finger_proximity[i] = static_cast<int32_t>(it->second[3]);
        } else {
            msg.finger_forces[i] = 0;
            msg.finger_tangentials[i] = 0;
            msg.finger_angles[i] = 0;
            msg.finger_proximity[i] = 0;
        }
    }
    for (int i = 1; i <= 9; ++i) {
        std::string palm_key = "palm_data_" + std::to_string(i);
        auto it = touchData.palmResults.find(palm_key);
        msg.palm_data[i - 1] = (it != touchData.palmResults.end()) ? static_cast<int32_t>(it->second) : 0;
    }
}

// 触觉 version2（压阻式）数据 → 消息（RH56H1 专用，不影响 F1 的 TouchData1）
void touch_to_msg_v2(const TouchDataResult& touchData, rh56h1_interfaces::msg::TouchData2& msg) {
    auto fill = [&](const char* finger, std::vector<int16_t>& tip_end, std::vector<int16_t>& tip_touch, float& fx,
                    float& fy, float& fz) {
        auto it = touchData.fingerResultsV2.find(finger);
        if (it != touchData.fingerResultsV2.end()) {
            tip_end.assign(it->second.tip_end.begin(), it->second.tip_end.end());
            tip_touch.assign(it->second.tip_touch.begin(), it->second.tip_touch.end());
            fx = it->second.force_x;
            fy = it->second.force_y;
            fz = it->second.force_z;
        }
    };
    fill("pinky", msg.pinky_tip_end, msg.pinky_tip_touch, msg.pinky_force_x, msg.pinky_force_y, msg.pinky_force_z);
    fill("ring", msg.ring_tip_end, msg.ring_tip_touch, msg.ring_force_x, msg.ring_force_y, msg.ring_force_z);
    fill("middle", msg.middle_tip_end, msg.middle_tip_touch, msg.middle_force_x, msg.middle_force_y,
         msg.middle_force_z);
    fill("index", msg.index_tip_end, msg.index_tip_touch, msg.index_force_x, msg.index_force_y, msg.index_force_z);
    fill("thumb", msg.thumb_tip_end, msg.thumb_tip_touch, msg.thumb_force_x, msg.thumb_force_y, msg.thumb_force_z);
    msg.palm_touch.assign(touchData.palmResultsV2.begin(), touchData.palmResultsV2.end());
}

} // namespace

void RH56H1InterfaceAdapter::wireTopics() {
    auto logger = getLogger();
    rclcpp::Node* node = backend_.ioNode();
    const int32_t hid = backend_.ioHandId();

    for (const auto& tc : config_.topics) {
        if (!tc.state_topic.empty()) {
            if (tc.name == "angle_control") {
                maps_.publishers[tc.state_topic] =
                    node->create_publisher<rh56h1_interfaces::msg::GetAngleAct1>(tc.state_topic, 10);
                logger->info("[{}] Publisher(GetAngleAct1): {}", backend_.ioNodeName(), tc.state_topic);
            } else if (tc.name == "force_control") {
                maps_.publishers[tc.state_topic] =
                    node->create_publisher<rh56h1_interfaces::msg::GetForceAct1>(tc.state_topic, 10);
                logger->info("[{}] Publisher(GetForceAct1): {}", backend_.ioNodeName(), tc.state_topic);
            } else if (tc.name == "current_control") {
                maps_.publishers[tc.state_topic] =
                    node->create_publisher<rh56h1_interfaces::msg::GetCurrentAct1>(tc.state_topic, 10);
                logger->info("[{}] Publisher(GetCurrentAct1): {}", backend_.ioNodeName(), tc.state_topic);
            } else if (tc.name == "touch_control") {
                if (tc.touch_version == 2) {
                    maps_.publishers[tc.state_topic] =
                        node->create_publisher<rh56h1_interfaces::msg::TouchData2>(tc.state_topic, 10);
                    logger->info("[{}] Publisher(TouchData2): {}", backend_.ioNodeName(), tc.state_topic);
                } else {
                    maps_.publishers[tc.state_topic] =
                        node->create_publisher<rh56h1_interfaces::msg::TouchData1>(tc.state_topic, 10);
                    logger->info("[{}] Publisher(TouchData1): {}", backend_.ioNodeName(), tc.state_topic);
                }
            } else if (tc.name == "pos_percent_control") {
                // 位置实际值百分比读取话题：读取 angleAct 后按各指角度范围换算为百分比发布
                maps_.publishers[tc.state_topic] =
                    node->create_publisher<rh56h1_interfaces::msg::GetPercentAct1>(tc.state_topic, 10);
                logger->info("[{}] Publisher(GetPercentAct1): {}", backend_.ioNodeName(), tc.state_topic);
            }
        }

        if (!tc.command_topic.empty() && !tc.write_registers.empty()) {
            const std::string reg = tc.write_registers[0];
            if (tc.name == "angle_control") {
                maps_.subscribers[tc.command_topic] = this->makeGroupedSubscription<rh56h1_interfaces::msg::SetAngle1>(
                    tc.command_topic, 10, [this, reg, hid](rh56h1_interfaces::msg::SetAngle1::SharedPtr msg) {
                        if (!rosIncomingHandIdTargetsThisNode(backend_, msg->hand_id)) {
                            getLogger()->warn("[{}] 忽略 SetAngle1: hand_id={}（本节点 Hand_ID={}）",
                                              backend_.ioNodeName(), msg->hand_id, hid);
                            return;
                        }
                        std::vector<int> vals(msg->joint_values.begin(), msg->joint_values.end());
                        const std::vector<int> clamped = clampAngleSetValues(vals);
                        logAngleSetClamped(getLogger(), backend_.ioNodeName(), vals, clamped);
                        backend_.ioWriteRegister(reg, clamped);
                    });
                logger->info("[{}] Subscriber(SetAngle1): {}", backend_.ioNodeName(), tc.command_topic);
            } else if (tc.name == "force_control") {
                maps_.subscribers[tc.command_topic] = this->makeGroupedSubscription<rh56h1_interfaces::msg::SetForce1>(
                    tc.command_topic, 10, [this, reg, hid](rh56h1_interfaces::msg::SetForce1::SharedPtr msg) {
                        if (!rosIncomingHandIdTargetsThisNode(backend_, msg->hand_id)) {
                            getLogger()->warn("[{}] 忽略 SetForce1: hand_id={}（本节点 Hand_ID={}）",
                                              backend_.ioNodeName(), msg->hand_id, hid);
                            return;
                        }
                        std::vector<int> vals(msg->joint_values.begin(), msg->joint_values.end());
                        backend_.ioWriteRegister(reg, vals);
                    });
                logger->info("[{}] Subscriber(SetForce1): {}", backend_.ioNodeName(), tc.command_topic);
            } else if (tc.name == "speed_control") {
                maps_.subscribers[tc.command_topic] = this->makeGroupedSubscription<rh56h1_interfaces::msg::SetSpeed1>(
                    tc.command_topic, 10, [this, reg, hid](rh56h1_interfaces::msg::SetSpeed1::SharedPtr msg) {
                        if (!rosIncomingHandIdTargetsThisNode(backend_, msg->hand_id)) {
                            getLogger()->warn("[{}] 忽略 SetSpeed1: hand_id={}（本节点 Hand_ID={}）",
                                              backend_.ioNodeName(), msg->hand_id, hid);
                            return;
                        }
                        std::vector<int> vals(msg->joint_values.begin(), msg->joint_values.end());
                        backend_.ioWriteRegister(reg, vals);
                    });
                logger->info("[{}] Subscriber(SetSpeed1): {}", backend_.ioNodeName(), tc.command_topic);
            } else if (tc.name == "current_control") {
                maps_.subscribers[tc.command_topic] =
                    this->makeGroupedSubscription<rh56h1_interfaces::msg::SetCurrent1>(
                        tc.command_topic, 10, [this, reg, hid](rh56h1_interfaces::msg::SetCurrent1::SharedPtr msg) {
                            if (!rosIncomingHandIdTargetsThisNode(backend_, msg->hand_id)) {
                                getLogger()->warn("[{}] 忽略 SetCurrent1: hand_id={}（本节点 Hand_ID={}）",
                                                  backend_.ioNodeName(), msg->hand_id, hid);
                                return;
                            }
                            std::vector<int> vals(msg->joint_values.begin(), msg->joint_values.end());
                            backend_.ioWriteRegister(reg, vals);
                        });
                logger->info("[{}] Subscriber(SetCurrent1): {}", backend_.ioNodeName(), tc.command_topic);
            } else if (tc.name == "pos_percent_control") {
                // 位置百分比：SetPercent1(0~100) → 按各指角度范围换算，固定写入 angleSet
                const std::string tname = tc.name;
                maps_.subscribers[tc.command_topic] =
                    this->makeGroupedSubscription<rh56h1_interfaces::msg::SetPercent1>(
                        tc.command_topic, 10,
                        [this, hid, tname](rh56h1_interfaces::msg::SetPercent1::SharedPtr msg) {
                            if (!rosIncomingHandIdTargetsThisNode(backend_, msg->hand_id)) {
                                getLogger()->warn("[{}] 忽略 {}(SetPercent1): hand_id={}（本节点 Hand_ID={}）",
                                                  backend_.ioNodeName(), tname, msg->hand_id, hid);
                                return;
                            }
                            std::vector<int> vals;
                            vals.reserve(msg->joint_values.size());
                            for (size_t i = 0; i < msg->joint_values.size(); ++i) {
                                vals.push_back(anglePercentToRaw(static_cast<double>(msg->joint_values[i]), i));
                            }
                            backend_.ioWriteRegister("angleSet", vals);
                        });
                logger->info("[{}] Subscriber(SetPercent1/{}): {} -> angleSet", backend_.ioNodeName(), tname,
                             tc.command_topic);
            } else if (tc.name == "speed_percent_control" || tc.name == "force_percent_control" ||
                       tc.name == "current_percent_control") {
                // 速度/力/电流百分比：按统一量程换算写入对应寄存器（0%->0，100%->量程上限）
                const double raw_max = percentRawMaxForRegister(reg);
                const std::string tname = tc.name;
                maps_.subscribers[tc.command_topic] =
                    this->makeGroupedSubscription<rh56h1_interfaces::msg::SetPercent1>(
                        tc.command_topic, 10,
                        [this, reg, hid, raw_max, tname](rh56h1_interfaces::msg::SetPercent1::SharedPtr msg) {
                            if (!rosIncomingHandIdTargetsThisNode(backend_, msg->hand_id)) {
                                getLogger()->warn("[{}] 忽略 {}(SetPercent1): hand_id={}（本节点 Hand_ID={}）",
                                                  backend_.ioNodeName(), tname, msg->hand_id, hid);
                                return;
                            }
                            std::vector<int> vals;
                            vals.reserve(msg->joint_values.size());
                            for (float p : msg->joint_values) {
                                vals.push_back(percentToRaw(static_cast<double>(p), raw_max));
                            }
                            backend_.ioWriteRegister(reg, vals);
                        });
                logger->info("[{}] Subscriber(SetPercent1/{}): {} -> {}", backend_.ioNodeName(), tname,
                             tc.command_topic, reg);
            }
        }
    }
}

void RH56H1InterfaceAdapter::publishRegisterData(const TopicConfig& topic_config, const std::vector<int>& values) {
    const int32_t hid = backend_.ioHandId();
    rclcpp::Node* node = backend_.ioNode();

    if (topic_config.name == "angle_control") {
        auto pub = std::dynamic_pointer_cast<rclcpp::Publisher<rh56h1_interfaces::msg::GetAngleAct1>>(
            maps_.publishers[topic_config.state_topic]);
        if (!pub) {
            return;
        }
        rh56h1_interfaces::msg::GetAngleAct1 msg;
        stamp_header(msg.header, node, config_.publish_frame_id);
        msg.hand_id = hid;
        for (size_t i = 0; i < kRH56H1Joints; ++i) {
            msg.joint_values[i] = (i < values.size()) ? static_cast<int32_t>(values[i]) : 0;
            msg.joint_names[i] = configuredJointName(config_.joint_names, i);
        }
        pub->publish(msg);
        return;
    }

    if (topic_config.name == "force_control") {
        auto pub = std::dynamic_pointer_cast<rclcpp::Publisher<rh56h1_interfaces::msg::GetForceAct1>>(
            maps_.publishers[topic_config.state_topic]);
        if (!pub) {
            return;
        }
        rh56h1_interfaces::msg::GetForceAct1 msg;
        stamp_header(msg.header, node, config_.publish_frame_id);
        msg.hand_id = hid;
        for (size_t i = 0; i < kRH56H1Joints; ++i) {
            msg.joint_values[i] = (i < values.size()) ? static_cast<int32_t>(values[i]) : 0;
            msg.joint_names[i] = configuredJointName(config_.joint_names, i);
        }
        pub->publish(msg);
        return;
    }

    if (topic_config.name == "current_control") {
        auto pub = std::dynamic_pointer_cast<rclcpp::Publisher<rh56h1_interfaces::msg::GetCurrentAct1>>(
            maps_.publishers[topic_config.state_topic]);
        if (!pub) {
            return;
        }
        rh56h1_interfaces::msg::GetCurrentAct1 msg;
        stamp_header(msg.header, node, config_.publish_frame_id);
        msg.hand_id = hid;
        for (size_t i = 0; i < kRH56H1Joints; ++i) {
            msg.joint_values[i] = (i < values.size()) ? static_cast<int32_t>(values[i]) : 0;
            msg.joint_names[i] = configuredJointName(config_.joint_names, i);
        }
        pub->publish(msg);
        return;
    }

    if (topic_config.name == "pos_percent_control") {
        // 角度实际值(angleAct) → 位置百分比发布（按各指角度范围，0%=握紧、100%=张开）
        auto pub = std::dynamic_pointer_cast<rclcpp::Publisher<rh56h1_interfaces::msg::GetPercentAct1>>(
            maps_.publishers[topic_config.state_topic]);
        if (!pub) {
            return;
        }
        rh56h1_interfaces::msg::GetPercentAct1 msg;
        stamp_header(msg.header, node, config_.publish_frame_id);
        msg.hand_id = hid;
        for (size_t i = 0; i < kRH56H1Joints; ++i) {
            msg.joint_values[i] = (i < values.size()) ? angleRawToPercent(values[i], i) : 0.0F;
            msg.joint_names[i] = configuredJointName(config_.joint_names, i);
        }
        pub->publish(msg);
    }
}

void RH56H1InterfaceAdapter::publishTouchData(const TopicConfig& topic_config, const TouchDataResult& touchData,
                                              int version) {
    rclcpp::Node* node = backend_.ioNode();
    // 触觉 version2（压阻式）：发布 TouchData2；其余版本沿用 TouchData1
    if (version == 2) {
        auto pub_v2 = std::dynamic_pointer_cast<rclcpp::Publisher<rh56h1_interfaces::msg::TouchData2>>(
            maps_.publishers[topic_config.state_topic]);
        if (!pub_v2) {
            return;
        }
        rh56h1_interfaces::msg::TouchData2 msg;
        stamp_header(msg.header, node, config_.publish_frame_id);
        touch_to_msg_v2(touchData, msg);
        pub_v2->publish(msg);
        return;
    }
    auto pub = std::dynamic_pointer_cast<rclcpp::Publisher<rh56h1_interfaces::msg::TouchData1>>(
        maps_.publishers[topic_config.state_topic]);
    if (!pub) {
        return;
    }
    rh56h1_interfaces::msg::TouchData1 msg;
    stamp_header(msg.header, node, config_.publish_frame_id);
    touch_to_msg(touchData, msg);
    pub->publish(msg);
}

void RH56H1InterfaceAdapter::wireServices() {
    auto logger = getLogger();

    for (const auto& sc : config_.services) {
        if (sc.is_write_register && !sc.set_service_name.empty()) {
            const std::string& reg = sc.register_name;

            if (reg == "angleSet") {
                maps_.services[sc.set_service_name] = this->makeGroupedService<rh56h1_interfaces::srv::Setangle>(
                    sc.set_service_name, [this, reg](const rh56h1_interfaces::srv::Setangle::Request::SharedPtr req,
                                                     rh56h1_interfaces::srv::Setangle::Response::SharedPtr res) {
                        if (!rosIncomingHandIdTargetsThisNode(backend_, req->hand_id)) {
                            res->accepted = false;
                            res->message = "rejected: hand_id mismatch";
                            getLogger()->warn("[{}] 拒绝 Setangle: hand_id={}（本节点 Hand_ID={}）",
                                              backend_.ioNodeName(), req->hand_id, backend_.ioHandId());
                            return;
                        }
                        std::vector<int> vals(req->joint_values.begin(), req->joint_values.end());
                        const std::vector<int> clamped = clampAngleSetValues(vals);
                        logAngleSetClamped(getLogger(), backend_.ioNodeName(), vals, clamped);
                        const IoError e = backend_.ioWriteRegister(reg, clamped);
                        res->accepted = isOk(e);
                        res->message = toString(e);
                    });
                logger->info("[{}] Service(SetAngle): {}", backend_.ioNodeName(), sc.set_service_name);
            } else if (reg == "id") {
                maps_.services[sc.set_service_name] = this->makeGroupedService<rh56h1_interfaces::srv::Setid>(
                    sc.set_service_name, [this](const rh56h1_interfaces::srv::Setid::Request::SharedPtr req,
                                                rh56h1_interfaces::srv::Setid::Response::SharedPtr res) {
                        if (!rosIncomingHandIdTargetsThisNode(backend_, req->hand_id)) {
                            res->accepted = false;
                            res->message = "rejected: hand_id mismatch";
                            getLogger()->warn("[{}] 拒绝 Setid: hand_id={}（本节点 Hand_ID={}）", backend_.ioNodeName(),
                                              req->hand_id, backend_.ioHandId());
                            return;
                        }
                        const IoError e = backend_.ioWriteRegister("id", {static_cast<int>(req->device_id)});
                        res->accepted = isOk(e);
                        res->message = toString(e);
                    });
                logger->info("[{}] Service(Setid): {}", backend_.ioNodeName(), sc.set_service_name);
            } else if (reg == "mode") {
                maps_.services[sc.set_service_name] = this->makeGroupedService<rh56h1_interfaces::srv::Setmode>(
                    sc.set_service_name, [this](const rh56h1_interfaces::srv::Setmode::Request::SharedPtr req,
                                                rh56h1_interfaces::srv::Setmode::Response::SharedPtr res) {
                        if (!rosIncomingHandIdTargetsThisNode(backend_, req->hand_id)) {
                            res->accepted = false;
                            res->message = "rejected: hand_id mismatch";
                            getLogger()->warn("[{}] 拒绝 Setmode: hand_id={}（本节点 Hand_ID={}）",
                                              backend_.ioNodeName(), req->hand_id, backend_.ioHandId());
                            return;
                        }
                        std::vector<int> vals(req->joint_values.begin(), req->joint_values.end());
                        const IoError e = backend_.ioWriteRegister("mode", vals);
                        res->accepted = isOk(e);
                        res->message = toString(e);
                    });
                logger->info("[{}] Service(SetMode): {}", backend_.ioNodeName(), sc.set_service_name);
            } else if (reg == "forceSet") {
                maps_.services[sc.set_service_name] = this->makeGroupedService<rh56h1_interfaces::srv::Setforce>(
                    sc.set_service_name, [this](const rh56h1_interfaces::srv::Setforce::Request::SharedPtr req,
                                                rh56h1_interfaces::srv::Setforce::Response::SharedPtr res) {
                        if (!rosIncomingHandIdTargetsThisNode(backend_, req->hand_id)) {
                            res->accepted = false;
                            res->message = "rejected: hand_id mismatch";
                            getLogger()->warn("[{}] 拒绝 Setforce: hand_id={}（本节点 Hand_ID={}）",
                                              backend_.ioNodeName(), req->hand_id, backend_.ioHandId());
                            return;
                        }
                        std::vector<int> vals(req->joint_values.begin(), req->joint_values.end());
                        const IoError e = backend_.ioWriteRegister("forceSet", vals);
                        res->accepted = isOk(e);
                        res->message = toString(e);
                    });
                logger->info("[{}] Service(SetForce): {}", backend_.ioNodeName(), sc.set_service_name);
            } else if (reg == "speedSet") {
                maps_.services[sc.set_service_name] = this->makeGroupedService<rh56h1_interfaces::srv::Setspeed>(
                    sc.set_service_name, [this](const rh56h1_interfaces::srv::Setspeed::Request::SharedPtr req,
                                                rh56h1_interfaces::srv::Setspeed::Response::SharedPtr res) {
                        if (!rosIncomingHandIdTargetsThisNode(backend_, req->hand_id)) {
                            res->accepted = false;
                            res->message = "rejected: hand_id mismatch";
                            getLogger()->warn("[{}] 拒绝 Setspeed: hand_id={}（本节点 Hand_ID={}）",
                                              backend_.ioNodeName(), req->hand_id, backend_.ioHandId());
                            return;
                        }
                        std::vector<int> vals(req->joint_values.begin(), req->joint_values.end());
                        const IoError e = backend_.ioWriteRegister("speedSet", vals);
                        res->accepted = isOk(e);
                        res->message = toString(e);
                    });
                logger->info("[{}] Service(SetSpeed): {}", backend_.ioNodeName(), sc.set_service_name);
            } else if (reg == "baudRate") {
                maps_.services[sc.set_service_name] = this->makeGroupedService<rh56h1_interfaces::srv::Setbaudrate>(
                    sc.set_service_name, [this, reg](const rh56h1_interfaces::srv::Setbaudrate::Request::SharedPtr req,
                                                     rh56h1_interfaces::srv::Setbaudrate::Response::SharedPtr res) {
                        if (!rosIncomingHandIdTargetsThisNode(backend_, req->hand_id)) {
                            res->accepted = false;
                            res->message = "rejected: hand_id mismatch";
                            getLogger()->warn("[{}] 拒绝 Setbaudrate: hand_id={}（本节点 Hand_ID={}）",
                                              backend_.ioNodeName(), req->hand_id, backend_.ioHandId());
                            return;
                        }
                        const IoError e = backend_.ioWriteRegister(reg, {static_cast<int>(req->baudrate)});
                        res->accepted = isOk(e);
                        res->message = toString(e);
                    });
                logger->info("[{}] Service(Setbaudrate): {}", backend_.ioNodeName(), sc.set_service_name);
            } else if (reg == "clearError") {
                maps_.services[sc.set_service_name] = this->makeGroupedService<rh56h1_interfaces::srv::Setclearerror>(
                    sc.set_service_name,
                    [this, reg](const rh56h1_interfaces::srv::Setclearerror::Request::SharedPtr req,
                                rh56h1_interfaces::srv::Setclearerror::Response::SharedPtr res) {
                        if (!rosIncomingHandIdTargetsThisNode(backend_, req->hand_id)) {
                            res->accepted = false;
                            res->message = "rejected: hand_id mismatch";
                            getLogger()->warn("[{}] 拒绝 Setclearerror: hand_id={}（本节点 Hand_ID={}）",
                                              backend_.ioNodeName(), req->hand_id, backend_.ioHandId());
                            return;
                        }
                        const IoError e = backend_.ioWriteRegister(reg, {static_cast<int>(req->clear_code)});
                        res->accepted = isOk(e);
                        res->message = toString(e);
                    });
                logger->info("[{}] Service(Setclearerror): {}", backend_.ioNodeName(), sc.set_service_name);
            } else if (reg == "save") {
                maps_.services[sc.set_service_name] = this->makeGroupedService<rh56h1_interfaces::srv::Setsave>(
                    sc.set_service_name, [this, reg](const rh56h1_interfaces::srv::Setsave::Request::SharedPtr req,
                                                     rh56h1_interfaces::srv::Setsave::Response::SharedPtr res) {
                        if (!rosIncomingHandIdTargetsThisNode(backend_, req->hand_id)) {
                            res->accepted = false;
                            res->message = "rejected: hand_id mismatch";
                            getLogger()->warn("[{}] 拒绝 Setsave: hand_id={}（本节点 Hand_ID={}）",
                                              backend_.ioNodeName(), req->hand_id, backend_.ioHandId());
                            return;
                        }
                        const IoError e = backend_.ioWriteRegister(reg, {static_cast<int>(req->save_code)});
                        res->accepted = isOk(e);
                        res->message = toString(e);
                    });
                logger->info("[{}] Service(Setsave): {}", backend_.ioNodeName(), sc.set_service_name);
            } else if (reg == "resetPara") {
                maps_.services[sc.set_service_name] = this->makeGroupedService<rh56h1_interfaces::srv::Setresetpara>(
                    sc.set_service_name, [this, reg](const rh56h1_interfaces::srv::Setresetpara::Request::SharedPtr req,
                                                     rh56h1_interfaces::srv::Setresetpara::Response::SharedPtr res) {
                        if (!rosIncomingHandIdTargetsThisNode(backend_, req->hand_id)) {
                            res->accepted = false;
                            res->message = "rejected: hand_id mismatch";
                            getLogger()->warn("[{}] 拒绝 Setresetpara: hand_id={}（本节点 Hand_ID={}）",
                                              backend_.ioNodeName(), req->hand_id, backend_.ioHandId());
                            return;
                        }
                        const IoError e = backend_.ioWriteRegister(reg, {static_cast<int>(req->confirm)});
                        res->accepted = isOk(e);
                        res->message = toString(e);
                    });
                logger->info("[{}] Service(Setresetpara): {}", backend_.ioNodeName(), sc.set_service_name);
            } else if (reg == "gestureForceClb") {
                maps_.services[sc.set_service_name] =
                    this->makeGroupedService<rh56h1_interfaces::srv::Setgestureforceclb>(
                        sc.set_service_name,
                        [this, reg](const rh56h1_interfaces::srv::Setgestureforceclb::Request::SharedPtr req,
                                    rh56h1_interfaces::srv::Setgestureforceclb::Response::SharedPtr res) {
                            if (!rosIncomingHandIdTargetsThisNode(backend_, req->hand_id)) {
                                res->accepted = false;
                                res->message = "rejected: hand_id mismatch";
                                getLogger()->warn("[{}] 拒绝 Setgestureforceclb: hand_id={}（本节点 Hand_ID={}）",
                                                  backend_.ioNodeName(), req->hand_id, backend_.ioHandId());
                                return;
                            }
                            std::vector<int> vals(req->calibration_values.begin(), req->calibration_values.end());
                            const IoError e = backend_.ioWriteRegister(reg, vals);
                            res->accepted = isOk(e);
                            res->message = toString(e);
                        });
                logger->info("[{}] Service(Setgestureforceclb): {}", backend_.ioNodeName(), sc.set_service_name);
            } else if (reg == "defaultSpeedSet") {
                maps_.services[sc.set_service_name] = this->makeGroupedService<rh56h1_interfaces::srv::Setdefaultspeed>(
                    sc.set_service_name,
                    [this, reg](const rh56h1_interfaces::srv::Setdefaultspeed::Request::SharedPtr req,
                                rh56h1_interfaces::srv::Setdefaultspeed::Response::SharedPtr res) {
                        if (!rosIncomingHandIdTargetsThisNode(backend_, req->hand_id)) {
                            res->accepted = false;
                            res->message = "rejected: hand_id mismatch";
                            getLogger()->warn("[{}] 拒绝 Setdefaultspeed: hand_id={}（本节点 Hand_ID={}）",
                                              backend_.ioNodeName(), req->hand_id, backend_.ioHandId());
                            return;
                        }
                        std::vector<int> vals(req->joint_values.begin(), req->joint_values.end());
                        const IoError e = backend_.ioWriteRegister(reg, vals);
                        res->accepted = isOk(e);
                        res->message = toString(e);
                    });
                logger->info("[{}] Service(Setdefaultspeed): {}", backend_.ioNodeName(), sc.set_service_name);
            } else if (reg == "defaultForceSet") {
                maps_.services[sc.set_service_name] =
                    this->makeGroupedService<rh56h1_interfaces::srv::Setdefaultforceset>(
                        sc.set_service_name,
                        [this, reg](const rh56h1_interfaces::srv::Setdefaultforceset::Request::SharedPtr req,
                                    rh56h1_interfaces::srv::Setdefaultforceset::Response::SharedPtr res) {
                            if (!rosIncomingHandIdTargetsThisNode(backend_, req->hand_id)) {
                                res->accepted = false;
                                res->message = "rejected: hand_id mismatch";
                                getLogger()->warn("[{}] 拒绝 Setdefaultforceset: hand_id={}（本节点 Hand_ID={}）",
                                                  backend_.ioNodeName(), req->hand_id, backend_.ioHandId());
                                return;
                            }
                            std::vector<int> vals(req->joint_values.begin(), req->joint_values.end());
                            const IoError e = backend_.ioWriteRegister(reg, vals);
                            res->accepted = isOk(e);
                            res->message = toString(e);
                        });
                logger->info("[{}] Service(Setdefaultforceset): {}", backend_.ioNodeName(), sc.set_service_name);
            } else if (reg == "pause") {
                maps_.services[sc.set_service_name] = this->makeGroupedService<rh56h1_interfaces::srv::Setpause>(
                    sc.set_service_name, [this, reg](const rh56h1_interfaces::srv::Setpause::Request::SharedPtr req,
                                                     rh56h1_interfaces::srv::Setpause::Response::SharedPtr res) {
                        if (!rosIncomingHandIdTargetsThisNode(backend_, req->hand_id)) {
                            res->accepted = false;
                            res->message = "rejected: hand_id mismatch";
                            getLogger()->warn("[{}] 拒绝 Setpause: hand_id={}（本节点 Hand_ID={}）",
                                              backend_.ioNodeName(), req->hand_id, backend_.ioHandId());
                            return;
                        }
                        const IoError e = backend_.ioWriteRegister(reg, {static_cast<int>(req->pause_flag)});
                        res->accepted = isOk(e);
                        res->message = toString(e);
                    });
                logger->info("[{}] Service(Setpause): {}", backend_.ioNodeName(), sc.set_service_name);
            } else if (reg == "stop") {
                maps_.services[sc.set_service_name] = this->makeGroupedService<rh56h1_interfaces::srv::Setstop>(
                    sc.set_service_name, [this, reg](const rh56h1_interfaces::srv::Setstop::Request::SharedPtr req,
                                                     rh56h1_interfaces::srv::Setstop::Response::SharedPtr res) {
                        if (!rosIncomingHandIdTargetsThisNode(backend_, req->hand_id)) {
                            res->accepted = false;
                            res->message = "rejected: hand_id mismatch";
                            getLogger()->warn("[{}] 拒绝 Setstop: hand_id={}（本节点 Hand_ID={}）",
                                              backend_.ioNodeName(), req->hand_id, backend_.ioHandId());
                            return;
                        }
                        const IoError e = backend_.ioWriteRegister(reg, {static_cast<int>(req->stop_flag)});
                        res->accepted = isOk(e);
                        res->message = toString(e);
                    });
                logger->info("[{}] Service(Setstop): {}", backend_.ioNodeName(), sc.set_service_name);
            } else if (reg == "actionSeqIndex") {
                maps_.services[sc.set_service_name] =
                    this->makeGroupedService<rh56h1_interfaces::srv::Setactionseqindex>(
                        sc.set_service_name,
                        [this, reg](const rh56h1_interfaces::srv::Setactionseqindex::Request::SharedPtr req,
                                    rh56h1_interfaces::srv::Setactionseqindex::Response::SharedPtr res) {
                            if (!rosIncomingHandIdTargetsThisNode(backend_, req->hand_id)) {
                                res->accepted = false;
                                res->message = "rejected: hand_id mismatch";
                                getLogger()->warn("[{}] 拒绝 Setactionseqindex: hand_id={}（本节点 Hand_ID={}）",
                                                  backend_.ioNodeName(), req->hand_id, backend_.ioHandId());
                                return;
                            }
                            const IoError e = backend_.ioWriteRegister(reg, {static_cast<int>(req->index)});
                            res->accepted = isOk(e);
                            res->message = toString(e);
                        });
                logger->info("[{}] Service(Setactionseqindex): {}", backend_.ioNodeName(), sc.set_service_name);
            } else if (reg == "actionLibraryIndex") {
                maps_.services[sc.set_service_name] =
                    this->makeGroupedService<rh56h1_interfaces::srv::Setactionlibraryindex>(
                        sc.set_service_name,
                        [this, reg](const rh56h1_interfaces::srv::Setactionlibraryindex::Request::SharedPtr req,
                                    rh56h1_interfaces::srv::Setactionlibraryindex::Response::SharedPtr res) {
                            if (!rosIncomingHandIdTargetsThisNode(backend_, req->hand_id)) {
                                res->accepted = false;
                                res->message = "rejected: hand_id mismatch";
                                getLogger()->warn("[{}] 拒绝 Setactionlibraryindex: hand_id={}（本节点 Hand_ID={}）",
                                                  backend_.ioNodeName(), req->hand_id, backend_.ioHandId());
                                return;
                            }
                            const IoError e = backend_.ioWriteRegister(reg, {static_cast<int>(req->index)});
                            res->accepted = isOk(e);
                            res->message = toString(e);
                        });
                logger->info("[{}] Service(Setactionlibraryindex): {}", backend_.ioNodeName(), sc.set_service_name);
            } else if (reg == "posSetPercent") {
                // 位置百分比服务：Setpercent(0~100) → 按各指角度范围换算写入 angleSet（0%=握紧、100%=张开）
                maps_.services[sc.set_service_name] = this->makeGroupedService<rh56h1_interfaces::srv::Setpercent>(
                    sc.set_service_name,
                    [this, reg](const rh56h1_interfaces::srv::Setpercent::Request::SharedPtr req,
                                rh56h1_interfaces::srv::Setpercent::Response::SharedPtr res) {
                        if (!rosIncomingHandIdTargetsThisNode(backend_, req->hand_id)) {
                            res->accepted = false;
                            res->message = "rejected: hand_id mismatch";
                            getLogger()->warn("[{}] 拒绝 Setpercent({}): hand_id={}（本节点 Hand_ID={}）",
                                              backend_.ioNodeName(), reg, req->hand_id, backend_.ioHandId());
                            return;
                        }
                        std::vector<int> vals;
                        vals.reserve(req->joint_values.size());
                        for (size_t i = 0; i < req->joint_values.size(); ++i) {
                            vals.push_back(anglePercentToRaw(static_cast<double>(req->joint_values[i]), i));
                        }
                        const IoError e = backend_.ioWriteRegister("angleSet", vals);
                        res->accepted = isOk(e);
                        res->message = toString(e);
                    });
                logger->info("[{}] Service(Setpercent/{}): {} -> angleSet", backend_.ioNodeName(), reg,
                             sc.set_service_name);
            } else if (reg == "speedSetPercent" || reg == "forceSetPercent" || reg == "currentSetPercent") {
                // 速度/力/电流百分比设置服务：按统一量程换算为原始值写入对应寄存器
                static const std::map<std::string, std::string> kPercentToRawReg = {
                    {"speedSetPercent", "speedSet"}, {"forceSetPercent", "forceSet"}, {"currentSetPercent",
                                                                                       "currentSet"}};
                const std::string raw_reg = kPercentToRawReg.at(reg);
                const double raw_max = percentRawMaxForRegister(reg);
                maps_.services[sc.set_service_name] = this->makeGroupedService<rh56h1_interfaces::srv::Setpercent>(
                    sc.set_service_name,
                    [this, reg, raw_reg, raw_max](const rh56h1_interfaces::srv::Setpercent::Request::SharedPtr req,
                                                  rh56h1_interfaces::srv::Setpercent::Response::SharedPtr res) {
                        if (!rosIncomingHandIdTargetsThisNode(backend_, req->hand_id)) {
                            res->accepted = false;
                            res->message = "rejected: hand_id mismatch";
                            getLogger()->warn("[{}] 拒绝 Setpercent({}): hand_id={}（本节点 Hand_ID={}）",
                                              backend_.ioNodeName(), reg, req->hand_id, backend_.ioHandId());
                            return;
                        }
                        std::vector<int> vals;
                        vals.reserve(req->joint_values.size());
                        for (float p : req->joint_values) {
                            vals.push_back(percentToRaw(static_cast<double>(p), raw_max));
                        }
                        const IoError e = backend_.ioWriteRegister(raw_reg, vals);
                        res->accepted = isOk(e);
                        res->message = toString(e);
                    });
                logger->info("[{}] Service(Setpercent/{}): {} -> {}", backend_.ioNodeName(), reg, sc.set_service_name,
                             raw_reg);
            } else {
                throw std::runtime_error("[RH56H1] 未映射的写寄存器服务: " + reg +
                                         "，请在 rh56h1_interfaces 增加专用 .srv 并接线");
            }
        }

        if (!sc.get_service_name.empty()) {
            const std::string& reg = sc.register_name;

            if (reg == "errorCode") {
                maps_.services[sc.get_service_name] = this->makeGroupedService<rh56h1_interfaces::srv::Geterror>(
                    sc.get_service_name, [this](const rh56h1_interfaces::srv::Geterror::Request::SharedPtr req,
                                                rh56h1_interfaces::srv::Geterror::Response::SharedPtr res) {
                        if (!rosIncomingHandIdTargetsThisNode(backend_, req->hand_id)) {
                            getLogger()->warn("[{}] 拒绝 Geterror: hand_id={}（本节点 Hand_ID={}）",
                                              backend_.ioNodeName(), req->hand_id, backend_.ioHandId());
                            for (size_t i = 0; i < kRH56H1Joints; ++i) {
                                res->joint_values[i] = 0;
                                res->joint_names[i] = "";
                            }
                            res->message = "rejected: hand_id mismatch";
                            return;
                        }
                        auto rr = backend_.ioReadRegister("errorCode");
                        const bool ok = rr.ok();
                        const auto& vals = rr.values;
                        for (size_t i = 0; i < kRH56H1Joints; ++i) {
                            res->joint_values[i] = (ok && i < vals.size()) ? static_cast<int32_t>(vals[i]) : 0;
                            res->joint_names[i] = configuredJointName(config_.joint_names, i);
                        }
                        res->message = toString(rr.error);
                    });
                logger->info("[{}] Service(GetError): {}", backend_.ioNodeName(), sc.get_service_name);
            } else if (reg == "temp") {
                maps_.services[sc.get_service_name] = this->makeGroupedService<rh56h1_interfaces::srv::Gettemp>(
                    sc.get_service_name, [this](const rh56h1_interfaces::srv::Gettemp::Request::SharedPtr req,
                                                rh56h1_interfaces::srv::Gettemp::Response::SharedPtr res) {
                        if (!rosIncomingHandIdTargetsThisNode(backend_, req->hand_id)) {
                            getLogger()->warn("[{}] 拒绝 Gettemp: hand_id={}（本节点 Hand_ID={}）",
                                              backend_.ioNodeName(), req->hand_id, backend_.ioHandId());
                            for (size_t i = 0; i < kRH56H1Joints; ++i) {
                                res->joint_values[i] = 0;
                                res->joint_names[i] = "";
                            }
                            res->message = "rejected: hand_id mismatch";
                            return;
                        }
                        auto rr = backend_.ioReadRegister("temp");
                        const bool ok = rr.ok();
                        const auto& vals = rr.values;
                        for (size_t i = 0; i < kRH56H1Joints; ++i) {
                            res->joint_values[i] = (ok && i < vals.size()) ? static_cast<int32_t>(vals[i]) : 0;
                            res->joint_names[i] = configuredJointName(config_.joint_names, i);
                        }
                        res->message = toString(rr.error);
                    });
                logger->info("[{}] Service(GetTemp): {}", backend_.ioNodeName(), sc.get_service_name);
            } else if (reg == "status") {
                maps_.services[sc.get_service_name] = this->makeGroupedService<rh56h1_interfaces::srv::Getstatus>(
                    sc.get_service_name, [this](const rh56h1_interfaces::srv::Getstatus::Request::SharedPtr req,
                                                rh56h1_interfaces::srv::Getstatus::Response::SharedPtr res) {
                        if (!rosIncomingHandIdTargetsThisNode(backend_, req->hand_id)) {
                            getLogger()->warn("[{}] 拒绝 Getstatus: hand_id={}（本节点 Hand_ID={}）",
                                              backend_.ioNodeName(), req->hand_id, backend_.ioHandId());
                            for (size_t i = 0; i < kRH56H1Joints; ++i) {
                                res->joint_values[i] = 0;
                                res->joint_names[i] = "";
                            }
                            res->message = "rejected: hand_id mismatch";
                            return;
                        }
                        auto rr = backend_.ioReadRegister("status");
                        const bool ok = rr.ok();
                        const auto& vals = rr.values;
                        for (size_t i = 0; i < kRH56H1Joints; ++i) {
                            res->joint_values[i] = (ok && i < vals.size()) ? static_cast<int32_t>(vals[i]) : 0;
                            res->joint_names[i] = configuredJointName(config_.joint_names, i);
                        }
                        res->message = toString(rr.error);
                    });
                logger->info("[{}] Service(Getstatus): {}", backend_.ioNodeName(), sc.get_service_name);
            } else if (reg == "posActPercent") {
                // 位置实际值百分比读取服务：读取 angleAct 后按各指角度范围换算返回（0%=握紧、100%=张开）
                maps_.services[sc.get_service_name] = this->makeGroupedService<rh56h1_interfaces::srv::Getpercent>(
                    sc.get_service_name, [this](const rh56h1_interfaces::srv::Getpercent::Request::SharedPtr req,
                                                rh56h1_interfaces::srv::Getpercent::Response::SharedPtr res) {
                        if (!rosIncomingHandIdTargetsThisNode(backend_, req->hand_id)) {
                            getLogger()->warn("[{}] 拒绝 Getpercent(posActPercent): hand_id={}（本节点 Hand_ID={}）",
                                              backend_.ioNodeName(), req->hand_id, backend_.ioHandId());
                            for (size_t i = 0; i < kRH56H1Joints; ++i) {
                                res->joint_values[i] = 0.0F;
                                res->joint_names[i] = "";
                            }
                            res->message = "rejected: hand_id mismatch";
                            return;
                        }
                        auto rr = backend_.ioReadRegister("angleAct");
                        const bool ok = rr.ok();
                        const auto& vals = rr.values;
                        for (size_t i = 0; i < kRH56H1Joints; ++i) {
                            res->joint_values[i] =
                                (ok && i < vals.size()) ? angleRawToPercent(vals[i], i) : 0.0F;
                            res->joint_names[i] = configuredJointName(config_.joint_names, i);
                        }
                        res->message = toString(rr.error);
                    });
                logger->info("[{}] Service(Getpercent/posActPercent): {}", backend_.ioNodeName(),
                             sc.get_service_name);
            } else {
                throw std::runtime_error("[RH56H1] 未映射的读寄存器服务: " + reg +
                                         "，请在 rh56h1_interfaces 增加专用 .srv 并接线");
            }
        }
    }
}

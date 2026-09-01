#include "EG2_4C2_interface_adapter.hpp"
#include "logger_manager.hpp"

#include <algorithm>
#include <stdexcept>
#include <unordered_set>

#include <std_msgs/msg/header.hpp>

#include <eg2_4c2_interfaces/msg/gripper_state.hpp>
#include <eg2_4c2_interfaces/msg/set_int32.hpp>
#include <eg2_4c2_interfaces/srv/force_mode_grasp.hpp>
#include <eg2_4c2_interfaces/srv/force_mode_open.hpp>
#include <eg2_4c2_interfaces/srv/get_scalar_for_hand.hpp>
#include <eg2_4c2_interfaces/srv/set_int32_value.hpp>
#include <eg2_4c2_interfaces/srv/trigger_for_hand.hpp>

namespace {

/**
 * 力控组合写步骤间隔（毫秒）：写入相邻寄存器之间留一点间隔给夹爪处理。
 * 该间隔在设备 worker 线程上执行（见 ioWriteSequence），不再阻塞 ROS 回调线程；
 * 整组步骤由 worker 串行化，无需再用经验暂停时长来"躲开"定时读。
 */
constexpr int kCompositeStepMs = 3;
constexpr int kClosedPosition = 0;
constexpr int kOpenPosition = 1000;

int clamp_speed(int32_t v) {
    if (v < 10) {
        return 10;
    }
    if (v > 1000) {
        return 1000;
    }
    return static_cast<int>(v);
}

int clamp_force_grasp(int32_t v) {
    if (v < 100) {
        return 0;
    }
    if (v > 1000) {
        return 1000;
    }
    return static_cast<int>(v);
}

bool clamp_force_open(int32_t v, int& out) {
    // ROS API 沿用负值表示张开方向；4C2 的 FORCE_SET 寄存器只接受正的力阈值。
    if (v > -100 || v < -1000) {
        return false;
    }
    out = static_cast<int>(-v);
    return true;
}

/**
 * 把组合写序列结果格式化为与历史一致的 message：
 *   成功 -> "<最后一步寄存器>: ok"；失败 -> "<失败步寄存器>: <错误码>"。
 */
std::string sequence_message(const std::vector<WriteStep>& steps, const SequenceResult& r) {
    if (steps.empty()) {
        return toString(r.error);
    }
    const size_t idx = r.ok() ? (steps.size() - 1) : r.failed_step;
    return steps[idx].reg + ": " + toString(r.error);
}

void stamp_header(std_msgs::msg::Header& h, rclcpp::Node* node, const std::string& frame_id) {
    h.stamp = node->now();
    h.frame_id = frame_id;
}

bool is_trigger_register(const std::string& reg) {
    static const std::unordered_set<std::string> kTriggers = {
        "save", "defaultPar", "clearError", "stop",
    };
    return kTriggers.count(reg) != 0;
}

} // namespace

void EG2_4C2InterfaceAdapter::wireTopics() {
    auto logger = getLogger();
    rclcpp::Node* node = backend_.ioNode();
    const int32_t hid = backend_.ioHandId();

    for (const auto& tc : config_.topics) {
        if (!tc.state_topic.empty() && tc.name == "gripper_state") {
            maps_.publishers[tc.state_topic] =
                node->create_publisher<eg2_4c2_interfaces::msg::GripperState>(tc.state_topic, 10);
            logger->info("[{}] Publisher(GripperState): {}", backend_.ioNodeName(), tc.state_topic);
        }

        if (!tc.command_topic.empty() && !tc.write_registers.empty()) {
            const std::string reg = tc.write_registers[0];

            if (tc.name == "open_len_set") {
                maps_.subscribers[tc.command_topic] = this->makeGroupedSubscription<eg2_4c2_interfaces::msg::SetInt32>(
                    tc.command_topic, 10, [this, reg, hid](eg2_4c2_interfaces::msg::SetInt32::SharedPtr msg) {
                        if (!rosIncomingHandIdTargetsThisNode(backend_, msg->hand_id)) {
                            getLogger()->warn("[{}] 忽略 SetInt32(open_len): hand_id={}（本节点 Hand_ID={}）",
                                              backend_.ioNodeName(), msg->hand_id, hid);
                            return;
                        }
                        // 与 gripper_demo_can_serial.py::move_gripper() 保持一致：
                        // 运动命令必须在同一帧连续写 position、speed、force。
                        const int position = std::clamp(static_cast<int>(msg->value), kClosedPosition, kOpenPosition);
                        const int speed = topic_speed_.load();
                        const int force = topic_force_.load();
                        backend_.ioWriteRegister(reg, {position, speed, force});
                    });
                logger->info("[{}] Subscriber(SetInt32 open_len): {}", backend_.ioNodeName(), tc.command_topic);
            } else if (tc.name == "speed_set") {
                maps_.subscribers[tc.command_topic] = this->makeGroupedSubscription<eg2_4c2_interfaces::msg::SetInt32>(
                    tc.command_topic, 10, [this, reg, hid](eg2_4c2_interfaces::msg::SetInt32::SharedPtr msg) {
                        if (!rosIncomingHandIdTargetsThisNode(backend_, msg->hand_id)) {
                            getLogger()->warn("[{}] 忽略 SetInt32(speed): hand_id={}（本节点 Hand_ID={}）",
                                              backend_.ioNodeName(), msg->hand_id, hid);
                            return;
                        }
                        const int speed = clamp_speed(msg->value);
                        topic_speed_.store(speed);
                        backend_.ioWriteRegister(reg, {speed});
                    });
                logger->info("[{}] Subscriber(SetInt32 speed): {}", backend_.ioNodeName(), tc.command_topic);
            } else if (tc.name == "force_set") {
                maps_.subscribers[tc.command_topic] = this->makeGroupedSubscription<eg2_4c2_interfaces::msg::SetInt32>(
                    tc.command_topic, 10, [this, reg, hid](eg2_4c2_interfaces::msg::SetInt32::SharedPtr msg) {
                        if (!rosIncomingHandIdTargetsThisNode(backend_, msg->hand_id)) {
                            getLogger()->warn("[{}] 忽略 SetInt32(force): hand_id={}（本节点 Hand_ID={}）",
                                              backend_.ioNodeName(), msg->hand_id, hid);
                            return;
                        }
                        const int force = std::clamp(static_cast<int>(msg->value), 100, 1000);
                        topic_force_.store(force);
                        backend_.ioWriteRegister(reg, {force});
                    });
                logger->info("[{}] Subscriber(SetInt32 force): {}", backend_.ioNodeName(), tc.command_topic);
            } else if (tc.name == "catch_mode_set") {
                maps_.subscribers[tc.command_topic] = this->makeGroupedSubscription<eg2_4c2_interfaces::msg::SetInt32>(
                    tc.command_topic, 10, [this, reg, hid](eg2_4c2_interfaces::msg::SetInt32::SharedPtr msg) {
                        if (!rosIncomingHandIdTargetsThisNode(backend_, msg->hand_id)) {
                            getLogger()->warn("[{}] 忽略 SetInt32(catch_mode): hand_id={}（本节点 Hand_ID={}）",
                                              backend_.ioNodeName(), msg->hand_id, hid);
                            return;
                        }
                        backend_.ioWriteRegister(reg, {static_cast<int>(msg->value)});
                    });
                logger->info("[{}] Subscriber(SetInt32 catch_mode): {}", backend_.ioNodeName(), tc.command_topic);
            }
        }
    }
}

void EG2_4C2InterfaceAdapter::publishRegisterData(const TopicConfig& topic_config, const std::vector<int>& values) {
    if (topic_config.name != "gripper_state") {
        return;
    }

    const int32_t hid = backend_.ioHandId();
    rclcpp::Node* node = backend_.ioNode();
    auto pub = std::dynamic_pointer_cast<rclcpp::Publisher<eg2_4c2_interfaces::msg::GripperState>>(
        maps_.publishers[topic_config.state_topic]);
    if (!pub) {
        return;
    }

    eg2_4c2_interfaces::msg::GripperState msg;
    stamp_header(msg.header, node, config_.publish_frame_id);
    msg.hand_id = hid;

    auto v = [&](size_t i) -> int32_t { return (i < values.size()) ? static_cast<int32_t>(values[i]) : 0; };

    msg.force_act = v(0);
    msg.open_len_act = v(1);
    msg.current_act = v(2);
    msg.temp_c = v(3);
    msg.error_code = static_cast<uint32_t>(std::max(0, v(4)));
    msg.status = static_cast<uint32_t>(std::max(0, v(5)));

    pub->publish(msg);
}

void EG2_4C2InterfaceAdapter::publishTouchData(const TopicConfig&, const TouchDataResult&, int) { (void)0; }

void EG2_4C2InterfaceAdapter::wireServices() {
    auto logger = getLogger();
    rclcpp::Node* node = backend_.ioNode();

    for (const auto& sc : config_.services) {
        if (sc.is_write_register && !sc.set_service_name.empty()) {
            const std::string& reg = sc.register_name;

            if (is_trigger_register(reg)) {
                maps_.services[sc.set_service_name] = this->makeGroupedService<eg2_4c2_interfaces::srv::TriggerForHand>(
                    sc.set_service_name,
                    [this, reg](const eg2_4c2_interfaces::srv::TriggerForHand::Request::SharedPtr req,
                                eg2_4c2_interfaces::srv::TriggerForHand::Response::SharedPtr res) {
                        if (!rosIncomingHandIdTargetsThisNode(backend_, req->hand_id)) {
                            res->accepted = false;
                            res->message = "rejected: hand_id mismatch";
                            getLogger()->warn("[{}] 拒绝 TriggerForHand({}): hand_id={}", backend_.ioNodeName(), reg,
                                              req->hand_id);
                            return;
                        }
                        const IoError e = backend_.ioWriteRegister(reg, {1});
                        res->accepted = isOk(e);
                        res->message = toString(e);
                    });
                logger->info("[{}] Service(TriggerForHand {}): {}", backend_.ioNodeName(), reg, sc.set_service_name);
            } else if (reg == "id" || reg == "baud" || reg == "catchMode" || reg == "maxOpenLen" ||
                       reg == "minOpenLen" || reg == "openLenSet" || reg == "speedSet" || reg == "forceSet") {
                maps_.services[sc.set_service_name] = this->makeGroupedService<eg2_4c2_interfaces::srv::SetInt32Value>(
                    sc.set_service_name,
                    [this, reg](const eg2_4c2_interfaces::srv::SetInt32Value::Request::SharedPtr req,
                                eg2_4c2_interfaces::srv::SetInt32Value::Response::SharedPtr res) {
                        if (!rosIncomingHandIdTargetsThisNode(backend_, req->hand_id)) {
                            res->accepted = false;
                            res->message = "rejected: hand_id mismatch";
                            return;
                        }
                        IoError e = IoError::InvalidArgument;
                        if (reg == "openLenSet") {
                            // 与 Topic 和 Python demo 一致：运动时同帧写位置、速度、力度。
                            const int position =
                                std::clamp(static_cast<int>(req->value), kClosedPosition, kOpenPosition);
                            e = backend_.ioWriteRegister(
                                reg, {position, topic_speed_.load(), topic_force_.load()});
                        } else if (reg == "speedSet") {
                            const int speed = clamp_speed(req->value);
                            topic_speed_.store(speed);
                            e = backend_.ioWriteRegister(reg, {speed});
                        } else if (reg == "forceSet") {
                            const int force = std::clamp(static_cast<int>(req->value), 100, 1000);
                            topic_force_.store(force);
                            e = backend_.ioWriteRegister(reg, {force});
                        } else {
                            e = backend_.ioWriteRegister(reg, {static_cast<int>(req->value)});
                        }
                        res->accepted = isOk(e);
                        res->message = toString(e);
                    });
                logger->info("[{}] Service(SetInt32Value {}): {}", backend_.ioNodeName(), reg, sc.set_service_name);
            } else {
                throw std::runtime_error("[EG2-4C2] 未映射的写寄存器服务: " + reg +
                                         "，请使用 TriggerForHand / SetInt32Value 或增加映射");
            }
            continue;
        }

        if (!sc.get_service_name.empty()) {
            const std::string& reg = sc.register_name;
            static const std::unordered_set<std::string> kScalarRead = {
                "errorCode", "temp", "status", "forceAct", "openLenAct", "currentAct",
            };
            if (kScalarRead.count(reg) == 0) {
                throw std::runtime_error("[EG2-4C2] 未映射的读寄存器服务: " + reg);
            }

            maps_.services[sc.get_service_name] = this->makeGroupedService<eg2_4c2_interfaces::srv::GetScalarForHand>(
                sc.get_service_name, [this, reg](const eg2_4c2_interfaces::srv::GetScalarForHand::Request::SharedPtr req,
                                                 eg2_4c2_interfaces::srv::GetScalarForHand::Response::SharedPtr res) {
                    if (!rosIncomingHandIdTargetsThisNode(backend_, req->hand_id)) {
                        res->value = 0;
                        res->message = "rejected: hand_id mismatch";
                        return;
                    }
                    const auto rr = backend_.ioReadRegister(reg, 0);
                    res->value = (rr.ok() && !rr.values.empty()) ? static_cast<int32_t>(rr.values[0]) : 0;
                    res->message = toString(rr.error);
                });
            logger->info("[{}] Service(GetScalarForHand {}): {}", backend_.ioNodeName(), reg, sc.get_service_name);
        }
    }

    // --- 夹取/张开组合 API（仅 hand_id + speed + force；步骤间隔 3ms）---
    // 4C2 手册规定：speedSet、forceSet 单独写入不会产生运动，写 openLenSet 才立即动作。
    // 因此 openLenSet 必须与速度、力组成连续三寄存器写入。
    // 整组写经 ioWriteSequence 在设备 worker 上原子串行执行；定时读与之天然互不交错，
    // 无需再用经验暂停时长。回调仅等待 worker 完成（不在回调内 sleep 持锁）。
    if (!node->has_parameter("eg2_4c2_composite_service_prefix")) {
        node->declare_parameter<std::string>("eg2_4c2_composite_service_prefix", "/gripper");
    }
    std::string composite_prefix = node->get_parameter("eg2_4c2_composite_service_prefix").as_string();
    while (!composite_prefix.empty() && composite_prefix.back() == '/') {
        composite_prefix.pop_back();
    }
    if (composite_prefix.empty()) {
        composite_prefix = "/gripper";
    }

    const std::string svc_fg = composite_prefix + "/force_mode_grasp";
    maps_.services[svc_fg] = this->makeGroupedService<eg2_4c2_interfaces::srv::ForceModeGrasp>(
        svc_fg, [this](const eg2_4c2_interfaces::srv::ForceModeGrasp::Request::SharedPtr req,
                       eg2_4c2_interfaces::srv::ForceModeGrasp::Response::SharedPtr res) {
            res->accepted = false;
            res->message = "";
            if (!rosIncomingHandIdTargetsThisNode(backend_, req->hand_id)) {
                res->message = "rejected: hand_id mismatch";
                getLogger()->warn("[{}] 拒绝 ForceModeGrasp: hand_id={}", backend_.ioNodeName(), req->hand_id);
                return;
            }
            const int sp = clamp_speed(req->speed);
            const int fg = clamp_force_grasp(req->force);
            if (fg < 100) {
                res->message = "invalid_argument: force 须为 100..1000（正夹取）";
                getLogger()->warn("[{}] ForceModeGrasp: force 须为 100..1000（正夹取），收到 {}", backend_.ioNodeName(),
                                  req->force);
                return;
            }
            const std::vector<WriteStep> steps = {
                {"catchMode", {1}, kCompositeStepMs},
                {"openLenSet", {kClosedPosition, sp, fg}, 0},
            };
            const SequenceResult r = backend_.ioWriteSequence(steps);
            res->accepted = r.ok();
            res->message = sequence_message(steps, r);
        });
    logger->info("[{}] Service(ForceModeGrasp): {}", backend_.ioNodeName(), svc_fg);

    const std::string svc_fo = composite_prefix + "/force_mode_open";
    maps_.services[svc_fo] = this->makeGroupedService<eg2_4c2_interfaces::srv::ForceModeOpen>(
        svc_fo, [this](const eg2_4c2_interfaces::srv::ForceModeOpen::Request::SharedPtr req,
                       eg2_4c2_interfaces::srv::ForceModeOpen::Response::SharedPtr res) {
            res->accepted = false;
            res->message = "";
            if (!rosIncomingHandIdTargetsThisNode(backend_, req->hand_id)) {
                res->message = "rejected: hand_id mismatch";
                getLogger()->warn("[{}] 拒绝 ForceModeOpen: hand_id={}", backend_.ioNodeName(), req->hand_id);
                return;
            }
            int fo = 0;
            if (!clamp_force_open(req->force, fo)) {
                res->message = "invalid_argument: force 须为 -1000..-100（负值表示张开方向）";
                getLogger()->warn("[{}] ForceModeOpen: force 须为 -1000..-100，收到 {}", backend_.ioNodeName(),
                                  req->force);
                return;
            }
            const int sp = clamp_speed(req->speed);
            const std::vector<WriteStep> steps = {
                {"catchMode", {0}, kCompositeStepMs},
                {"openLenSet", {kOpenPosition, sp, fo}, 0},
            };
            const SequenceResult r = backend_.ioWriteSequence(steps);
            res->accepted = r.ok();
            res->message = sequence_message(steps, r);
        });
    logger->info("[{}] Service(ForceModeOpen): {}", backend_.ioNodeName(), svc_fo);
}

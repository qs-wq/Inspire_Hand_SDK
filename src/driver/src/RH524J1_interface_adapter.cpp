#include "RH524J1_interface_adapter.hpp"
#include "logger_manager.hpp"

#include <cstdint>
#include <std_msgs/msg/header.hpp>
#include <stdexcept>
#include <vector>

#include <rh524j1_interfaces/msg/get_angle_act1.hpp>
#include <rh524j1_interfaces/msg/get_current_act1.hpp>
#include <rh524j1_interfaces/msg/get_force_act1.hpp>
#include <rh524j1_interfaces/msg/set_angle1.hpp>
#include <rh524j1_interfaces/msg/set_current1.hpp>
#include <rh524j1_interfaces/msg/set_force1.hpp>
#include <rh524j1_interfaces/msg/set_speed1.hpp>

#include <rh524j1_interfaces/srv/getangleact.hpp>
#include <rh524j1_interfaces/srv/getcurrentact.hpp>
#include <rh524j1_interfaces/srv/geterror.hpp>
#include <rh524j1_interfaces/srv/getforceact.hpp>
#include <rh524j1_interfaces/srv/getstatus.hpp>
#include <rh524j1_interfaces/srv/gettemp.hpp>
#include <rh524j1_interfaces/srv/setangle.hpp>
#include <rh524j1_interfaces/srv/setbaudrate.hpp>
#include <rh524j1_interfaces/srv/setclearerror.hpp>
#include <rh524j1_interfaces/srv/setdefaultforceset.hpp>
#include <rh524j1_interfaces/srv/setdefaultspeed.hpp>
#include <rh524j1_interfaces/srv/setforce.hpp>
#include <rh524j1_interfaces/srv/setgestureforceclb.hpp>
#include <rh524j1_interfaces/srv/setid.hpp>
#include <rh524j1_interfaces/srv/setmode.hpp>
#include <rh524j1_interfaces/srv/setpause.hpp>
#include <rh524j1_interfaces/srv/setresetpara.hpp>
#include <rh524j1_interfaces/srv/setspeed.hpp>
#include <rh524j1_interfaces/srv/setstop.hpp>

namespace {

constexpr size_t kRH524J1Joints = 24;

void stamp_header(std_msgs::msg::Header& h, rclcpp::Node* node, const std::string& frame_id) {
    h.stamp = node->now();
    h.frame_id = frame_id;
}

} // namespace

void RH524J1InterfaceAdapter::wireTopics() {
    auto logger = getLogger();
    rclcpp::Node* node = backend_.ioNode();
    const int32_t hid = backend_.ioHandId();

    for (const auto& tc : config_.topics) {
        if (!tc.state_topic.empty()) {
            if (tc.name == "angle_control") {
                maps_.publishers[tc.state_topic] =
                    node->create_publisher<rh524j1_interfaces::msg::GetAngleAct1>(tc.state_topic, 10);
                logger->info("[{}] Publisher(GetAngleAct1): {}", backend_.ioNodeName(), tc.state_topic);
            } else if (tc.name == "force_control") {
                maps_.publishers[tc.state_topic] =
                    node->create_publisher<rh524j1_interfaces::msg::GetForceAct1>(tc.state_topic, 10);
                logger->info("[{}] Publisher(GetForceAct1): {}", backend_.ioNodeName(), tc.state_topic);
            } else if (tc.name == "current_control") {
                maps_.publishers[tc.state_topic] =
                    node->create_publisher<rh524j1_interfaces::msg::GetCurrentAct1>(tc.state_topic, 10);
                logger->info("[{}] Publisher(GetCurrentAct1): {}", backend_.ioNodeName(), tc.state_topic);
            }
        }

        if (!tc.command_topic.empty() && !tc.write_registers.empty()) {
            const std::string reg = tc.write_registers[0];
            if (tc.name == "angle_control") {
                maps_.subscribers[tc.command_topic] = this->makeGroupedSubscription<rh524j1_interfaces::msg::SetAngle1>(
                    tc.command_topic, 10, [this, reg, hid](rh524j1_interfaces::msg::SetAngle1::SharedPtr msg) {
                        if (!rosIncomingHandIdTargetsThisNode(backend_, msg->hand_id)) {
                            getLogger()->warn("[{}] 忽略 SetAngle1: hand_id={}（本节点 Hand_ID={}）",
                                              backend_.ioNodeName(), msg->hand_id, hid);
                            return;
                        }
                        std::vector<int> vals(msg->joint_values.begin(), msg->joint_values.end());
                        backend_.ioWriteRegister(reg, vals);
                    });
                logger->info("[{}] Subscriber(SetAngle1): {}", backend_.ioNodeName(), tc.command_topic);
            } else if (tc.name == "force_control") {
                maps_.subscribers[tc.command_topic] = this->makeGroupedSubscription<rh524j1_interfaces::msg::SetForce1>(
                    tc.command_topic, 10, [this, reg, hid](rh524j1_interfaces::msg::SetForce1::SharedPtr msg) {
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
                maps_.subscribers[tc.command_topic] = this->makeGroupedSubscription<rh524j1_interfaces::msg::SetSpeed1>(
                    tc.command_topic, 10, [this, reg, hid](rh524j1_interfaces::msg::SetSpeed1::SharedPtr msg) {
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
                    this->makeGroupedSubscription<rh524j1_interfaces::msg::SetCurrent1>(
                        tc.command_topic, 10, [this, reg, hid](rh524j1_interfaces::msg::SetCurrent1::SharedPtr msg) {
                            if (!rosIncomingHandIdTargetsThisNode(backend_, msg->hand_id)) {
                                getLogger()->warn("[{}] 忽略 SetCurrent1: hand_id={}（本节点 Hand_ID={}）",
                                                  backend_.ioNodeName(), msg->hand_id, hid);
                                return;
                            }
                            std::vector<int> vals(msg->joint_values.begin(), msg->joint_values.end());
                            backend_.ioWriteRegister(reg, vals);
                        });
                logger->info("[{}] Subscriber(SetCurrent1): {}", backend_.ioNodeName(), tc.command_topic);
            }
        }
    }
}

void RH524J1InterfaceAdapter::publishRegisterData(const TopicConfig& topic_config, const std::vector<int>& values) {
    const int32_t hid = backend_.ioHandId();
    rclcpp::Node* node = backend_.ioNode();

    if (topic_config.name == "angle_control") {
        auto pub = std::dynamic_pointer_cast<rclcpp::Publisher<rh524j1_interfaces::msg::GetAngleAct1>>(
            maps_.publishers[topic_config.state_topic]);
        if (!pub) {
            return;
        }
        rh524j1_interfaces::msg::GetAngleAct1 msg;
        stamp_header(msg.header, node, config_.publish_frame_id);
        msg.hand_id = hid;
        for (size_t i = 0; i < kRH524J1Joints; ++i) {
            msg.joint_values[i] = (i < values.size()) ? static_cast<int32_t>(values[i]) : 0;
            msg.joint_names[i] = configuredJointName(config_.joint_names, i);
        }
        pub->publish(msg);
        return;
    }

    if (topic_config.name == "force_control") {
        auto pub = std::dynamic_pointer_cast<rclcpp::Publisher<rh524j1_interfaces::msg::GetForceAct1>>(
            maps_.publishers[topic_config.state_topic]);
        if (!pub) {
            return;
        }
        rh524j1_interfaces::msg::GetForceAct1 msg;
        stamp_header(msg.header, node, config_.publish_frame_id);
        msg.hand_id = hid;
        for (size_t i = 0; i < kRH524J1Joints; ++i) {
            msg.joint_values[i] = (i < values.size()) ? static_cast<int32_t>(values[i]) : 0;
            msg.joint_names[i] = configuredJointName(config_.joint_names, i);
        }
        pub->publish(msg);
        return;
    }

    if (topic_config.name == "current_control") {
        auto pub = std::dynamic_pointer_cast<rclcpp::Publisher<rh524j1_interfaces::msg::GetCurrentAct1>>(
            maps_.publishers[topic_config.state_topic]);
        if (!pub) {
            return;
        }
        rh524j1_interfaces::msg::GetCurrentAct1 msg;
        stamp_header(msg.header, node, config_.publish_frame_id);
        msg.hand_id = hid;
        for (size_t i = 0; i < kRH524J1Joints; ++i) {
            msg.joint_values[i] = (i < values.size()) ? static_cast<int32_t>(values[i]) : 0;
            msg.joint_names[i] = configuredJointName(config_.joint_names, i);
        }
        pub->publish(msg);
    }
}

void RH524J1InterfaceAdapter::publishTouchData(const TopicConfig& topic_config, const TouchDataResult& touchData,
                                              int version) {
    (void)topic_config;
    (void)touchData;
    (void)version;
}

void RH524J1InterfaceAdapter::wireServices() {
    auto logger = getLogger();

    for (const auto& sc : config_.services) {
        if (sc.is_write_register && !sc.set_service_name.empty()) {
            const std::string& reg = sc.register_name;

            if (reg == "angleSet") {
                maps_.services[sc.set_service_name] = this->makeGroupedService<rh524j1_interfaces::srv::Setangle>(
                    sc.set_service_name, [this, reg](const rh524j1_interfaces::srv::Setangle::Request::SharedPtr req,
                                                     rh524j1_interfaces::srv::Setangle::Response::SharedPtr res) {
                        if (!rosIncomingHandIdTargetsThisNode(backend_, req->hand_id)) {
                            res->accepted = false;
                            res->message = "rejected: hand_id mismatch";
                            getLogger()->warn("[{}] 拒绝 Setangle: hand_id={}（本节点 Hand_ID={}）",
                                              backend_.ioNodeName(), req->hand_id, backend_.ioHandId());
                            return;
                        }
                        std::vector<int> vals(req->joint_values.begin(), req->joint_values.end());
                        const IoError e = backend_.ioWriteRegister(reg, vals);
                        res->accepted = isOk(e);
                        res->message = toString(e);
                    });
                logger->info("[{}] Service(SetAngle): {}", backend_.ioNodeName(), sc.set_service_name);
            } else if (reg == "id") {
                maps_.services[sc.set_service_name] = this->makeGroupedService<rh524j1_interfaces::srv::Setid>(
                    sc.set_service_name, [this](const rh524j1_interfaces::srv::Setid::Request::SharedPtr req,
                                                rh524j1_interfaces::srv::Setid::Response::SharedPtr res) {
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
                maps_.services[sc.set_service_name] = this->makeGroupedService<rh524j1_interfaces::srv::Setmode>(
                    sc.set_service_name, [this](const rh524j1_interfaces::srv::Setmode::Request::SharedPtr req,
                                                rh524j1_interfaces::srv::Setmode::Response::SharedPtr res) {
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
                maps_.services[sc.set_service_name] = this->makeGroupedService<rh524j1_interfaces::srv::Setforce>(
                    sc.set_service_name, [this](const rh524j1_interfaces::srv::Setforce::Request::SharedPtr req,
                                                rh524j1_interfaces::srv::Setforce::Response::SharedPtr res) {
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
                maps_.services[sc.set_service_name] = this->makeGroupedService<rh524j1_interfaces::srv::Setspeed>(
                    sc.set_service_name, [this](const rh524j1_interfaces::srv::Setspeed::Request::SharedPtr req,
                                                rh524j1_interfaces::srv::Setspeed::Response::SharedPtr res) {
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
                maps_.services[sc.set_service_name] = this->makeGroupedService<rh524j1_interfaces::srv::Setbaudrate>(
                    sc.set_service_name, [this, reg](const rh524j1_interfaces::srv::Setbaudrate::Request::SharedPtr req,
                                                     rh524j1_interfaces::srv::Setbaudrate::Response::SharedPtr res) {
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
                maps_.services[sc.set_service_name] = this->makeGroupedService<rh524j1_interfaces::srv::Setclearerror>(
                    sc.set_service_name,
                    [this, reg](const rh524j1_interfaces::srv::Setclearerror::Request::SharedPtr req,
                                rh524j1_interfaces::srv::Setclearerror::Response::SharedPtr res) {
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
            } else if (reg == "resetPara") {
                maps_.services[sc.set_service_name] = this->makeGroupedService<rh524j1_interfaces::srv::Setresetpara>(
                    sc.set_service_name, [this, reg](const rh524j1_interfaces::srv::Setresetpara::Request::SharedPtr req,
                                                     rh524j1_interfaces::srv::Setresetpara::Response::SharedPtr res) {
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
                    this->makeGroupedService<rh524j1_interfaces::srv::Setgestureforceclb>(
                        sc.set_service_name,
                        [this, reg](const rh524j1_interfaces::srv::Setgestureforceclb::Request::SharedPtr req,
                                    rh524j1_interfaces::srv::Setgestureforceclb::Response::SharedPtr res) {
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
                maps_.services[sc.set_service_name] = this->makeGroupedService<rh524j1_interfaces::srv::Setdefaultspeed>(
                    sc.set_service_name,
                    [this, reg](const rh524j1_interfaces::srv::Setdefaultspeed::Request::SharedPtr req,
                                rh524j1_interfaces::srv::Setdefaultspeed::Response::SharedPtr res) {
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
                    this->makeGroupedService<rh524j1_interfaces::srv::Setdefaultforceset>(
                        sc.set_service_name,
                        [this, reg](const rh524j1_interfaces::srv::Setdefaultforceset::Request::SharedPtr req,
                                    rh524j1_interfaces::srv::Setdefaultforceset::Response::SharedPtr res) {
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
                maps_.services[sc.set_service_name] = this->makeGroupedService<rh524j1_interfaces::srv::Setpause>(
                    sc.set_service_name, [this, reg](const rh524j1_interfaces::srv::Setpause::Request::SharedPtr req,
                                                     rh524j1_interfaces::srv::Setpause::Response::SharedPtr res) {
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
                maps_.services[sc.set_service_name] = this->makeGroupedService<rh524j1_interfaces::srv::Setstop>(
                    sc.set_service_name, [this, reg](const rh524j1_interfaces::srv::Setstop::Request::SharedPtr req,
                                                     rh524j1_interfaces::srv::Setstop::Response::SharedPtr res) {
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
            } else {
                throw std::runtime_error("[RH524J1] 未映射的写寄存器服务: " + reg +
                                         "，请在 rh524j1_interfaces 增加专用 .srv 并接线");
            }
        }

        if (!sc.get_service_name.empty()) {
            const std::string& reg = sc.register_name;

            if (reg == "errorCode") {
                maps_.services[sc.get_service_name] = this->makeGroupedService<rh524j1_interfaces::srv::Geterror>(
                    sc.get_service_name, [this](const rh524j1_interfaces::srv::Geterror::Request::SharedPtr req,
                                                rh524j1_interfaces::srv::Geterror::Response::SharedPtr res) {
                        if (!rosIncomingHandIdTargetsThisNode(backend_, req->hand_id)) {
                            getLogger()->warn("[{}] 拒绝 Geterror: hand_id={}（本节点 Hand_ID={}）",
                                              backend_.ioNodeName(), req->hand_id, backend_.ioHandId());
                            for (size_t i = 0; i < kRH524J1Joints; ++i) {
                                res->joint_values[i] = 0;
                                res->joint_names[i] = "";
                            }
                            res->message = "rejected: hand_id mismatch";
                            return;
                        }
                        auto rr = backend_.ioReadRegister("errorCode");
                        const bool ok = rr.ok();
                        const auto& vals = rr.values;
                        for (size_t i = 0; i < kRH524J1Joints; ++i) {
                            res->joint_values[i] = (ok && i < vals.size()) ? static_cast<int32_t>(vals[i]) : 0;
                            res->joint_names[i] = configuredJointName(config_.joint_names, i);
                        }
                        res->message = toString(rr.error);
                    });
                logger->info("[{}] Service(GetError): {}", backend_.ioNodeName(), sc.get_service_name);
            } else if (reg == "temp") {
                maps_.services[sc.get_service_name] = this->makeGroupedService<rh524j1_interfaces::srv::Gettemp>(
                    sc.get_service_name, [this](const rh524j1_interfaces::srv::Gettemp::Request::SharedPtr req,
                                                rh524j1_interfaces::srv::Gettemp::Response::SharedPtr res) {
                        if (!rosIncomingHandIdTargetsThisNode(backend_, req->hand_id)) {
                            getLogger()->warn("[{}] 拒绝 Gettemp: hand_id={}（本节点 Hand_ID={}）",
                                              backend_.ioNodeName(), req->hand_id, backend_.ioHandId());
                            for (size_t i = 0; i < kRH524J1Joints; ++i) {
                                res->joint_values[i] = 0;
                                res->joint_names[i] = "";
                            }
                            res->message = "rejected: hand_id mismatch";
                            return;
                        }
                        auto rr = backend_.ioReadRegister("temp");
                        const bool ok = rr.ok();
                        const auto& vals = rr.values;
                        for (size_t i = 0; i < kRH524J1Joints; ++i) {
                            res->joint_values[i] = (ok && i < vals.size()) ? static_cast<int32_t>(vals[i]) : 0;
                            res->joint_names[i] = configuredJointName(config_.joint_names, i);
                        }
                        res->message = toString(rr.error);
                    });
                logger->info("[{}] Service(GetTemp): {}", backend_.ioNodeName(), sc.get_service_name);
            } else if (reg == "status") {
                maps_.services[sc.get_service_name] = this->makeGroupedService<rh524j1_interfaces::srv::Getstatus>(
                    sc.get_service_name, [this](const rh524j1_interfaces::srv::Getstatus::Request::SharedPtr req,
                                                rh524j1_interfaces::srv::Getstatus::Response::SharedPtr res) {
                        if (!rosIncomingHandIdTargetsThisNode(backend_, req->hand_id)) {
                            getLogger()->warn("[{}] 拒绝 Getstatus: hand_id={}（本节点 Hand_ID={}）",
                                              backend_.ioNodeName(), req->hand_id, backend_.ioHandId());
                            for (size_t i = 0; i < kRH524J1Joints; ++i) {
                                res->joint_values[i] = 0;
                                res->joint_names[i] = "";
                            }
                            res->message = "rejected: hand_id mismatch";
                            return;
                        }
                        auto rr = backend_.ioReadRegister("status");
                        const bool ok = rr.ok();
                        const auto& vals = rr.values;
                        for (size_t i = 0; i < kRH524J1Joints; ++i) {
                            res->joint_values[i] = (ok && i < vals.size()) ? static_cast<int32_t>(vals[i]) : 0;
                            res->joint_names[i] = configuredJointName(config_.joint_names, i);
                        }
                        res->message = toString(rr.error);
                    });
                logger->info("[{}] Service(Getstatus): {}", backend_.ioNodeName(), sc.get_service_name);
            } else if (reg == "currentAct") {
                maps_.services[sc.get_service_name] = this->makeGroupedService<rh524j1_interfaces::srv::Getcurrentact>(
                    sc.get_service_name, [this](const rh524j1_interfaces::srv::Getcurrentact::Request::SharedPtr req,
                                                rh524j1_interfaces::srv::Getcurrentact::Response::SharedPtr res) {
                        if (!rosIncomingHandIdTargetsThisNode(backend_, req->hand_id)) {
                            getLogger()->warn("[{}] 拒绝 Getcurrentact: hand_id={}（本节点 Hand_ID={}）",
                                              backend_.ioNodeName(), req->hand_id, backend_.ioHandId());
                            for (size_t i = 0; i < kRH524J1Joints; ++i) {
                                res->joint_values[i] = 0;
                                res->joint_names[i] = "";
                            }
                            res->message = "rejected: hand_id mismatch";
                            return;
                        }
                        auto rr = backend_.ioReadRegister("currentAct");
                        const bool ok = rr.ok();
                        const auto& vals = rr.values;
                        for (size_t i = 0; i < kRH524J1Joints; ++i) {
                            res->joint_values[i] = (ok && i < vals.size()) ? static_cast<int32_t>(vals[i]) : 0;
                            res->joint_names[i] = configuredJointName(config_.joint_names, i);
                        }
                        res->message = toString(rr.error);
                    });
                logger->info("[{}] Service(Getcurrentact): {}", backend_.ioNodeName(), sc.get_service_name);
            } else if (reg == "forceAct") {
                maps_.services[sc.get_service_name] = this->makeGroupedService<rh524j1_interfaces::srv::Getforceact>(
                    sc.get_service_name, [this](const rh524j1_interfaces::srv::Getforceact::Request::SharedPtr req,
                                                rh524j1_interfaces::srv::Getforceact::Response::SharedPtr res) {
                        if (!rosIncomingHandIdTargetsThisNode(backend_, req->hand_id)) {
                            getLogger()->warn("[{}] 拒绝 Getforceact: hand_id={}（本节点 Hand_ID={}）",
                                              backend_.ioNodeName(), req->hand_id, backend_.ioHandId());
                            for (size_t i = 0; i < kRH524J1Joints; ++i) {
                                res->joint_values[i] = 0;
                                res->joint_names[i] = "";
                            }
                            res->message = "rejected: hand_id mismatch";
                            return;
                        }
                        auto rr = backend_.ioReadRegister("forceAct");
                        const bool ok = rr.ok();
                        const auto& vals = rr.values;
                        for (size_t i = 0; i < kRH524J1Joints; ++i) {
                            res->joint_values[i] = (ok && i < vals.size()) ? static_cast<int32_t>(vals[i]) : 0;
                            res->joint_names[i] = configuredJointName(config_.joint_names, i);
                        }
                        res->message = toString(rr.error);
                    });
                logger->info("[{}] Service(Getforceact): {}", backend_.ioNodeName(), sc.get_service_name);
            } else if (reg == "angleAct") {
                maps_.services[sc.get_service_name] = this->makeGroupedService<rh524j1_interfaces::srv::Getangleact>(
                    sc.get_service_name, [this](const rh524j1_interfaces::srv::Getangleact::Request::SharedPtr req,
                                                rh524j1_interfaces::srv::Getangleact::Response::SharedPtr res) {
                        if (!rosIncomingHandIdTargetsThisNode(backend_, req->hand_id)) {
                            getLogger()->warn("[{}] 拒绝 Getangleact: hand_id={}（本节点 Hand_ID={}）",
                                              backend_.ioNodeName(), req->hand_id, backend_.ioHandId());
                            for (size_t i = 0; i < kRH524J1Joints; ++i) {
                                res->joint_values[i] = 0;
                                res->joint_names[i] = "";
                            }
                            res->message = "rejected: hand_id mismatch";
                            return;
                        }
                        auto rr = backend_.ioReadRegister("angleAct");
                        const bool ok = rr.ok();
                        const auto& vals = rr.values;
                        for (size_t i = 0; i < kRH524J1Joints; ++i) {
                            res->joint_values[i] = (ok && i < vals.size()) ? static_cast<int32_t>(vals[i]) : 0;
                            res->joint_names[i] = configuredJointName(config_.joint_names, i);
                        }
                        res->message = toString(rr.error);
                    });
                logger->info("[{}] Service(Getangleact): {}", backend_.ioNodeName(), sc.get_service_name);
            } else {
                throw std::runtime_error("[RH524J1] 未映射的读寄存器服务: " + reg +
                                         "，请在 rh524j1_interfaces 增加专用 .srv 并接线");
            }
        }
    }
}

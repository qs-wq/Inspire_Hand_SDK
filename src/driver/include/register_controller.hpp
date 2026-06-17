#pragma once

#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "device_interface_types.hpp"
#include "device_worker.hpp"
#include "interface_adapter.hpp"
#include "protocol.hpp"
#include "register_io_backend.hpp"
#include "ring_buffer.hpp"
#include "serial_port.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief 设备节点：定时循环读写寄存器；ROS 消息/服务类型由 InterfaceAdapter 按产品线实现。
 */
class RegisterController : public rclcpp::Node, public IRegisterIoBackend {
public:
    RegisterController(
        const std::string& node_name,
        const DeviceNodeConfig& config,
        std::shared_ptr<SerialPortBase> device,
        std::shared_ptr<Protocol> protocol);

    ~RegisterController() override;

    void initialize();
    void start();
    void stop();

    const DeviceNodeConfig& getConfig() const { return config_; }

    // --- IRegisterIoBackend ---
    rclcpp::Node* ioNode() override { return this; }

    rclcpp::CallbackGroup::SharedPtr ioServiceCallbackGroup() override {
        return service_cb_group_;
    }

    RegisterReadResult ioReadRegister(
        const std::string& register_name, size_t length = 0) override;

    IoError ioWriteRegister(
        const std::string& register_name, const std::vector<int>& values) override;

    SequenceResult ioWriteSequence(const std::vector<WriteStep>& steps) override;

    TouchReadResult ioReadTouchData(int version) override;

    int32_t ioHandId() const override;

    std::string ioNodeName() const override;

protected:
    RegisterReadResult readRegister(const std::string& reg_name, size_t length = 0);

    IoError writeRegister(const std::string& reg_name, const std::vector<int>& values);

    TouchReadResult readTouchData(int version = 0);

    void controlLoop();

    DeviceNodeConfig config_;
    std::shared_ptr<SerialPortBase> device_;
    std::shared_ptr<Protocol> protocol_;
    std::unique_ptr<RingBuffer> ring_buffer_;

    std::map<std::string, rclcpp::PublisherBase::SharedPtr> publishers_;
    std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> subscribers_;
    std::map<std::string, rclcpp::ServiceBase::SharedPtr> services_;

private:
    std::unique_ptr<InterfaceAdapter> interface_adapter_;

    // 每设备串口事务串行化执行器：所有读写均经此单线程顺序执行，保证事务原子。
    std::unique_ptr<DeviceWorker> worker_;

    // 回调组：定时器与服务/订阅分组，使二者可在多线程执行器下并行进入。
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
    rclcpp::CallbackGroup::SharedPtr service_cb_group_;

    rclcpp::TimerBase::SharedPtr control_timer_;
    std::atomic<bool> running_{false};

    void timerCallback();

    // 定时读背压：上一次读任务尚未完成时跳过本次提交，避免队列堆积。
    std::atomic<bool> read_in_flight_{false};
};

#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "protocol.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief 组合写序列中的一步：寄存器名 + 写入值 + 该步执行后的间隔（毫秒）。
 *
 * 用于 EG-5CD1 力控/触控等需要「按顺序写多个寄存器、步骤间留固定间隔」的组合操作。
 * 整组步骤会在设备 worker 上作为单个原子任务串行执行，期间不会与定时读交错。
 */
struct WriteStep {
    std::string reg;
    std::vector<int> values;
    int post_delay_ms = 0;  ///< 本步写完后的等待时长（在 worker 线程上 sleep）
};

/**
 * @brief 组合写序列结果。error==IoError::Ok 表示全部成功；否则 failed_step 指出失败步序号。
 */
struct SequenceResult {
    size_t failed_step = 0;          ///< 失败步在序列中的下标（error!=Ok 时有效）
    IoError error = IoError::Ok;     ///< 第一个失败步的错误码；Ok 表示整组成功

    bool ok() const { return error == IoError::Ok; }
};

/**
 * @brief 寄存器读写能力抽象，供 InterfaceAdapter 调用，与具体 ROS 节点解耦。
 *
 * 所有读写均经由设备 worker 串行化执行（见 DeviceWorker），从结构上保证串口事务原子，
 * 因此不再需要旧的 ioPauseTimer 暂停定时器机制。
 */
class IRegisterIoBackend {
public:
    virtual ~IRegisterIoBackend() = default;

    virtual rclcpp::Node* ioNode() = 0;

    /** @brief 服务/订阅回调应归入的回调组（与定时器分组以实现并发）。 */
    virtual rclcpp::CallbackGroup::SharedPtr ioServiceCallbackGroup() = 0;

    virtual RegisterReadResult ioReadRegister(
        const std::string& register_name, size_t length = 0) = 0;

    virtual IoError ioWriteRegister(
        const std::string& register_name, const std::vector<int>& values) = 0;

    /**
     * @brief 原子化执行一组写步骤（遇到第一个失败即停止）。
     * @param steps 写步骤序列（含每步后的间隔）。
     * @return SequenceResult：全部成功或首个失败步及其错误码。
     */
    virtual SequenceResult ioWriteSequence(const std::vector<WriteStep>& steps) = 0;

    virtual TouchReadResult ioReadTouchData(int version) = 0;

    virtual int32_t ioHandId() const = 0;

    virtual std::string ioNodeName() const = 0;
};

/**
 * @brief 发布/订阅/服务实体存放处（由 RegisterController 持有，适配器仅引用）。
 */
struct RosEntityMaps {
    std::map<std::string, rclcpp::PublisherBase::SharedPtr>& publishers;
    std::map<std::string, rclcpp::SubscriptionBase::SharedPtr>& subscribers;
    std::map<std::string, rclcpp::ServiceBase::SharedPtr>& services;
};

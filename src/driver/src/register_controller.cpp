#include "register_controller.hpp"
#include "logger_manager.hpp"

#include <chrono>
#include <thread>
#include <utility>

RegisterController::RegisterController(const std::string& node_name, const DeviceNodeConfig& config,
                                       std::shared_ptr<SerialPortBase> device, std::shared_ptr<Protocol> protocol)
    : rclcpp::Node(node_name), config_(config), device_(device), protocol_(protocol),
      ring_buffer_(std::make_unique<RingBuffer>(1024)), worker_(std::make_unique<DeviceWorker>(config.device_name)) {
    LoggerManager::setThreadDeviceName(config_.device_name);

    // 定时器与服务/订阅分到不同回调组：定时器组互斥（同一时刻仅一个读任务在排队），
    // 服务组可重入，使服务回调中对 worker future 的等待不会阻塞定时器线程；
    // 真正的串口事务串行化由 worker 单线程保证。
    timer_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    auto logger = getLogger();
    logger->info("[{}] RegisterController创建: device={}, profile={}, topics={}, services={}", node_name,
                 config_.device_name, config_.interfaces_profile, config_.topics.size(), config_.services.size());
}

RegisterController::~RegisterController() {
    stop();
    // 先停 worker（确保不再有任务触碰 protocol_/device_/ring_buffer_），再释放适配器。
    if (worker_) {
        worker_->stop();
    }
    interface_adapter_.reset();
    auto logger = getLogger();
    logger->info("[{}] RegisterController销毁", get_name());
}

void RegisterController::initialize() {
    auto logger = getLogger();
    logger->info("[{}] 初始化设备节点 (interfaces_profile={})...", get_name(), config_.interfaces_profile);

    RosEntityMaps maps{publishers_, subscribers_, services_};
    interface_adapter_ = makeInterfaceAdapter(config_.interfaces_profile, *this, config_, maps);

    if (!config_.topics.empty()) {
        interface_adapter_->wireTopics();
    }

    if (!config_.services.empty()) {
        interface_adapter_->wireServices();
    }

    logger->info("[{}] 设备节点初始化完成", get_name());
}

void RegisterController::applyInitialRegisters() {
    if (config_.initial_registers.empty()) {
        return;
    }

    auto logger = getLogger();
    for (const auto& initial : config_.initial_registers) {
        try {
            const IoError err = worker_
                                    ->submit([this, initial]() {
                                        return writeRegister(initial.register_name, initial.values);
                                    })
                                    .get();
            if (isOk(err)) {
                logger->info("[{}] 启动初始写入 {} 成功 ({} 个值)", get_name(), initial.register_name,
                             initial.values.size());
            } else {
                logger->warn("[{}] 启动初始写入 {} 失败: {}", get_name(), initial.register_name, toString(err));
            }
        } catch (const std::exception& e) {
            logger->warn("[{}] 启动初始写入 {} 异常: {}", get_name(), initial.register_name, e.what());
        }
    }
}

void RegisterController::start() {
    if (running_.load()) {
        return;
    }

    applyInitialRegisters();

    running_.store(true);

    auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / config_.update_rate));

    control_timer_ = create_wall_timer(period, std::bind(&RegisterController::timerCallback, this), timer_cb_group_);

    auto logger = getLogger();
    logger->info("[{}] 设备节点已启动，更新频率: {} Hz", get_name(), config_.update_rate);
}

void RegisterController::stop() {
    if (!running_.load()) {
        return;
    }

    running_.store(false);

    if (control_timer_) {
        control_timer_->cancel();
        control_timer_.reset();
    }

    auto logger = getLogger();
    logger->info("[{}] 设备节点已停止", get_name());
}

void RegisterController::timerCallback() {
    if (!running_.load()) {
        return;
    }

    // 背压/合并：上一次读任务尚未完成则跳过本次，避免 worker 队列堆积。
    bool expected = false;
    if (!read_in_flight_.compare_exchange_strong(expected, true)) {
        return;
    }

    try {
        worker_->submit([this]() {
            try {
                controlLoop();
            } catch (const std::exception& e) {
                auto logger = getLogger();
                logger->error("[{}] 控制循环异常: {}", get_name(), e.what());
            }
            read_in_flight_.store(false);
        });
    } catch (const std::exception& e) {
        // worker 已停止等异常：复位标志，避免永久卡死后续定时读。
        read_in_flight_.store(false);
        auto logger = getLogger();
        logger->warn("[{}] 提交定时读任务失败: {}", get_name(), e.what());
    }
}

void RegisterController::controlLoop() {
    if (!interface_adapter_) {
        return;
    }

    for (const auto& topic_config : config_.topics) {
        for (const auto& reg_name : topic_config.read_registers) {
            if (reg_name == "touchAct") {
                auto touchResult = readTouchData(topic_config.touch_version);
                if (touchResult.ok()) {
                    interface_adapter_->publishTouchData(topic_config, touchResult.data, topic_config.touch_version);
                }
            } else {
                auto readResult = readRegister(reg_name);
                if (readResult.ok()) {
                    interface_adapter_->publishRegisterData(topic_config, readResult.values);
                }
            }
        }
    }
}

// 以下三个 raw 助手仅在 worker 线程上执行（由 ioXxx / controlLoop 调用），
// 直接访问 device_/ring_buffer_/protocol_，无需额外加锁。事务起始清空串口 RX 残留。
RegisterReadResult RegisterController::readRegister(const std::string& reg_name, size_t length) {
    try {
        device_->clearBuffer();
        return protocol_->readRegister(device_, *ring_buffer_, reg_name, length);
    } catch (const std::exception& e) {
        auto logger = getLogger();
        logger->error("[{}] 读取寄存器 {} 失败: {}", get_name(), reg_name, e.what());
        return {IoError::DeviceError, {}};
    }
}

IoError RegisterController::writeRegister(const std::string& reg_name, const std::vector<int>& values) {
    try {
        device_->clearBuffer();
        return protocol_->writeRegister(device_, reg_name, values);
    } catch (const std::exception& e) {
        auto logger = getLogger();
        logger->error("[{}] 写入寄存器 {} 失败: {}", get_name(), reg_name, e.what());
        return IoError::DeviceError;
    }
}

TouchReadResult RegisterController::readTouchData(int version) {
    try {
        device_->clearBuffer();
        return protocol_->readTouchData(device_, *ring_buffer_, version);
    } catch (const std::exception& e) {
        auto logger = getLogger();
        logger->error("[{}] 读取触觉数据失败: {}", get_name(), e.what());
        return {IoError::DeviceError, {}};
    }
}

RegisterReadResult RegisterController::ioReadRegister(const std::string& register_name, size_t length) {
    try {
        return worker_->submit([this, register_name, length]() { return readRegister(register_name, length); }).get();
    } catch (const std::exception& e) {
        auto logger = getLogger();
        logger->error("[{}] 读取寄存器 {} 任务失败: {}", get_name(), register_name, e.what());
        return {IoError::DeviceError, {}};
    }
}

IoError RegisterController::ioWriteRegister(const std::string& register_name, const std::vector<int>& values) {
    try {
        return worker_->submit([this, register_name, values]() { return writeRegister(register_name, values); }).get();
    } catch (const std::exception& e) {
        auto logger = getLogger();
        logger->error("[{}] 写入寄存器 {} 任务失败: {}", get_name(), register_name, e.what());
        return IoError::DeviceError;
    }
}

SequenceResult RegisterController::ioWriteSequence(const std::vector<WriteStep>& steps) {
    try {
        return worker_
            ->submit([this, steps]() -> SequenceResult {
                SequenceResult result;
                for (size_t i = 0; i < steps.size(); ++i) {
                    const auto& step = steps[i];
                    const IoError e = writeRegister(step.reg, step.values);
                    if (!isOk(e)) {
                        result.failed_step = i;
                        result.error = e;
                        return result;
                    }
                    if (step.post_delay_ms > 0) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(step.post_delay_ms));
                    }
                }
                return result; // error 默认 Ok
            })
            .get();
    } catch (const std::exception& e) {
        auto logger = getLogger();
        logger->error("[{}] 组合写序列任务失败: {}", get_name(), e.what());
        return {0, IoError::DeviceError};
    }
}

TouchReadResult RegisterController::ioReadTouchData(int version) {
    try {
        return worker_->submit([this, version]() { return readTouchData(version); }).get();
    } catch (const std::exception& e) {
        auto logger = getLogger();
        logger->error("[{}] 读取触觉数据任务失败: {}", get_name(), e.what());
        return {IoError::DeviceError, {}};
    }
}

int32_t RegisterController::ioHandId() const { return static_cast<int32_t>(protocol_->getDeviceId()); }

std::string RegisterController::ioNodeName() const { return std::string(get_name()); }

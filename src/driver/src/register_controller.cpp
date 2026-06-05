#include "register_controller.hpp"
#include "logger_manager.hpp"

#include <chrono>
#include <utility>

RegisterController::RegisterController(
    const std::string& node_name,
    const DeviceNodeConfig& config,
    std::shared_ptr<SerialPortBase> device,
    std::shared_ptr<Protocol> protocol)
    : rclcpp::Node(node_name),
      config_(config),
      device_(device),
      protocol_(protocol),
      ring_buffer_(std::make_unique<RingBuffer>(1024))
{
    LoggerManager::setThreadDeviceName(config_.device_name);

    auto logger = getLogger();
    logger->info("[{}] RegisterController创建: device={}, profile={}, topics={}, services={}",
        node_name, config_.device_name, config_.interfaces_profile,
        config_.topics.size(), config_.services.size());
}

RegisterController::~RegisterController() {
    stop();
    interface_adapter_.reset();
    auto logger = getLogger();
    logger->info("[{}] RegisterController销毁", get_name());
}

void RegisterController::initialize() {
    auto logger = getLogger();
    logger->info("[{}] 初始化设备节点 (interfaces_profile={})...", get_name(),
        config_.interfaces_profile);

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

void RegisterController::start() {
    if (running_.load()) {
        return;
    }

    running_.store(true);

    auto period = std::chrono::milliseconds(
        static_cast<int>(1000.0 / config_.update_rate));

    control_timer_ = create_wall_timer(
        period,
        std::bind(&RegisterController::timerCallback, this));

    auto logger = getLogger();
    logger->info("[{}] 设备节点已启动，更新频率: {} Hz",
        get_name(), config_.update_rate);
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

    if (timer_paused_.load()) {
        auto now = std::chrono::steady_clock::now();
        if (now < timer_resume_time_) {
            return;
        }
        timer_paused_.store(false);
    }

    try {
        controlLoop();
    } catch (const std::exception& e) {
        auto logger = getLogger();
        logger->error("[{}] 控制循环异常: {}", get_name(), e.what());
    }
}

void RegisterController::pauseTimer(int duration_ms) {
    timer_paused_.store(true);
    timer_resume_time_ = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(duration_ms);

    auto logger = getLogger();
    logger->debug("[{}] 定时器已暂停 {} ms", get_name(), duration_ms);
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
                    interface_adapter_->publishTouchData(
                        topic_config, touchResult.data, topic_config.touch_version);
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

RegisterReadResult RegisterController::readRegister(
    const std::string& reg_name,
    size_t length)
{
    try {
        return protocol_->readRegister(device_, *ring_buffer_, reg_name, length);
    } catch (const std::exception& e) {
        auto logger = getLogger();
        logger->error("[{}] 读取寄存器 {} 失败: {}",
            get_name(), reg_name, e.what());
        return {IoError::DeviceError, {}};
    }
}

IoError RegisterController::writeRegister(
    const std::string& reg_name,
    const std::vector<int>& values)
{
    try {
        return protocol_->writeRegister(device_, reg_name, values);
    } catch (const std::exception& e) {
        auto logger = getLogger();
        logger->error("[{}] 写入寄存器 {} 失败: {}",
            get_name(), reg_name, e.what());
        return IoError::DeviceError;
    }
}

TouchReadResult RegisterController::readTouchData(int version) {
    try {
        return protocol_->readTouchData(device_, *ring_buffer_, version);
    } catch (const std::exception& e) {
        auto logger = getLogger();
        logger->error("[{}] 读取触觉数据失败: {}", get_name(), e.what());
        return {IoError::DeviceError, {}};
    }
}

RegisterReadResult RegisterController::ioReadRegister(
    const std::string& register_name,
    size_t length)
{
    return readRegister(register_name, length);
}

IoError RegisterController::ioWriteRegister(
    const std::string& register_name,
    const std::vector<int>& values)
{
    return writeRegister(register_name, values);
}

TouchReadResult RegisterController::ioReadTouchData(int version) {
    return readTouchData(version);
}

void RegisterController::ioPauseTimer(int duration_ms) {
    pauseTimer(duration_ms);
}

int32_t RegisterController::ioHandId() const {
    return static_cast<int32_t>(protocol_->getDeviceId());
}

std::string RegisterController::ioNodeName() const {
    return std::string(get_name());
}

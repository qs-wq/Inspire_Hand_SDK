#include "logger_manager.hpp"
#include <cctype>                     // 用于toupper
#include <filesystem>                 // C++17标准库，需要CMakeLists.txt中设置C++17
#include <spdlog/pattern_formatter.h> // 用于自定义格式化器
#include <sstream>
#include <yaml-cpp/yaml.h>

// 静态成员初始化
std::mutex LoggerManager::mutex_;
LoggerManager::LogConfig LoggerManager::current_config_;

// 线程局部存储：每个线程独立的设备名
thread_local std::string thread_device_name_;

spdlog::level::level_enum LoggerManager::parseLogLevel(const std::string& level_str) {
    std::string upper_level = level_str;
    // 转换为大写
    for (auto& c : upper_level) {
        c = std::toupper(c);
    }

    if (upper_level == "TRACE")
        return spdlog::level::trace;
    if (upper_level == "DEBUG")
        return spdlog::level::debug;
    if (upper_level == "INFO")
        return spdlog::level::info;
    if (upper_level == "WARN" || upper_level == "WARNING")
        return spdlog::level::warn;
    if (upper_level == "ERROR")
        return spdlog::level::err;
    if (upper_level == "CRITICAL")
        return spdlog::level::critical;
    if (upper_level == "OFF")
        return spdlog::level::off;

    // 默认返回INFO级别
    return spdlog::level::info;
}

void LoggerManager::createSinks(const LogConfig& config, std::vector<spdlog::sink_ptr>& sinks) {
    sinks.clear();

    // 控制台输出sink
    if (config.console_enable) {
        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        console_sink->set_level(spdlog::level::trace); // 控制台显示所有级别
        sinks.push_back(console_sink);
    }

    // 文件输出sink（使用轮转）
    if (config.file_enable) {
        // 确保日志目录存在
        std::filesystem::path log_path(config.file_path);
        auto log_dir = log_path.parent_path();
        if (!log_dir.empty() && !std::filesystem::exists(log_dir)) {
            std::filesystem::create_directories(log_dir);
        }

        // 使用轮转文件sink
        auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(config.file_path, config.max_file_size,
                                                                                config.max_files);
        file_sink->set_level(spdlog::level::trace); // 文件记录所有级别
        sinks.push_back(file_sink);
    }
}

void LoggerManager::updateLogger(const std::vector<spdlog::sink_ptr>& sinks, spdlog::level::level_enum level) {
    // 创建自定义格式化器，支持设备名前缀
    class DeviceNameFormatter : public spdlog::custom_flag_formatter {
    public:
        void format(const spdlog::details::log_msg& msg, const std::tm&, spdlog::memory_buf_t& dest) override {
            // 获取线程局部存储的设备名
            extern thread_local std::string thread_device_name_;
            if (!thread_device_name_.empty()) {
                std::string prefix = "[" + thread_device_name_ + "] ";
                dest.append(prefix.data(), prefix.data() + prefix.size());
            }
        }

        std::unique_ptr<spdlog::custom_flag_formatter> clone() const override {
            return spdlog::details::make_unique<DeviceNameFormatter>();
        }
    };

    // 创建新的logger
    auto logger = std::make_shared<spdlog::logger>("multi_sink_logger", sinks.begin(), sinks.end());
    logger->set_level(level);

    // 创建格式化器并注册自定义标志 'D' 用于设备名
    auto formatter = std::make_unique<spdlog::pattern_formatter>();
    formatter->add_flag<DeviceNameFormatter>('D');
    formatter->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %D%v");
    logger->set_formatter(std::move(formatter));

    // 设置默认logger
    spdlog::set_default_logger(logger);
}

void LoggerManager::initialize(const LogConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);

    current_config_ = config;
    auto level = parseLogLevel(config.level);

    std::vector<spdlog::sink_ptr> sinks;
    createSinks(config, sinks);

    if (!sinks.empty()) {
        updateLogger(sinks, level);

        // 输出初始化信息
        auto logger = getLogger();
        logger->info("日志系统初始化成功 - 级别: {}, 文件: {}, 最大大小: {}MB, 保留文件数: {}", config.level,
                     config.file_path, config.max_file_size / (1024 * 1024), config.max_files);
    } else {
        throw std::runtime_error("日志系统初始化失败：没有可用的日志输出目标");
    }
}

void LoggerManager::initialize(const YAML::Node& logging_node) {
    LogConfig config;

    if (logging_node["level"]) {
        config.level = logging_node["level"].as<std::string>();
    }

    if (logging_node["file"]) {
        config.file_path = logging_node["file"].as<std::string>();
    }

    // 控制台输出开关（默认开启）
    if (logging_node["console"]) {
        config.console_enable = logging_node["console"].as<bool>();
    }

    // 文件输出开关（默认开启）
    if (logging_node["file_enable"]) {
        config.file_enable = logging_node["file_enable"].as<bool>();
    }

    // 日志文件最大大小（MB）
    if (logging_node["max_file_size_mb"]) {
        config.max_file_size = logging_node["max_file_size_mb"].as<size_t>() * 1024 * 1024;
    }

    // 保留的日志文件数量
    if (logging_node["max_files"]) {
        config.max_files = logging_node["max_files"].as<size_t>();
    }

    initialize(config);
}

void LoggerManager::setLogLevel(const std::string& level) { setLogLevel(parseLogLevel(level)); }

void LoggerManager::setLogLevel(spdlog::level::level_enum level) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto logger = getLogger();
    if (logger) {
        logger->set_level(level);
        current_config_.level = [level]() -> std::string {
            switch (level) {
            case spdlog::level::trace:
                return "TRACE";
            case spdlog::level::debug:
                return "DEBUG";
            case spdlog::level::info:
                return "INFO";
            case spdlog::level::warn:
                return "WARN";
            case spdlog::level::err:
                return "ERROR";
            case spdlog::level::critical:
                return "CRITICAL";
            case spdlog::level::off:
                return "OFF";
            default:
                return "INFO";
            }
        }();

        logger->info("日志级别已更新为: {}", current_config_.level);
    }
}

std::string LoggerManager::getLogLevel() {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_config_.level;
}

std::shared_ptr<spdlog::logger> LoggerManager::getLogger() {
    auto logger = spdlog::default_logger();
    if (!logger || logger->name() == "null_logger") {
        // 如果没有初始化，创建一个默认的logger
        std::lock_guard<std::mutex> lock(mutex_);
        LogConfig default_config;
        initialize(default_config);
        logger = spdlog::default_logger();
    }
    return logger;
}

void LoggerManager::reconfigure(const LogConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);

    // 记录重新配置信息
    auto old_logger = getLogger();
    if (old_logger) {
        old_logger->info("重新配置日志系统...");
    }

    initialize(config);
}

void LoggerManager::flush() {
    auto logger = getLogger();
    if (logger) {
        logger->flush();
    }
}

void LoggerManager::shutdown() {
    std::lock_guard<std::mutex> lock(mutex_);
    spdlog::shutdown();
}

void LoggerManager::setThreadDeviceName(const std::string& device_name) { thread_device_name_ = device_name; }

std::string LoggerManager::getThreadDeviceName() { return thread_device_name_; }

void LoggerManager::clearThreadDeviceName() { thread_device_name_.clear(); }

// 便捷函数实现
std::shared_ptr<spdlog::logger> getLogger() { return LoggerManager::getLogger(); }

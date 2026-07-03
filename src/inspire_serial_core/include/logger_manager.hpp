#pragma once

#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// 前向声明
namespace YAML {
class Node;
}

/**
 * @brief 日志管理器类
 *
 * 提供统一的日志管理功能：
 * - 支持日志文件轮转（按大小和数量）
 * - 支持动态调整日志级别
 * - 支持结构化日志输出
 * - 线程安全的日志配置
 */
class LoggerManager {
public:
    /**
     * @brief 日志配置结构
     */
    struct LogConfig {
        std::string level = "INFO";                                   // 日志级别：DEBUG/INFO/WARN/ERROR
        std::string file_path = "logs/app.log";                       // 日志文件路径
        bool console_enable = true;                                   // 是否输出到控制台
        bool file_enable = true;                                      // 是否输出到文件
        size_t max_file_size = static_cast<size_t>(10) * 1024 * 1024; // 单个日志文件最大大小（字节），默认10MB
        size_t max_files = 5;                                         // 保留的日志文件数量，默认5个
    };

    /**
     * @brief 初始化日志系统（从配置加载）
     * @param config 日志配置
     */
    static void initialize(const LogConfig& config);

    /**
     * @brief 初始化日志系统（从YAML节点加载）
     * @param logging_node YAML日志配置节点
     */
    static void initialize(const YAML::Node& logging_node);

    /**
     * @brief 动态设置日志级别
     * @param level 日志级别字符串：DEBUG/INFO/WARN/ERROR
     */
    static void setLogLevel(const std::string& level);

    /**
     * @brief 动态设置日志级别
     * @param level spdlog日志级别枚举
     */
    static void setLogLevel(spdlog::level::level_enum level);

    /**
     * @brief 获取当前日志级别
     * @return 日志级别字符串
     */
    static std::string getLogLevel();

    /**
     * @brief 获取日志对象（用于直接使用spdlog功能）
     * @return 共享的日志对象指针
     */
    static std::shared_ptr<spdlog::logger> getLogger();

    /**
     * @brief 设置当前线程的设备名（用于日志前缀）
     * @param device_name 设备名称
     */
    static void setThreadDeviceName(const std::string& device_name);

    /**
     * @brief 获取当前线程的设备名
     * @return 设备名称，如果未设置则返回空字符串
     */
    static std::string getThreadDeviceName();

    /**
     * @brief 清除当前线程的设备名
     */
    static void clearThreadDeviceName();

    /**
     * @brief 重新配置日志系统（运行时动态修改）
     * @param config 新的日志配置
     */
    static void reconfigure(const LogConfig& config);

    /**
     * @brief 刷新日志缓冲区（确保所有日志都写入文件）
     */
    static void flush();

    /**
     * @brief 关闭日志系统（程序退出时调用）
     */
    static void shutdown();

private:
    // 将字符串转换为spdlog日志级别
    static spdlog::level::level_enum parseLogLevel(const std::string& level_str);

    // 创建日志sink
    static void createSinks(const LogConfig& config, std::vector<spdlog::sink_ptr>& sinks);

    // 更新默认logger
    static void updateLogger(const std::vector<spdlog::sink_ptr>& sinks, spdlog::level::level_enum level);

    static std::mutex mutex_;         // 保护日志配置的互斥锁
    static LogConfig current_config_; // 当前配置
};

// 便捷函数：获取默认日志对象
std::shared_ptr<spdlog::logger> getLogger();

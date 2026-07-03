#include <gtest/gtest.h>

#include "logger_manager.hpp"

// 自定义测试入口：初始化一个“静默”日志配置，避免协议代码内部 getLogger()
// 自动创建带控制台输出和 logs/app.log 文件的默认日志（污染测试输出与工作目录）。
int main(int argc, char** argv) {
    LoggerManager::LogConfig quiet;
    quiet.level = "OFF";         // 关闭所有级别输出
    quiet.console_enable = true; // 至少保留一个 sink，避免初始化抛异常
    quiet.file_enable = false;   // 测试不写日志文件
    LoggerManager::initialize(quiet);

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

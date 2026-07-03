#pragma once
#include <atomic>
#include <boost/asio.hpp>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

class Protocol;

class SerialPortBase {
public:
    SerialPortBase(const std::string& port, unsigned int baudrate);
    ~SerialPortBase();

    // 阻塞式写入
    size_t write(const std::vector<uint8_t>& data);

    // 阻塞式读取（带超时）
    std::vector<uint8_t> read(std::chrono::milliseconds timeout = std::chrono::milliseconds(25));

    // 非阻塞式读取（返回当前缓冲区数据）
    std::vector<uint8_t> readAvailable();

    // 清空接收缓冲区
    void clearBuffer();

    // 检查串口状态
    bool isOpen() const;

    void setProtocol(std::shared_ptr<Protocol> protocol);

private:
    // 异步读取处理
    void startAsyncRead();
    void handleRead(const boost::system::error_code& error, size_t bytes_transferred);

    // ASIO 相关
    boost::asio::io_context io_context_;
    boost::asio::serial_port serial_;
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard_;
    std::thread io_thread_;

    // 接收缓冲区
    static constexpr size_t BUFFER_SIZE = 4096;
    std::array<uint8_t, BUFFER_SIZE> temp_buffer_; // 临时接收缓冲
    std::vector<uint8_t> receive_buffer_;          // 累积缓冲区

    // 线程同步
    mutable std::mutex mutex_;
    std::condition_variable cv_;

    // 状态标志
    std::atomic<bool> is_running_{true};

    // 协议处理器
    std::shared_ptr<Protocol> current_protocol_;
};

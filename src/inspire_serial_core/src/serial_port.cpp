// serial_port.cpp
#include "serial_port.hpp"
#include "protocol.hpp"
#include <iostream>

SerialPortBase::SerialPortBase(const std::string& port, unsigned int baudrate)
    : serial_(io_context_),
      work_guard_(boost::asio::make_work_guard(io_context_)),
      port_name_(port),
      baudrate_(baudrate) {
    
    try {
        if (!configureOpenSerial()) {
            throw std::runtime_error("serial open failed");
        }
        
        // 启动异步读取
        startAsyncRead();
        
        // 启动 IO 线程
        io_thread_ = std::thread([this]() {
            try {
                io_context_.run();
            } catch (const std::exception& e) {
                std::cerr << "IO thread exception: " << e.what() << std::endl;
            }
        });
        
        std::cout << "Serial port " << port << " opened successfully at "
                  << baudrate << " baud" << std::endl;
                  
    } catch (const boost::system::system_error& e) {
        throw std::runtime_error("Failed to open serial port " + port +
                                ": " + e.what());
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to open serial port " + port + ": " + e.what());
    }
}

SerialPortBase::~SerialPortBase() {
    is_running_ = false;
    
    // 停止所有异步操作
    boost::system::error_code ec;
    if (serial_.is_open()) {
        serial_.cancel(ec);
        serial_.close(ec);
    }
    
    // 停止 io_context
    work_guard_.reset();
    io_context_.stop();
    
    // 等待 IO 线程结束
    if (io_thread_.joinable()) {
        io_thread_.join();
    }
    
    std::cout << "Serial port closed" << std::endl;
}

void SerialPortBase::startAsyncRead() {
    serial_.async_read_some(
        boost::asio::buffer(temp_buffer_),
        [this](const boost::system::error_code& error, size_t bytes_transferred) {
            handleRead(error, bytes_transferred);
        }
    );
}

void SerialPortBase::handleRead(const boost::system::error_code& error, 
                                 size_t bytes_transferred) {
    if (!is_running_) {
        return;
    }
    
    if (!error && bytes_transferred > 0) {
        // 加锁保护共享数据
        {
            std::lock_guard<std::mutex> lock(mutex_);
            
            // 追加到接收缓冲区
            receive_buffer_.insert(
                receive_buffer_.end(),
                temp_buffer_.begin(),
                temp_buffer_.begin() + bytes_transferred
            );
            
            // 防止缓冲区无限增长
            if (receive_buffer_.size() > 65536) {
                std::cerr << "Warning: Receive buffer overflow, clearing old data" 
                         << std::endl;
                receive_buffer_.erase(
                    receive_buffer_.begin(),
                    receive_buffer_.begin() + (receive_buffer_.size() - 32768)
                );
            }
        }
        
        // 通知等待的线程
        cv_.notify_all();
        
        // 继续异步读取
        startAsyncRead();
        
    } else if (error == boost::asio::error::operation_aborted) {
        // 操作被取消（正常关闭）
        std::cout << "Read operation cancelled" << std::endl;
        
    } else {
        // 断链（EOF/设备消失）按旧代码期望：报错后尝试重连，避免持续刷屏。
        if (error == boost::asio::error::eof ||
            error == boost::asio::error::bad_descriptor ||
            error == boost::asio::error::connection_reset ||
            error == boost::asio::error::not_found) {
            std::cerr << "Read error: " << error.message()
                      << " (serial disconnected, trying reconnect)" << std::endl;
            tryReconnect();
            return;
        }

        // 其他错误：保留一次错误并继续读
        std::cerr << "Read error: " << error.message() << std::endl;
        if (is_running_ && serial_.is_open()) {
            startAsyncRead();
        }
    }
}

bool SerialPortBase::configureOpenSerial() {
    if (!serial_.is_open()) {
        serial_.open(port_name_);
    }
    serial_.set_option(boost::asio::serial_port_base::baud_rate(baudrate_));
    serial_.set_option(boost::asio::serial_port_base::character_size(8));
    serial_.set_option(boost::asio::serial_port_base::stop_bits(
        boost::asio::serial_port_base::stop_bits::one));
    serial_.set_option(boost::asio::serial_port_base::parity(
        boost::asio::serial_port_base::parity::none));
    serial_.set_option(boost::asio::serial_port_base::flow_control(
        boost::asio::serial_port_base::flow_control::none));
    return true;
}

void SerialPortBase::tryReconnect() {
    if (!is_running_) {
        return;
    }
    bool expected = false;
    if (!reconnecting_.compare_exchange_strong(expected, true)) {
        return;
    }

    std::cerr << "Serial reconnect start: " << port_name_ << std::endl;
    size_t attempts = 0;
    while (is_running_) {
        ++attempts;
        boost::system::error_code ec;
        if (serial_.is_open()) {
            serial_.cancel(ec);
            serial_.close(ec);
        }

        try {
            configureOpenSerial();
            {
                std::lock_guard<std::mutex> lock(mutex_);
                receive_buffer_.clear();
            }
            cv_.notify_all();
            std::cerr << "Serial reconnect success: " << port_name_
                      << " (attempt " << attempts << ")" << std::endl;
            reconnecting_ = false;
            startAsyncRead();
            return;
        } catch (const std::exception& e) {
            if (attempts % 10 == 1) {
                std::cerr << "Serial reconnect failed: " << e.what() << std::endl;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    reconnecting_ = false;
}

size_t SerialPortBase::write(const std::vector<uint8_t>& data) {
    if (!serial_.is_open()) {
        throw std::runtime_error("Serial port is not open");
    }
    
    if (data.empty()) {
        return 0;
    }
    
    try {
        // 使用同步写入
        std::lock_guard<std::mutex> lock(mutex_);
        size_t bytes_written = boost::asio::write(
            serial_,
            boost::asio::buffer(data)
        );
        
        return bytes_written;
        
    } catch (const boost::system::system_error& e) {
        throw std::runtime_error("Write error: " + std::string(e.what()));
    }
}

std::vector<uint8_t> SerialPortBase::read(std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    
    // 等待数据到达或超时
    bool has_data = cv_.wait_for(
        lock,
        timeout,
        [this]() { return !receive_buffer_.empty() || !is_running_; }
    );
    
    if (!is_running_) {
        return {};
    }
    
    if (!has_data || receive_buffer_.empty()) {
        // 超时或无数据
        return {};
    }
    
    // 返回所有可用数据
    std::vector<uint8_t> result = std::move(receive_buffer_);
    receive_buffer_.clear();
    
    return result;
}

std::vector<uint8_t> SerialPortBase::readAvailable() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (receive_buffer_.empty()) {
        return {};
    }
    
    std::vector<uint8_t> result = std::move(receive_buffer_);
    receive_buffer_.clear();
    
    return result;
}

void SerialPortBase::clearBuffer() {
    std::lock_guard<std::mutex> lock(mutex_);
    receive_buffer_.clear();
}

bool SerialPortBase::isOpen() const {
    return serial_.is_open();
}

void SerialPortBase::setProtocol(std::shared_ptr<Protocol> protocol) {
    std::lock_guard<std::mutex> lock(mutex_);
    current_protocol_ = protocol;
}


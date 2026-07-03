#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>

/**
 * @brief 每设备串口事务串行化执行器（请求队列 + 单工作线程）。
 *
 * 设计目的：在多线程执行器下，对同一设备的「写命令 + 读应答 + 解析」整组事务
 * 必须永不交错。本类用一个独立工作线程顺序消费任务队列，从结构上保证：
 * 任何通过 submit() 提交的任务都在同一线程上按 FIFO 串行执行，互不交叠。
 *
 * 典型用法：定时器回调提交「读取并发布」任务（fire-and-forget）；服务/话题回调
 * 提交「写寄存器」任务并对返回的 future 调用 get() 取结果。worker 单线程保证
 * 串口事务原子，而调用线程的等待不会与 worker 互相持锁。
 *
 * 线程安全：submit()/stop() 可从任意线程调用。
 */
class DeviceWorker {
public:
    /**
     * @brief 构造并启动工作线程。
     * @param name 设备名（用于线程内日志前缀，可空）。
     */
    explicit DeviceWorker(std::string name = std::string()) : name_(std::move(name)) {
        thread_ = std::thread([this]() { runLoop(); });
    }

    ~DeviceWorker() { stop(); }

    DeviceWorker(const DeviceWorker&) = delete;
    DeviceWorker& operator=(const DeviceWorker&) = delete;

    /**
     * @brief 提交一个任务到队列，返回与其结果关联的 future。
     *
     * 任务在工作线程上串行执行；调用方可对返回的 future 调用 get() 同步等待结果。
     * 若执行器已停止，则立即抛出 std::runtime_error（调用方应自行兜底）。
     *
     * @tparam F 可调用对象类型（无参）。
     * @param f 待执行任务。
     * @return std::future，承载任务返回值（或异常）。
     */
    template <class F> auto submit(F&& f) -> std::future<std::invoke_result_t<F>> {
        using Ret = std::invoke_result_t<F>;

        auto task = std::make_shared<std::packaged_task<Ret()>>(std::forward<F>(f));
        std::future<Ret> fut = task->get_future();

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!running_.load()) {
                throw std::runtime_error("DeviceWorker 已停止，拒绝提交任务");
            }
            queue_.emplace([task]() { (*task)(); });
        }
        cv_.notify_one();
        return fut;
    }

    /**
     * @brief 停止执行器：标记停止、唤醒并 join 工作线程。
     *
     * 幂等。停止时队列中尚未执行的任务会被丢弃；其关联的 packaged_task 在析构时
     * 会让对应 future 抛出 std::future_error(broken_promise)，调用方需 try/catch 兜底。
     * 正在执行的任务会先执行完再退出。
     */
    void stop() {
        bool expected = true;
        if (!running_.compare_exchange_strong(expected, false)) {
            // 已停止：仍确保线程被 join（处理重复调用）
            if (thread_.joinable()) {
                thread_.join();
            }
            return;
        }
        cv_.notify_all();
        if (thread_.joinable()) {
            thread_.join();
        }
    }

    /** @brief 当前是否仍在运行。 */
    bool running() const { return running_.load(); }

    /** @brief 设备名。 */
    const std::string& name() const { return name_; }

private:
    void runLoop() {
        for (;;) {
            std::function<void()> task;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [this]() { return !running_.load() || !queue_.empty(); });
                if (!running_.load() && queue_.empty()) {
                    return;
                }
                task = std::move(queue_.front());
                queue_.pop();
            }
            // 任务自身已用 packaged_task 捕获异常，这里无需再 try/catch
            task();
        }
    }

    std::string name_;
    std::thread thread_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::queue<std::function<void()>> queue_;
    std::atomic<bool> running_{true};
};

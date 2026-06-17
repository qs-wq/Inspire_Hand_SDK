#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <future>
#include <thread>
#include <vector>

#include "device_worker.hpp"

// submit 返回值能通过 future 正确取回
TEST(DeviceWorkerTest, SubmitReturnsValue) {
    DeviceWorker w("t");
    auto fut = w.submit([]() { return 21 * 2; });
    EXPECT_EQ(fut.get(), 42);
}

// 任务异常通过 future 透传
TEST(DeviceWorkerTest, SubmitPropagatesException) {
    DeviceWorker w("t");
    auto fut = w.submit([]() -> int { throw std::runtime_error("boom"); });
    EXPECT_THROW(fut.get(), std::runtime_error);
}

// 同一线程顺序提交按 FIFO 执行
TEST(DeviceWorkerTest, ExecutesInFifoOrder) {
    DeviceWorker w("t");
    std::vector<int> order;
    std::vector<std::future<void>> futs;
    for (int i = 0; i < 100; ++i) {
        futs.push_back(w.submit([&order, i]() { order.push_back(i); }));
    }
    for (auto& f : futs) {
        f.get();
    }
    ASSERT_EQ(order.size(), 100u);
    for (int i = 0; i < 100; ++i) {
        EXPECT_EQ(order[i], i);
    }
}

// 任务串行执行：绝不交叠（用一个非原子计数器 + “同时进入”探测）
TEST(DeviceWorkerTest, TasksNeverOverlap) {
    DeviceWorker w("t");
    std::atomic<int> concurrent{0};
    std::atomic<int> max_concurrent{0};
    std::vector<std::future<void>> futs;

    for (int i = 0; i < 50; ++i) {
        futs.push_back(w.submit([&]() {
            int cur = concurrent.fetch_add(1) + 1;
            int prev_max = max_concurrent.load();
            while (cur > prev_max && !max_concurrent.compare_exchange_weak(prev_max, cur)) {
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            concurrent.fetch_sub(1);
        }));
    }
    for (auto& f : futs) {
        f.get();
    }
    EXPECT_EQ(max_concurrent.load(), 1);  // 单线程串行，最大并发恒为 1
}

// 多线程并发 submit，所有任务都被执行且各自结果正确
TEST(DeviceWorkerTest, ConcurrentSubmitFromManyThreads) {
    DeviceWorker w("t");
    constexpr int kThreads = 8;
    constexpr int kPerThread = 100;
    std::atomic<int> sum{0};
    std::vector<std::thread> producers;

    for (int t = 0; t < kThreads; ++t) {
        producers.emplace_back([&]() {
            std::vector<std::future<int>> futs;
            for (int i = 0; i < kPerThread; ++i) {
                futs.push_back(w.submit([]() { return 1; }));
            }
            for (auto& f : futs) {
                sum.fetch_add(f.get());
            }
        });
    }
    for (auto& p : producers) {
        p.join();
    }
    EXPECT_EQ(sum.load(), kThreads * kPerThread);
}

// stop() 后再 submit 抛异常；stop() 幂等
TEST(DeviceWorkerTest, SubmitAfterStopThrows) {
    DeviceWorker w("t");
    w.stop();
    EXPECT_FALSE(w.running());
    EXPECT_THROW(w.submit([]() { return 0; }), std::runtime_error);
    EXPECT_NO_THROW(w.stop());  // 幂等
}

// 析构时安全停止（不挂起）：构造后直接析构
TEST(DeviceWorkerTest, DestructorStopsCleanly) {
    {
        DeviceWorker w("t");
        w.submit([]() { std::this_thread::sleep_for(std::chrono::milliseconds(2)); });
    }
    SUCCEED();
}

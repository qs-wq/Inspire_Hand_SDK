# 单元测试：覆盖 RingBuffer 与各 485 协议的纯逻辑（命令构建 / 响应解析 / 校验和）。
# 使用系统 GTest（libgtest-dev）。自定义 main（test_main.cpp）以静默日志。

add_executable(inspire_serial_core_tests
    test_main.cpp
    test_ring_buffer.cpp
    test_device_worker.cpp
    test_rh56f1_485_protocol.cpp
    test_rh5dg2_485_protocol.cpp
    test_rh56dfx_serial_can_protocol.cpp
    test_eg5cd1_485_protocol.cpp
    test_eg2_4c2_serial_can_protocol.cpp
    test_rh524j1_485_protocol.cpp
)

target_include_directories(inspire_serial_core_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(inspire_serial_core_tests PRIVATE
    inspire_serial_core
    GTest::gtest
)

# 注册到 CTest（支持 `ctest` 与 `colcon test`）
include(GoogleTest)
gtest_discover_tests(inspire_serial_core_tests)

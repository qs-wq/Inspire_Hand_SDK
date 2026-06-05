cmake_minimum_required(VERSION 3.16)
project(inspire_serial_core CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 是否构建独立示例可执行（非 ROS 路径）。在 colcon/ROS2 工作区中默认关闭即可，
# 单独 `cmake -S src/inspire_serial_core` 时默认开启，便于无 ROS 环境快速验证。
option(INSPIRE_SERIAL_CORE_BUILD_EXAMPLES "Build standalone example executables" ON)

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)

# ---------------------------------------------------------------------------
# 核心库 inspire_serial_core：协议 / 串口 / 配置 / 日志。
# 采用 SHARED 库：
#   1) 可被 driver 等其它 colcon 包通过 find_package 复用（跨包链接）；
#   2) 各协议 .cpp 中 REGISTER_PROTOCOL 的静态自注册对象随 .so 加载执行，
#      不会像 STATIC 归档那样被链接器当作“未引用符号”丢弃。
# ---------------------------------------------------------------------------
add_library(inspire_serial_core SHARED
    src/serial_port.cpp
    src/device_manager.cpp
    src/ring_buffer.cpp
    src/RH56F1_485_protocol.cpp
    src/RH5DG2_485_protocol.cpp
    src/EG5CD1_485_protocol.cpp
    src/RH56F1_canfd_protocol.cpp
    src/RH5DG2_canfd_protocol.cpp
    src/config_loader.cpp
    src/protocol_factory.cpp
    src/logger_manager.cpp
)
add_library(inspire_serial_core::inspire_serial_core ALIAS inspire_serial_core)

target_include_directories(inspire_serial_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${Boost_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
)

target_link_libraries(inspire_serial_core PUBLIC
    ${Boost_LIBRARIES}
    yaml-cpp
    pthread
    spdlog::spdlog
)

target_compile_features(inspire_serial_core PUBLIC cxx_std_17)

# ---------------------------------------------------------------------------
# 安装 + 导出：供其它包 find_package(inspire_serial_core) 使用。
# ---------------------------------------------------------------------------
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS inspire_serial_core
    EXPORT inspire_serial_coreTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION include
)

install(DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.hpp"
)

install(EXPORT inspire_serial_coreTargets
    FILE inspire_serial_coreTargets.cmake
    NAMESPACE inspire_serial_core::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/inspire_serial_core
)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/inspire_serial_coreConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/inspire_serial_coreConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/inspire_serial_core
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/inspire_serial_coreConfig.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/inspire_serial_core
)

# ---------------------------------------------------------------------------
# 独立可执行（多设备并行控制示例，无 ROS 依赖）
# ---------------------------------------------------------------------------
if(INSPIRE_SERIAL_CORE_BUILD_EXAMPLES)
    add_executable(serial_hand_control_node examples/main.cpp)
    target_link_libraries(serial_hand_control_node PRIVATE inspire_serial_core)
endif()

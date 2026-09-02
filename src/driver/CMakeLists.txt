cmake_minimum_required(VERSION 3.16)
project(inspire_control_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rh5dg2_interfaces REQUIRED)
find_package(rh56f1_interfaces REQUIRED)
find_package(rh56dfx_interfaces REQUIRED)
find_package(eg5cd1_interfaces REQUIRED)

# ---------------------------------------------------------------------------
# 复用同一 colcon 工作区内的裸库核心包 inspire_serial_core。
# 它是独立的纯 CMake 包（src/inspire_serial_core），安装并导出了
# inspire_serial_core::inspire_serial_core 目标，include 目录与
# Boost/yaml-cpp/spdlog 依赖经由该目标的 PUBLIC 属性自动传递。
# ---------------------------------------------------------------------------
find_package(inspire_serial_core REQUIRED)

add_executable(inspire_control_node
    src/register_controller.cpp
    src/interface_adapter_factory.cpp
    src/RH5DG2_interface_adapter.cpp
    src/RH56F1_interface_adapter.cpp
    src/RH56H1_interface_adapter.cpp
    src/RH56DFX_interface_adapter.cpp
    src/EG5CD1_interface_adapter.cpp
    src/inspire_controller_factory.cpp
    src/inspire_control_node.cpp
)

target_include_directories(inspire_control_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(inspire_control_node
    ament_index_cpp
    rclcpp
    std_msgs
    rh5dg2_interfaces
    rh56f1_interfaces
    rh56dfx_interfaces
    eg5cd1_interfaces
)

target_link_libraries(inspire_control_node
    inspire_serial_core::inspire_serial_core
)

install(TARGETS inspire_control_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
    FILES_MATCHING PATTERN "*.yaml"
)

install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
    FILES_MATCHING PATTERN "*.launch.py"
)

ament_package()

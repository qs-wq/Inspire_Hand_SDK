#!/bin/bash
# 灵巧手控制系统 - 依赖安装脚本
# 适用于 Ubuntu 22.04+

set -e  # 遇到错误立即退出

echo "=========================================="
echo "  灵巧手控制系统 - 依赖安装脚本"
echo "=========================================="
echo ""

# 检查是否为root用户
if [ "$EUID" -eq 0 ]; then 
   echo "错误：请不要使用root用户运行此脚本"
   exit 1
fi

# 检测操作系统
if [ ! -f /etc/os-release ]; then
    echo "错误：无法检测操作系统"
    exit 1
fi

. /etc/os-release

if [ "$ID" != "ubuntu" ] && [ "$ID" != "debian" ]; then
    echo "警告：此脚本主要针对Ubuntu/Debian系统，其他系统可能需要手动安装依赖"
    read -p "是否继续？(y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo "检测到操作系统: $PRETTY_NAME"
echo ""

# 更新软件包列表
echo "=== 步骤 1/6: 更新软件包列表 ==="
sudo apt update
echo ""

# 安装系统依赖
echo "=== 步骤 2/6: 安装系统依赖 ==="
sudo apt install -y \
    build-essential \
    cmake \
    pkg-config \
    git \
    wget \
    curl \
    gcc \
    g++ \
    make \
    libc6-dev

echo ""

# 检查CMake版本
CMAKE_VERSION=$(cmake --version | head -n1 | cut -d' ' -f3)
CMAKE_MAJOR=$(echo $CMAKE_VERSION | cut -d'.' -f1)
CMAKE_MINOR=$(echo $CMAKE_VERSION | cut -d'.' -f2)

if [ "$CMAKE_MAJOR" -lt 3 ] || ([ "$CMAKE_MAJOR" -eq 3 ] && [ "$CMAKE_MINOR" -lt 10 ]); then
    echo "警告：CMake版本 ($CMAKE_VERSION) 低于要求 (3.10+)"
    echo "建议升级CMake或从源码编译"
else
    echo "✓ CMake版本检查通过: $CMAKE_VERSION"
fi
echo ""

# 安装Boost库
echo "=== 步骤 3/6: 安装Boost库 ==="
sudo apt install -y \
    libboost-system-dev \
    libboost-thread-dev \
    libboost-dev

# 验证Boost安装
if pkg-config --exists boost; then
    BOOST_VERSION=$(pkg-config --modversion boost)
    echo "✓ Boost安装成功: $BOOST_VERSION"
else
    echo "警告：无法验证Boost安装，但可能已成功安装"
fi
echo ""

# 安装yaml-cpp库
echo "=== 步骤 4/6: 安装yaml-cpp库 ==="
sudo apt install -y libyaml-cpp-dev

# 验证yaml-cpp安装
if pkg-config --exists yaml-cpp; then
    YAML_VERSION=$(pkg-config --modversion yaml-cpp)
    echo "✓ yaml-cpp安装成功: $YAML_VERSION"
else
    echo "警告：无法验证yaml-cpp安装，但可能已成功安装"
fi
echo ""

# 安装spdlog库
echo "=== 步骤 5/6: 安装spdlog库 ==="
if sudo apt install -y libspdlog-dev; then
    echo "✓ spdlog安装成功"
else
    echo "警告：apt安装spdlog失败，尝试从源码编译..."
    
    # 从源码编译spdlog
    cd /tmp
    if [ -d "spdlog" ]; then
        rm -rf spdlog
    fi
    
    git clone https://github.com/gabime/spdlog.git
    cd spdlog
    mkdir build && cd build
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
    make -j$(nproc)
    sudo make install
    cd /tmp
    rm -rf spdlog
    
    echo "✓ spdlog从源码编译安装成功"
fi
echo ""

# 配置串口权限
echo "=== 步骤 6/6: 配置串口权限 ==="
if groups | grep -q dialout; then
    echo "✓ 用户已在dialout组中"
else
    sudo usermod -a -G dialout $USER
    echo "✓ 已将用户添加到dialout组"
    echo "  注意：需要重新登录或运行 'newgrp dialout' 才能生效"
fi
echo ""

# 检查ROS2安装
echo "=== 检查ROS2安装 ==="
if [ -f /opt/ros/humble/setup.bash ]; then
    echo "✓ 检测到ROS2 Humble已安装"
    source /opt/ros/humble/setup.bash
    
    # 检查ROS2包
    if ros2 pkg list | grep -q rclcpp; then
        echo "✓ ROS2包检查通过"
    else
        echo "警告：ROS2包可能不完整，建议运行："
        echo "  sudo apt install -y ros-humble-desktop"
    fi
else
    echo "⚠ ROS2 Humble未安装"
    echo ""
    echo "请按照以下步骤安装ROS2 Humble："
    echo ""
    echo "1. 设置locale:"
    echo "   sudo apt install -y locales"
    echo "   sudo locale-gen en_US en_US.UTF-8"
    echo "   sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8"
    echo ""
    echo "2. 添加ROS2源:"
    echo "   sudo apt install -y software-properties-common"
    echo "   sudo add-apt-repository universe"
    echo "   sudo apt update && sudo apt install -y curl gnupg lsb-release"
    echo "   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -"
    echo "   sudo sh -c 'echo \"deb [arch=\$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu \$(lsb_release -cs) main\" > /etc/apt/sources.list.d/ros2-latest.list'"
    echo ""
    echo "3. 安装ROS2:"
    echo "   sudo apt update"
    echo "   sudo apt install -y ros-humble-desktop"
    echo "   sudo apt install -y ros-humble-rclcpp ros-humble-std-msgs ros-humble-std-srvs"
    echo "   sudo apt install -y ros-humble-rosidl-default-generators ros-humble-rosidl-default-runtime"
    echo "   sudo apt install -y python3-colcon-common-extensions python3-rosdep"
    echo ""
    echo "4. 初始化rosdep:"
    echo "   sudo rosdep init"
    echo "   rosdep update"
    echo ""
    echo "5. 设置环境变量（添加到~/.bashrc）:"
    echo "   echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc"
    echo "   source ~/.bashrc"
fi
echo ""

# 总结
echo "=========================================="
echo "  依赖安装完成！"
echo "=========================================="
echo ""
echo "已安装的依赖："
echo "  ✓ 系统构建工具 (CMake, GCC, Make等)"
echo "  ✓ Boost库 (system, thread)"
echo "  ✓ yaml-cpp库"
echo "  ✓ spdlog库"
echo "  ✓ 串口权限配置"
echo ""
echo "ROS2 侧（随源码编译的包，非 apt）："
echo "  • rh5dg2_interfaces / rh56f1_interfaces — 机型专用 msg/srv"
echo "  • inspire_control_ros2 — 灵巧手节点（依赖上述接口包）"
echo ""
echo "下一步："
echo "  1. 如果串口权限已更改，请重新登录或运行: newgrp dialout"
echo "  2. 如果ROS2未安装，请按照上述说明安装ROS2 Humble"
echo "  3. 编译 ROS2 工作区（接口包 + 节点包，首次必须带 interfaces）："
echo "     cd src/ros2"
echo "     source /opt/ros/humble/setup.bash"
echo "     colcon build --packages-select rh5dg2_interfaces rh56f1_interfaces inspire_control_ros2"
echo "     source install/setup.bash"
echo "  仅修改节点时可简化为: colcon build --packages-select inspire_control_ros2"
echo ""
echo "验证安装："
echo "  cmake --version"
echo "  g++ --version"
echo "  pkg-config --modversion boost"
echo "  pkg-config --modversion yaml-cpp"
echo ""

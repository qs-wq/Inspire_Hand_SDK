# 灵巧手控制系统（Inspire / ROS2）

基于 C++ 与 ROS2 的多设备灵巧手控制系统，底层通过 RS485 / CANFD 等与多台 Inspire 系列灵巧手通信。节点包名为 **`inspire_control_ros2`**。

## 项目简介

本项目是一个模块化的灵巧手控制系统，支持：
- ✅ **多设备支持**：同时控制多个灵巧手设备（如左手、右手）
- ✅ **多协议支持**：通过工厂模式支持多种通信协议（RH56F1_485、RH5DG2_485、EG5CD1_485 等）
- ✅ **动态配置**：通过 YAML 配置设备协议与 ROS2 话题/服务
- ✅ **双通信模式**：支持话题（实时控制）和服务（按需调用）两种方式
- ✅ **异步串口通信**：基于Boost.Asio的异步串口通信，支持超时和错误处理
- ✅ **统一日志系统**：全局日志管理器，支持文件轮转和级别控制

## 项目结构

本仓库即一个 **colcon 工作区根目录**，`src/` 下为各平级包（单层 `src`，无嵌套工作区）：

```
serial_control/                        # = git 根 = colcon 工作区根
├── src/                               # colcon 包目录（唯一一层）
│   ├── inspire_serial_core/           # ① 裸库（纯 CMake 包，可脱离 ROS 独立构建）
│   │   ├── package.xml                #    <build_type>cmake</build_type>，供 colcon 排序
│   │   ├── CMakeLists.txt             #    构建 SHARED 库 + 安装/导出 find_package 配置
│   │   ├── cmake/inspire_serial_coreConfig.cmake.in   # 导出配置模板（源文件，需入库）
│   │   ├── include/                   #    protocol.hpp / io_error.hpp / *_protocol.hpp / serial_port.hpp ...
│   │   ├── src/                       #    协议 / 串口 / 配置 / 日志实现
│   │   ├── examples/                  #    main.cpp 多设备并行控制示例（serial_hand_control_node）
│   │   └── config/                    #    device_protocol_config.yaml、device_protocol_rh56f1_example.yaml、device_protocol_rh5dg2_example.yaml ...
│   ├── driver/                        # ② 功能包 inspire_control_ros2（find_package(inspire_serial_core)）
│   │   ├── src/                       #    节点、RegisterController、机型适配器
│   │   ├── include/
│   │   ├── config/                    #    device_protocol_*.yaml、ros2_controller_*.yaml
│   │   └── launch/                    #    inspire_control_*.launch.py
│   └── interfaces/                    # ③ 接口包
│       ├── RH5DG2/                    #    rh5dg2_interfaces（13 自由度）
│       ├── RH56F1/                    #    rh56f1_interfaces（6 自由度）
│       └── EG5CD1/                    #    eg5cd1_interfaces（EG-5CD1 夹爪）
├── docs/                              # 全部文档集中存放（架构/模块/依赖/协议规则/厂商手册）
├── install_dependencies.sh           # 依赖安装脚本（一键安装）
├── .gitignore
└── README.md                         # 本文件
```

> **依赖关系**：`driver` 通过 `find_package(inspire_serial_core)` 链接裸库导出的 `inspire_serial_core::inspire_serial_core` 目标；colcon 依 `package.xml` 的 `<depend>inspire_serial_core</depend>` 自动保证先构建裸库。裸库采用 **SHARED** 库，使各协议 `REGISTER_PROTOCOL` 自注册对象随 `.so` 加载执行（避免 STATIC 归档丢符号）。

### ROS2 接口说明（重构后）

| 包名 | 作用 |
|------|------|
| **inspire_control_ros2** | 节点与驱动逻辑：`inspire_control_node`、`RegisterController`、`RH5DG2InterfaceAdapter` / `RH56F1InterfaceAdapter` / **`EG5CD1InterfaceAdapter`**，配置文件安装在 `share/inspire_control_ros2/config`。 |
| **rh5dg2_interfaces** | RH5DG2（13 自由度）专用 `msg`/`srv`，例如 `SetAngle1`、`GetAngleAct1`、`Setforce`、`Geterror` 等。 |
| **rh56f1_interfaces** | RH56 系列（6 自由度）专用 `msg`/`srv`。 |
| **eg5cd1_interfaces** | **因时 EG-5CD1** 电动夹爪 RS485：`GripperState`、`SetInt32`、`TriggerForHand`、`SetInt32Value`、`GetScalarForHand`；**组合服务** `ForceModeGrasp` / `ForceModeOpen` / `TouchModeGrasp` / `TouchModeOpen`（仅 `hand_id`+`speed`+`force`，内部按文档顺序经 `ioWriteSequence` 在设备 `DeviceWorker` 上**原子串行**写寄存器，见下）。 |

在 **`device_protocol_config.yaml`** 中设置 **`protocol.type`**（如 **`RH5DG2_485`**、**`RH56F1_485`**、**`EG5CD1_485`** 等），启动时自动推导 **`interfaces_profile`**（`RH5DG2` / `RH56F1` / **`EG5CD1`**）并创建对应适配器。

### EG-5CD1 夹爪全链路说明

- **协议实现**：`EG5CD1_485_Protocol`（`REGISTER_PROTOCOL("EG5CD1_485", …)`），帧头主发 `EB 90`、应答 `EE 16`，读命令 `0x00`、写命令 `0x01`，寄存器名与文档一致（如 `openLenSet`、`gripperStatusBlock` 一次读 1120–1132 共 14 字节）。
- **示例配置**（随包安装到 `share/inspire_control_ros2/config`）：
  - `device_protocol_eg5cd1_example.yaml`：`protocol.type: EG5CD1_485` 与串口设备名。
  - `ros2_controller_eg5cd1_example.yaml`：话题名需与适配器约定一致：`gripper_state`、`open_len_set`、`speed_set`、`force_set`、`catch_mode_set`。
  - **力控 / 触控组合服务**（节点启动后自动创建，默认前缀见参数）：`{prefix}/force_mode_grasp`、`force_mode_open`、`touch_mode_grasp`、`touch_mode_open`。请求字段均为 `hand_id`、`speed`（0–1000）、`force`（力控夹取 1–2000；力控张开 -2000..0；触控 0–2000）。整组写经 `ioWriteSequence` 在该设备的 `DeviceWorker` 单线程上**原子串行执行**（步骤间隔 3ms 在 worker 线程内），与定时读状态天然互不交错，无需再暂停状态轮询。前缀由 ROS 参数 **`eg5cd1_composite_service_prefix`** 控制（默认 `/gripper`），与示例话题的 `/gripper/...` 对齐。
- **启动示例**：

```bash
ros2 run inspire_control_ros2 inspire_control_node -- \
  --device-config $(ros2 pkg prefix inspire_control_ros2)/share/inspire_control_ros2/config/device_protocol_eg5cd1_example.yaml \
  --controller-config $(ros2 pkg prefix inspire_control_ros2)/share/inspire_control_ros2/config/ros2_controller_eg5cd1_example.yaml
```

（将示例里的 `port`、`device` 改成你的实际串口与 `device_protocol` 中设备名一致。）

编译时需与工作区内接口包一起构建（见 [编译项目](#3-编译项目)）。

## 快速开始

> **💡 快速安装**：推荐使用自动化安装脚本一键安装所有依赖
> ```bash
> ./install_dependencies.sh
> ```
> 详细说明见 [依赖安装](#2-依赖安装) 章节

### 1. 环境要求

- **操作系统**：Linux (Ubuntu 22.04+)
- **ROS2**：Humble或更高版本
- **C++标准**：C++17
- **编译器**：GCC 9+ 或 Clang 10+
- **构建工具**：CMake 3.10+

### 2. 依赖安装

#### 2.1 系统依赖

**Ubuntu/Debian系统**：

```bash
# 更新软件包列表
sudo apt update

# 安装基础构建工具
sudo apt install -y \
    build-essential \
    cmake \
    pkg-config \
    git \
    wget \
    curl

# 安装C++编译器和工具链
sudo apt install -y \
    gcc \
    g++ \
    make \
    libc6-dev
```

#### 2.2 ROS2依赖

**安装ROS2 Humble（如果未安装）**：

```bash
# 设置locale
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 添加ROS2源
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl gnupg lsb-release

# 添加ROS2 GPG密钥
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# 安装ROS2 Humble
sudo apt update
sudo apt install -y ros-humble-desktop

# 安装ROS2开发工具
sudo apt install -y \
    ros-humble-rclcpp \
    ros-humble-std-msgs \
    ros-humble-std-srvs \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    python3-colcon-common-extensions \
    python3-rosdep

# 初始化rosdep
sudo rosdep init
rosdep update

# 设置ROS2环境（添加到~/.bashrc）
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### 2.3 第三方库依赖

**安装Boost库**：

```bash
# 安装Boost开发库（包含Boost.Asio）
sudo apt install -y \
    libboost-system-dev \
    libboost-thread-dev \
    libboost-dev
```

**安装yaml-cpp库**：

```bash
# 安装yaml-cpp开发库
sudo apt install -y libyaml-cpp-dev
```

**安装spdlog库**：

```bash
# 方式1：通过apt安装（推荐）
sudo apt install -y libspdlog-dev

# 方式2：从源码编译安装（如果apt版本不满足要求）
cd /tmp
git clone https://github.com/gabime/spdlog.git
cd spdlog
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)
sudo make install
```

#### 2.4 串口权限配置

**配置串口访问权限**：

```bash
# 方式1：添加用户到dialout组（推荐，永久生效）
sudo usermod -a -G dialout $USER

# 方式2：临时设置权限（每次重启后需重新设置）
sudo chmod 666 /dev/ttyUSB0

# 注意：方式1需要重新登录才能生效
# 验证权限
groups | grep dialout
```

**验证串口设备**：

```bash
# 查看串口设备
ls -l /dev/ttyUSB*

# 查看串口信息
dmesg | grep ttyUSB
```

#### 2.5 完整依赖清单

**系统级依赖**：
- `build-essential` - 基础构建工具
- `cmake` (>= 3.10) - 构建系统
- `pkg-config` - 包配置工具
- `gcc` / `g++` (>= 9) - C++编译器
- `make` - 构建工具

**ROS2依赖（apt）**：
- `ros-humble-desktop` - ROS2桌面版（或按需安装 `ros-humble-rclcpp` 等）
- `ros-humble-rclcpp` - ROS2 C++客户端库
- `ros-humble-std-msgs` - ROS2标准消息
- `ros-humble-rosidl-default-generators` - ROS2接口生成器
- `ros-humble-rosidl-default-runtime` - ROS2接口运行时
- `python3-colcon-common-extensions` - Colcon构建工具扩展
- `python3-rosdep` - ROS依赖管理工具（可选）

**本仓库 ROS2 工作区包（源码编译，非 apt）**：`rh5dg2_interfaces`、`rh56f1_interfaces`、`inspire_control_ros2`，详见上文「ROS2 接口说明」与 `docs/依赖清单.md`。

**第三方库依赖**：
- `libboost-system-dev` - Boost系统库（包含Boost.Asio）
- `libboost-thread-dev` - Boost线程库
- `libboost-dev` - Boost开发库
- `libyaml-cpp-dev` - yaml-cpp开发库
- `libspdlog-dev` - spdlog开发库

**一键安装脚本**：

```bash
#!/bin/bash
# 完整依赖安装脚本

echo "=== 安装系统依赖 ==="
sudo apt update
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

echo "=== 安装Boost库 ==="
sudo apt install -y \
    libboost-system-dev \
    libboost-thread-dev \
    libboost-dev

echo "=== 安装yaml-cpp库 ==="
sudo apt install -y libyaml-cpp-dev

echo "=== 安装spdlog库 ==="
sudo apt install -y libspdlog-dev

echo "=== 配置串口权限 ==="
sudo usermod -a -G dialout $USER

echo "=== 依赖安装完成 ==="
echo "注意：串口权限配置需要重新登录才能生效"
echo "请运行: newgrp dialout 或重新登录"
```

#### 2.6 一键安装脚本（推荐）

**使用自动化安装脚本**：

```bash
# 运行依赖安装脚本
cd /home/ubuntu/serial_control
chmod +x install_dependencies.sh
./install_dependencies.sh
```

脚本会自动：
- 检测操作系统
- 安装所有系统依赖
- 安装Boost、yaml-cpp、spdlog库
- 配置串口权限
- 检查ROS2安装状态
- 提供详细的安装反馈

#### 2.7 验证安装

**验证系统依赖**：

```bash
# 检查CMake版本
cmake --version  # 应 >= 3.10

# 检查GCC版本
gcc --version    # 应 >= 9

# 检查G++版本
g++ --version    # 应 >= 9
```

**验证ROS2安装**：

```bash
# 检查ROS2环境
echo $ROS_DISTRO  # 应显示: humble

# 检查ROS2包
ros2 pkg list | grep rclcpp

# 检查colcon
colcon --version
```

**验证第三方库**：

```bash
# 检查Boost
pkg-config --modversion boost

# 检查yaml-cpp
pkg-config --modversion yaml-cpp

# 检查spdlog（如果通过apt安装）
dpkg -l | grep spdlog
```

**验证串口权限**：

```bash
# 检查用户组
groups | grep dialout

# 检查串口设备
ls -l /dev/ttyUSB*  # 应显示用户有读写权限
```

### 3. 编译项目

#### 一键编译整个工作区（推荐）

仓库根目录即 colcon 工作区，裸库与 ROS 包一起构建，依赖顺序自动解析：

```bash
cd /home/ubuntu/serial_control
source /opt/ros/humble/setup.bash   # 或本机已安装的 ROS2 distro
colcon build
source install/setup.bash
```

仅改节点代码时可只编译节点包：`colcon build --packages-select inspire_control_ros2`；改了裸库或接口包时需带上对应包（或直接全量 `colcon build`）。

#### 仅编译核心库（无 ROS 环境时）

裸库是纯 CMake 包，可脱离 ROS 单独构建（含 `serial_hand_control_node` 示例）：

```bash
cd /home/ubuntu/serial_control/src/inspire_serial_core
cmake -S . -B build && cmake --build build -j
```

#### 运行单元测试

核心库自带 gtest 单元测试，覆盖：`RingBuffer` 环形缓冲、`DeviceWorker` 串口事务串行化（FIFO 执行、异常传播、并发提交无重叠、关停语义），以及 RH56F1 / RH5DG2 / EG5CD1 三个 485 协议的命令构建、响应解析、校验和等**纯逻辑**。全部用例不依赖真实串口硬件，测试源码位于 `src/inspire_serial_core/tests/`。

- **colcon 工作区方式**（推荐）：

```bash
cd /home/ubuntu/serial_control
source /opt/ros/humble/setup.bash
colcon build --packages-select inspire_serial_core
colcon test --packages-select inspire_serial_core
colcon test-result --all          # 查看测试汇总
```

- **独立 CMake 方式**（无 ROS 环境）：

```bash
cd /home/ubuntu/serial_control/src/inspire_serial_core
cmake -S . -B build && cmake --build build -j
ctest --test-dir build --output-on-failure
```

> 测试默认随核心库一起构建（CMake 选项 `INSPIRE_SERIAL_CORE_BUILD_TESTS=ON`）；若环境未安装 GTest（`libgtest-dev`），构建会自动跳过测试而不影响主库。关闭测试可加 `-DINSPIRE_SERIAL_CORE_BUILD_TESTS=OFF`。

### 4. 配置设备

编辑 **`src/driver/config/device_protocol_config.yaml`**（或与 launch 一致的 `--device-config` 路径）：

```yaml
protocol:
  type: RH56F1_485

devices:
  - name: hand_left
    port: /dev/ttyUSB0
    baudrate: 115200
    Hand_ID: 1
```

### 5. 启动节点

#### 单设备模式

```bash
ros2 launch inspire_control_ros2 inspire_control_single_device.launch.py \
  device_name:=hand_left
```

#### 多设备模式

```bash
ros2 launch inspire_control_ros2 inspire_control_multi_device.launch.py
```

### 6. 使用示例

以下示例假定 **`protocol.type`** 为 RH5DG2 系列（**13** 个关节）。若为 **RH56F1** 系列，请将包名改为 **`rh56f1_interfaces`**，且 **`joint_values` 长度为 6**。也可用 `ros2 interface show <包名>/<类型>` 查看字段。

**`hand_id` 与节点绑定**：入站 Topic/Service 中的 **`hand_id`** 须与 **`device_protocol_config.yaml`** 里该设备的 **`Hand_ID`** 一致，否则节点会拒绝写寄存器（`accepted: false`）或忽略订阅回调；**`hand_id: 0`** 视为未指定，仍会被本节点接受（兼容不指定id）。

#### 发布控制命令（话题模式）

```bash
# 角度命令（示例数值请按现场标定修改）
ros2 topic pub --once /hand_left/angle_set rh5dg2_interfaces/msg/SetAngle1 \
  "{hand_id: 1, joint_values: [100,100,100,100,100,100,100,100,100,100,100,100,100]}"
```

#### 订阅状态数据（话题模式）

```bash
ros2 topic echo /hand_left/angle_actual
```

#### 调用服务（服务模式）

```bash
# 角度设置服务（与寄存器 angleSet 对应）
ros2 service call /hand_left/set_angle rh5dg2_interfaces/srv/Setangle \
  "{command: '', hand_id: 1, joint_values: [100,100,100,100,100,100,100,100,100,100,100,100,100]}"

# 读取故障码（示例）
ros2 service call /hand_left/get_errorCode rh5dg2_interfaces/srv/Geterror \
  "{query: '', hand_id: 1}"

# 设置设备通信 ID
ros2 service call /hand_left/set_id rh5dg2_interfaces/srv/Setid \
  "{hand_id: 1, device_id: 1}"
```

## 文档说明

### 项目架构说明

📖 **[docs/项目架构说明.md](docs/项目架构说明.md)**

包含：
- 系统整体架构图
- 各模块关系和数据流
- 线程模型
- 启动流程
- 扩展点说明

### 模块使用说明

📖 **[docs/模块使用说明.md](docs/模块使用说明.md)**

包含：
- 各模块功能简介
- 主要类和函数说明
- 配置参数说明
- 使用示例
- 数据流和通信方式

### 依赖清单

📖 **[docs/依赖清单.md](docs/依赖清单.md)**

包含：
- 完整的依赖项列表
- 版本要求
- 安装命令
- 验证方法
- 常见问题

### 协议格式说明

📖 **[docs/RH56F1_485协议格式说明.md](docs/RH56F1_485协议格式说明.md)**（另见 `docs/RH5DG2_485协议格式说明.md`、`docs/EG5CD1协议格式说明.md`、`docs/EG5CD1_ROS2_API.md`）

包含：
- 读写请求格式
- 读写回复格式
- 各字节含义
- 校验和计算
- 完整示例

## 核心模块

### 1. 串口通信模块 (SerialPortBase)

基于Boost.Asio的异步串口通信，支持阻塞式读写和超时机制。

**主要功能**：
- 异步接收数据
- 阻塞式发送数据
- 超时读取
- 线程安全

### 2. 协议抽象层 (Protocol)

协议抽象基类，定义统一的协议接口。支持多种协议实现（RH56F1_485、RH5DG2_485等）。

**主要功能**：
- 命令构建
- 响应解析
- 校验和验证
- 寄存器读写

**统一错误类型 `IoError`（`src/inspire_serial_core/include/io_error.hpp`）**：

读写接口不再返回简单的 `bool`，而是返回结构化错误码，贯穿「协议层 → `IRegisterIoBackend` → `InterfaceAdapter`」，让上层能区分失败原因：

| 接口 | 返回类型 | 说明 |
|------|----------|------|
| `writeRegister(...)` | `IoError` | `Ok` 成功；其余为错误码 |
| `readRegister(...)` | `RegisterReadResult` | `{ IoError error; std::vector<int> values; }`，`.ok()` 判断成功 |
| `readTouchData(...)` | `TouchReadResult` | `{ IoError error; TouchDataResult data; }`，`.ok()` 判断成功 |

`IoError` 取值：`Ok / Timeout（无应答）/ ChecksumError（校验失败）/ BadResponse（帧非法）/ UnknownRegister（寄存器名未注册）/ InvalidArgument（参数越界）/ NotSupported（机型不支持）/ DeviceError（串口/设备异常）`。可用 `isOk(e)` 判断成功、`toString(e)` 取可读字符串用于日志。

**错误码已透传到 Service 响应**：所有 `.srv` 响应均新增 `string message` 字段，由适配器写入 `toString(IoError)`：

- **写服务**（`bool accepted` + `string message`）：`accepted = isOk(e)`，`message` 为错误码字符串（如 `timeout`、`checksum_error`）；`hand_id` 不匹配时 `message = "rejected: hand_id mismatch"`。
- **读服务**（原有 `value` / `joint_values` 等 + `string message`）：`message = toString(rr.error)`，读失败时数值填 0 且 `message` 给出原因。
- **组合服务**（EG-5CD1 力控/触控）：`message` 标明失败步骤，如 `speedSet: timeout`、`invalid_argument: force ...`；全部成功则为 `catchModeClose: ok` 等。

调用方据此即可在程序里区分失败原因，无需再翻日志。

### 3. 设备管理器 (DeviceManager)

管理多个串口设备，维护端口到设备对象的映射关系。

**主要功能**：
- 设备添加/移除
- 设备查询
- 多设备管理

### 4. ROS2控制器 (RegisterController)

ROS2 设备控制节点，通过 **`InterfaceAdapter`** 使用 **`rh5dg2_interfaces` / `rh56f1_interfaces`** 中的消息与服务类型。

**主要功能**：
- 话题：订阅命令、发布状态（消息类型由 **`device_protocol_config.yaml`** 的 **`protocol.type`** 推导的机型决定）
- 服务：各功能对应独立 `.srv`，不再使用统一 Register 服务
- 定时器循环：默认 50Hz（`update_rate` 可配）

**并发模型（串口事务串行化）**：

每个设备节点持有一个 **`DeviceWorker`**（请求队列 + 单工作线程，见 `inspire_serial_core/include/device_worker.hpp`）。所有读寄存器、写寄存器、组合写序列（`ioWriteSequence`）都被提交到该 worker，由单线程按 FIFO 执行——这从结构上保证对同一串口的「写命令 → 读应答 → 解析」整组事务**永不交错**。

同时 `RegisterController` 把**定时器**与**服务/订阅**放进不同的回调组（定时器=互斥组，服务=可重入组），配合 `MultiThreadedExecutor`，使「定时读状态」与「服务/话题写寄存器」可在不同线程**并行进入**，而真正落到串口时仍由 worker 串行化。要点：

- 服务回调对 worker 的 `future.get()` 等待不会阻塞定时器线程（不同回调组）。
- 定时读做**合并背压**：上一次读任务未完成则跳过本次提交，避免队列堆积。
- 每次事务起始清空串口 RX 缓冲，去除历史帧残留。
- 回调内不再 `sleep` 持锁；EG-5CD1 组合序列作为单个原子任务在 worker 上执行。

> **真机验证**：已用 RH5DG2（`/dev/ttyUSB0`，115200，Hand_ID 1）做硬件冒烟测试——50Hz 定时读、状态话题发布、只读服务并发调用、`set_angle` 写入与「读+写+服务」混合并发压测均通过；约数万次读取仅出现 1 次瞬时读失败且下一周期立即自恢复、未污染后续帧，验证了「每次事务清空 RX + worker 串行化」对偶发失败的隔离效果。

### 5. 配置系统 (ConfigLoader)

从YAML文件加载配置，支持设备配置和日志配置。

**主要功能**：
- 设备配置加载
- 协议对象创建
- 日志系统配置

### 6. 日志系统 (LoggerManager)

统一的日志管理，基于spdlog实现。

**主要功能**：
- 控制台和文件输出
- 日志轮转
- 级别控制
- 线程安全

## 通信方式

### 话题模式（Topic）

**特点**：
- 实时性高
- 适合连续控制
- 定时器循环读取和发布

**使用场景**：
- 实时角度控制
- 实时力控制
- 状态监控

### 服务模式（Service）

**特点**：
- 按需调用
- 不参与定时循环
- 适合单次操作

**使用场景**：
- 设备配置（ID、波特率等）
- 错误查询
- 状态查询

## 配置文件

### 设备协议配置 (device_protocol_config.yaml)

```yaml
protocol:
  type: RH56F1_485

devices:
  - name: hand_left
    port: /dev/ttyUSB0
    baudrate: 115200
    Hand_ID: 1

logging:
  level: DEBUG
  file: logs/hand_control.log
  console: true
  file_enable: true
  max_file_size_mb: 10
  max_files: 5
```

### ROS2控制器配置 (ros2_controller_config.yaml)

```yaml
device_nodes:
  - device: hand_left
    update_rate: 50
    publish_header:
      frame_id: "hand_left"
    joint_names:
      - "hand_left/joint_0"
      # ... 共 13 项（RH5DG2）或 6 项（RH56F1）

    topics:
      - name: angle_control
        registers:
          write: ["angleSet"]
          read: ["angleAct"]
        command_topic: "/hand_left/angle_set"
        state_topic: "/hand_left/angle_actual"

    services:
      - register_name: "angleSet"
        set_service_name: "/hand_left/set_angle"
        is_write_register: true
```

## 常见问题

### 1. 依赖安装问题

#### CMake版本过低

```bash
# 检查CMake版本
cmake --version

# 如果版本 < 3.10，升级CMake
# Ubuntu 22.04默认CMake版本通常满足要求
# 如果需要升级，可以从源码编译或使用snap
sudo snap install cmake --classic
```

#### Boost库找不到

```bash
# 检查Boost安装
pkg-config --modversion boost

# 如果找不到，重新安装
sudo apt install --reinstall libboost-system-dev libboost-thread-dev libboost-dev

# 检查库文件位置
dpkg -L libboost-system-dev | grep .so
```

#### yaml-cpp库找不到

```bash
# 检查yaml-cpp安装
pkg-config --modversion yaml-cpp

# 如果找不到，重新安装
sudo apt install --reinstall libyaml-cpp-dev

# 检查库文件位置
dpkg -L libyaml-cpp-dev | grep .so
```

#### spdlog库找不到

```bash
# 方式1：通过apt安装（推荐）
sudo apt install libspdlog-dev

# 方式2：从源码编译
cd /tmp
git clone https://github.com/gabime/spdlog.git
cd spdlog
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)
sudo make install
sudo ldconfig
```

#### ROS2未安装或版本不对

```bash
# 检查ROS2版本
echo $ROS_DISTRO

# 如果未设置，安装ROS2 Humble
# 参考上述"安装ROS2依赖"部分

# 如果版本不对，卸载旧版本后重新安装
```

#### 编译时找不到头文件

```bash
# 检查库的头文件位置
dpkg -L libboost-dev | grep include
dpkg -L libyaml-cpp-dev | grep include

# 如果找不到，重新安装开发包
sudo apt install --reinstall libboost-dev libyaml-cpp-dev
```

### 2. 串口权限问题

```bash
# 添加用户到dialout组
sudo usermod -a -G dialout $USER

# 重新登录后生效，或立即生效
newgrp dialout

# 验证权限
groups | grep dialout

# 或临时设置权限
sudo chmod 666 /dev/ttyUSB0
```

### 3. 设备未找到

- 检查串口设备：`ls -l /dev/ttyUSB*`
- 检查配置文件中的端口路径
- 检查设备是否已连接
- 检查USB转串口驱动：`lsmod | grep usbserial`

### 4. 通信超时

- 检查波特率配置
- 检查设备ID（Hand_ID）配置
- 检查串口连接
- 查看日志文件排查问题
- 检查串口是否被其他程序占用：`lsof /dev/ttyUSB0`

### 5. ROS2节点未启动

- 检查配置文件路径
- 检查ROS2环境：`source install/setup.bash`
- 检查ROS2包是否编译：`colcon list`
- 查看日志：`ros2 run inspire_control_ros2 inspire_control_node --ros-args --log-level debug`
- 检查节点是否已运行：`ros2 node list`

### 6. 编译错误

#### 找不到ROS2包

```bash
# 确保已source ROS2环境
source /opt/ros/humble/setup.bash

# 检查ROS2包
ros2 pkg list | grep rclcpp
```

#### 链接错误

```bash
# 检查库文件是否存在
ldconfig -p | grep boost
ldconfig -p | grep yaml
ldconfig -p | grep spdlog

# 更新动态链接库缓存
sudo ldconfig
```

#### CMake找不到包

```bash
# 检查pkg-config路径
echo $PKG_CONFIG_PATH

# 如果为空，添加默认路径
export PKG_CONFIG_PATH=/usr/lib/pkgconfig:/usr/local/lib/pkgconfig
```

## 扩展开发

### 添加新协议

1. 创建新协议类，继承`Protocol`
2. 实现所有纯虚函数
3. 使用`REGISTER_PROTOCOL`宏注册
4. 在配置文件中指定协议类型

### 添加新寄存器

1. 在协议类的 `REGISTER_MAP` 中添加寄存器地址（及读长度等）
2. 在对应机型的 interfaces 中增加专用 `srv`/`msg`（若需对外暴露）
3. 在 **`(device)_interface_adapter.cpp`** 中为该寄存器接线
4. 在 **`ros2_controller_config.yaml`** 中增加 `topics` 或 `services` 项

### 添加新设备

1. 在`device_protocol_config.yaml`中添加设备配置
2. 在`ros2_controller_config.yaml`中添加设备节点配置
3. 系统自动识别并启动

---

**文档版本**：v1.1
**最后更新**：2026-06-17

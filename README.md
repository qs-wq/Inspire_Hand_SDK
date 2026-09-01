# 灵巧手控制系统（Inspire / ROS2）

[![CI](https://github.com/jsadin/Inspire_Hand_SDK/actions/workflows/ci.yml/badge.svg)](https://github.com/jsadin/Inspire_Hand_SDK/actions/workflows/ci.yml)

基于 C++ 与 ROS2 的多设备灵巧手控制系统，底层通过 RS485 / CANFD 等与多台 Inspire 系列灵巧手通信。节点包名为 **`inspire_control_ros2`**。

## 项目简介

本项目是一个模块化的灵巧手控制系统，支持：
- ✅ **多设备支持**：同时控制多个灵巧手设备（如左手、右手）
- ✅ **多协议支持**：通过工厂模式支持多种通信协议（**RH524J1_485**、RH56F1_485、**RH56H1_485** / **RH56H1_canfd**、**RH56DFX_serial_can**、RH5DG2_485、EG5CD1_485、**EG2_4C2_serial_can** 等）
- ✅ **动态配置**：通过 YAML 配置设备协议与 ROS2 话题/服务
- ✅ **双通信模式**：支持话题（实时控制）和服务（按需调用）两种方式
- ✅ **异步串口通信**：基于Boost.Asio的异步串口通信，支持超时和错误处理
- ✅ **统一日志系统**：全局日志管理器，支持文件轮转和级别控制
- ✅ **持续集成（CI）**：GitHub Actions 自动编译、跑单元测试与静态检查

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
│   │   ├── config/                    #    device_protocol_config.yaml、device_protocol_rh56f1_example.yaml、device_protocol_rh5dg2_example.yaml ...
│   │   └── tests/                     #    gtest 单元测试（RingBuffer / DeviceWorker / 三款 485 协议，不依赖硬件）
│   ├── driver/                        # ② 功能包 inspire_control_ros2（find_package(inspire_serial_core)）
│   │   ├── src/                       #    节点、RegisterController、机型适配器
│   │   ├── include/
│   │   ├── config/                    #    device_protocol_*.yaml、ros2_controller_*.yaml
│   │   └── launch/                    #    inspire_control_*.launch.py
│   └── interfaces/                    # ③ 接口包
│       ├── RH524J1/                   #    rh524j1_interfaces（24 自由度腱绳手）
│       ├── RH5DG2/                    #    rh5dg2_interfaces（13 自由度）
│       ├── RH56F1/                    #    rh56f1_interfaces（6 自由度）
│       ├── RH56H1/                    #    rh56h1_interfaces（6 自由度，触觉 version2 压阻式）
│       ├── RH56DFX/                   #    rh56dfx_interfaces（Serial-CAN 灵巧手）
│       ├── EG5CD1/                    #    eg5cd1_interfaces（EG-5CD1 夹爪）
│       └── EG2_4C2/                   #    eg2_4c2_interfaces（EG2-4C2 Serial-CAN 夹爪）
├── docs/                              # 全部文档集中存放（架构/模块/依赖/协议规则/厂商手册）
├── scripts/                           # CI 辅助脚本（clang-format / clang-tidy 检查）
├── .github/workflows/                 # GitHub Actions CI 配置
├── install_dependencies.sh           # 依赖安装脚本（一键安装）
├── .gitignore
└── README.md                         # 本文件
```

> **依赖关系**：`driver` 通过 `find_package(inspire_serial_core)` 链接裸库导出的 `inspire_serial_core::inspire_serial_core` 目标；colcon 依 `package.xml` 的 `<depend>inspire_serial_core</depend>` 自动保证先构建裸库。裸库采用 **SHARED** 库，使各协议 `REGISTER_PROTOCOL` 自注册对象随 `.so` 加载执行（避免 STATIC 归档丢符号）。

### ROS2 接口说明（重构后）

| 包名 | 作用 |
|------|------|
| **inspire_control_ros2** | 节点与驱动逻辑：`inspire_control_node`、`RegisterController`、**`RH524J1InterfaceAdapter`** / `RH5DG2InterfaceAdapter` / `RH56F1InterfaceAdapter` / **`RH56H1InterfaceAdapter`** / **`RH56DFXInterfaceAdapter`** / **`EG5CD1InterfaceAdapter`** / **`EG2_4C2InterfaceAdapter`**，配置文件安装在 `share/inspire_control_ros2/config`。 |
| **rh524j1_interfaces** | RH524J1（065 腱绳手，24 自由度）专用 `msg`/`srv`，`joint_values` 长度 24，顺序为电缸 ID1～ID24。话题含角度/力/速度/电流；服务含设置角度力速度、读温度/故障码/状态/电流/力/角度等。 |
| **rh5dg2_interfaces** | RH5DG2（13 自由度）专用 `msg`/`srv`，例如 `SetAngle1`、`GetAngleAct1`、`Setforce`、`Geterror` 等。 |
| **rh56f1_interfaces** | RH56F1（6 自由度）专用 `msg`/`srv`。 |
| **rh56h1_interfaces** | RH56H1（6 自由度）专用 `msg`/`srv`，与 `rh56f1_interfaces` 字段一致（寄存器/帧相同），但作为**独立接口包**与其他机械手架构对齐；触觉使用 version2（压阻式）的 `TouchData2`。 |
| **rh56dfx_interfaces** | RH56DFX Serial-CAN 灵巧手专用 `msg`/`srv`，服务集与 RH5DG2/RH56F1 对齐（`Setangle`/`Setforce`/`Setspeed`/`Setid`/`Setbaudrate`/`Setclearerror`/`Setactionseqindex`/`Geterror`/`Getstatus`/`Gettemp` 等已支持；`Setmode`/`Setpause`/`Setstop`/`Setresetpara`/`Setgestureforceclb`/`Setactionlibraryindex` 在当前 CAN 协议中暂未定义，调用返回 `not_supported`；另含 DFX 特有 `Setsave`）。电流话题 `SetCurrent1`/`GetCurrentAct1` 已映射至 CAN 寄存器 `currentSet`（1020 `CURRENT_LIMIT`）与 `currentAct`（1594 `CURRENT`）；`touchAct` 在当前机型无触觉硬件，话题保留但不发布数据。 |
| **eg5cd1_interfaces** | **因时 EG-5CD1** 电动夹爪 RS485：`GripperState`、`SetInt32`、`TriggerForHand`、`SetInt32Value`、`GetScalarForHand`；**组合服务** `ForceModeGrasp` / `ForceModeOpen` / `TouchModeGrasp` / `TouchModeOpen`（仅 `hand_id`+`speed`+`force`，内部按文档顺序经 `ioWriteSequence` 在设备 `DeviceWorker` 上**原子串行**写寄存器，见下）。 |
| **eg2_4c2_interfaces** | **因时 EG2-4C2** 电动夹爪 Serial-CAN（USB-CAN 转串口模块，详见 `docs/4C2夹爪CAN转Serial通信规则.md`）：`GripperState`、`SetInt32`、`TriggerForHand`、`SetInt32Value`、`GetScalarForHand`；**组合服务** `ForceModeGrasp` / `ForceModeOpen`（4C2 手册只定义 `catchMode` 0/1 两档位置/力控，无触控模式，故不提供 `TouchModeGrasp/Open`）。与 `eg5cd1_interfaces` 服务签名一致，**架构上与其他机械手夹爪对齐**，可通过 `eg2_4c2_composite_service_prefix` 改前缀（默认 `/gripper`）。 |

在 **`device_protocol_config.yaml`** 中设置 **`protocol.type`**（如 **`RH524J1_485`**、**`RH5DG2_485`**、**`RH56F1_485`**、**`RH56H1_485`** / **`RH56H1_canfd`**、**`RH56DFX_serial_can`**、**`EG5CD1_485`**、**`EG2_4C2_serial_can`** 等），启动时自动推导 **`interfaces_profile`**（**`RH524J1`** / `RH5DG2` / `RH56F1` / **`RH56H1`** / **`RH56DFX`** / **`EG5CD1`** / **`EG2_4C2`**）并创建对应适配器。

**RH56H1** 与 **RH56F1** 寄存器与帧格式相同，支持 **485** 与 **CAN-FD** 两种 `protocol.type`；ROS 接口使用**独立包 `rh56h1_interfaces`**（字段与 `rh56f1_interfaces` 一致，架构上与其他机械手对齐，不再复用 F1 包）。二者唯一区别是**触觉传感器类型**：RH56F1 为 version1（电容式），RH56H1 为 **version2（压阻式）**。解码规则（有符号 int16、temp/errCode 取低字节、触觉 version2 布局与 float 合力等）已逐条与厂商参考 `RH56H1_SDK` 核对一致。

**RH56H1 触觉 version2（压阻式）** 已在 `RH56H1_485_Protocol` 中实现（参考 `RH56H1_SDK`），结构与 RH56F1 完全一致，仅 `readTouchData` / `parseTouchData` 两个触觉函数实现 version2 逻辑：
- **寄存器布局**：每根手指 `tip_end` 指端 `2*2=4` 个 int16、`tip_touch` 指尖 `6*5=30` 个 int16、`force` 合力 `x/y/z` 三个 float32；掌心 `palm` `15*6=90` 个 int16。手指顺序 `pinky/ring/middle/index/thumb`。
- **读取方式**：因数据量约 580 字节超过单帧 485 读取上限，按段多次读取（5 指各 3 段 + 掌心 1 段）后拼装再解析。
- **ROS 发布**：专用消息 **`rh56h1_interfaces/msg/TouchData2`** 承载 version2 完整数据（RH56F1 仍用自身的 `TouchData1`）。控制器配置中将 `touch_control` 的 **`touch_version` 设为 2** 时，`RH56H1InterfaceAdapter` 自动发布 `TouchData2`；可参考 **`ros2_controller_rh56h1_example.yaml`**。
- **CAN-FD**：`RH56H1_canfd_Protocol` 继承自 `RH56H1_485_Protocol`，其 `readTouchData` 的 version2 分支按 CAN-FD 合法字节长度（含 >64 自动拆帧）逐段读取后拼装，复用继承来的 `parseTouchData(version=2)` 解析；485 与 CAN-FD 的 version2 表现一致。

### RH56H1 百分比接口（0.0~100.0）

在 `rh56h1_interfaces` 中新增了**百分比**接口，与现有 raw（寄存器原始值）接口**并列存在、互不影响**。百分比接口内部按 RH56H1 用户手册 V1.2 的量程，将 `0.0~100.0` 线性换算成寄存器原始值后，**复用原本的寄存器读写通路**（`ioWriteRegister`/`ioReadRegister`）；读取时反向把原始值换算成百分比返回。超出 `0~100` 的输入会**自动裁剪**。

**位置百分比基于「角度 `angleSet/angleAct`」换算**（手册 2.5.9 明确不建议用电缸位置 `posSet` 设角度，推荐用 `angleSet`）。因各指角度范围不同，位置采用**逐关节**换算，方向约定 **0%=握紧（最小角度）、100%=张开（最大角度）**；速度/力/电流为 0~量程上限的统一线性换算。

| 项目 | 换算寄存器 | 量程（0% ↔ 100%） | 手册依据 | 话题（`SetPercent1`/`GetPercentAct1`，float32[6]） | 服务（`Setpercent`/`Getpercent`，float32[6]） |
|------|-----------|------------------|------|--------------------|------------------|
| 设置位置 | `angleSet` | 逐关节：四指 870→1690、拇指弯曲 950→1350、拇指旋转 700→1700（0%→100%） | 表35（2.5.10） | `/hand_left/pos_percent_set` | `/hand_left/set_pos_percent` |
| 设置速度 | `speedSet` | 0 – 3000（前 5 指） | 表38（2.5.12） | `/hand_left/speed_percent_set` | `/hand_left/set_speed_percent` |
| 设置力 | `forceSet` | 0 – 900（前 5 指） | 表37（2.5.11） | `/hand_left/force_percent_set` | `/hand_left/set_force_percent` |
| 设置电流 | `currentSet` | 0 – 1500（前 5 指） | 表31（2.5.6，电缸电流保护值） | `/hand_left/current_percent_set` | `/hand_left/set_current_percent` |
| 读取位置 | `angleAct` | 逐关节，同「设置位置」范围 | 表40（2.5.14） | `/hand_left/pos_percent_actual`（发布） | `/hand_left/get_pos_percent` |

> **说明与例外**（依据 RH56H1 手册 V1.2）：
> - **位置为何用角度**：`posSet`(0~2000) 是**电缸位置**（0=张开、2000=握紧），手册 2.5.9 明确"不建议用它设定手指位置角度"；`angleSet` 才是推荐的角度寄存器，单位 0.1°，各指范围为四指 870~1690、拇指弯曲 950~1350、拇指旋转 700~1700，**角度越大越张开**。**四指 angleSet 上限 1690，不是 posSet 的 2000**。因此位置百分比按各指范围逐关节换算，`50%` 表示该指角度范围中点。`angle_set` 超范围输入由驱动自动裁剪到手册合法区间。
> - **速度第 6 指（大拇指旋转）例外**：手册大拇指旋转 `speedSet(1057)` 范围为 **0~20**，而非 0~3000，当前代码对该指仍按 0~3000 换算，故 `speed_percent` 对大拇指旋转**不准确**；请对其速度改用 raw 接口（`speedSet`）直接给 0~20。
> - **力 / 电流第 6 指**：手册对大拇指旋转 `forceSet(1051)`、`currentSet(1021)` 标注 `\`（舵机通道未单独定义量程），百分比换算对该指仅为近似。

- **配置**：`ros2_controller_config.yaml` 与 `ros2_controller_rh56h1_example.yaml` 均已包含百分比接口；launch 默认加载前者。
- **启动**（修改 `device_protocol_config.yaml` 中 `port`、`Hand_ID` 后执行）：

```bash
source install/setup.bash
ros2 launch inspire_control_ros2 inspire_control_single_device.launch.py device_name:=hand_left
```

#### 关节顺序（`joint_values[6]`）

| 下标 | 关节 | 简称 |
|------|------|------|
| 0 | 小拇指 | pinky |
| 1 | 无名指 | ring |
| 2 | 中指 | middle |
| 3 | 食指 | index |
| 4 | 大拇指弯曲 | thumb_bend |
| 5 | 大拇指旋转 | thumb_rot |

#### 百分比话题与服务（0.0 ~ 100.0）

方向约定：**位置** `0%`=握紧、`100%`=张开（最大角度）；**速度/力/电流** `0%`=0、`100%`=量程上限。输入超范围自动裁剪。

**位置百分比**（写入 `angleSet`，读取 `angleAct`；**非**电缸 `posSet`）：

| 百分比 | 四指 angleSet | 拇指弯曲 | 拇指旋转 |
|--------|--------------|---------|---------|
| 0%（握紧） | 870 | 950 | 700 |
| 50%（中间） | 1280 | 1150 | 1200 |
| 100%（张开） | 1690 | 1350 | 1700 |

```bash
# --- 位置百分比：话题 ---
ros2 topic pub --once /hand_left/pos_percent_set rh56h1_interfaces/msg/SetPercent1 \
  "{hand_id: 1, joint_values: [50,50,50,50,50,50]}"

ros2 topic pub --once /hand_left/pos_percent_set rh56h1_interfaces/msg/SetPercent1 \
  "{hand_id: 1, joint_values: [100,100,100,100,100,100]}"
# 上式 100% 等价于 angle_set [1690,1690,1690,1690,1350,1700]（完全张开）

ros2 topic echo /hand_left/pos_percent_actual

# --- 位置百分比：服务 ---
ros2 service call /hand_left/set_pos_percent rh56h1_interfaces/srv/Setpercent \
  "{command: '', hand_id: 1, joint_values: [50,50,50,50,50,50]}"

ros2 service call /hand_left/get_pos_percent rh56h1_interfaces/srv/Getpercent \
  "{query: '', hand_id: 1}"

# --- 速度/力/电流百分比：话题（前5指量程见上表；第6指速度见例外说明）---
ros2 topic pub --once /hand_left/speed_percent_set rh56h1_interfaces/msg/SetPercent1 \
  "{hand_id: 1, joint_values: [50,50,50,50,50,50]}"

ros2 topic pub --once /hand_left/force_percent_set rh56h1_interfaces/msg/SetPercent1 \
  "{hand_id: 1, joint_values: [60,60,60,60,60,60]}"

ros2 topic pub --once /hand_left/current_percent_set rh56h1_interfaces/msg/SetPercent1 \
  "{hand_id: 1, joint_values: [50,50,50,50,50,50]}"

# --- 速度/力/电流百分比：服务 ---
ros2 service call /hand_left/set_speed_percent rh56h1_interfaces/srv/Setpercent \
  "{command: '', hand_id: 1, joint_values: [50,50,50,50,50,50]}"

ros2 service call /hand_left/set_force_percent rh56h1_interfaces/srv/Setpercent \
  "{command: '', hand_id: 1, joint_values: [60,60,60,60,60,60]}"

ros2 service call /hand_left/set_current_percent rh56h1_interfaces/srv/Setpercent \
  "{command: '', hand_id: 1, joint_values: [50,50,50,50,50,50]}"
```

> 速度/力/电流**无**百分比读取话题或服务（仅位置有 `pos_percent_actual` / `get_pos_percent`）。

#### 原始值(raw)话题与服务

**角度 `angleSet` / `angleAct`**（推荐；单位 0.1°，角度越大越张开，`-1`=该指不动）：

> **勿与 posSet 混淆**：`posSet`（电缸位置）量程 **0~2000**；`angleSet`（角度）四指最大 **1690**，不是 2000。向 `angle_set` 写入 2000 会被驱动**自动裁剪为 1690**（并打 warn 日志）。

| 下标 | angleSet 合法范围 | 物理角度 | 完全张开 | 完全握紧 |
|------|------------------|---------|---------|---------|
| 0~3 四指 | 870 ~ 1690，-1 | 87° ~ 169° | **1690** | **870** |
| 4 拇指弯曲 | 950 ~ 1350，-1 | 95° ~ 135° | **1350** | **950** |
| 5 拇指旋转 | 700 ~ 1700 | 70° ~ 170° | **1700** | **700** |

```bash
ros2 topic pub --once /hand_left/angle_set rh56h1_interfaces/msg/SetAngle1 \
  "{hand_id: 1, joint_values: [1280,1280,1280,1280,1150,1200]}"

ros2 topic pub --once /hand_left/angle_set rh56h1_interfaces/msg/SetAngle1 \
  "{hand_id: 1, joint_values: [870,870,870,870,950,700]}"

ros2 topic echo /hand_left/angle_actual

ros2 service call /hand_left/set_angle rh56h1_interfaces/srv/Setangle \
  "{command: '', hand_id: 1, joint_values: [1280,1280,1280,1280,1150,1200]}"
```

**力 `forceSet` / `forceAct`**（0~4：0~900 g；第6指手册未定义）：

```bash
ros2 topic pub --once /hand_left/force_set rh56h1_interfaces/msg/SetForce1 \
  "{hand_id: 1, joint_values: [600,600,600,600,600,600]}"

ros2 topic echo /hand_left/force_actual

ros2 service call /hand_left/set_force rh56h1_interfaces/srv/Setforce \
  "{command: '', hand_id: 1, joint_values: [600,600,600,600,600,600]}"
```

**速度 `speedSet`**（0~4：0~3000；第6指：**0~20**）：

```bash
ros2 topic pub --once /hand_left/speed_set rh56h1_interfaces/msg/SetSpeed1 \
  "{hand_id: 1, joint_values: [2000,2000,2000,2000,2000,10]}"

ros2 service call /hand_left/set_speed rh56h1_interfaces/srv/Setspeed \
  "{command: '', hand_id: 1, joint_values: [2000,2000,2000,2000,2000,10]}"
```

**电流 `currentSet` / `currentAct`**（0~4：0~1500 mA；第6指手册未定义）：

```bash
ros2 topic pub --once /hand_left/current_set rh56h1_interfaces/msg/SetCurrent1 \
  "{hand_id: 1, joint_values: [800,800,800,800,800,800]}"

ros2 topic echo /hand_left/current_actual
```

**触觉**（只读，`TouchData2`）：

```bash
ros2 topic echo /hand_left/touch_data
```

#### 系统管理与只读服务

| 服务 | 范围/含义 | 命令示例 |
|------|----------|---------|
| `set_id` | ID 1~254 | `ros2 service call /hand_left/set_id rh56h1_interfaces/srv/Setid "{hand_id: 1, device_id: 2}"` |
| `set_baudRate` | 0~3（485: 0=115200,1=57600,2=19200；CANFD 见手册） | `ros2 service call /hand_left/set_baudRate rh56h1_interfaces/srv/Setbaudrate "{hand_id: 1, baudrate: 0}"` |
| `set_clearError` | 写 1 清故障 | `ros2 service call /hand_left/set_clearError rh56h1_interfaces/srv/Setclearerror "{hand_id: 1, clear_code: 1}"` |
| `set_save` | 写 1 保存 Flash | `ros2 service call /hand_left/set_save rh56h1_interfaces/srv/Setsave "{hand_id: 1, save_code: 1}"` |
| `set_resetPara` | 写 1 恢复出厂 | `ros2 service call /hand_left/set_resetPara rh56h1_interfaces/srv/Setresetpara "{hand_id: 1, confirm: 1}"` |
| `set_defaultSpeed` | 0~4: 0~3000；第6指 0~20 | `ros2 service call /hand_left/set_defaultSpeed rh56h1_interfaces/srv/Setdefaultspeed "{hand_id: 1, joint_values: [2000,2000,2000,2000,2000,10]}"` |
| `set_defaultForceSet` | 0~4: 0~900 g | `ros2 service call /hand_left/set_defaultForceSet rh56h1_interfaces/srv/Setdefaultforceset "{hand_id: 1, joint_values: [600,600,600,600,600,600]}"` |
| `set_mode` | 0（速度力保护模式） | `ros2 service call /hand_left/set_mode rh56h1_interfaces/srv/Setmode "{command: '', hand_id: 1, joint_values: [0,0,0,0,0,0]}"` |
| `set_pause` | 写 1 暂停 | `ros2 service call /hand_left/set_pause rh56h1_interfaces/srv/Setpause "{hand_id: 1, pause_flag: 1}"` |
| `set_stop` | 写 1 急停 | `ros2 service call /hand_left/set_stop rh56h1_interfaces/srv/Setstop "{hand_id: 1, stop_flag: 1}"` |
| `set_actionSeqIndex` | 动作序列号 | `ros2 service call /hand_left/set_actionSeqIndex rh56h1_interfaces/srv/Setactionseqindex "{hand_id: 1, index: 1}"` |
| `get_errorCode` | 故障位（只读） | `ros2 service call /hand_left/get_errorCode rh56h1_interfaces/srv/Geterror "{query: '', hand_id: 1}"` |
| `get_status` | 状态码 0~8（只读） | `ros2 service call /hand_left/get_status rh56h1_interfaces/srv/Getstatus "{query: '', hand_id: 1}"` |
| `get_temp` | 0~100 ℃（只读） | `ros2 service call /hand_left/get_temp rh56h1_interfaces/srv/Gettemp "{query: '', hand_id: 1}"` |

> **手册有、SDK 未暴露**：电缸位置 `posSet`/`posAct`（0~2000，不建议用于设角度）、速度实际值 `speedAct` 等，需直接操作寄存器或后续扩展配置。

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

### EG2-4C2 夹爪全链路说明

因时 EG2-4C2 电动夹爪通过 **USB-CAN 转串口模块** 与主机通信（详见 `docs/4C2夹爪CAN转Serial通信规则.md` 与 `gripper_demo_can_serial.py`），架构与 RH56DFX_serial_can（同样是 Serial-CAN 灵巧手）和 EG5CD1（同样是单自由度夹爪）对齐：

- **协议实现**：`EG2_4C2_serial_can_Protocol`（`REGISTER_PROTOCOL("EG2_4C2_serial_can", …)`）。
  - 串口封包 21 字节：`AA AA | ExtId(4B 小端) | Data[8] | Meta[4] | Checksum | 55 55`，与 RH56DFX_serial_can 完全一致。
  - 29 位 ExtId 编码（手册第 4 章，与 demo `build_ext_id` 对齐）：
    `ExtId = (op_type & 0x3) << 26 | (reg_addr & 0xFFF) << 14 | (hand_id & 0x3FFF)`，
    `op_type`：00=读、01=写、02=定位、03=随动（实现只用读/写）。
    寄存器地址用 **Modbus 地址**（USB-CAN 串口专用），与手册第五节左列一致。
  - 写帧 Data[8] 不足部分用 `0xFF` 填充；写帧 Meta[0] **固定填 `0x08`**（与 `gripper_demo_can_serial.py:build_write_frame` 第 123 行行为一致：无论写 1/2/3/4 个寄存器，Meta[0] 均为 8）。
  - 读帧 Meta[0] 固定填 `0x01`（与 demo `READ_META = (0x01, 0x00, 0x01, 0x00)` 对齐；手册此处描述与 demo 不一致，**以 demo 实际跑通行为为准**）。
  - 校验和：`sum(ExtId..Meta) & 0xFF`（手册 §3.1 + demo 第 100 行一致）。
  - 应答解析：固定偏移 `frame[6..14]` 为 CAN 数据区，`0xA5` 转义规则同 RH56DFX_serial_can。
- **寄存器**（与 `gripper_demo_can_serial.py:REG` + 状态寄存器一致）：

  | 名称 | Modbus 地址 | 用途 | 默认读取长度 |
  |------|------------|------|-------------|
  | `save` / `defaultPar` | 1001 / 1002 | 保存 / 恢复默认 | 2 |
  | `id` / `baud` | 1003 / 1004 | 设备 ID / 波特率索引 | 2 |
  | `catchMode` | 1005 | 0=一次夹取，1=持续夹取 | 2 |
  | `stop` / `clearError` | 1006 / 1007 | 急停 / 清除故障 | 2 |
  | `openLenSet` / `speedSet` / `forceSet` | 1010 / 1011 / 1012 | 开口度 / 速度 / 力度 | 2 |
  | `maxOpenLen` / `minOpenLen` | 1016 / 1017 | 最大 / 最小开口 | 2 |
  | `forceAct` / `openLenAct` / `currentAct` / `temp` | 1060 / 1061 / 1062 / 1063 | 状态（只读） | 2 |
  | `errorCode` / `status` | 1064 / 1065 | 故障位 / 状态码（只读） | 2 |
  | `gripperStatusBlock` | 1060 | 一次读 1060–1065 共 12 字节（`forceAct / openLenAct / currentAct / temp / errorCode / status`），跨 8+4 两帧 | 12 |

- **示例配置**（随包安装到 `share/inspire_control_ros2/config`）：
  - `device_protocol_eg2_4c2_example.yaml`：`protocol.type: EG2_4C2_serial_can`，串口默认 `/dev/ttyUSB0` @ 115200。
  - `ros2_controller_eg2_4c2_example.yaml`：话题名与适配器约定一致——`gripper_state` / `open_len_set` / `speed_set` / `force_set` / `catch_mode_set`；服务含 `clear_error` / `save_params` / `restore_default` / `stop` / `set_id` / `set_baud_index` / `set_mode_service` / `set_max_open_len` / `set_min_open_len` / `get_error` / `get_temp` / `get_status`。
  - **夹取/张开组合服务**（节点启动后自动创建，前缀由参数 `eg2_4c2_composite_service_prefix` 控制，默认 `/gripper`）：
    - `{prefix}/force_mode_grasp`：夹取，内部写入持续夹取模式和目标开口度 `0`。
      - `speed`：`10～1000`
      - `force`：`100～1000`，必须填正数；数值越大，夹持力越大。
    - `{prefix}/force_mode_open`：张开，内部写入一次夹取模式和目标开口度 `1000`。
      - `speed`：`10～1000`
      - `force`：`-1000～-100`，必须填负数。负号只是 ROS API 用来表示“张开方向”，驱动实际向设备写入其绝对值。
    - `speedSet`、`forceSet` 单独写入只会修改参数，不会让夹爪运动；写入 `openLenSet` 才会触发运动。
    - 重复发送相同目标时，如果夹爪已经完全张开或闭合，不会再次产生肉眼可见的运动。重复测试应按“张开 → 夹取 → 张开”的顺序进行。
    - 服务返回 `accepted=true`、`message='openLenSet: ok'` 表示设备对寄存器写入作出了有效应答，不等于一定产生了位移；应同时查看 `/gripper/state` 中的 `open_len_act` 和 `status`。
    - 整组寄存器写入经 `ioWriteSequence` 在同一设备 worker 上串行执行，不会与定时状态读取交错。
  - **4C2 特有约束**：`catchMode` 只支持 0/1（无触控模式），故**不提供 `TouchModeGrasp/Open`**；当前机型无触觉硬件，`touchAct` 寄存器未实现，调用返回 `NotSupported`。
- **启动示例**：

```bash
ros2 run inspire_control_ros2 inspire_control_node -- \
  --device-config $(ros2 pkg prefix inspire_control_ros2)/share/inspire_control_ros2/config/device_protocol_eg2_4c2_example.yaml \
  --controller-config $(ros2 pkg prefix inspire_control_ros2)/share/inspire_control_ros2/config/ros2_controller_eg2_4c2_example.yaml
```

#### EG2-4C2 夹爪运动自检

当 `protocol.type: EG2_4C2_serial_can` 时，建议严格按照下面的顺序测试。先在一个终端持续查看状态：

```bash
source install/setup.bash

ros2 topic echo /gripper/state eg2_4c2_interfaces/msg/GripperState
```

重点查看：

- `open_len_act`：实际开口度，正常范围约为 `0～1000`；`0` 表示最小开口，`1000` 表示最大开口。
- `status`：`1` 已完全张开、`2` 已完全闭合、`3` 已停止、`4` 正在夹取、`5` 正在张开、`6` 夹取物体后力控停止。
- `error_code`：正常应为 `0`。

在另一个终端执行运动测试：

```bash
source install/setup.bash

# 1. 清除可能存在的故障
ros2 service call /gripper/clear_error \
  eg2_4c2_interfaces/srv/TriggerForHand \
  "{hand_id: 1}"

# 2. 张开：force 必须是 -1000～-100 的负数
ros2 service call /gripper/force_mode_open \
  eg2_4c2_interfaces/srv/ForceModeOpen \
  "{hand_id: 1, speed: 600, force: -400}"

# 3. 等夹爪完全张开后再夹取；force 必须是 100～1000 的正数
ros2 service call /gripper/force_mode_grasp \
  eg2_4c2_interfaces/srv/ForceModeGrasp \
  "{hand_id: 1, speed: 600, force: 300}"

# 4. 再次张开，确认能够往返运动
ros2 service call /gripper/force_mode_open \
  eg2_4c2_interfaces/srv/ForceModeOpen \
  "{hand_id: 1, speed: 600, force: -400}"
```

以下张开命令是错误示例，因为 `force` 使用了正数，驱动会返回 `accepted=false`，不会向夹爪发送运动序列：

```bash
ros2 service call /gripper/force_mode_open \
  eg2_4c2_interfaces/srv/ForceModeOpen \
  "{hand_id: 1, speed: 600, force: 900}"
```

若正确命令返回 `accepted=true`，但夹爪没有肉眼可见的运动：

1. 如果 `open_len_act` 已接近命令目标（张开约 `1000`、闭合约 `0`），说明夹爪已经到位，因此重复发送不会再次运动。
2. 如果 `open_len_act` 没有变化，检查电源、CAN 侧 `500 Kbps`、串口 `/dev/ttyUSB0 @ 115200`、设备 ID 和故障码。
3. 如果 `open_len_act` 超出 `0～1000` 或 `status` 长期为 `0`，说明状态数据异常，不能只凭 `accepted=true` 判断夹爪执行成功，应优先排查 USB-CAN 转串口模块的帧格式及 CAN 寄存器地址配置。

📖 **[docs/4C2夹爪CAN转Serial通信规则.md](docs/4C2夹爪CAN转Serial通信规则.md)** 通信协议详细规则。

### RH524J1 腱绳手全链路说明

因时 RH524J1（065 腱绳驱动，24 自由度）走 **RS485**，帧格式与 RH5DG2 相同（请求 `EB 90`），寄存器地址来自 `065demo/hand_param.h`（`angleSet=320`，一次写 24 路）。

- **协议**：`RH524J1_485_Protocol`（`REGISTER_PROTOCOL("RH524J1_485", …)`）
- **接口包**：`rh524j1_interfaces`（`joint_values` 固定 24 个整数）
- **示例配置**（随包安装到 `share/inspire_control_ros2/config`）：
  - `device_protocol_rh524j1_example.yaml`：`protocol.type: RH524J1_485`
  - `ros2_controller_rh524j1_example.yaml`：话题 `/hand_left/angle_set` 等，服务 `/hand_left/set_angle` 等
- **无触觉**、无力控组合服务。当前机型无 `touchAct`。

**启动**（改代码/配置后必须 **重新编译并重启节点**；正在跑的进程不会出现新服务）：

```bash
# 1) 在正在跑 launch 的终端按 Ctrl+C 停掉节点
# 2) 编译并加载
colcon build --packages-select inspire_control_ros2
source install/setup.bash

# 3) 用原来的命令重启即可（不必换 launch 文件）
ros2 launch inspire_control_ros2 inspire_control_single_device.launch.py device_name:=hand_left
```

也可使用 RH524J1 专用 launch（内容与上面默认配置一致）：

```bash
ros2 launch inspire_control_ros2 inspire_control_rh524j1.launch.py device_name:=hand_left
```

请先只编译本机型相关包，避免工作区里其它 ROS1 包拖垮构建：

```bash
colcon build --packages-select rh524j1_interfaces inspire_serial_core inspire_control_ros2
source install/setup.bash
```

> **`get_currentAct` 一直 `waiting for service`**：节点还是旧进程。先 `Ctrl+C`，再执行上面的 launch。重启后 `ros2 service list | grep get_currentAct` 应能看到服务。不等重启时，电流可用话题：`ros2 topic echo /hand_left/current_actual`。

#### `joint_values` 按下标 = 电缸 ID − 1

数组一共 24 个数，**第几个位置就对应几号电缸**：下标 `0` = 电缸 ID1，下标 `23` = 电缸 ID24。不要按手指重新排。

规则：

- **弯曲关节（除 ID1 外）：`0` = 张开，数值越大越弯曲。**
- **小指翻折（ID1 / 下标 0）实机范围 `-150 ~ 0`：`0` = 张开，`-150` = 尽量弯曲（负数方向）。**
- **不想动的关节填 `-1`**（保持当前角度）。**不要填 `0`**：`0` 会把该路拉回张开/零位。
- **只改 1 个关节时**：仅 1 路填角度、其余全 `-1`，SDK 会发**短帧**（与实机 `EB 90 … 05 12 40 01 …` 单路写相同）；多路同时改才发整包 24 路。
- 实机拇指指尖（ID20）和掌指（ID22）接线对调，协议已自动交换。你只要按表填写：下标 19 = 指尖、下标 21 = 掌指。

| 电缸 ID | 下标 | 关节名 | 部位 | 角度范围 |
|---------|------|--------|------|----------|
| 1 | 0 | `pinky_fold` | 小指翻折 | **-150 ~ 0**（0 张开，-150 弯曲） |
| 2 | 1 | `pinky_side` | 小指侧摆 | **-200 ~ 200** |
| 3 | 2 | `ring_side` | 无名指侧摆 | **-200 ~ 200** |
| 4 | 3 | `middle_side` | 中指侧摆 | **-200 ~ 200** |
| 5 | 4 | `index_side` | 食指侧摆 | **-200 ~ 200** |
| 6 | 5 | `pinky_mcp` | 小指掌指 | 0 ~ 900 |
| 7 | 6 | `ring_mcp` | 无名指掌指 | 0 ~ 900 |
| 8 | 7 | `middle_mcp` | 中指掌指 | 0 ~ 900 |
| 9 | 8 | `index_mcp` | 食指掌指 | 0 ~ 900 |
| 10 | 9 | `pinky_pip` | 小指指中 | 0 ~ 900 |
| 11 | 10 | `ring_pip` | 无名指指中 | 0 ~ 900 |
| 12 | 11 | `middle_pip` | 中指指中 | 0 ~ 900 |
| 13 | 12 | `index_pip` | 食指指中 | 0 ~ 900 |
| 14 | 13 | `pinky_dip` | 小指指尖 | 0 ~ 900 |
| 15 | 14 | `ring_dip` | 无名指指尖 | 0 ~ 900 |
| 16 | 15 | `middle_dip` | 中指指尖 | 0 ~ 900 |
| 17 | 16 | `index_dip` | 食指指尖 | 0 ~ 900 |
| 18 | 17 | `thumb_rot` | 拇指旋转 | 0 ~ 1000 |
| 19 | 18 | `thumb_side` | 拇指侧摆 | 0 ~ 1000 |
| 20 | 19 | `thumb_dip` | 拇指指尖 | 0 ~ 900 |
| 21 | 20 | `thumb_pip` | 拇指指中 | 0 ~ 900 |
| 22 | 21 | `thumb_mcp` | 拇指掌指 | 0 ~ 900 |
| 23 | 22 | `wrist_1` | 手腕 1 | 0 ~ 1000 |
| 24 | 23 | `wrist_2` | 手腕 2 | 0 ~ 1000 |

示例：小指掌指（ID6）700、中指掌指（ID8）800、食指指中（ID13）900、食指指尖（ID17）900、拇指掌指（ID22）600，其余张开：

```bash
ros2 service call /hand_left/set_angle rh524j1_interfaces/srv/Setangle \
  "{command: '', hand_id: 1, joint_values: [0,0,0,0,0,700,0,800,0,0,0,0,900,0,0,0,900,0,0,0,0,0,600,0]}"
```

只动拇指旋转（ID18 / 下标 17），其它关节保持不动：

```bash
ros2 service call /hand_left/set_angle rh524j1_interfaces/srv/Setangle \
  "{command: '', hand_id: 1, joint_values: [-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,400,-1,-1,-1,-1,-1,-1]}"
```

`hand_id` 必须和配置文件里的 `Hand_ID` 一致，否则话题会被忽略、服务会返回 `accepted: false`。下面所有命令都假定 `Hand_ID: 1`。24 路数组一律 **下标 = 电缸 ID − 1**。

#### 话题一览

节点按 `update_rate`（示例 50 Hz）循环读状态并发布。速度只有写入话题，没有实际速度反馈话题。

| 用途 | 写入话题 | 消息类型 | 读取话题 | 消息类型 | 寄存器 |
|------|----------|----------|----------|----------|--------|
| 角度 | `/hand_left/angle_set` | `SetAngle1` | `/hand_left/angle_actual` | `GetAngleAct1` | `angleSet` / `angleAct` |
| 力 | `/hand_left/force_set` | `SetForce1` | `/hand_left/force_actual` | `GetForceAct1` | `forceSet` / `forceAct` |
| 速度 | `/hand_left/speed_set` | `SetSpeed1` | （无） | — | `speedSet` |
| 电流 | `/hand_left/current_set` | `SetCurrent1` | `/hand_left/current_actual` | `GetCurrentAct1` | `currentSet` / `currentAct` |

无触觉话题（当前机型无 `touchAct`）。**日常位置控制只需「速度 + 角度」**；力接口见下节，可忽略。

> **角度 vs 速度单位**：`angleSet`/`angleAct` 单位为 **0.1°**（如 900=90°）。`speedSet` 为控制器内部 **标量档位**，**无 m/s、°/s 等物理单位**；065 实机满速刻度 **16384**（2¹⁴），0=最慢、16384=满速。下文示例取 **8192（约半速）**。

#### 角度话题与服务

弯曲关节 **`0` = 张开**，数值越大越弯曲；**`-1` = 该路保持当前角度**（仅 `angleSet` 支持）。先设速度再设角度更不容易“看起来没动”。

```bash
# --- 话题：写角度 / 读实际角度 ---
ros2 topic pub --once /hand_left/angle_set rh524j1_interfaces/msg/SetAngle1 \
  "{hand_id: 1, joint_values: [0,0,0,0,0,700,0,800,0,0,0,0,900,0,0,0,900,0,0,0,0,0,600,600]}"

ros2 topic echo /hand_left/angle_actual

# --- 服务：写角度 / 读实际角度 ---
ros2 service call /hand_left/set_angle rh524j1_interfaces/srv/Setangle \
  "{command: '', hand_id: 1, joint_values: [0,0,0,0,0,700,0,800,0,0,0,0,900,0,0,0,900,0,0,0,0,0,600,600]}"

ros2 service call /hand_left/get_angleAct rh524j1_interfaces/srv/Getangleact \
  "{query: '', hand_id: 1}"
```

成功时服务返回 `accepted: true`、`message: ok`（读服务看 `message: ok` 和 24 路 `joint_values`）。

#### 力话题与服务（可选）

本机型**无独立力/触觉传感器**；`forceSet`/`forceAct` 为 24 路电缸内部张力估算，**不做力控时可不发**。若使用：demo 只按 16 位截断，**SDK 不做量程裁剪**；不想改的路可填 `0` 或与当前值相同。

```bash
ros2 topic pub --once /hand_left/force_set rh524j1_interfaces/msg/SetForce1 \
  "{hand_id: 1, joint_values: [500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500]}"

ros2 topic echo /hand_left/force_actual

ros2 service call /hand_left/set_force rh524j1_interfaces/srv/Setforce \
  "{command: '', hand_id: 1, joint_values: [500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500]}"

ros2 service call /hand_left/get_forceAct rh524j1_interfaces/srv/Getforceact \
  "{query: '', hand_id: 1}"
```

#### 速度话题与服务

`speedSet` 为 24 路速度标量（**0~16384**，16384=满速）。运动前建议先设速度，再设角度。示例取 **8192（半速）**；更慢可试 4096，更快可试 12288（勿一次拉满）。没有 `speedAct` 反馈话题。

```bash
ros2 topic pub --once /hand_left/speed_set rh524j1_interfaces/msg/SetSpeed1 \
  "{hand_id: 1, joint_values: [8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192]}"

ros2 service call /hand_left/set_speed rh524j1_interfaces/srv/Setspeed \
  "{command: '', hand_id: 1, joint_values: [8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192]}"
```

#### 电流话题与服务

`currentSet` 是 24 路电流保护上限，`currentAct` 是实际电流。写电流保护**不会让手运动**，只改保护值。

**读电流有两种方式**（效果相同，下标 = 电缸 ID − 1）：

```bash
# 方式 1：话题（节点起来就能用，推荐）
ros2 topic echo /hand_left/current_actual

# 方式 2：服务（需用新版配置重启节点后才有）
ros2 service call /hand_left/get_currentAct rh524j1_interfaces/srv/Getcurrentact \
  "{query: '', hand_id: 1}"
```

写电流保护上限：

```bash
ros2 topic pub --once /hand_left/current_set rh524j1_interfaces/msg/SetCurrent1 \
  "{hand_id: 1, joint_values: [800,800,800,800,800,800,800,800,800,800,800,800,800,800,800,800,800,800,800,800,800,800,800,800]}"
```

#### 温度、故障码、状态（只读服务）

这三项都是 **24 路**，每个电缸一个数，顺序同样是下标 = 电缸 ID − 1。`0` 通常表示正常/无故障。

```bash
# 温度（℃，只读）
ros2 service call /hand_left/get_temp rh524j1_interfaces/srv/Gettemp \
  "{query: '', hand_id: 1}"

# 故障码（只读；非 0 表示该电缸有故障）
ros2 service call /hand_left/get_errorCode rh524j1_interfaces/srv/Geterror \
  "{query: '', hand_id: 1}"

# 运行状态（只读）
ros2 service call /hand_left/get_status rh524j1_interfaces/srv/Getstatus \
  "{query: '', hand_id: 1}"
```

有故障时先清错再继续运动：

```bash
ros2 service call /hand_left/set_clearError rh524j1_interfaces/srv/Setclearerror \
  "{hand_id: 1, clear_code: 1}"
```

#### 系统管理服务

| 服务 | 含义 | 命令示例 |
|------|------|---------|
| `set_id` | 改设备通信 ID（改完后 yaml 的 `Hand_ID` 也要改） | `ros2 service call /hand_left/set_id rh524j1_interfaces/srv/Setid "{hand_id: 1, device_id: 1}"` |
| `set_baudRate` | 改设备波特率索引（改完后 yaml 的 `baudrate` 也要改，常见串口 115200） | `ros2 service call /hand_left/set_baudRate rh524j1_interfaces/srv/Setbaudrate "{hand_id: 1, baudrate: 0}"` |
| `set_clearError` | 写 1 清除故障 | `ros2 service call /hand_left/set_clearError rh524j1_interfaces/srv/Setclearerror "{hand_id: 1, clear_code: 1}"` |
| `set_resetPara` | 写 1 恢复出厂参数 | `ros2 service call /hand_left/set_resetPara rh524j1_interfaces/srv/Setresetpara "{hand_id: 1, confirm: 1}"` |
| `set_defaultSpeed` | 上电默认速度（24 路，标量 0~16384） | `ros2 service call /hand_left/set_defaultSpeed rh524j1_interfaces/srv/Setdefaultspeed "{hand_id: 1, joint_values: [8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192]}"` |
| `set_defaultForceSet` | 上电默认力（24 路） | `ros2 service call /hand_left/set_defaultForceSet rh524j1_interfaces/srv/Setdefaultforceset "{hand_id: 1, joint_values: [500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500]}"` |
| `set_mode` | 24 路电缸运行模式 | `ros2 service call /hand_left/set_mode rh524j1_interfaces/srv/Setmode "{command: '', hand_id: 1, joint_values: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}"` |
| `set_pause` | 写 1 暂停，写 0 继续 | `ros2 service call /hand_left/set_pause rh524j1_interfaces/srv/Setpause "{hand_id: 1, pause_flag: 1}"` |
| `set_stop` | 写 1 急停 | `ros2 service call /hand_left/set_stop rh524j1_interfaces/srv/Setstop "{hand_id: 1, stop_flag: 1}"` |
| `set_gestureForceClb` | 力校准 | `ros2 service call /hand_left/set_gestureForceClb rh524j1_interfaces/srv/Setgestureforceclb "{hand_id: 1, calibration_values: [1]}"` |
| `get_errorCode` | 读 24 路故障码 | `ros2 service call /hand_left/get_errorCode rh524j1_interfaces/srv/Geterror "{query: '', hand_id: 1}"` |
| `get_status` | 读 24 路状态 | `ros2 service call /hand_left/get_status rh524j1_interfaces/srv/Getstatus "{query: '', hand_id: 1}"` |
| `get_temp` | 读 24 路温度 | `ros2 service call /hand_left/get_temp rh524j1_interfaces/srv/Gettemp "{query: '', hand_id: 1}"` |
| `get_currentAct` | 读 24 路实际电流 | `ros2 service call /hand_left/get_currentAct rh524j1_interfaces/srv/Getcurrentact "{query: '', hand_id: 1}"` |
| `get_forceAct` | 读 24 路实际力 | `ros2 service call /hand_left/get_forceAct rh524j1_interfaces/srv/Getforceact "{query: '', hand_id: 1}"` |
| `get_angleAct` | 读 24 路实际角度 | `ros2 service call /hand_left/get_angleAct rh524j1_interfaces/srv/Getangleact "{query: '', hand_id: 1}"` |

> **协议有、当前 ROS 未暴露**：保存 Flash（`save`）、电缸位置 `posSet`/`posAct`（不建议用来设角度）、`speedAct`、手势号 `Setgestureno` 等。无触觉、无动作序列/动作库、无百分比接口。

#### RH524J1 话题/服务自检

改配置或重新编译后，**先重启节点**，再另开终端：

```bash
source install/setup.bash

# 确认节点和服务已起来
ros2 node list
ros2 topic list
ros2 service list | grep hand_left

# 只读：温度 / 故障码 / 状态 / 电流 / 力 / 角度
ros2 service call /hand_left/get_temp rh524j1_interfaces/srv/Gettemp "{query: '', hand_id: 1}"
ros2 service call /hand_left/get_errorCode rh524j1_interfaces/srv/Geterror "{query: '', hand_id: 1}"
ros2 service call /hand_left/get_status rh524j1_interfaces/srv/Getstatus "{query: '', hand_id: 1}"
ros2 service call /hand_left/get_currentAct rh524j1_interfaces/srv/Getcurrentact "{query: '', hand_id: 1}"
ros2 service call /hand_left/get_forceAct rh524j1_interfaces/srv/Getforceact "{query: '', hand_id: 1}"
ros2 service call /hand_left/get_angleAct rh524j1_interfaces/srv/Getangleact "{query: '', hand_id: 1}"

# 话题：先看实际值，再发一条速度 + 角度
ros2 topic echo /hand_left/angle_actual
ros2 topic echo /hand_left/current_actual
ros2 topic echo /hand_left/force_actual

ros2 topic pub --once /hand_left/speed_set rh524j1_interfaces/msg/SetSpeed1 \
  "{hand_id: 1, joint_values: [8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192,8192]}"

ros2 topic pub --once /hand_left/angle_set rh524j1_interfaces/msg/SetAngle1 \
  "{hand_id: 1, joint_values: [-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,400,-1,-1,-1,-1,-1,-1]}"
```

判定：

- **服务正常**：有 `response:`，且 `message: ok`。
- **通信异常**：有 `response:` 但 `message` 不是 `ok`（ROS 通了，串口/设备没应答好）。
- **服务未就绪**：一直 `waiting for service` → 节点未启动，或未用 RH524J1 配置重启。执行 `ros2 service list | grep get_currentAct`；若没有，请 `Ctrl+C` 停节点后运行 `ros2 launch inspire_control_ros2 inspire_control_rh524j1.launch.py`。
- **话题正常**：发 `angle_set` 后，`angle_actual` 对应电缸会变化。
- **话题没反应**：检查 `hand_id` 是否等于 yaml 的 `Hand_ID`；其它关节不要填 `0`（会拉回张开），应填 `-1`。

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

**本仓库 ROS2 工作区包（源码编译，非 apt）**：`rh5dg2_interfaces`、`rh56f1_interfaces`、`rh56h1_interfaces`、`rh56dfx_interfaces`、`eg5cd1_interfaces`、`eg2_4c2_interfaces`、`inspire_control_ros2`，详见上文「ROS2 接口说明」与 `docs/依赖清单.md`。

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
cd src/inspire_serial_core
cmake -S . -B build && cmake --build build -j
```

#### 纯 C++ 控制灵巧手（无 ROS）

同一程序 `serial_hand_control_node` 通过 YAML 切换机型，**运行时**指定角度，无需改源码重编译：

| 参数 | 说明 |
|------|------|
| `--config` / `-c` | 设备协议 YAML（如 `config/device_protocol_rh56dfx_example.yaml`） |
| `--angles v1,v2,...` | 固定角度，逗号分隔（RH56DFX/RH56F1/RH56H1=6 个，RH5DG2=13 个） |
| `--angles-file <path>` | 从文件读取一行角度（逗号或空格分隔） |
| `--demo` | 自动开合演示（默认，未指定 `--angles` 时） |
| `--read-only` | 只读 `angleAct`，不写 `angleSet` |

```bash
cd src/inspire_serial_core

# 自动演示（默认）
./build/serial_hand_control_node --config config/device_protocol_rh56dfx_example.yaml

# 握拳：六个关节固定角度（RH56DFX）
./build/serial_hand_control_node -c config/device_protocol_rh56dfx_example.yaml \
  --angles 1000,1000,1000,1000,1200,1800

# 从文件读角度（方便脚本反复调用）
echo "1800,1800,1800,1800,1350,1800" > /tmp/hand_pose.txt
./build/serial_hand_control_node -c config/device_protocol_rh56dfx_example.yaml \
  --angles-file /tmp/hand_pose.txt

# 只读当前角度
./build/serial_hand_control_node -c config/device_protocol_rh56dfx_example.yaml --read-only
```

快捷脚本（机型别名 + 透传额外参数）：

```bash
./scripts/run_cpp_hand.sh rh56dfx --angles 1000,1000,1000,1000,1200,1800
./scripts/run_cpp_hand.sh rh56dfx --read-only
```

> 需要 ROS 话题/服务控制时，请使用 `inspire_control_ros2` 节点；纯 C++ 版适合无 ROS 环境或快速真机验证。

#### 运行单元测试

核心库自带 gtest 单元测试，覆盖：`RingBuffer` 环形缓冲、`DeviceWorker` 串口事务串行化（FIFO 执行、异常传播、并发提交无重叠、关停语义），以及 RH56F1 / RH5DG2 / RH524J1 / EG5CD1 等 485 协议 + RH56DFX_serial_can / EG2_4C2_serial_can 两个 Serial-CAN 协议的命令构建、响应解析、校验和、A5 转义、ExtId 编码等**纯逻辑**。全部用例不依赖真实串口硬件，测试源码位于 `src/inspire_serial_core/tests/`。

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

#### 持续集成（CI）

每次向 `master`/`main` 分支 **push** 或发起 **Pull Request** 时，GitHub Actions 会自动执行（见 [`.github/workflows/ci.yml`](.github/workflows/ci.yml)）：

| 步骤 | 内容 |
|------|------|
| `colcon build` | 编译整个工作区（7 个包，含 RH56H1 独立接口包） |
| `colcon test` | 运行 `inspire_serial_core` 的 41 个 gtest 用例 |
| `clang-format` | 校验 C++ 代码格式（规则见根目录 `.clang-format`） |
| `clang-tidy` | 对核心库与驱动包做静态分析（规则见 `.clang-tidy`） |

本地复现 CI 检查（需先 `colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`）：

```bash
./scripts/check_clang_format.sh
./scripts/run_clang_tidy.sh
```

状态徽章见 README 顶部；详细运行日志在 GitHub 仓库的 **Actions** 页。

### 4. 配置设备

编辑 **`src/driver/config/device_protocol_config.yaml`**（或与 launch 一致的 `--device-config` 路径）。`protocol.type` 决定机型（仓库内默认值为 `RH56DFX_serial_can`，下例以 `RH56F1_485` 演示，按需替换为 `RH5DG2_485` / `RH56DFX_serial_can` / `EG5CD1_485` 等）：

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

以下示例假定 **`protocol.type`** 为 RH5DG2 系列（**13** 个关节）。若为 **RH56F1** 请用 **`rh56f1_interfaces`**、**RH56H1** 请用 **`rh56h1_interfaces`**，且 **`joint_values` 长度为 6**。也可用 `ros2 interface show <包名>/<类型>` 查看字段。

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

#### RH5DG2 角度循环测试脚本

`test/` 目录提供两个 Python 脚本，用于在真机上做 min→max→min 三角波角度循环联调。**默认测试全部 13 个关节**（`--joints all --mode sync`）：

```bash
source install/setup.bash

# 话题模式：13 关节同步循环
python3 test/test_rh5dg2_angle_topic.py --device hand_left --hand-id 1

# 服务模式：13 关节同步循环
python3 test/test_rh5dg2_angle_service.py --device hand_left --hand-id 1

# 逐关节依次测试（每次只动一个关节，其余保持固定姿态）
python3 test/test_rh5dg2_angle_topic.py --mode sequential
python3 test/test_rh5dg2_angle_service.py --mode sequential

# 只测部分关节（例如前四指）
python3 test/test_rh5dg2_angle_topic.py --joints 0,1,2,3
```

常用参数：`--mode sync|sequential`、`--joints all|0,1,...`、`--step` 步进、`--min`/`--max` 循环范围（默认 965~1800）、`--rate` 发布频率、`--fixed-angles` sequential 模式下非活动关节固定姿态。

脚本可拷贝到任意目录运行，任选一种方式加载环境即可：

```bash
# 方式 1：source 工作区（推荐）
source /opt/ros/jazzy/setup.bash
source /path/to/Inspire_Hand_SDK-master/install/setup.bash
python3 /任意路径/test_rh5dg2_angle_topic.py

# 方式 2：环境变量指定 install 目录
export INSPIRE_HAND_SDK_INSTALL=/path/to/Inspire_Hand_SDK-master/install
python3 /任意路径/test_rh5dg2_angle_topic.py
```

启动节点示例：

```bash
ros2 launch inspire_control_ros2 inspire_control_single_device.launch.py device_name:=hand_left
```

#### RH56DFX 服务/Topic 收发自检

当 `protocol.type: RH56DFX_serial_can` 时，可用以下步骤快速确认「服务是否可调用」「Topic 是否正常收发」：

```bash
# 0) 确保环境干净（避免多节点重名导致结果混乱）
pkill -f inspire_control_node || true

# 1) 启动 RH56DFX 单设备节点
ros2 launch inspire_control_ros2 inspire_control_single_device.launch.py \
  device_name:=hand_left
```

另开一个终端执行：

```bash
source install/setup.bash

# 2) 节点与服务类型检查（应只有 1 个 /hand_left/hand_left_node）
ros2 node list
ros2 service type /hand_left/get_status

# 3) 服务读测试（RH56DFX 接口包）
ros2 service call /hand_left/get_status rh56dfx_interfaces/srv/Getstatus \
  "{query: '', hand_id: 1}"
ros2 service call /hand_left/get_errorCode rh56dfx_interfaces/srv/Geterror \
  "{query: '', hand_id: 1}"
ros2 service call /hand_left/get_temp rh56dfx_interfaces/srv/Gettemp \
  "{query: '', hand_id: 1}"

# 4) Topic 写+读联调（6 个关节）
ros2 topic echo /hand_left/angle_actual
ros2 topic pub --once /hand_left/angle_set rh56dfx_interfaces/msg/SetAngle1 \
  "{hand_id: 1, joint_values: [100,100,100,100,100,100]}"
```

判定建议：

- **服务链路正常**：命令出现 `response:`，且 `message='ok'`。
- **设备通信异常**：有 `response:` 但 `message='device_error'`（说明 ROS2 服务通，但设备侧收发失败）。
- **服务未就绪**：长时间 `waiting for service to become available...`（通常是节点未启动/命名空间不匹配）。
- **Topic 正常**：`/hand_left/angle_set` 下发后，`/hand_left/angle_actual` 在后续周期出现可观测变化。
- **Topic 被忽略**：`hand_id` 与配置 `Hand_ID` 不一致时，订阅回调会忽略该命令。

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

### 依赖清单

📖 **[docs/依赖清单.md](docs/依赖清单.md)**

包含：
- 完整的依赖项列表
- 版本要求
- 安装命令
- 验证方法
- 常见问题

### 协议格式说明

📖 **[docs/RH56F1_485协议格式说明.md](docs/RH56F1_485协议格式说明.md)**（另见 `docs/RH5DG2_485协议格式说明.md`、`docs/RH56DFX_Serial_CAN协议解析.md`、`docs/夹爪485寄存器规则.md`、`docs/EG5CD1协议格式说明.md`、`docs/EG5CD1_ROS2_API.md`、`docs/4C2夹爪CAN转Serial通信规则.md`）

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

ROS2 设备控制节点，通过 **`InterfaceAdapter`** 使用 **`rh5dg2_interfaces` / `rh56f1_interfaces` / `rh56h1_interfaces`** 等中的消息与服务类型。

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

> **硬件验证**：已在 RH5DG2 真机环境（115200，Hand_ID 1）验证 50Hz 定时读、状态话题发布、只读服务调用与 `set_angle` 写入；读写并发场景下通信稳定，偶发单次读失败可在下一周期自恢复。

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
      # ... 共 13 项（RH5DG2）或 6 项（RH56F1 / RH56H1）

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

**文档版本**：v1.3  
**最后更新**：2026-07-07（RH56H1 位置百分比：100%=张开/0%=握紧）

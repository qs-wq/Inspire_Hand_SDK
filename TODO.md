# 待办与重构清单（TODO）

> 本文件记录项目的待办事项与重构计划，按优先级分级（P0 最高）。
> 完成一项请把 `[ ]` 改为 `[x]` 并在「已完成」区追加一行说明。
> 详细的问题背景与设计分析见 `docs/项目架构说明.md`。

最近更新：2026-06-05

---

## 已完成 ✅

### P0 — 卫生 / 立即收益
- [x] 提交 `.gitignore`，并对历史产物执行 `git rm --cached`
- [x] 移出 30MB 的 `Inspire_Hand_Serial_Driver.zip`（不进源码库）
- [x] 统一配置文件命名（去除中文括号），明确唯一权威配置目录

### P1 — 结构 / 解耦
- [x] 裸库抽成独立 CMake target `inspire_serial_core`，ROS 包改用 `find_package` 依赖，去掉 `../../../` 相对路径
- [x] 引入统一错误类型 `IoError`，贯穿 `Protocol → IRegisterIoBackend → 适配器 → ROS Service 响应`
- [x] 消除双层 `src` 嵌套，重构为 colcon 标准布局（`src/` 下平级 `inspire_serial_core` / `driver` / `interfaces`）
- [x] 全部 58 个 `.srv` 响应段新增 `string message` 字段，错误信息透传给 ROS Service 调用方
- [x] 文档集中到 `docs/`，更新 README 与各说明文档

---

## 待办 TODO

### P2 — 并发 / 性能（重要）

- [ ] **串口并发模型重构（最高优先）**
  - 现状：之前的 `std::mutex` 补丁已回退，`register_controller` 目前无锁。多线程执行器下「定时读状态」与「Topic/Service 写寄存器」会并发访问同一串口和 `RingBuffer`，存在帧错乱风险。
  - 方向：单 `io_context` + 每设备 `strand` 串行化；回调内不再 `sleep` 持锁；组合序列改为异步状态机 / 请求队列。
  - 涉及：`inspire_serial_core/src/serial_port.cpp`、`driver/src/register_controller.cpp`、EG5CD1 组合服务回调。

- [ ] **EG5CD1 组合服务暂停时长自适应**
  - 现状：`kPauseForceCompositeMs=220` / `kPauseTouchCompositeMs=280` 为经验估算值，慢波特率下可能不足。
  - 方向：按波特率 / 实测 RTT 自适应计算暂停时长。

### P3 — 质量 / 功能

- [ ] **补单元测试**
  - 覆盖：`RingBuffer` 找帧头、各协议 `buildXxxCommand` / `parseResponse` / 校验和等纯逻辑。
  - 工具：gtest / ament_cmake_gtest。

- [ ] **触觉 version 2（压阻式）适配**
  - 现状：配置注释明确「2 暂未适配」，为功能缺口。
  - 涉及：协议层 `readTouchData` 的 version 2 解析分支。

- [ ] **补 EG-5CD1 的 CANFD 通道**
  - 现状：夹爪目前仅有 485；核里仅有 `RH56F1_canfd` / `RH5DG2_canfd`，缺 `EG5CD1_canfd`。

- [ ] **接入 CI**
  - 内容：`colcon build` + 运行测试 + `clang-format` / `clang-tidy`。

---

## 优先级建议

1. 先搭 **P3 单元测试**（给后续重构兜底，性价比高）。
2. 再做 **P2 串口并发重构**（直接关系多设备 / 高频下的稳定性，收益最高）。
3. 其余 P3 功能项（触觉 v2、EG-5CD1 CANFD、CI）按需推进。

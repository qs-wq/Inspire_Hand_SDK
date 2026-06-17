# 待办与重构清单（TODO）

> 本文件记录项目的待办事项与重构计划，按优先级分级（P0 最高）。
> 完成一项请把 `[ ]` 改为 `[x]` 并在「已完成」区追加一行说明。
> 详细的问题背景与设计分析见 `docs/项目架构说明.md`。

最近更新：2026-06-17

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

### P2 — 并发 / 性能（重要）
- [x] **串口并发模型重构**：每设备引入 `DeviceWorker`（请求队列 + 单工作线程），所有读写/组合序列均经 worker **串行化**，从结构上保证「写命令 + 读应答 + 解析」事务永不交错；`RegisterController` 把定时器与服务/订阅拆到不同回调组（timer 互斥 / service 可重入），配合 `MultiThreadedExecutor` 实现「定时读 ‖ 服务写」真正并发；定时读做合并背压（`read_in_flight_`），事务起始清空串口 RX 残留。删除旧的 `ioPauseTimer`/`pauseTimer` 暂停机制。涉及 `inspire_serial_core/include/device_worker.hpp`、`driver/src/register_controller.cpp` 及三个适配器。
- [x] **EG5CD1 组合服务暂停时长自适应（已被上一项消解）**：组合写改为 `ioWriteSequence` 在 worker 上原子执行（步骤间隔 `kCompositeStepMs=3ms` 在 worker 线程内），不再需要 `kPauseForceCompositeMs/kPauseTouchCompositeMs` 经验暂停，相关常量与回调内 `sleep` 已删除。

### P3 — 质量 / 功能
- [x] 补单元测试（gtest）：覆盖 `RingBuffer`（push/pop/advance/越界/覆盖/回绕）、`DeviceWorker`（FIFO/串行不交叠/多线程并发提交/异常透传/停后提交/析构安全）与 RH56F1 / RH5DG2 / EG5CD1 三个 485 协议的 `buildReadCommand`/`buildWriteCommand`/`parseResponse`/`validateChecksum`。共 41 个用例，`colcon test` 全绿。代码在 `src/inspire_serial_core/tests/`。

---

## 待办 TODO

### P3 — 质量 / 功能

- [ ] **触觉 version 2（压阻式）适配**
  - 现状：配置注释明确「2 暂未适配」，为功能缺口。
  - 涉及：协议层 `readTouchData` 的 version 2 解析分支。

- [ ] **补 EG-5CD1 的 CANFD 通道**
  - 现状：夹爪目前仅有 485；核里仅有 `RH56F1_canfd` / `RH5DG2_canfd`，缺 `EG5CD1_canfd`。

- [ ] **接入 CI**
  - 内容：`colcon build` + 运行测试 + `clang-format` / `clang-tidy`。

---

## 优先级建议

1. ~~先搭 **P3 单元测试**~~ ✅ 已完成（纯逻辑已有回归兜底）。
2. ~~**P2 串口并发重构**~~ ✅ 已完成（worker 串行化 + 双回调组并发）。
3. 其余 P3 功能项（触觉 v2、EG-5CD1 CANFD、CI）按需推进；CI 现已具备 `colcon test` 基础，接入相对容易。

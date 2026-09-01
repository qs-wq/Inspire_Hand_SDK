## 因时机器人 EG2-4C2 电动夹爪 - CAN 经 USB 转串口通信指南

---

### 一、概述

本指南适用于 **CAN 转 USB 串口模块** 控制 EG2-4C2 夹爪：

- **夹爪侧**：CAN 500K，29 位扩展帧（ExtId 算法见下文）
- **电脑侧**：`/dev/ttyUSB0` 等串口，115200 8N1
- **寄存器地址**：ExtId 中填 **Modbus 地址**（与 `4C2夹爪CAN寄存器规则.md` 第十节对照表左列一致），**不是**原生 CAN 地址
- **组帧格式**：参考 `4c2/demo_can.py`（`AA AA` 帧头 + ExtId + 数据区 + 校验 + `55 55` 帧尾）

---

### 二、串口配置

| 参数 | 值 |
|------|-----|
| 串口设备 | `/dev/ttyUSB0` |
| 波特率 | 115200 |
| CAN 波特率 | 500 Kbps（模块侧配置） |
| 设备 ID | 1～16383，默认 1 |

---

### 三、串口帧格式

#### 3.1 帧结构

```
AA AA | ExtId(变长,小端) | Data[8] | Meta[4] | Checksum | 55 55
```

| 字段 | 说明 |
|------|------|
| 帧头 | `0xAA 0xAA` |
| ExtId | 扩展标识符字节，**低字节在前** |
| Data[8] | 固定 8 字节数据区 |
| Meta[4] | 读/写附加字段（见下） |
| Checksum | `sum(ExtId..Meta) & 0xFF` |
| 帧尾 | `0x55 0x55` |

#### 3.2 ExtId 编码（手册第 4 章，非 demo_can 字符串拼接）

手册定义 29 位扩展标识符（从低位到高位）：

| 位域 | 含义 |
|------|------|
| bit31～28 | 预留 |
| bit27～26 | 操作类型：00=读，01=写 |
| bit25～14 | 寄存器起始地址（12 位） |
| bit13～0 | 设备 ID（14 位） |

```python
ExtId = (op_type << 26) | (reg_addr << 14) | (dev_id & 0x3FFF)
```

> **USB-CAN 串口模块地址说明**：经 `demo_can.py` 同类 USB-CAN 模块控制时，ExtId 中的 `reg_addr` 应填 **Modbus 寄存器地址**（如 1010、1060），**不是** 原生 CAN 总线手册里的 +10/+60 偏移址（1020、1120）。原生 socketcan 直连仍用 `4C2夹爪CAN寄存器规则.md` 中的 CAN 地址。

手册示例（**原生 CAN 总线**地址）：

| 操作 | CAN 地址 | ID | ExtId |
|------|----------|-----|-------|
| 读 | 1120 | 1 | `0x01180001` |
| 读 | 1128 | 1 | `0x011A0001` |
| 写 | 1020 | 1 | `0x04FF0001` |

**USB-CAN 串口模块**对应示例（Modbus 地址）：

| 操作 | 地址 | ID | ExtId |
|------|------|-----|-------|
| 读状态 | 1060 | 1 | `0x01090001` |
| 读故障/状态 | 1064 | 1 | `0x010A0001` |
| 写开口度 | 1010 | 1 | `0x04FC8001` |

串口帧中 ExtId 以 **4 字节小端** 嵌入（如 `0x01090001` → `01 00 09 01`）。

> **注意**：`demo_can.py` 的 `0000000/0000010 + bin(address)` 算法与手册不同，写操作 ExtId 会算错；应使用上式位域编码。

#### 3.3 读帧 Data + Meta

```
Data:  [read_len, 00, 00, 00, 00, 00, 00, 00]
Meta:  [01, 00, 01, 00]
```

`read_len` = 欲读取字节数（1～8）。

#### 3.4 写帧 Data + Meta

```
Data:  int16 低字节在前，不足 8 字节用 0xFF 填充
Meta:  [有效字节数, 00, 01, 00]
```

单次最多写 4 个寄存器（8 字节）。

#### 3.5 应答解析

- 应答帧结构同发送帧
- 有效数据在 ExtId 之后 8 字节处
- 需处理 `0xA5` 转义：若 `A5` 后跟 `55/AA/A5`，跳过 `A5` 保留下一字节
- int16 为低字节在前；值 >60000 视为 0

---

### 四、组帧示例

#### 读寄存器（地址 1060，ID=1，读 8 字节）

```
AA AA 01 00 09 01 08 00 00 00 00 00 00 00 01 00 01 00 [CHK] 55 55
```

#### 写寄存器（地址 1010，开口800/速度800/力500）

```
AA AA 01 80 FC 04 20 03 20 03 F4 01 FF FF 06 00 01 00 [CHK] 55 55
```

---

### 五、常用寄存器（USB-CAN 串口 ExtId 地址）

| 地址 | 名称 | 说明 |
|------|------|------|
| 1010 | OPEN_LEN_SET | 开口度设置（0~1000） |
| 1011 | SPEED_SET | 速度 |
| 1012 | FORCE_SET | 夹持力 |
| 1005 | CATCH_MODE | 夹取模式（0/1） |
| 1006 | STOP | 急停 |
| 1007 | CLEAR_ERROR | 清除故障 |
| 1060 | FORCE_ACT | 实际受力 |
| 1061 | OPENLEN_ACT | 实际开口度 |
| 1064 | ERROR_CODE | 故障码 |
| 1065 | STATUS | 状态码 |

> 原生 CAN（socketcan）地址见 `4C2夹爪CAN寄存器规则.md` 第五节及第十节对照表右列；**不能**简单用 Modbus+10/+60 换算。

---

### 六、Python 使用

```python
from gripper_demo_can_serial import CanSerialBus, configure_gripper, move_gripper, read_gripper_status

bus = CanSerialBus.open("/dev/ttyUSB0", baudrate=115200)
configure_gripper(bus, dev_id=1, speed=800, force=500)
move_gripper(bus, dev_id=1, position=500, speed=800, force=500)
print(read_gripper_status(bus, dev_id=1))
bus.close()
```

---

**文档版本**：基于 EG2-4XX 手册 V1.06 + `4c2/demo_can.py` 组帧  
**对应代码**：`gripper_demo_can_serial.py`

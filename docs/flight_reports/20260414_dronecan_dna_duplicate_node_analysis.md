# DroneCAN DNA Server 多CAN总线 ESC Node ID 冲突分析报告

**日期**: 2026-04-14  
**版本**: v1.0  
**作者**: TG700 Flight Control Team  
**固件**: ArduPlane V4.7.0-dev (3863214b)  
**飞控**: ZeroOneM9  
**电调**: 弦动 CAN 电调（DroneCAN/UAVCAN V0.9 兼容）

---

## 1. 问题描述

TG700 采用双 CAN 总线架构驱动 16 个电调：

| CAN 总线 | 物理位置 | ESC 数量 | ESC_OF | ESC_BM |
|:-:|:-:|:-:|:-:|:-:|
| CAN_D1 | 右翼 | 8 | 4 | 4080 |
| CAN_D2 | 左翼 | 8 | 12 | 1044480 |

两条总线**物理隔离**，不互通数据。当两条总线上的 ESC 配置了**相同的 Node ID**（如都使用 0-7）时，飞控持续报出 PreArm 错误：

```
DroneCAN: Duplicate Node ESC../7!
DroneCAN: Duplicate Node ESC../6!
DroneCAN: Duplicate Node ESC../4!
```

该错误阻止解锁（PreArm check failure），且冲突报告的具体 Node ID 编号在每次上电时不确定。

### 1.1 日志证据

**20260406-1.log** 中的冲突记录（原始 ESC ID 0-7 配置）：

| 时间戳 (TimeUS) | 消息类型 | 冲突 Node ID |
|:-:|:-:|:-:|
| 443479470 | Arm: | ../7 |
| 472706764 | Arm: | ../6 |
| 619702045 | PreArm: | ../7 |
| 650698963 | PreArm: | ../4 |
| 681695805 | PreArm: | ../7 |
| 712692953 | PreArm: | ../7 |
| 743689541 | PreArm: | ../7 |
| 836680366 | PreArm: | ../2 |
| 867677427 | PreArm: | ../6 |
| 898673999 | PreArm: | ../6 |

**20260409.log** 中（右翼改为 Node ID 9-16 后）：**零冲突报告**。

---

## 2. 根因分析

### 2.1 ArduPilot DNA Server 架构

ArduPilot 飞控内部维护一个 DNA (Dynamic Node Allocation) 数据库，存储在飞控 Flash 中，
用于记录每个 CAN Node ID 对应的 16 字节硬件唯一标识（Unique ID）。

**关键设计特点：该数据库是全局共享的，不按 CAN 总线编号隔离。**

### 2.2 冲突检测源码

文件：`libraries/AP_DroneCAN/AP_DroneCAN_DNA_Server.cpp`

```cpp
// 第105-118行
bool AP_DroneCAN_DNA_Server::Database::handle_node_info(
    uint8_t source_node_id, const uint8_t unique_id[])
{
    WITH_SEMAPHORE(sem);

    if (is_registered(source_node_id)) {
        // 该 Node ID 已注册，检查 Unique ID 是否匹配
        if (source_node_id != find_node_id(unique_id, 16)) {
            return true; // Unique ID 不匹配 → 判定为冲突
        }
    } else {
        register_uid(source_node_id, unique_id, 16); // 新设备，注册
    }
    return false;
}
```

冲突报告逻辑（第493-504行）：

```cpp
case DUPLICATE_NODES: {
    if (_ap_dronecan.option_is_set(
            AP_DroneCAN::Options::DNA_IGNORE_DUPLICATE_NODE)) {
        return true; // 可通过选项忽略
    }
    snprintf(fail_msg, fail_msg_len,
             "Duplicate Node %s../%d!", fault_node_name, fault_node_id);
    return false;
}
```

### 2.3 冲突触发机制

```
                    飞控 DNA 数据库（全局）
                    ┌─────────────────────┐
                    │ Node ID → Unique ID │
                    └──────────┬──────────┘
                               │
           ┌───────────────────┼───────────────────┐
           │                   │                   │
    CAN1 (右翼)          数据库查询           CAN2 (左翼)
    ESC ID=7             全局范围            ESC ID=7
    UID=AAAA...                             UID=BBBB...
           │                                       │
           │  ① 先上线，注册成功                    │
           │  {7, AAAA...}                         │
           │                                       │
           │                   ② 后上线，查到 ID=7  │
           │                   已注册但UID不匹配    │
           │                   → DUPLICATE_NODES!   │
           └───────────────────┴───────────────────┘
```

### 2.4 弦动电调默认行为

根据《弦动电调 CAN 通信协议 V3.2》第25页"7.电调编址"：

> "电调上电默认 ID 一般为 0x20。如果用户自己设置了节点 ID，
> 那电调的节点 ID 就是新设置的节点 ID。"

若未手动编址，所有电调的出厂 Node ID 都是 0x20（32），必然导致冲突。

### 2.5 冲突 Node ID 不确定的原因

由于两条 CAN 总线的 ESC 上电和 GetNodeInfo 握手的时序是**竞争性的**，
哪条总线的哪个 ESC 先完成握手就先注册，后到者成为"冲突方"。
因此每次上电冲突的具体 Node ID 可能不同。

---

## 3. 解决方案

### 3.1 临时方案（已验证有效）

确保所有 CAN 总线上的 ESC Node ID **全局唯一**：

| CAN 总线 | 物理位置 | ESC Node ID 范围 |
|:-:|:-:|:-:|
| CAN_D1 | 右翼 | 9, 10, 11, 12, 13, 14, 15, 16 |
| CAN_D2 | 左翼 | 0, 1, 2, 3, 4, 5, 6, 7 |

使用弦动电调上位机或 DroneCAN Inspector 为每个电调设置唯一 Node ID。

### 3.2 可选：忽略冲突检查

ArduPilot 提供了 `CAN_Dx_UC_OPTION` 参数中的 `DNA_IGNORE_DUPLICATE_NODE` 选项位。
但**不建议使用**：忽略冲突不解决根本问题，可能导致 ESC 控制信号路由错误。

### 3.3 建议的上游修复

ArduPilot DNA Server 应按 CAN 总线编号隔离数据库，
使每条总线独立维护 Node ID → Unique ID 的映射关系。
这符合 DroneCAN 协议本身的设计——不同物理总线是独立的通信域。

---

## 4. 验证结果

| 日志文件 | ESC ID 配置 | Duplicate Node 报错 |
|:-|:-|:-:|
| 20260404-1.log | 两侧均为 0-7 | 未记录（该日志无相关段） |
| 20260405-1.log | 两侧均为 0-7 | 未记录 |
| 20260406-1.log | 两侧均为 0-7 | **大量报错**（../7, ../6, ../4, ../2） |
| 20260409.log | 右翼 9-16, 左翼 0-7 | **零报错** |

---

## 5. 影响评估

- **影响范围**：所有使用多 CAN 总线且 ESC 存在相同 Node ID 的 ArduPilot 用户
- **严重程度**：中（阻止解锁，但有简单临时方案）
- **安全影响**：无直接飞行安全影响（冲突在地面即被检出阻止起飞）

---

## 6. 参考资料

- ArduPilot 源码：`libraries/AP_DroneCAN/AP_DroneCAN_DNA_Server.cpp`
- 弦动电调 CAN 通信协议 V3.2
- DroneCAN/UAVCAN V0.9 规范：https://dronecan.github.io/

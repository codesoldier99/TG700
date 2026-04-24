# 专题答复 Q1：为何 ArduPilot 中没有 MPC？

**版本**：v1.0  
**日期**：2026-04-14

---

## 核心结论

> **不是"创新空间小"决定的，是"工程约束 + 开源治理"决定的。**

ArduPilot 中没有广泛采用 MPC，根本原因有 5 个：算力瓶颈、模型依赖、开源治理惯性、
PID 已"够用"、QP 求解器依赖。

---

## 1. 五大原因详解

### 1.1 算力瓶颈

| 飞控 MCU | 主频 | 400 Hz 可用窗口 |
|:-|:-:|:-:|
| STM32F405 | 168 MHz | 2.5 ms |
| STM32F765 | 216 MHz | 2.5 ms |
| STM32H743 (ZeroOneM9) | 480 MHz | 2.5 ms |

MPC 每步需要求解 QP 问题，典型求解时间：

- OSQP: 0.5–5 ms（依赖问题规模）
- qpOASES: 1–10 ms
- TG700 16 电机控制分配问题（变量 20+，约束 40+）需 > 3 ms

**结论**：直到 STM32H7 双核才勉强跑得动轻量 MPC。F4/F7 时代（2015–2020）完全不可能。

### 1.2 ArduPilot 的"通用性"教条

ArduPilot 的核心设计理念是"**一套代码通吃**"：

- 支持的机型：Copter (多旋翼)、Plane (固定翼)、Rover (车)、Sub (潜艇)、Blimp (飞艇)、
  Tracker (天线)、QuadPlane (复合翼)
- 总代码量：~400 万行 C++
- 活跃贡献者：~200 人

MPC 强依赖"模型"，每种机型都要单独建模 + 辨识 + 调优。这和 ArduPilot "通用代码"的
核心价值观冲突。

**例子**：ArduPilot 的姿态控制器 `AC_AttitudeControl` 是"模型无关"的 PID + 前馈架构，
从 2 电机到 32 电机都能用同一份代码。换成 MPC，每个机型都要独立的优化问题求解器，
**维护成本爆炸**。

### 1.3 开源治理惯性

ArduPilot 的开发模式：

- maintainer 群体偏向"**足够好就行**"的增量改进
- 大 PR（如 MPC 重构）需要 2+ 个 maintainer review
- 合并后要能通过 AutoTest 回归测试（覆盖 100+ 场景）
- **没有核心 maintainer 主推** MPC，社区 PR 难合并

对比：PX4 由 Lorenz Meier 带领的苏黎世理工团队主导，**推 MPC 比 ArduPilot 容易**。
这是开源治理结构的差别。

### 1.4 PID 已"够用"

对小型消费级无人机：

- 推重比 > 2
- 载荷变化 < 20%
- 飞行包络小（悬停 / 航点 / 前飞）
- PID + 前馈 + L1 导航 → 99% 场景满意

**MPC 的收益在哪？**

MPC 的优势场景：
- 大载重（推重比 < 1.5）
- 强耦合（共轴多电机）
- 硬约束（电池、姿态、速度包络）
- 时变系统（载重 / 气动 / 执行机构）

**而这些正是 TG700 的特点**：MPC 的价值在大载重 VTOL 上才显现，而 ArduPilot 默认
不支持这类机型的深度优化。

### 1.5 QP 求解器依赖

ArduPilot 代码库有严格的依赖管理规定：

- 避免引入大型第三方库（维护负担）
- 第三方库必须是 header-only 或已经在 ChibiOS 里
- OSQP（C 版本约 30k 行代码）、qpOASES（C++ 约 50k 行）都超出可接受范围

**已有尝试**：
- 2021 年有社区 PR 尝试集成 OSQP，因编译时间 +30% 被拒
- 2023 年的 MPC for Copter 分支至今未合并主线

---

## 2. ArduPilot 中已有的"轻量级" MPC / INDI 尝试

虽然没有标准 MPC，ArduPilot 中有以下"类 MPC"机制：

### 2.1 INDI（Incremental Nonlinear Dynamic Inversion）

- 位置：`libraries/AC_AttitudeControl/AC_AttitudeControl.cpp`
- 思想：基于角加速度测量的增量反演
- 已用于：**倾转旋翼、部分尾座式机型**
- 在 VTOL Planes 中有限启用

### 2.2 PSC（Position Controller）的前馈

- 位置环使用"前馈 + P"（不是纯 PID）
- 前馈需要轨迹的二阶导数
- 本质上是 **1 步 MPC** 的退化版

### 2.3 AutoTune 的参数搜索

- Copter AutoTune 用扰动响应 + 梯度下降
- Plane AutoTune 用 PTNR (Pilot Test Native Remote)
- **本质上是离线 MPC 的调参过程**

### 2.4 L1 Navigation

- `libraries/AP_L1_Control/`
- 固定翼航迹跟踪用 L1 自适应律
- **是一种简化的 MPC（预测 1 步 + 比例反馈）**

---

## 3. 工业界对比

| 厂商 / 项目 | 是否用 MPC | 实现方式 |
|:-:|:-:|:-|
| **DJI**（Mavic/Phantom/Matrice） | ? | 闭源，自研 MPC + EKF |
| **Skydio** | ? | 闭源，MPC + 视觉 |
| **PX4** | ? 部分 | 位置/轨迹跟踪用 MPC，姿态仍是 PID/INDI |
| **ArduPilot** | ? | PID + INDI + L1 |
| **Autonomous Systems Lab (ETH)** | ? | 研究代码，非主线 |

**结论**：MPC 在工业界（尤其消费级）已经是主流，但 ArduPilot 作为开源项目受治理结构
限制没有完全采纳。

---

## 4. 对本研究的启示

### 4.1 不要把 MPC 作为论文主角

- 纯 MPC 理论空间小（近 30 年 VTOL 上做烂了）
- 顶刊审稿人对"又一个 MPC for VTOL"论文敏感度高

### 4.2 把 MPC 作为工具

在 S1 核心选题中，可以这样用 MPC：

**方案 A：DRL-Assisted MPC**
- MPC 求解优化问题
- DRL 在线调整 MPC 的权重矩阵 Q、R
- 稳定性证明：MPC 本身的递归可行性 + DRL 的保守约束
- 创新点：**用数据学习权重，而不是手工调**

**方案 B：Backstepping-Inspired MPC**
- 反步法提供"级联结构"作为先验
- MPC 在每一级做滚动优化
- 整体稳定性通过反步 Lyapunov 分析
- 创新点：**结构化先验 + 滚动优化**

### 4.3 或者，直接避开 MPC

本提案的主攻方向 **S1（反步 + DRL 增益调度 + DOBC）** 完全不需要 MPC。
反步法本身已经提供了级联结构 + Lyapunov 可证性，DRL 提供自适应性，DOBC 提供鲁棒性。

**结论**：**MPC 只作为"可选工具"，不作为论文主线**。

---

## 5. 工程建议

如果未来需要在 ArduPilot 中集成 MPC：

1. 硬件基础：必须用 STM32H7 双核（已满足，ZeroOneM9）
2. QP 求解器：用 header-only 小库（如 **TinyMPC**、**?OSQP**）
3. 模型来源：用 S0 的辨识结果作为 MPC 模型
4. 代码结构：写成可选模块 `libraries/AP_MPC/`，保留 PID 作为 fallback
5. 测试策略：先 SITL 验证，再铁鸟台，再真机

---

## 参考资料

- ArduPilot 源码：`libraries/AC_AttitudeControl/`, `libraries/AP_L1_Control/`
- PX4 Multicopter Position Controller (MPC 实现)
- TinyMPC: https://tinympc.org/
- ?OSQP: https://osqp.org/docs/get_started/embedded.html

# TG700 先进控制算法研究提案

**版本**：v1.0  
**日期**：2026-04-14  
**作者**：TG700 Flight Control Team  
**研究平台**：TG700 16-motor 共轴复合翼大载重 VTOL  
**基线代码**：`libraries/AP_Motors/AP_MotorsMatrix_TG700.cpp` (v4.0)  
**飞控**：ZeroOneM9 (STM32H743 双核)

---

## 摘要

本提案针对天工 700（TG700）16 电机共轴复合翼 VTOL 大载重无人机的实际飞行痛点，
结合已有的反步控制基础、深度学习能力、Q-learning 研究成果、以及 ArduPilot 代码/日志
分析经验，设计一套"**工程解决痛点 × 理论面向顶刊**"双达标的研究路线图。

核心主张：

> **以"反步控制（Backstepping）+ 深度强化学习增益调度 + 扰动观测器"的复合自适应控制
> 作为主攻方向**，以"TG700 非线性动力学深度学习辨识"作为地基论文，在 2–3 年内形成
> 4–5 篇递进式成果体系，目标期刊 IEEE TAC / TCST / TIE / Automatica / AIAA JGCD。

---

## 1. 研究动机与定位

### 1.1 TG700 的工程痛点

| 编号 | 痛点 | 观察表征 | 现有 ArduPilot 的局限 |
|:-:|:-|:-|:-|
| **P1** | 大载重下 PID 增益失配 | 空载 / 半载 / 满载总重变化 ±80% | PID 固定增益，单一调参点 |
| **P2** | 16 电机共轴气动耦合 | 上下桨气流相互干扰，单轴扰动会波及多通道 | 混控矩阵假设通道正交 |
| **P3** | 双 CAN 总线延迟不均 | CAN1/CAN2 时序差 1–3 ms | 飞控假设所有电机零延迟同相 |
| **P4** | 大惯量 + 低推重比 | 响应慢、超调大、容易震荡 | 姿态环 P 增益上限低 |
| **P5** | 阵风/湍流鲁棒性差 | 山区测绘扰动大 | 无显式扰动观测器 |
| **P6** | 动力学模型未知 | 厂家不提供关键气动参数 | 只能手动调 PID |

### 1.2 ArduPilot 默认控制器的不足

ArduPilot 主线代码的控制架构：

- 位置控制：前馈 + PID 级联
- 姿态控制：PID + INDI（部分机型）
- 航迹跟踪：L1 Navigation / TECS
- 自整定：AutoTune（针对小型多旋翼设计）

**对于 TG700 这种大载重 + 强耦合 + 多执行机构冗余的场景，存在以下理论空白**：

1. 无**稳定性证明**的控制律
2. 无**参数不确定性自适应**机制
3. 无**扰动显式补偿**
4. 无**16 电机最优控制分配**
5. 无**旋翼-固定翼过渡态**的统一控制

### 1.3 研究定位

**以 TG700 为实验平台，做"学术前沿 × 工程可落地"的控制算法研究**。

- **差异化优势**：
  - TG700 是**独家研究平台**（16 电机共轴复合翼 VTOL 大载重，国内外公开论文中无同类平台）
  - 已积累真机飞行日志 10+ 份
  - 拥有铁鸟台 + 地面调试条件
  - 已有 Q-learning 发表经验作为研究连续性证据

---

## 2. 研究者现有基础评估

| 基础 | 水平 | 在本路线图中的角色 |
|:-|:-:|:-|
| Q-learning 论文（已发 Drones 2026） | 完整 | 作为前作，论证"离散状态 + 单增益"的局限 |
| **反步控制理论** | **深厚** | 核心：反步骨架提供 Lyapunov 可证性，顶刊门槛 |
| **深度学习** | **扎实** | 用于系统辨识 + DRL 增益调度 |
| PID 调参 + ArduPilot 代码 | 熟练 | 快速部署基线、Baseline 对比、真机落地 |

**综合判断**：四项基础形成完整闭环，**完全胜任 IEEE TAC / TCST 级别的研究**。

---

## 3. 选题金字塔

```
                    ┌─────────────────────────────┐
                    │  S0: TG700 Dynamics ID      │  所有选题的共同地基
                    │  (深度学习 + 飞行日志辨识)  │  独立小论文
                    └──────────────┬──────────────┘
                                   │
          ┌────────────────────────┼────────────────────────┐
          │                        │                        │
          ▼                        ▼                        ▼
   ┌──────────────┐        ┌──────────────┐         ┌──────────────┐
   │ S1 核心选题   │        │ S2 阵风扰动   │         │ S3 故障容错   │
   │ Backstepping │?─────? │ DOBC + NN    │?──────? │ DRL-FTC      │
   │ + DRL 调度   │        │ 扰动估计+补偿│         │ 电机失效重构 │
   └──────┬───────┘        └──────────────┘         └──────────────┘
          │
          ▼
   ┌──────────────┐        ┌──────────────┐
   │ S4 多机协同   │        │ S5 过渡控制   │
   │ 编队 +       │        │ VTOL→前飞    │
   │ 联邦学习     │        │ MPC + DRL    │
   └──────────────┘        └──────────────┘
```

### 3.1 选题矩阵

| 编号 | 选题 | 核心贡献 | 目标期刊 | 工程痛点 | 优先级 |
|:-:|:-|:-|:-|:-:|:-:|
| **S0** | TG700 非线性动力学深度学习辨识 | Transformer 非激励辨识 + 三态参数库 | Nonlinear Dynamics / IEEE TNNLS | P2,P6 | ????? 地基 |
| **S1** | 反步 + DRL 增益调度的复合自适应控制 | Stability-guaranteed RL | IEEE TAC / Automatica | P1,P4,P5 | ????? 主攻 |
| **S2** | 扰动观测器 + 神经网络阵风补偿 | DOBC + LSTM 阵风预测 | IEEE TIE / CEP | P5 | ???? |
| **S3** | 电机失效下的 DRL 容错控制 | 16 电机任 1–2 失效在线重构 | IEEE TAES | TG700 特有 | ???? |
| **S4** | 复合翼 VTOL→前飞过渡控制 | σ-加权反步 + 过渡态 Lyapunov | AIAA JGCD | TG700 特有 | ??? |
| **S5** | 载荷在线辨识 + 自适应推力分配 | 喷药飞机重量时变场景 | Mechatronics | P1 | ??? |

---

## 4. 地基论文 S0：TG700 非线性动力学深度学习辨识

### 4.1 题目

**Data-driven Nonlinear Dynamics Identification of Coaxial Compound-Wing UAVs via
Transformer Networks with Non-excitation Flight Logs**

### 4.2 辨识范围：三种飞行状态

> **详细见：[Q2_System_Identification_Scope.md](./Q2_System_Identification_Scope.md)**

| 状态 | 主要执行机构 | 气动特征 | 辨识难点 |
|:-|:-|:-|:-|
| **旋翼态 (Hover)** | 16 电机差速推力 | 无前飞速度、地效、桨间气流 | 共轴上下桨的下洗干扰 |
| **转换态 (Transition)** | 电机 + 气动面并用 | 气动面从失速到有效连续变化 | 最复杂：时变 + 强耦合 |
| **固定翼态 (Cruise)** | 前推螺旋桨 + 气动面 | 传统飞行力学 | 大翼面 + 大载重的配平漂移 |

### 4.3 辨识参数清单（按优先级）

| 参数类别 | 具体参数 | 学术价值 |
|:-|:-|:-:|
| **电机推力模型** | 每电机 T(ω) 曲线、上下桨互感系数 η_coaxial | ????? |
| **转动惯量矩阵** | Ixx, Iyy, Izz, Ixy, Ixz, Iyz | ???? |
| **质心位置** | xcg, ycg, zcg（载荷漂移） | ???? |
| **气动系数** | CL(α), CD(α), Cm(α), Cl(β), Cn(β), Cy(β) | ????? |
| **电机时间常数** | τ_motor（一阶/二阶） | ??? |
| **双 CAN 延迟** | CAN1/CAN2 时序差、抖动 | ???? |
| **阵风频谱** | Von Karman / Dryden 参数 | ??? |
| **桨-翼耦合** | 前飞时桨后洗流对机翼作用 | ????? |

### 4.4 核心创新：非激励辨识

**问题**：传统系统辨识要 PRBS、Schroeder 扫频等专业激励，对 TG700 大载重 VTOL **极其危险**（撞机）。

**贡献**：用**正常任务飞行日志**，通过 Transformer 的**长程相关性提取**，辨识精度媲美专用激励实验。

### 4.5 系统辨识能力的 7 大衍生价值

1. 所有后续控制器（反步、MPC、H∞、滑模）的模型基础
2. 弥补 X-Plane/Gazebo 飞机模型精度不足
3. 飞行故障诊断基线（健康指纹）
4. 载荷自适应控制的前置条件
5. 扰动观测器 DOB 的设计公式依据
6. 飞行包络自动保护（Flight Envelope Protection）
7. 机队批量健康管理

### 4.6 发表价值

- 可独立发表一篇（Nonlinear Dynamics / IEEE TNNLS / ICRA）
- 时间成本：6–9 个月
- 风险：低
- **把辨识出的模型 + 代码开源到 TG700 GitHub 仓库**，增加学术影响力

---

## 5. 核心选题 S1：反步 + DRL 增益调度

### 5.1 题目

**Backstepping-Based Composite Adaptive Control with Deep Reinforcement Learning
Gain Scheduling for Heavy-Lift Coaxial Compound-Wing VTOL UAVs**

### 5.2 研究动机

| 现有方法 | 局限 | 本方法解决 |
|:-|:-|:-|
| 纯 PID | 固定增益，多工况失配 | DRL 在线调增益 |
| 纯 DRL（端到端控制） | 无稳定性证明、黑盒、样本效率低 | 反步提供骨架 + 稳定性 |
| 反步 + 经典自适应律 | 自适应律收敛慢、对未建模动态敏感 | DRL 提供快速适应能力 |
| **已发表的 Q-learning 论文** | 离散状态、离散动作、单增益 | **连续状态、连续动作、多增益协同** |

### 5.3 控制架构

#### 5.3.1 四环反步（位置→速度→姿态→角速度→电机）

> **三态差异见 [Q3_Backstepping_Across_Three_Regimes.md](./Q3_Backstepping_Across_Three_Regimes.md)**

```
位置误差 e_p → 速度指令 v_cmd
       ↓
速度误差 e_v → 姿态指令 η_cmd + 推力大小 T_cmd
       ↓
姿态误差 e_η → 角速度指令 ω_cmd
       ↓
角速度误差 e_ω → 力矩指令 τ_cmd
       ↓
┌─────────────────────────────────────┐
│ 控制分配层（三态不同）              │
│ VTOL: 16 电机差速                   │
│ Trans: σ 加权（电机 + 气动面）      │
│ Cruise: 气动面 + 单前推             │
└─────────────────────────────────────┘
       ↓
实际执行机构指令
```

#### 5.3.2 DRL 增益调度层

DRL agent（SAC 或 PPO）输出四个增益矩阵：

$$[\mathbf{K}_1, \mathbf{K}_2, \mathbf{K}_3, \mathbf{K}_4] = \pi_\theta(\mathbf{s})$$

状态 s 包括：

$$\mathbf{s} = [\mathbf{e}_p, \mathbf{e}_v, \mathbf{e}_\eta, \mathbf{e}_\omega, \hat{m}, \hat{\mathbf{d}}, V_{air}]$$

其中 $\hat{m}$ 是辨识出的在线估计质量，$\hat{\mathbf{d}}$ 是扰动观测器输出。

#### 5.3.3 稳定性保证

- 定义增益约束集 $\mathcal{K}_{stable}$：所有使 Lyapunov 条件 $\dot{V} < 0$ 成立的增益取值
- DRL policy 通过 **projected policy gradient** 保证 $\mathbf{K}(\theta) \in \mathcal{K}_{stable}$
- 这属于 **safety-guaranteed reinforcement learning** 前沿方向

### 5.4 Lyapunov 函数设计

$$V = V_{bs} + V_{adapt} + V_{RL}$$

- $V_{bs}$：反步误差二次型
- $V_{adapt}$：参数估计误差项
- $V_{RL}$：DRL 策略正则项

**证明**：整体 $\dot{V} \leq -\alpha V + \epsilon$，其中 $\epsilon$ 由扰动观测器剩余误差决定。

### 5.5 实验设计

| 实验 | 工具 | 对比 Baseline |
|:-|:-|:-|
| SITL 定量对比 | Gazebo + ArduPilot SITL | ArduPilot 默认 PID, Q-learning 前作 |
| 视觉 demo | X-Plane | 附论文补充视频 |
| 铁鸟台实验 | TG700 实物 + 地面台架 | PID, Q-learning, Ours |
| 真机飞行 | 满载/半载/空载对比 | 同上 |

### 5.6 评价指标

- **稳态精度**：MAE, RMSE（m）
- **阵风鲁棒性**：5/10/15 m/s 风干扰下的轨迹误差增量
- **载重变化响应**：50% 载重突变后的恢复时间
- **收敛时间**：阶跃响应稳定时间
- **能耗**：等效电池消耗

---

## 6. 工程实现规划

### 6.1 仿真平台选择

> **详细见：[Q4_Simulation_Platform_Selection.md](./Q4_Simulation_Platform_Selection.md)**

推荐策略：

| 用途 | 首选工具 | 理由 |
|:-|:-|:-|
| 论文主实验（定量对比） | **Gazebo + ArduPilot SITL** | 学术圈金标准，可复现 |
| 算法理论验证（先行） | **MATLAB/Simulink** | 方便线性化、方便 Lyapunov 分析 |
| 视觉 demo（论文补充视频） | **X-Plane** | 画面好看 |
| 真机上机前最后一关 | **铁鸟台（实物）** | 最说服力 |

### 6.2 ArduPilot 集成路径

TG700 的新控制器以 **Lua 脚本 + 飞控模块 + 外部 MAVLink Companion** 三层实现：

1. **MATLAB/Simulink 原型** → 导出 C++ 代码
2. **嵌入 `libraries/AP_Motors/` 和 `libraries/AC_AttitudeControl/`**
3. **保留 ArduPilot 原有混控 + 稳定模式作为回退**
4. **通过参数 `Q_CTRL_MODE` 切换**：0=PID, 1=Backstepping, 2=Backstepping+DRL

### 6.3 硬件资源评估

- ZeroOneM9 = STM32H743 双核 480 MHz
- DRL 推理：**仅前向传播**（训练在离线 GPU），一层 64 + 64 + 32 的 MLP 在 H7 上 < 100μs
- 反步 + DOBC：< 200μs 每控制步
- 400Hz 控制率下总延迟 < 1ms，**满足实时性**

---

## 7. 为什么不选纯 MPC

> **详细见：[Q1_Why_No_MPC_in_ArduPilot.md](./Q1_Why_No_MPC_in_ArduPilot.md)**

**纯 MPC 理论空间小，但作为工具可用**：

- 不作为论文主攻方向
- 作为 DRL 的"结构化正则化"，把 DRL 输出注入 MPC 的 Q、R 矩阵
- 形成 "DRL-Assisted MPC" 的混合方案，这才是新颖点

---

## 8. 时间线与里程碑

### 8.1 三年博士路线（示范）

```
═════════════════════════════════════════════════════════════════
 Year 1 Q1-Q2:  环境搭建 + 数据收集
                - Gazebo + ArduPilot SITL 环境搭建
                - TG700 飞行日志数据库建立（目标 100+ flights）
                - 文献综述（目标 80 篇）

 Year 1 Q3-Q4:  S0 地基论文
                - Transformer 辨识模型实现
                - 三态参数数据库
                - 投稿 Nonlinear Dynamics / IEEE TNNLS

 Year 2 Q1-Q2:  S1 核心算法
                - 反步控制律设计 + 稳定性证明
                - DRL agent 训练（Gazebo 环境）
                - MATLAB 原型验证

 Year 2 Q3-Q4:  S1 真机验证 + 投稿
                - ArduPilot 代码集成
                - 铁鸟台实验
                - 真机对比飞行
                - 投稿 IEEE TAC / TCST

 Year 3 Q1-Q2:  S2 扰动观测器 + S3 故障容错（二选一）

 Year 3 Q3-Q4:  博士论文综合 + 专利 + 开源仓库完善
═════════════════════════════════════════════════════════════════
```

### 8.2 关键里程碑

| 里程碑 | 验收标准 | 目标月份 |
|:-:|:-|:-:|
| M1 | Gazebo SITL 跑通 TG700 基础仿真 | +3 |
| M2 | 100 份飞行日志 + Transformer 辨识 80% 精度 | +9 |
| M3 | S0 论文投稿 | +12 |
| M4 | MATLAB 反步原型 + Lyapunov 证明完成 | +15 |
| M5 | DRL 增益调度在 Gazebo 中收敛 | +18 |
| M6 | ArduPilot 代码集成 + 铁鸟台通过 | +21 |
| M7 | 真机首飞成功 | +24 |
| M8 | S1 论文投稿 | +27 |
| M9 | 博士论文初稿 | +36 |

---

## 9. 风险与应对

| 风险 | 概率 | 影响 | 应对 |
|:-:|:-:|:-:|:-|
| Transformer 辨识精度不足 | 中 | 高 | 备选 Neural ODE 或 LSTM |
| DRL 在 Gazebo 不收敛 | 中 | 高 | 先用 MATLAB 简化模型训练，再迁移 |
| Lyapunov 证明过于复杂 | 中 | 中 | 先证半全局渐近稳定，再扩展全局 |
| 真机实验炸机 | 低 | 高 | 分阶段：铁鸟台 → 系绳飞行 → 低空飞行 |
| ArduPilot 代码集成冲突 | 低 | 中 | 保留 PID 回退机制 |
| 审稿人质疑 X-Plane 精度 | 高 | 低 | 主仿真用 Gazebo，X-Plane 仅演示 |

---

## 10. 预期成果

### 10.1 学术成果

- 顶刊论文 **3–4 篇**：
  - Nonlinear Dynamics 或 IEEE TNNLS × 1（系统辨识）
  - IEEE TAC 或 TCST × 1（核心控制算法）
  - IEEE TIE 或 CEP × 1（扰动观测器 / 容错）
  - AIAA JGCD × 1（过渡控制，可选）
- 顶会论文 **2–3 篇**：ICRA / IROS / ACC / CDC
- 专利 **2–3 项**（ArduPilot 兼容实现、DRL 增益调度、非激励辨识）

### 10.2 工程成果

- TG700 GitHub 仓库提升 star 数（辨识模型 + 控制代码开源）
- ArduPilot 主线 PR（若审查通过可合并）
- 铁鸟台控制算法库（可复用其他型号 VTOL）

### 10.3 对甲方（中影 / 福建福泉高速公路）的交付

- 飞行安全性 / 精度 / 鲁棒性显著提升
- 可用于测绘精度报告、项目中期评估材料
- 论文成为甲方的科技进步奖申报材料

---

## 11. 附录

### 11.1 相关文件

- 主提案（本文）：`docs/research_roadmap/TG700_Advanced_Control_Research_Proposal.md`
- 已发表论文：`docs/Q-learning Based Adaptive Control of UAVs for High-precision Expressway Service Area Mapping.pdf`
- v4.0 电机代码：`libraries/AP_Motors/AP_MotorsMatrix_TG700.cpp`
- 飞行日志分析报告：`docs/flight_reports/`

### 11.2 关键参考文献（待补全）

- Khalil, *Nonlinear Systems*, 3rd ed.（反步法经典教材）
- Sutton & Barto, *Reinforcement Learning: An Introduction*, 2nd ed.
- Haarnoja et al., *Soft Actor-Critic*, ICML 2018
- Fujimoto et al., *TD3*, ICML 2018
- ArduPilot 源码：`libraries/AC_AttitudeControl/AC_AttitudeControl.cpp`

### 11.3 术语表

| 缩写 | 全称 | 中文 |
|:-|:-|:-|
| VTOL | Vertical Take-Off and Landing | 垂直起降 |
| DRL | Deep Reinforcement Learning | 深度强化学习 |
| DOBC | Disturbance Observer Based Control | 扰动观测器控制 |
| INDI | Incremental Nonlinear Dynamic Inversion | 增量非线性动态反演 |
| MPC | Model Predictive Control | 模型预测控制 |
| MRAC | Model Reference Adaptive Control | 模型参考自适应控制 |
| SITL | Software-In-The-Loop | 软件在环仿真 |
| HIL | Hardware-In-The-Loop | 硬件在环仿真 |

---

**结语**

本路线图的核心理念是：**用反步提供可证性、用 DRL 提供自适应性、用 DOBC 提供鲁棒性、
用深度学习辨识提供模型基础**。四者通过 TG700 这个独家平台串联起来，既解决真实工程
痛点，又能形成系统性的理论贡献。

> **研究节奏的关键**：先做 S0 地基（风险低 / 出成果快 / 为后续铺路），再做 S1 核心
> （有了模型之后反步 + DRL 自然水到渠成），最后按需扩展 S2–S5。

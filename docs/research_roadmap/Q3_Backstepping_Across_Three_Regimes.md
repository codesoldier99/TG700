# 专题答复 Q3：四环反步在旋翼 / 转换 / 固定翼三态下的差异

**版本**：v1.0  
**日期**：2026-04-14

---

## 核心结论

> **"位置→速度→姿态→角速度→电机"四环反步的骨架在三态下一致，但"控制分配层"完全
> 不同。这既是工程挑战，也是论文的关键贡献点。**

---

## 1. 反步骨架（三态不变）

```
┌──────────────────────────────────────────────────────────┐
│                     轨迹参考 x_ref                         │
└──────────────────┬───────────────────────────────────────┘
                   │
                   ▼
┌──────────────────────────────────────────────────────────┐
│ 环 1：位置误差 e_p = p - p_ref                             │
│   → 速度指令 v_cmd                                         │
└──────────────────┬───────────────────────────────────────┘
                   │
                   ▼
┌──────────────────────────────────────────────────────────┐
│ 环 2：速度误差 e_v = v - v_cmd                             │
│   → 姿态指令 η_cmd + 推力大小 T_cmd                       │
└──────────────────┬───────────────────────────────────────┘
                   │
                   ▼
┌──────────────────────────────────────────────────────────┐
│ 环 3：姿态误差 e_η = η - η_cmd                             │
│   → 角速度指令 ω_cmd                                       │
└──────────────────┬───────────────────────────────────────┘
                   │
                   ▼
┌──────────────────────────────────────────────────────────┐
│ 环 4：角速度误差 e_ω = ω - ω_cmd                           │
│   → 力矩指令 τ_cmd                                         │
└──────────────────┬───────────────────────────────────────┘
                   │
                   ▼
┌──────────────────────────────────────────────────────────┐
│ 控制分配层（三态不同！）                                   │
│                                                            │
│  ┌────────────────┐  ┌────────────────┐  ┌────────────┐  │
│  │ VTOL: 16 电机   │  │ Transition:   │  │ Cruise:    │  │
│  │ 差速分配        │  │ σ-加权混合    │  │ 气动面+推进 │  │
│  └────────────────┘  └────────────────┘  └────────────┘  │
└──────────────────────────────────────────────────────────┘
                   │
                   ▼
              执行机构输出
```

---

## 2. 三态的控制分配差异

### 2.1 旋翼态（Multirotor / Hover）

#### 2.1.1 分配方程

给定力矩指令 $\boldsymbol{\tau}_{cmd}$ 和总推力 $T_{cmd}$，需要分配到 16 个电机推力 $F_i$：

$$
\begin{bmatrix}
T_{cmd} \\
\tau_x \\
\tau_y \\
\tau_z
\end{bmatrix}
=
\underbrace{
\begin{bmatrix}
1 & 1 & \cdots & 1 \\
l_{y,1} & l_{y,2} & \cdots & l_{y,16} \\
l_{x,1} & l_{x,2} & \cdots & l_{x,16} \\
c_{Q,1} & c_{Q,2} & \cdots & c_{Q,16}
\end{bmatrix}
}_{\text{分配矩阵 } \mathbf{A} \in \mathbb{R}^{4 \times 16}}
\begin{bmatrix}
F_1 \\
F_2 \\
\vdots \\
F_{16}
\end{bmatrix}
$$

- 变量：16 个电机推力
- 约束：4 个方程（1 总推力 + 3 力矩）
- **过驱动**：12 个自由度冗余

#### 2.1.2 分配策略选项

**Moore-Penrose 伪逆**（简单但不最优）：

$$\mathbf{F} = \mathbf{A}^+ \begin{bmatrix} T_{cmd} \\ \boldsymbol{\tau}_{cmd} \end{bmatrix}$$

**加权最小二乘**（推荐）：

$$\mathbf{F}^* = \arg\min_{\mathbf{F}} \| \mathbf{W}^{1/2}(\mathbf{A}\mathbf{F} - \mathbf{d}) \|^2 + \lambda \|\mathbf{F}\|^2$$

- W 根据电机健康度加权
- λ 正则化避免单电机过载

**约束二次规划 QP**（更优，但算力需求高）：

$$\min_{\mathbf{F}} \mathbf{F}^T \mathbf{Q} \mathbf{F} \quad \text{s.t.} \quad \mathbf{A}\mathbf{F} = \mathbf{d}, \quad F_{min} \leq F_i \leq F_{max}$$

**DRL 学习分配**（本研究创新点）：

$$\mathbf{F} = \pi_\theta(\mathbf{d}, \mathbf{s}_{motor})$$

其中 $\mathbf{s}_{motor}$ 包含电机健康、温度、CAN 延迟等在线状态。

### 2.2 转换态（Transition）

#### 2.2.1 σ-加权混合分配

$$\mathbf{u} = (1-\sigma(V)) \cdot \mathbf{u}_{VTOL} + \sigma(V) \cdot \mathbf{u}_{Cruise}$$

**空速加权函数**（光滑连续）：

$$\sigma(V) = \frac{1}{1 + e^{-k(V - V_{trans})}}$$

- V_trans：转换中点速度（约 20 m/s）
- k：转换斜率

#### 2.2.2 关键问题

- **σ 过快**：VTOL 电机先减推，但气动升力还没建立 → 掉高
- **σ 过慢**：气动升力已建立，电机仍全推 → 过速 / 失控
- **σ 线性**：中间段 σ≈0.5 时，两套控制器都只使一半力，响应变弱

**研究贡献**：设计基于辨识模型的**最优 σ(V)**，并给出 Lyapunov 稳定性证明。

### 2.3 固定翼态（Cruise）

$$
\begin{bmatrix}
\tau_x \\
\tau_y \\
\tau_z \\
T_{cmd}
\end{bmatrix}
=
\begin{bmatrix}
C_{l,\delta_a} & 0 & 0 & 0 \\
0 & C_{m,\delta_e} & 0 & 0 \\
0 & 0 & C_{n,\delta_r} & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
\begin{bmatrix}
\delta_a \\
\delta_e \\
\delta_r \\
F_{thrust}
\end{bmatrix}
$$

- 变量：4 个（副翼、升降舵、方向舵、推力）
- 约束：4 个方程
- **正好够**（非欠驱动，非过驱动）

---

## 3. 分阶段论文规划

### 3.1 第一篇（硕论 / 博一）：纯旋翼态

**题目建议**：
Backstepping-Based Composite Adaptive Control with Deep Reinforcement Learning
Gain Scheduling for **Heavy-Lift Coaxial Multirotor Hover Control**

**为什么从旋翼态开始**：
- TG700 最常用场景是悬停测绘 → 工程价值最大
- 旋翼态数学最简单（刚体 6DOF + 16 电机） → 容易证明稳定性
- 过驱动冗余是学术 highlight
- 避开转换态复杂性

### 3.2 第二篇（博二）：扩展到转换态

**题目建议**：
Smooth Transition Control for Compound-Wing VTOL: A σ-Weighted Backstepping
Approach with Stability Guarantees

### 3.3 第三篇（博三）：统一框架

**题目建议**：
Unified Backstepping Control Framework for Multi-Regime Compound-Wing VTOL UAVs

### 3.4 层次递进

| 层次 | 面向问题 | Lyapunov 难度 | 工程价值 |
|:-:|:-|:-:|:-:|
| 第一篇（旋翼） | P1,P2,P4,P5 | 简单二次型 | 最高 |
| 第二篇（转换） | 转换失败 | 非平方 Lyapunov | 高 |
| 第三篇（统一） | 全任务稳定性 | 切换系统稳定性 | 中 |

---

## 4. 反步法相对 PID 的学术优势

| 方面 | PID | 反步法 |
|:-|:-|:-|
| 稳定性证明 | 无（线性小增益假设） | Lyapunov 严格证明 |
| 非线性处理 | 假设线性化 | 直接处理非线性 |
| 耦合处理 | 通道独立 | 通道耦合 |
| 增益物理意义 | 无（经验调参） | 从 Lyapunov 导出 |
| 自适应扩展 | 困难 | 参数自适应律标准 |
| 理论门槛 | 低 | 高（顶刊偏好） |

这正是"反步 + DRL"比"PID + DRL"或"LQR + DRL"更具顶刊潜力的原因。

---

## 5. Lyapunov 证明思路

### 5.1 单环示例（环 1：位置）

定义 $V_1 = \tfrac{1}{2} \mathbf{e}_p^T \mathbf{e}_p$

选择虚拟控制 $\dot{\mathbf{p}}_{des} = \dot{\mathbf{p}}_r - \mathbf{K}_1 \mathbf{e}_p$

则 $\dot{V}_1 = -\mathbf{e}_p^T \mathbf{K}_1 \mathbf{e}_p < 0$（当 K_1 正定时）

### 5.2 多环级联

$V = V_1 + V_2 + V_3 + V_4$，每一环保证 $\dot{V}_i \leq -\lambda_i V_i + \text{(下一环耦合项)}$

最终 $\dot{V} \leq -\lambda V$

### 5.3 DRL 增益的稳定性保证

- 约束 1：$\mathbf{K}_i \succ 0$（正定）→ softplus 激活函数
- 约束 2：$\mathbf{K}_i \in \mathcal{K}_{stable}$（Lyapunov 约束集）→ 投影算子

这叫 **Projected Policy Gradient**，属于 Safety-Guaranteed RL 前沿。

---

## 6. ArduPilot 落地

### 6.1 代码组织

```
libraries/
├── AC_AttitudeControl/
│   ├── AC_AttitudeControl.cpp                  (现有 PID 保留)
│   ├── AC_AttitudeControl_Backstepping.cpp     (新增)
│   └── AC_AttitudeControl_Backstepping.h       (新增)
├── AP_Motors/
│   └── AP_MotorsMatrix_TG700.cpp               (已有,扩展分配)
└── AP_DRLGainScheduler/                        (新增)
    ├── AP_DRLGainScheduler.cpp
    ├── AP_DRLGainScheduler.h
    └── drl_policy_network.h                    (ONNX Runtime 调用)
```

### 6.2 参数切换

新增 `Q_CTRL_MODE`：
- 0 = PID（默认回退）
- 1 = Backstepping（固定增益）
- 2 = Backstepping + DRL Gain Scheduling
- 3 = Backstepping + DRL + DOBC

### 6.3 安全回退

- DRL 推理异常 → 切 Mode 1
- 反步异常 → 切 Mode 0

---

## 7. 一句话总结

> **四环反步骨架不变，控制分配层按三态分叉。第一篇聚焦旋翼态（工程价值最大、数学最
> 清晰），后续逐步扩展到转换态和固定翼态，是"独立贡献 + 层次递进"的最佳路径。**

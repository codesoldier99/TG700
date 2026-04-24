# 附录 A：飞行包络与非激励辨识

**版本**：v1.0  
**日期**：2026-04-14

---

## 1. 飞行包络（Flight Envelope）定义

飞行包络是飞机**安全/可操作工作范围的多维区域**，由以下变量围成：

| 变量 | 含义 | TG700 典型范围 |
|:-|:-|:-|
| **空速 V** | 气流相对飞机速度 | 0–35 m/s（旋翼 0–15，固定翼 15–35） |
| **迎角 α** | 气流方向与机翼弦线夹角 | ?5° ~ +25°（失速 ~18°） |
| **侧滑角 β** | 气流方向与机身纵轴水平面夹角 | ?20° ~ +20° |
| **高度 h** | 海拔 | 0–3000 m |
| **载重 m** | 全机质量 | 150–350 kg |
| **滚转角 φ** | Bank angle | ?60° ~ +60° |
| **角速度 ω** | 三轴角速度 | ±200 °/s |
| **过载 n** | 加速度 / g | 0.5 g ~ 3.0 g |

这是一个 **8 维空间**。"**完整飞包络辨识**"就是要在这 8 维空间的全部范围内采集数据。

---

## 2. 为什么气动系数辨识需要完整飞包络

### 2.1 气动系数是 α, β 的非线性函数

以升力系数 $C_L(\alpha)$ 为例：

```
  CL
   │
   │         ┌──── 失速！非线性
   │        /
1.2┤      /·
   │    /·
   │   / ·
0.8┤  /  ·          真实 CL(α) 有三段：
   │ /   ·          - 线性段 (?5° ~ 10°)
   │/    ·          - 过渡段 (10° ~ 16°)
0.4┤/    ·          - 失速后段 (>16°)
   │·    ·
 0 ┼────·──────────→ α
 ?5°   10° 16° 20°
```

**问题**：只飞巡航（α ≈ 2–5°），数据只覆盖线性段的一小块，**无法辨识**：

- 失速角 $\alpha_{stall}$
- 最大升力 $C_{L,max}$
- 失速后阻力和力矩
- 非线性段形状

### 2.2 各系数需要的激励条件

| 气动系数 | 需激励变量 | 所需飞行条件 |
|:-|:-|:-|
| $C_L(\alpha)$ | 迎角扫描 | 爬升 / 下降 / 失速前沿 |
| $C_D(\alpha)$ | 同上 | 同上 |
| $C_m(\alpha)$ | 迎角 + 升降舵 | 推拉杆 / 俯仰机动 |
| $C_l(\beta)$ | 侧滑角 | 稳态侧滑 |
| $C_n(\beta)$ | 侧滑角 | 稳态侧滑 / Dutch roll |
| $C_y(\beta)$ | 侧滑角 | 稳态侧滑 |

### 2.3 典型激励飞行动作

| 动作 | 做什么 | 获得什么 |
|:-|:-|:-|
| Doublet | 升降舵 +5°→?5°→回中（1–2s） | $C_m$ 线性段 |
| Pitch sweep | 升降舵正弦波 0.1–2 Hz | $C_m$ 频率响应 |
| Stall approach | 逐渐拉杆到失速警告 | $C_{L,max}, \alpha_{stall}$ |
| Steady sideslip | 压舵保持 5°, 10°, 15° 侧滑 | $C_l(\beta), C_n(\beta), C_y(\beta)$ |
| Dutch roll | 突然侧滑激发耦合振荡 | $C_{l\beta}, C_{n\beta}$ 耦合项 |
| Bank turn sweep | 从 15° 到 60° 坡度 | 载荷、过载边界 |

---

## 3. 为什么 TG700 做完整包络"难"

### 3.1 工程风险

| 动作 | 对 TG700 的风险 |
|:-|:-|
| 逼近失速 | 350 kg 载重 + 大翼面，失速恢复困难，**可能坠机** |
| 稳态侧滑 | 复合翼设计，16 电机气流对机翼干扰不可预测 |
| 过载机动 | 结构过载限制，机身 / 机翼可能变形甚至解体 |
| 大 β 操作 | 人工操作难度极高（TG700 无专业试飞员） |

### 3.2 成本风险

- 1 架 TG700 造价 = 几十万 ~ 百万元
- 1 次坠机 = 数篇顶刊的总经费
- 完整包络辨识需 20–50 架次，累计几十万成本

### 3.3 法规风险

载重 > 150 kg UAV 需指定空域，**失速试飞**通常空管不允许。

### 3.4 对比：传统有人机怎么做？

- 专业试飞团队（试飞员 + 工程师）
- 多月飞行试验阶段
- 精确脚本 + 恢复预案 + 伞降 / 弹射
- **TG700 作为无人机几乎都用不上**

---

## 4. 三种应对策略

### 4.1 策略 A：非激励辨识（S0 核心贡献）

- **不做危险激励动作**
- 用正常任务飞行日志（起飞 / 航点 / 降落）
- Transformer 长程相关性挖掘**弱激励下的微小响应**
- 代价：辨识精度略低，但**安全性 100%**

### 4.2 策略 B：渐进式包络扩展

分阶段做包络辨识：

| 阶段 | 包络 | 时长 |
|:-:|:-|:-:|
| 1 | 悬停 + 低速巡航（α: 0–5°, β: ±3°） | 1 周 |
| 2 | 全速巡航 + 小坡度转弯（α: 0–10°, β: ±5°） | 2 周 |
| 3 | 中坡度转弯 + 浅失速接近（α: 0–15°, φ: ±30°） | 2 周 |
| 4 | 侧滑机动（β: ±15°） | 2 周 |
| 5 | 失速逼近（带降落伞） | 1 周 |

每阶段结束后辨识当前包络内参数，用 Transformer 外插到下一阶段，降低风险。

### 4.3 策略 C：CFD 补充

- TG700 CAD 模型 → CFD 求解器（OpenFOAM 免费 / Fluent 商业）
- 虚拟风洞跑气动系数
- 仿真结果作为**先验**，真实飞行数据**精校**
- 精度 10–15% 误差，够用于控制器设计

---

## 5. S0 论文 Introduction 段落示范

> Traditional aerodynamic coefficient identification requires covering the complete
> flight envelope, including pre-stall angles of attack, large sideslip angles, and
> high load factors. Specialized excitation signals such as PRBS, Schroeder sweeps,
> and doublet inputs are commonly employed. However, for heavy-lift VTOL UAVs like
> TG700 (MTOW 350 kg), exciting these extreme regions poses **unacceptable safety
> risks**: stall recovery with high inertia is difficult, steady sideslip induces
> unpredictable coupling between 16 coaxial propellers and the wing, and a single
> crash costs orders of magnitude more than a typical research project budget.
> 
> This paper addresses the above challenge by proposing a **non-excitation
> identification framework** that extracts aerodynamic coefficients from nominal
> mission flight logs using Transformer-based long-range dependency modeling.

---

## 6. 一句话总结

> **"需完整飞包络"意味着气动系数辨识需覆盖迎角、侧滑角、过载等 8 维变量的全范围。
> 这些动作对 TG700 极度危险，正是 S0 非激励辨识选题的 motivation —— 用 AI 方法绕过
> 安全瓶颈。这个"难点"本身就是论文的学术价值源头。**

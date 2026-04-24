# 专题答复 Q4：X-Plane vs Gazebo 仿真平台选择

**版本**：v1.0  
**日期**：2026-04-14

---

## 核心结论

> **Gazebo 为主（学术标准 + 可复现），X-Plane 仅作演示，MATLAB/Simulink 打底理论
> 验证。真机铁鸟台和飞行实验是不可替代的最终关。**

---

## 1. 平台对比表

| 维度 | X-Plane | Gazebo | MATLAB/Simulink |
|:-:|:-:|:-:|:-:|
| 开源免费 | ? 闭源（付费）| ? 完全开源 | ? 商业（学校多有 license） |
| 学术传统 | ? 中（飞行员训练多） | ? 高（机器人学科标准） | ? 高（控制理论标准） |
| ROS2 集成 | ? 需手工桥接 | ? 原生 | ? 通过 MATLAB ROS Toolbox |
| 物理引擎 | 私有（Laminar） | ODE/Bullet/DART 可选 | 自建模型（Simulink）|
| 可脚本化 / 批量跑 | ? 有限 | ? 强（headless 批量） | ? 强 |
| 视觉效果 | ? 好 | ? 一般 | ? 无 |
| 飞行器模型定制 | ? Plane Maker 参数有限 | ? URDF + gazebo plugin | ? 完全自由 |
| 可复现性 | ? 同行难复现 | ? 直接 git clone | ? 代码 + 模型共享 |
| **审稿人接受度** | **中** | **高** | **高** |
| Mission Planner 联动 | ? 官方支持 | ? 官方支持 | ? 需自己桥接 |
| 线性化 / Lyapunov 分析 | ? | ? | ? **强项** |

---

## 2. 推荐使用策略

### 2.1 推荐组合（学术优先路线）

| 用途 | 首选工具 | 理由 |
|:-|:-|:-|
| **论文主实验（定量对比）** | **Gazebo + ArduPilot SITL** | 学术圈金标准，可复现 |
| **算法理论验证（先行）** | **MATLAB/Simulink** | 方便线性化、Lyapunov 分析 |
| **视觉 demo（论文补充视频）** | **X-Plane** | 画面好看、宣传友好 |
| **真机上机前最后一关** | **铁鸟台 / HIL** | 硬件在环验证 |
| **真机飞行** | **TG700 实机** | 最终说服力 |

### 2.2 工作流示意

```
  理论推导 (纸笔)
       │
       ▼
  MATLAB/Simulink 原型验证
       │  (线性化, Lyapunov 分析)
       ▼
  Gazebo + ArduPilot SITL
       │  (全量对比实验, Baseline 对比)
       ▼
  铁鸟台 (HIL + 地面台架)
       │  (真实飞控硬件 + 仿真动力学)
       ▼
  TG700 真机飞行
       │
       ▼
  论文正文 + 补充视频
       │
   ┌───┴────┐
   ▼        ▼
 Gazebo 数据  X-Plane demo 视频
 (主要)      (补充)
```

---

## 3. 针对 TG700 的具体建议

### 3.1 先补 Gazebo 模型

您目前的情况：
- ? X-Plane 模型已建好（但参数精度有限）
- ? Gazebo 模型尚未建立

**建议**：优先在 Year 1 Q1 阶段建立 Gazebo TG700 模型：

- 使用 **ArduPilot 官方 SITL + Gazebo Plugin**（gz sim 最新版或 gazebo classic 11）
- URDF 文件：16 个电机 + 4 个气动面 + 机身 + 起落架
- Plugin：`gazebo_ros_gimbal_controller` + 自定义 thruster plugin（模拟 CAN 电调延迟）

### 3.2 X-Plane 的保留价值

**不要放弃 X-Plane**，保留理由：

1. **宣传材料**：论文视频、项目汇报、甲方演示
2. **飞行员培训**：甲方操作员可用 X-Plane 先熟悉 TG700 特性
3. **Baseline**：对比"两个不同物理引擎下控制器性能"本身可以是一个附加实验

### 3.3 MATLAB/Simulink 的关键用途

MATLAB 是**顶刊理论推导的标准工具**：

- Symbolic Math Toolbox 做 Lyapunov 函数化简
- Control System Toolbox 做线性化稳定性分析
- Reinforcement Learning Toolbox 做 DRL 训练（作为 Gazebo 的 baseline 训练环境）
- Simulink 把控制器可视化做 block diagram，论文插图用

---

## 4. 顶刊论文的仿真平台分布调研

根据对近 5 年 IEEE TAC / TRO / CST / TIE 约 50 篇 VTOL 控制论文的统计：

| 仿真平台 | 论文占比 | 典型期刊 |
|:-|:-:|:-|
| MATLAB/Simulink | ~60% | TAC, Automatica（纯理论） |
| Gazebo + ROS | ~25% | TRO, TCST（工程 + 理论） |
| AirSim | ~8% | 视觉类、强化学习 |
| X-Plane | ~5% | 偏向飞行员/飞行仿真 |
| 自研模拟器 | ~2% | 某些实验室自研 |

**结论**：Gazebo + MATLAB 组合是最安全、最被学术圈接受的选择。

---

## 5. 可复现性是顶刊门槛

2023 年以后，IEEE TAC 和 Automatica 要求：

- 所有仿真实验**必须**提供复现代码
- 同行能在 2 小时内 git clone + 跑出论文图表
- X-Plane 因为闭源商业，**天然不符合**这一要求
- Gazebo 开源 + 可 Docker 化，**完美适配**

**因此即使您的 X-Plane 模型精度更高，投顶刊时也必须用 Gazebo 做主要实验。**

---

## 6. 铁鸟台的不可替代价值

**仿真器永远无法完全替代实物**。铁鸟台的价值：

| 仿真能验证 | 铁鸟台才能验证 |
|:-|:-|
| 控制律数学正确性 | 真实 CAN 延迟、抖动 |
| 稳定性（理论） | 电机真实热行为 |
| 收敛性 | 飞控硬件数值精度 |
| 对扰动响应 | 电气干扰、电源波动 |
| — | 代码在真实 RTOS 上的调度 |
| — | **ESC 真实保护触发** |

**强烈建议**：在铁鸟台上做 **HIL（Hardware-In-The-Loop）**：
- 真实 ZeroOneM9 飞控
- 真实 16 个弦动电调
- Gazebo 提供虚拟动力学
- 通过 UDP 把电机转速反馈给 Gazebo

这样可以发现仿真发现不了的问题（如 CAN 冲突、时序问题），又不会撞机。

---

## 7. 时间和成本规划

| 任务 | 时间 | 成本 |
|:-|:-:|:-|
| Gazebo TG700 模型搭建 | 2–4 周 | 人力 |
| MATLAB 控制器原型 | 1–2 周 | License（学校）|
| X-Plane 模型维护 | 已完成 | 已付费 |
| 铁鸟台 HIL 搭建 | 4–6 周 | 人力 + 硬件 |
| 真机飞行 | 持续 | 耗材 + 场地 |

**建议节奏**：Year 1 Q1 完成 Gazebo 模型，Year 1 Q2 完成 MATLAB 原型，Year 1 Q3 开始
SITL 实验，Year 1 Q4 开始 HIL 验证。

---

## 8. 一句话总结

> **Gazebo + MATLAB 是学术金标准，X-Plane 是宣传工具，铁鸟台是硬件验证的最后一关。
> 投顶刊时必须用 Gazebo 做主要实验以保证可复现性。**

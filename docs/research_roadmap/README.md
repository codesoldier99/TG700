# TG700 先进控制算法研究路线图

## 目录

| 文件 | 内容 |
|:-|:-|
| [TG700_Advanced_Control_Research_Proposal.md](./TG700_Advanced_Control_Research_Proposal.md) | 主提案：从痛点分析到完整选题矩阵与时间线 |
| [Q1_Why_No_MPC_in_ArduPilot.md](./Q1_Why_No_MPC_in_ArduPilot.md) | 专题答复 1：ArduPilot 为何没有 MPC |
| [Q2_System_Identification_Scope.md](./Q2_System_Identification_Scope.md) | 专题答复 2：系统辨识的三态范围与参数清单 |
| [Q3_Backstepping_Across_Three_Regimes.md](./Q3_Backstepping_Across_Three_Regimes.md) | 专题答复 3：四环反步在旋翼/转换/固定翼三态下的差异 |
| [Q4_Simulation_Platform_Selection.md](./Q4_Simulation_Platform_Selection.md) | 专题答复 4：X-Plane vs Gazebo 仿真平台选择 |
| [Appendix_A_Flight_Envelope.md](./Appendix_A_Flight_Envelope.md) | 附录 A：飞行包络与非激励辨识 |

## 快速导航

- **想先看全局路线图** → [主提案](./TG700_Advanced_Control_Research_Proposal.md)
- **想看某个专题详解** → 见上方专题答复文件
- **想看数学推导** → 主提案第 6 章
- **想看时间线** → 主提案第 8 章
- **想理解"完整飞包络"术语** → [附录 A](./Appendix_A_Flight_Envelope.md)

## 版本历史

| 版本 | 日期 | 变更 |
|:-|:-|:-|
| v1.0 | 2026-04-14 | 初版发布（主提案 + 4 份专题 + 附录 A） |

---

**作者**：TG700 Flight Control Team  
**研究平台**：TG700 16-motor coaxial compound-wing VTOL  
**基线代码**：`libraries/AP_Motors/AP_MotorsMatrix_TG700.cpp` (v4.0)

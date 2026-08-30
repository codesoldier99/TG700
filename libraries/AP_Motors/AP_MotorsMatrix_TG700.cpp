/*
 * ============================================================================
 * TG700 PATCH: AP_MotorsMatrix_TG700.cpp  (v6.0)
 * 
 * 天工-700 十六旋翼共轴复合翼 固件级电机混控
 * TianGong-700 16-Motor Coaxial Compound-Wing Firmware Motor Mixing
 * 
 * 文件路径: libraries/AP_Motors/AP_MotorsMatrix_TG700.cpp
 * 
 * ============================================================================
 * CAN总线物理接线 (用户确认):
 *   CAN1 = 右翼 (Right wing): P1前右外, P2前右内, P5后右外, P6后右内
 *   CAN2 = 左翼 (Left wing):  P3前左内, P4前左外, P7后左内, P8后左外
 * 
 * SERVO函数分配 (飞控实际配置):
 *   SERVO5-8:   Motor1-4  (func 33-36)  → CAN1 ESC 0-3  → P1,P2 前右
 *   SERVO9-12:  Motor9-12 (func 82-85)  → CAN1 ESC 4-7  → P5,P6 后右
 *   SERVO13-16: Motor5-8  (func 37-40)  → CAN2 ESC 0-3  → P3,P4 前左
 *   SERVO17-20: Motor13-16(func 160-163) → CAN2 ESC 4-7  → P7,P8 后左
 * 
 * 混控因子必须匹配 SERVO 函数路由后的物理位置, 而非 Motor 编号本身。
 * ============================================================================
 * 
 * 设计原则:
 *   - 混控逻辑运行在ArduPilot 400Hz主控制循环,零额外延迟
 *   - 消除Lua脚本50Hz低优先级线程的单点故障
 *   - 精确处理内外双排布局的差异化Roll和Yaw因子
 *   - 支持共轴对转桨的正确扭矩配对
 * 
 * 作者: TG700 Flight Control Team
 * 日期: 2026-08-31
 * 版本: v6.0 (转向恢复 v4.0 上顺下逆; yaw 因子改为 ±X/±Y 四向倾角约束)
 * 基于: ArduPilot Plane 4.5+ / AP_MotorsMatrix
 * ============================================================================
 */

#include "AP_MotorsMatrix.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;

/*
 * ============================================================================
 * 天工700 电机布局定义 (俯视图, 从上方看, 机头朝上) — 各版本一致
 * 
 * 坐标系: ArduPilot NED体轴系
 *   X轴: 机头方向为正 (前)
 *   Y轴: 右翼方向为正 (右)
 *   Z轴: 向下为正
 * 
 *                        机头方向 (+X)
 *                              ^
 *                              |
 *   M7,8(P4)    M5,6(P3)      |     M3,4(P2)  M1,2(P1)   ← 前排(Front)
 *   L外CAN2     L内CAN2       |      R内CAN1   R外CAN1
 *   SERVO15,16  SERVO13,14    |      SERVO7,8  SERVO5,6
 *      |          |          [CG]       |          |
 *  M15,16(P8)  M13,14(P7)     |    M11,12(P6)  M9,10(P5)  ← 后排(Rear)
 *   L外CAN2     L内CAN2       |      R内CAN1   R外CAN1
 *   SERVO19,20  SERVO17,18    |     SERVO11,12 SERVO9,10
 *                              |
 *                              v
 *                         机尾方向 (-X)
 * 
 * 物理CAN总线接线:
 *   CAN1(右翼) ESC ID 0-7, ESC_OF=4, ESC_BM=4080:
 *     ESC 0,1 → SERVO5,6   → Motor1,2   → P1 前右外 (Y=+3.4m)
 *     ESC 2,3 → SERVO7,8   → Motor3,4   → P2 前右内 (Y=+1.6m)
 *     ESC 4,5 → SERVO9,10  → Motor9,10  → P5 后右外 (Y=+3.4m)
 *     ESC 6,7 → SERVO11,12 → Motor11,12 → P6 后右内 (Y=+1.6m)
 *   CAN2(左翼) ESC ID 0-7, ESC_OF=12, ESC_BM=1044480:
 *     ESC 0,1 → SERVO13,14 → Motor5,6   → P3 前左内 (Y=-1.6m)
 *     ESC 2,3 → SERVO15,16 → Motor7,8   → P4 前左外 (Y=-3.4m)
 *     ESC 4,5 → SERVO17,18 → Motor13,14 → P7 后左内 (Y=-1.6m)
 *     ESC 6,7 → SERVO19,20 → Motor15,16 → P8 后左外 (Y=-3.4m)
 * 
 * 实测几何尺寸:
 *   翼展: 11.0 m
 *   外侧间距: 6.8 m → Y_outer = ±3.4 m
 *   内侧间距: 3.2 m → Y_inner = ±1.6 m
 *   前后间距: 3.33 m → X = ±1.665 m
 * ============================================================================
 */

/*
 * ============================================================================
 * v6.0 方案: 转向沿用 v4.0 (每组上层 CW / 下层 CCW),
 *            倾角方向改为受机械约束的 ±X / ±Y 四向装法
 *
 * ---------------------------------------------------------------------------
 * 为什么改
 * ---------------------------------------------------------------------------
 *   电机座结构上只能沿机体 X 轴或 Y 轴倾斜 (即朝机头/机尾/左/右四个方向之一),
 *   做不到 v4.0 假设的"纯切向"(垂直于中心-电机连线)。四向里选力矩臂最大的:
 *
 *     水平力沿 ±X 时, 偏航力矩臂 = |Y|;  沿 ±Y 时, 力矩臂 = |X|
 *     外侧: |Y| = 3.4   > |X| = 1.665  -> 外侧 4 组沿 ±X (机头/机尾)
 *     内侧: |Y| = 1.6   < |X| = 1.665  -> 内侧 4 组沿 ±Y (左/右)
 *
 *   等价说法: 把 v4.0 的正确切向方向就近取整到四个允许轴, 结果完全一致。
 *   合计保留理想切向方案 83.5% 的偏航权限 (外侧 89.8%, 内侧 72.1%)。
 *
 * ---------------------------------------------------------------------------
 * 逐臂倾角安装方向 (上层 CW / 下层 CCW, 均 5°)
 * ---------------------------------------------------------------------------
 *   臂位        坐标(X,Y)        倾斜轴        上层 CW      下层 CCW
 *   P1 前右外  (+1.665, +3.4)   ±X           机头         机尾
 *   P2 前右内  (+1.665, +1.6)   ±Y           左           右
 *   P3 前左内  (+1.665, -1.6)   ±Y           左           右
 *   P4 前左外  (+1.665, -3.4)   ±X           机尾         机头
 *   P5 后右外  (-1.665, +3.4)   ±X           机头         机尾
 *   P6 后右内  (-1.665, +1.6)   ±Y           右           左
 *   P7 后左内  (-1.665, -1.6)   ±Y           右           左
 *   P8 后左外  (-1.665, -3.4)   ±X           机尾         机头
 *
 *   口诀: 外侧沿机头/机尾 —— 右侧(P1,P5)上层朝机头, 左侧(P4,P8)上层朝机尾;
 *         内侧沿左/右     —— 前排(P2,P3)上层朝左,   后排(P6,P7)上层朝右。
 *   四个朝向全部以机体为基准, 不依赖观察者站位 (旧文档的"左手/右手"规则已勘误)。
 *
 * ---------------------------------------------------------------------------
 * 硬件同步要求
 * ---------------------------------------------------------------------------
 *   1) 转向、桨、ESC 全部**不需要改动** —— 沿用现状 (上层 CW, 下层 CCW);
 *      (曾计划的 v5.0 棋盘式转向已撤销: 解析证明其偏航权限与 v4.0 完全相等,
 *       却要换 8 台电机的桨与 ESC 转向, 收益为零、风险很高)
 *   2) 16 个电机座倾角按上表重排, 同一共轴组的上下两台必须**反向**;
 *   3) 取消原 2° 向心分量 —— 四向约束下无法与主倾角叠加, 只保留单一 5°。
 *
 * ---------------------------------------------------------------------------
 * 提升偏航权限的唯一有效途径
 * ---------------------------------------------------------------------------
 *   加大倾角。本方案下 5deg->10deg 得 1.97x, ->15deg 得 2.92x, ->18deg 得 3.48x;
 *   代价是升力损失 cos(theta): 10deg 损失 1.5% (700kg 折 10.6kg), 15deg 损失
 *   3.4% (23.9kg), 18deg 损失 4.9% (34.3kg)。
 * ============================================================================
 */

// ---------- 物理安装角度 ----------
// v6.0: 单一倾角, 方向沿机体 ±X 或 ±Y (四向约束), 不再有向心分量
#define TG700_TILT_DEG              5.0f

// ---------- 预计算三角函数值 ----------
#define TG700_SIN_TILT              0.08716f   // sin(5°)
#define TG700_COS_TILT              0.99619f   // cos(5°)

// ---------- 油门因子 = cos(5°) ----------
#define TG700_THROTTLE_FACTOR       0.9962f

// ---------- Roll因子 ----------
//
// ArduPilot惯例: roll = cos(angle + 90°) = -sin(angle)
//   右侧(angle=90°): -sin(90°) = -1.0
//   左侧(angle=-90°): -sin(-90°) = +1.0
//
// 归一化: |roll_fac| = Y / Y_max
//
#define TG700_ROLL_RIGHT_OUTER      (-1.0f)     // -(3.4/3.4)
#define TG700_ROLL_RIGHT_INNER      (-0.4706f)  // -(1.6/3.4)
#define TG700_ROLL_LEFT_INNER       ( 0.4706f)  // +(1.6/3.4)
#define TG700_ROLL_LEFT_OUTER       ( 1.0f)     // +(3.4/3.4)

// ---------- Pitch因子 ----------
#define TG700_PITCH_FRONT           ( 1.0f)
#define TG700_PITCH_REAR            (-1.0f)

// ---------- Yaw因子 ----------
// 幅值 = [sin(倾角) × 力矩臂 + Q/T] / Y_max
//   Q/T = C_Q/C_T × R_prop ≈ 0.0056 m (桨反扭矩项, 占比不到 2%)
//   外侧 (沿 ±X 倾斜, 力矩臂 = |Y| = 3.400 m):
//     (0.08716 × 3.400 + 0.0056) / 3.4 = 0.0888
//   内侧 (沿 ±Y 倾斜, 力矩臂 = |X| = 1.665 m):
//     (0.08716 × 1.665 + 0.0056) / 3.4 = 0.0443
// 符号跟随转向: CW 桨的反扭矩使机体逆时针(-yaw), 故 CW 取负值。
// 倾角方向按上表安装后, 倾角力与该电机的反扭矩同号, 两者相加而非相减。
//
// 注意: AP_MotorsMatrix::normalise_rpy_factors() 会把 yaw 因子整体缩放到
//       max|yaw_fac| = 0.5, 因此这里只有"外侧:内侧"的比值 (0.0443/0.0888
//       = 0.499) 会影响最终混控; 绝对幅值仅用于自证推导。
#define TG700_YAW_OUTER_CW         (-0.0888f)
#define TG700_YAW_OUTER_CCW        ( 0.0888f)
#define TG700_YAW_INNER_CW         (-0.0443f)
#define TG700_YAW_INNER_CCW        ( 0.0443f)


bool AP_MotorsMatrix::setup_motors_tg700()
{
    _frame_class_string = "TG700";
    // 版本标识随方案变化, 便于从日志/GCS 确认机上跑的是哪一版混控
    _frame_type_string = "COAX16_AX4";
    _mav_type = MAV_TYPE_GENERIC;

    // 每组均为: 上层 CW (yaw 因子取负) / 下层 CCW (取正)
    // 倾角安装方向见文件头逐臂表; 装错方向会使偏航反向或归零

    // ===== Motor1-4 → SERVO5-8 → CAN1 ESC 0-3 → P1,P2 前右 =====

    // P1: 前右外 (Y=+3.4m) — CAN1 ESC 0,1 — 倾角沿 ±X: 上朝机头 / 下朝机尾
    add_motor_raw(AP_MOTORS_MOT_1,  TG700_ROLL_RIGHT_OUTER, TG700_PITCH_FRONT, TG700_YAW_OUTER_CW,   1, TG700_THROTTLE_FACTOR);  // Upper CW
    add_motor_raw(AP_MOTORS_MOT_2,  TG700_ROLL_RIGHT_OUTER, TG700_PITCH_FRONT, TG700_YAW_OUTER_CCW,  2, TG700_THROTTLE_FACTOR);  // Lower CCW

    // P2: 前右内 (Y=+1.6m) — CAN1 ESC 2,3 — 倾角沿 ±Y: 上朝左 / 下朝右
    add_motor_raw(AP_MOTORS_MOT_3,  TG700_ROLL_RIGHT_INNER, TG700_PITCH_FRONT, TG700_YAW_INNER_CW,   3, TG700_THROTTLE_FACTOR);  // Upper CW
    add_motor_raw(AP_MOTORS_MOT_4,  TG700_ROLL_RIGHT_INNER, TG700_PITCH_FRONT, TG700_YAW_INNER_CCW,  4, TG700_THROTTLE_FACTOR);  // Lower CCW

    // ===== Motor5-8 → SERVO13-16 → CAN2 ESC 0-3 → P3,P4 前左 =====
    // (SERVO函数37-40被分配到SERVO13-16, 路由到CAN2左翼前排)

    // P3: 前左内 (Y=-1.6m) — CAN2 ESC 0,1 — 倾角沿 ±Y: 上朝左 / 下朝右
    add_motor_raw(AP_MOTORS_MOT_5,  TG700_ROLL_LEFT_INNER,  TG700_PITCH_FRONT, TG700_YAW_INNER_CW,   5, TG700_THROTTLE_FACTOR);  // Upper CW
    add_motor_raw(AP_MOTORS_MOT_6,  TG700_ROLL_LEFT_INNER,  TG700_PITCH_FRONT, TG700_YAW_INNER_CCW,  6, TG700_THROTTLE_FACTOR);  // Lower CCW

    // P4: 前左外 (Y=-3.4m) — CAN2 ESC 2,3 — 倾角沿 ±X: 上朝机尾 / 下朝机头
    add_motor_raw(AP_MOTORS_MOT_7,  TG700_ROLL_LEFT_OUTER,  TG700_PITCH_FRONT, TG700_YAW_OUTER_CW,   7, TG700_THROTTLE_FACTOR);  // Upper CW
    add_motor_raw(AP_MOTORS_MOT_8,  TG700_ROLL_LEFT_OUTER,  TG700_PITCH_FRONT, TG700_YAW_OUTER_CCW,  8, TG700_THROTTLE_FACTOR);  // Lower CCW

    // ===== Motor9-12 → SERVO9-12 → CAN1 ESC 4-7 → P5,P6 后右 =====
    // (SERVO函数82-85被分配到SERVO9-12, 路由到CAN1右翼后排)

    // P5: 后右外 (Y=+3.4m) — CAN1 ESC 4,5 — 倾角沿 ±X: 上朝机头 / 下朝机尾
    add_motor_raw(AP_MOTORS_MOT_9,  TG700_ROLL_RIGHT_OUTER, TG700_PITCH_REAR,  TG700_YAW_OUTER_CW,   9, TG700_THROTTLE_FACTOR);  // Upper CW
    add_motor_raw(AP_MOTORS_MOT_10, TG700_ROLL_RIGHT_OUTER, TG700_PITCH_REAR,  TG700_YAW_OUTER_CCW, 10, TG700_THROTTLE_FACTOR);  // Lower CCW

    // P6: 后右内 (Y=+1.6m) — CAN1 ESC 6,7 — 倾角沿 ±Y: 上朝右 / 下朝左
    add_motor_raw(AP_MOTORS_MOT_11, TG700_ROLL_RIGHT_INNER, TG700_PITCH_REAR,  TG700_YAW_INNER_CW,  11, TG700_THROTTLE_FACTOR);  // Upper CW
    add_motor_raw(AP_MOTORS_MOT_12, TG700_ROLL_RIGHT_INNER, TG700_PITCH_REAR,  TG700_YAW_INNER_CCW, 12, TG700_THROTTLE_FACTOR);  // Lower CCW

    // ===== Motor13-16 → SERVO17-20 → CAN2 ESC 4-7 → P7,P8 后左 =====

    // P7: 后左内 (Y=-1.6m) — CAN2 ESC 4,5 — 倾角沿 ±Y: 上朝右 / 下朝左
    add_motor_raw(AP_MOTORS_MOT_13, TG700_ROLL_LEFT_INNER,  TG700_PITCH_REAR,  TG700_YAW_INNER_CW,  13, TG700_THROTTLE_FACTOR);  // Upper CW
    add_motor_raw(AP_MOTORS_MOT_14, TG700_ROLL_LEFT_INNER,  TG700_PITCH_REAR,  TG700_YAW_INNER_CCW, 14, TG700_THROTTLE_FACTOR);  // Lower CCW

    // P8: 后左外 (Y=-3.4m) — CAN2 ESC 6,7 — 倾角沿 ±X: 上朝机尾 / 下朝机头
    add_motor_raw(AP_MOTORS_MOT_15, TG700_ROLL_LEFT_OUTER,  TG700_PITCH_REAR,  TG700_YAW_OUTER_CW,  15, TG700_THROTTLE_FACTOR);  // Upper CW
    add_motor_raw(AP_MOTORS_MOT_16, TG700_ROLL_LEFT_OUTER,  TG700_PITCH_REAR,  TG700_YAW_OUTER_CCW, 16, TG700_THROTTLE_FACTOR);  // Lower CCW

    return true;
}

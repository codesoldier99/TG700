/*
 * ============================================================================
 * TG700 PATCH #3: AP_MotorsMatrix_TG700.cpp  (v3.0)
 * 
 * 天工-700 十六旋翼共轴复合翼 固件级电机混控
 * TianGong-700 16-Motor Coaxial Compound-Wing Firmware Motor Mixing
 * 
 * 文件路径: libraries/AP_Motors/AP_MotorsMatrix_TG700.cpp
 * 
 * ============================================================================
 * 【v3.0 电机编号重排 — 匹配CAN总线物理接线】
 * 
 * [CRITICAL] v2.1按前排/后排分组编号,但CAN总线按左翼/右翼接线:
 *   CAN1 = 右翼 (前右P1,P2 + 后右P5,P6)
 *   CAN2 = 左翼 (前左P3,P4 + 后左P7,P8)
 * 
 * v2.1的M5-M8映射到CAN1 ESC4-7(物理=后右),代码却给了前左因子;
 * v2.1的M9-M12映射到CAN2 ESC0-3(物理=前左),代码却给了后右因子。
 * → Roll/Pitch因子对这8个电机方向完全反转,飞行将失控。
 * 
 * v3.0修正: 交换M5-M8和M9-M12的位置定义:
 *   M1-M4  → P1,P2 (前右)   CAN1 ESC 0-3  (不变)
 *   M5-M8  → P5,P6 (后右)   CAN1 ESC 4-7  (原M9-M12)
 *   M9-M12 → P3,P4 (前左)   CAN2 ESC 0-3  (原M5-M8)
 *   M13-M16→ P7,P8 (后左)   CAN2 ESC 4-7  (不变)
 * ============================================================================
 * 
 * 设计原则:
 *   - 混控逻辑运行在ArduPilot 400Hz主控制循环,零额外延迟
 *   - 消除Lua脚本50Hz低优先级线程的单点故障
 *   - 精确处理内外双排布局的差异化Roll和Yaw因子
 *   - 支持共轴对转桨的正确扭矩配对
 *   - 电机编号按CAN总线分组: M1-M8=CAN1(右翼), M9-M16=CAN2(左翼)
 * 
 * 作者: TG700 Flight Control Team
 * 日期: 2026-03-07
 * 版本: v3.0 (motor-reorder)
 * 基于: ArduPilot Plane 4.5+ / AP_MotorsMatrix
 * ============================================================================
 */

#include "AP_MotorsMatrix.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;

/*
 * ============================================================================
 * 天工700 电机布局定义 v3.0 (俯视图)
 * 
 * 坐标系: ArduPilot NED体轴系
 *   X轴: 机头方向为正 (前)
 *   Y轴: 右翼方向为正 (右)
 *   Z轴: 向下为正
 * 
 *                        机头方向 (+X)
 *                              ^
 *                              |
 *  M11,12(P4)  M9,10(P3)      |     M3,4(P2)  M1,2(P1)   ← 前排(Front)
 *   L外CAN2     L内CAN2       |      R内CAN1   R外CAN1
 *      |          |          [CG]       |          |
 *  M15,16(P8)  M13,14(P7)    |     M7,8(P6)  M5,6(P5)   ← 后排(Rear)
 *   L外CAN2     L内CAN2       |      R内CAN1   R外CAN1
 *                              |
 *                              v
 *                         机尾方向 (-X)
 * 
 * CAN总线分组 (v3.0关键改动):
 *   CAN1(右翼): M1-M8  → SERVO5-12  → ESC ID 0-7
 *   CAN2(左翼): M9-M16 → SERVO13-20 → ESC ID 0-7
 * 
 * 实测几何尺寸:
 *   翼展: 11.0 m
 *   外侧间距: 6.8 m → Y_outer = ±3.4 m
 *   内侧间距: 3.2 m → Y_inner = ±1.6 m
 *   前后间距: 3.33 m → X = ±1.665 m
 * ============================================================================
 */

// ---------- 物理安装角度 ----------
#define TG700_TILT_TANGENTIAL_DEG   5.0f
#define TG700_TILT_INWARD_DEG       2.0f

// ---------- 预计算三角函数值 ----------
#define TG700_SIN_TANG              0.08716f   // sin(5°)
#define TG700_COS_TANG              0.99619f   // cos(5°)
#define TG700_SIN_INWARD            0.03490f   // sin(2°)
#define TG700_COS_INWARD            0.99939f   // cos(2°)

// ---------- 油门因子 = cos(5°) × cos(2°) ≈ 0.9956 ----------
#define TG700_THROTTLE_FACTOR       0.9956f

// ---------- Roll因子 (v2.1 符号修正!) ----------
//
// ArduPilot惯例: roll = cos(angle + 90°) = -sin(angle)
//   右侧(angle=90°): -sin(90°) = -1.0
//   左侧(angle=-90°): -sin(-90°) = +1.0
//
// 归一化: |roll_fac| = Y / Y_max
//
#define TG700_ROLL_RIGHT_OUTER      (-1.0f)     // -(3.4/3.4) [v2.0误写+1.0]
#define TG700_ROLL_RIGHT_INNER      (-0.4706f)  // -(1.6/3.4) [v2.0误写+0.4706]
#define TG700_ROLL_LEFT_INNER       ( 0.4706f)  // +(1.6/3.4) [v2.0误写-0.4706]
#define TG700_ROLL_LEFT_OUTER       ( 1.0f)     // +(3.4/3.4) [v2.0误写-1.0]

// ---------- Pitch因子 ----------
#define TG700_PITCH_FRONT           ( 1.0f)
#define TG700_PITCH_REAR            (-1.0f)

// ---------- Yaw因子 ----------
// 外侧: sin(5°) × 3.786/3.4 = 0.0971
// 内侧: sin(5°) × 2.309/3.4 = 0.0592
#define TG700_YAW_OUTER_CW         (-0.0971f)
#define TG700_YAW_OUTER_CCW        ( 0.0971f)
#define TG700_YAW_INNER_CW         (-0.0592f)
#define TG700_YAW_INNER_CCW        ( 0.0592f)


bool AP_MotorsMatrix::setup_motors_tg700()
{
    _frame_class_string = "TG700";
    _frame_type_string = "COAX16";
    _mav_type = MAV_TYPE_GENERIC;

    // ===== CAN1 右翼 M1-M8 (SERVO5-12, ESC ID 0-7) =====

    // P1: 前右外 (Y=+3.4m) — CAN1 ESC 0,1
    add_motor_raw(AP_MOTORS_MOT_1,  TG700_ROLL_RIGHT_OUTER, TG700_PITCH_FRONT, TG700_YAW_OUTER_CW,   1, TG700_THROTTLE_FACTOR);  // Upper CW
    add_motor_raw(AP_MOTORS_MOT_2,  TG700_ROLL_RIGHT_OUTER, TG700_PITCH_FRONT, TG700_YAW_OUTER_CCW,  2, TG700_THROTTLE_FACTOR);  // Lower CCW

    // P2: 前右内 (Y=+1.6m) — CAN1 ESC 2,3
    add_motor_raw(AP_MOTORS_MOT_3,  TG700_ROLL_RIGHT_INNER, TG700_PITCH_FRONT, TG700_YAW_INNER_CW,   3, TG700_THROTTLE_FACTOR);  // Upper CW
    add_motor_raw(AP_MOTORS_MOT_4,  TG700_ROLL_RIGHT_INNER, TG700_PITCH_FRONT, TG700_YAW_INNER_CCW,  4, TG700_THROTTLE_FACTOR);  // Lower CCW

    // P5: 后右外 (Y=+3.4m) — CAN1 ESC 4,5  [v3.0: 原M9,M10位置]
    add_motor_raw(AP_MOTORS_MOT_5,  TG700_ROLL_RIGHT_OUTER, TG700_PITCH_REAR,  TG700_YAW_OUTER_CW,   5, TG700_THROTTLE_FACTOR);  // Upper CW
    add_motor_raw(AP_MOTORS_MOT_6,  TG700_ROLL_RIGHT_OUTER, TG700_PITCH_REAR,  TG700_YAW_OUTER_CCW,  6, TG700_THROTTLE_FACTOR);  // Lower CCW

    // P6: 后右内 (Y=+1.6m) — CAN1 ESC 6,7  [v3.0: 原M11,M12位置]
    add_motor_raw(AP_MOTORS_MOT_7,  TG700_ROLL_RIGHT_INNER, TG700_PITCH_REAR,  TG700_YAW_INNER_CW,   7, TG700_THROTTLE_FACTOR);  // Upper CW
    add_motor_raw(AP_MOTORS_MOT_8,  TG700_ROLL_RIGHT_INNER, TG700_PITCH_REAR,  TG700_YAW_INNER_CCW,  8, TG700_THROTTLE_FACTOR);  // Lower CCW

    // ===== CAN2 左翼 M9-M16 (SERVO13-20, ESC ID 0-7) =====

    // P3: 前左内 (Y=-1.6m) — CAN2 ESC 0,1  [v3.0: 原M5,M6位置]
    add_motor_raw(AP_MOTORS_MOT_9,  TG700_ROLL_LEFT_INNER,  TG700_PITCH_FRONT, TG700_YAW_INNER_CW,   9, TG700_THROTTLE_FACTOR);  // Upper CW
    add_motor_raw(AP_MOTORS_MOT_10, TG700_ROLL_LEFT_INNER,  TG700_PITCH_FRONT, TG700_YAW_INNER_CCW, 10, TG700_THROTTLE_FACTOR);  // Lower CCW

    // P4: 前左外 (Y=-3.4m) — CAN2 ESC 2,3  [v3.0: 原M7,M8位置]
    add_motor_raw(AP_MOTORS_MOT_11, TG700_ROLL_LEFT_OUTER,  TG700_PITCH_FRONT, TG700_YAW_OUTER_CW,  11, TG700_THROTTLE_FACTOR);  // Upper CW
    add_motor_raw(AP_MOTORS_MOT_12, TG700_ROLL_LEFT_OUTER,  TG700_PITCH_FRONT, TG700_YAW_OUTER_CCW, 12, TG700_THROTTLE_FACTOR);  // Lower CCW

    // P7: 后左内 (Y=-1.6m) — CAN2 ESC 4,5
    add_motor_raw(AP_MOTORS_MOT_13, TG700_ROLL_LEFT_INNER,  TG700_PITCH_REAR,  TG700_YAW_INNER_CW,  13, TG700_THROTTLE_FACTOR);  // Upper CW
    add_motor_raw(AP_MOTORS_MOT_14, TG700_ROLL_LEFT_INNER,  TG700_PITCH_REAR,  TG700_YAW_INNER_CCW, 14, TG700_THROTTLE_FACTOR);  // Lower CCW

    // P8: 后左外 (Y=-3.4m) — CAN2 ESC 6,7
    add_motor_raw(AP_MOTORS_MOT_15, TG700_ROLL_LEFT_OUTER,  TG700_PITCH_REAR,  TG700_YAW_OUTER_CW,  15, TG700_THROTTLE_FACTOR);  // Upper CW
    add_motor_raw(AP_MOTORS_MOT_16, TG700_ROLL_LEFT_OUTER,  TG700_PITCH_REAR,  TG700_YAW_OUTER_CCW, 16, TG700_THROTTLE_FACTOR);  // Lower CCW

    return true;
}

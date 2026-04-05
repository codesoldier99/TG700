/*
 * ============================================================================
 * TG700 PATCH: AP_MotorsMatrix_TG700.cpp  (v4.0)
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
 * 日期: 2026-04-05
 * 版本: v4.0 (fix M5-M8/M9-M12 factors to match actual SERVO routing)
 * 基于: ArduPilot Plane 4.5+ / AP_MotorsMatrix
 * ============================================================================
 */

#include "AP_MotorsMatrix.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;

/*
 * ============================================================================
 * 天工700 电机布局定义 v4.0 (俯视图, 从上方看, 机头朝上)
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

    // ===== Motor1-4 → SERVO5-8 → CAN1 ESC 0-3 → P1,P2 前右 =====

    // P1: 前右外 (Y=+3.4m) — CAN1 ESC 0,1
    add_motor_raw(AP_MOTORS_MOT_1,  TG700_ROLL_RIGHT_OUTER, TG700_PITCH_FRONT, TG700_YAW_OUTER_CW,   1, TG700_THROTTLE_FACTOR);  // Upper CW
    add_motor_raw(AP_MOTORS_MOT_2,  TG700_ROLL_RIGHT_OUTER, TG700_PITCH_FRONT, TG700_YAW_OUTER_CCW,  2, TG700_THROTTLE_FACTOR);  // Lower CCW

    // P2: 前右内 (Y=+1.6m) — CAN1 ESC 2,3
    add_motor_raw(AP_MOTORS_MOT_3,  TG700_ROLL_RIGHT_INNER, TG700_PITCH_FRONT, TG700_YAW_INNER_CW,   3, TG700_THROTTLE_FACTOR);  // Upper CW
    add_motor_raw(AP_MOTORS_MOT_4,  TG700_ROLL_RIGHT_INNER, TG700_PITCH_FRONT, TG700_YAW_INNER_CCW,  4, TG700_THROTTLE_FACTOR);  // Lower CCW

    // ===== Motor5-8 → SERVO13-16 → CAN2 ESC 0-3 → P3,P4 前左 =====
    // (SERVO函数37-40被分配到SERVO13-16, 路由到CAN2左翼前排)

    // P3: 前左内 (Y=-1.6m) — CAN2 ESC 0,1
    add_motor_raw(AP_MOTORS_MOT_5,  TG700_ROLL_LEFT_INNER,  TG700_PITCH_FRONT, TG700_YAW_INNER_CW,   5, TG700_THROTTLE_FACTOR);  // Upper CW
    add_motor_raw(AP_MOTORS_MOT_6,  TG700_ROLL_LEFT_INNER,  TG700_PITCH_FRONT, TG700_YAW_INNER_CCW,  6, TG700_THROTTLE_FACTOR);  // Lower CCW

    // P4: 前左外 (Y=-3.4m) — CAN2 ESC 2,3
    add_motor_raw(AP_MOTORS_MOT_7,  TG700_ROLL_LEFT_OUTER,  TG700_PITCH_FRONT, TG700_YAW_OUTER_CW,   7, TG700_THROTTLE_FACTOR);  // Upper CW
    add_motor_raw(AP_MOTORS_MOT_8,  TG700_ROLL_LEFT_OUTER,  TG700_PITCH_FRONT, TG700_YAW_OUTER_CCW,  8, TG700_THROTTLE_FACTOR);  // Lower CCW

    // ===== Motor9-12 → SERVO9-12 → CAN1 ESC 4-7 → P5,P6 后右 =====
    // (SERVO函数82-85被分配到SERVO9-12, 路由到CAN1右翼后排)

    // P5: 后右外 (Y=+3.4m) — CAN1 ESC 4,5
    add_motor_raw(AP_MOTORS_MOT_9,  TG700_ROLL_RIGHT_OUTER, TG700_PITCH_REAR,  TG700_YAW_OUTER_CW,   9, TG700_THROTTLE_FACTOR);  // Upper CW
    add_motor_raw(AP_MOTORS_MOT_10, TG700_ROLL_RIGHT_OUTER, TG700_PITCH_REAR,  TG700_YAW_OUTER_CCW, 10, TG700_THROTTLE_FACTOR);  // Lower CCW

    // P6: 后右内 (Y=+1.6m) — CAN1 ESC 6,7
    add_motor_raw(AP_MOTORS_MOT_11, TG700_ROLL_RIGHT_INNER, TG700_PITCH_REAR,  TG700_YAW_INNER_CW,  11, TG700_THROTTLE_FACTOR);  // Upper CW
    add_motor_raw(AP_MOTORS_MOT_12, TG700_ROLL_RIGHT_INNER, TG700_PITCH_REAR,  TG700_YAW_INNER_CCW, 12, TG700_THROTTLE_FACTOR);  // Lower CCW

    // ===== Motor13-16 → SERVO17-20 → CAN2 ESC 4-7 → P7,P8 后左 =====

    // P7: 后左内 (Y=-1.6m) — CAN2 ESC 4,5
    add_motor_raw(AP_MOTORS_MOT_13, TG700_ROLL_LEFT_INNER,  TG700_PITCH_REAR,  TG700_YAW_INNER_CW,  13, TG700_THROTTLE_FACTOR);  // Upper CW
    add_motor_raw(AP_MOTORS_MOT_14, TG700_ROLL_LEFT_INNER,  TG700_PITCH_REAR,  TG700_YAW_INNER_CCW, 14, TG700_THROTTLE_FACTOR);  // Lower CCW

    // P8: 后左外 (Y=-3.4m) — CAN2 ESC 6,7
    add_motor_raw(AP_MOTORS_MOT_15, TG700_ROLL_LEFT_OUTER,  TG700_PITCH_REAR,  TG700_YAW_OUTER_CW,  15, TG700_THROTTLE_FACTOR);  // Upper CW
    add_motor_raw(AP_MOTORS_MOT_16, TG700_ROLL_LEFT_OUTER,  TG700_PITCH_REAR,  TG700_YAW_OUTER_CCW, 16, TG700_THROTTLE_FACTOR);  // Lower CCW

    return true;
}

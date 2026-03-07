/*
 * ============================================================================
 * TG700 PATCH #2: AP_MotorsMatrix_TG700.cpp  (v2.1 - Bug Fixed)
 * 
 * 天工-700 十六旋翼共轴复合翼 固件级电机混控
 * TianGong-700 16-Motor Coaxial Compound-Wing Firmware Motor Mixing
 * 
 * 文件路径: libraries/AP_Motors/AP_MotorsMatrix_TG700.cpp
 * 
 * ============================================================================
 * 【v2.1 BUG修正记录】
 * 
 * 1. [CRITICAL] Roll因子符号修正:
 *    v2.0 错误地将右侧电机设为正值、左侧设为负值。
 *    ArduPilot惯例: 右侧=负值(正Roll时减推), 左侧=正值(正Roll时加推)。
 *    验证: add_motor()中 roll=cos(angle+90°)=-sin(angle),
 *    角度90°(右)→roll=-1.0, 角度-90°(左)→roll=+1.0。
 * 
 * 2. [CRITICAL] 删除不存在的remove_all_motors()调用,
 *    setup_motors()已在调用前清除所有电机。
 * 
 * 3. [IMPORTANT] 添加_frame_class_string, _frame_type_string, _mav_type。
 * ============================================================================
 * 
 * 设计原则:
 *   - 混控逻辑运行在ArduPilot 400Hz主控制循环,零额外延迟
 *   - 消除Lua脚本50Hz低优先级线程的单点故障
 *   - 精确处理内外双排布局的差异化Roll和Yaw因子
 *   - 支持共轴对转桨的正确扭矩配对
 * 
 * 作者: TG700 Flight Control Team
 * 日期: 2026-03-07
 * 版本: v2.1 (bug-fixed)
 * 基于: ArduPilot Plane 4.5+ / AP_MotorsMatrix
 * ============================================================================
 */

#include "AP_MotorsMatrix.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;

/*
 * ============================================================================
 * 天工700 电机布局定义 v2.1 (俯视图)
 * 
 * 坐标系: ArduPilot NED体轴系
 *   X轴: 机头方向为正 (前)
 *   Y轴: 右翼方向为正 (右)
 *   Z轴: 向下为正
 * 
 *                     机头方向 (+X)
 *                           ^
 *                           |
 *   P4(L外) P3(L内)          |       P2(R内) P1(R外)  ← 前排 (Front Row)
 *      |       |            |          |       |
 *      |       |          [CG]         |       |      ← 4根纵梁
 *      |       |            |          |       |
 *   P8(L外) P7(L内)          |       P6(R内) P5(R外)  ← 后排 (Rear Row)
 *                           |
 *                           v
 *                      机尾方向 (-X)
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
    // v2.1: 设置帧信息 (v2.0缺失)
    _frame_class_string = "TG700";
    _frame_type_string = "COAX16";
    _mav_type = MAV_TYPE_GENERIC;

    // ===== 前排 (Front Row, X = +1.665m) =====

    // P1: 前右外 (Y=+3.4m)
    add_motor_raw(AP_MOTORS_MOT_1,  TG700_ROLL_RIGHT_OUTER, TG700_PITCH_FRONT, TG700_YAW_OUTER_CW,   1, TG700_THROTTLE_FACTOR);  // Upper CW
    add_motor_raw(AP_MOTORS_MOT_2,  TG700_ROLL_RIGHT_OUTER, TG700_PITCH_FRONT, TG700_YAW_OUTER_CCW,  2, TG700_THROTTLE_FACTOR);  // Lower CCW

    // P2: 前右内 (Y=+1.6m)
    add_motor_raw(AP_MOTORS_MOT_3,  TG700_ROLL_RIGHT_INNER, TG700_PITCH_FRONT, TG700_YAW_INNER_CW,   3, TG700_THROTTLE_FACTOR);  // Upper CW
    add_motor_raw(AP_MOTORS_MOT_4,  TG700_ROLL_RIGHT_INNER, TG700_PITCH_FRONT, TG700_YAW_INNER_CCW,  4, TG700_THROTTLE_FACTOR);  // Lower CCW

    // P3: 前左内 (Y=-1.6m)
    add_motor_raw(AP_MOTORS_MOT_5,  TG700_ROLL_LEFT_INNER,  TG700_PITCH_FRONT, TG700_YAW_INNER_CW,   5, TG700_THROTTLE_FACTOR);  // Upper CW
    add_motor_raw(AP_MOTORS_MOT_6,  TG700_ROLL_LEFT_INNER,  TG700_PITCH_FRONT, TG700_YAW_INNER_CCW,  6, TG700_THROTTLE_FACTOR);  // Lower CCW

    // P4: 前左外 (Y=-3.4m)
    add_motor_raw(AP_MOTORS_MOT_7,  TG700_ROLL_LEFT_OUTER,  TG700_PITCH_FRONT, TG700_YAW_OUTER_CW,   7, TG700_THROTTLE_FACTOR);  // Upper CW
    add_motor_raw(AP_MOTORS_MOT_8,  TG700_ROLL_LEFT_OUTER,  TG700_PITCH_FRONT, TG700_YAW_OUTER_CCW,  8, TG700_THROTTLE_FACTOR);  // Lower CCW

    // ===== 后排 (Rear Row, X = -1.665m) =====

    // P5: 后右外 (Y=+3.4m)
    add_motor_raw(AP_MOTORS_MOT_9,  TG700_ROLL_RIGHT_OUTER, TG700_PITCH_REAR, TG700_YAW_OUTER_CW,   9, TG700_THROTTLE_FACTOR);  // Upper CW
    add_motor_raw(AP_MOTORS_MOT_10, TG700_ROLL_RIGHT_OUTER, TG700_PITCH_REAR, TG700_YAW_OUTER_CCW, 10, TG700_THROTTLE_FACTOR);  // Lower CCW

    // P6: 后右内 (Y=+1.6m)
    add_motor_raw(AP_MOTORS_MOT_11, TG700_ROLL_RIGHT_INNER, TG700_PITCH_REAR, TG700_YAW_INNER_CW,  11, TG700_THROTTLE_FACTOR);  // Upper CW
    add_motor_raw(AP_MOTORS_MOT_12, TG700_ROLL_RIGHT_INNER, TG700_PITCH_REAR, TG700_YAW_INNER_CCW, 12, TG700_THROTTLE_FACTOR);  // Lower CCW

    // P7: 后左内 (Y=-1.6m)
    add_motor_raw(AP_MOTORS_MOT_13, TG700_ROLL_LEFT_INNER,  TG700_PITCH_REAR, TG700_YAW_INNER_CW,  13, TG700_THROTTLE_FACTOR);  // Upper CW
    add_motor_raw(AP_MOTORS_MOT_14, TG700_ROLL_LEFT_INNER,  TG700_PITCH_REAR, TG700_YAW_INNER_CCW, 14, TG700_THROTTLE_FACTOR);  // Lower CCW

    // P8: 后左外 (Y=-3.4m)
    add_motor_raw(AP_MOTORS_MOT_15, TG700_ROLL_LEFT_OUTER,  TG700_PITCH_REAR, TG700_YAW_OUTER_CW,  15, TG700_THROTTLE_FACTOR);  // Upper CW
    add_motor_raw(AP_MOTORS_MOT_16, TG700_ROLL_LEFT_OUTER,  TG700_PITCH_REAR, TG700_YAW_OUTER_CCW, 16, TG700_THROTTLE_FACTOR);  // Lower CCW

    return true;
}

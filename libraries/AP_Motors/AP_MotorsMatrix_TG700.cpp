/*
 * ============================================================================
 * TG700 PATCH #2: AP_MotorsMatrix_TG700.cpp  (v2.0)
 * 
 * 天工-700 十六旋翼共轴复合翼 固件级电机混控
 * TianGong-700 16-Motor Coaxial Compound-Wing Firmware Motor Mixing
 * 
 * 文件路径: libraries/AP_Motors/AP_MotorsMatrix_TG700.cpp
 * 
 * ============================================================================
 * 【v2.0 关键修正】
 * 
 * v1.0 错误地将布局描述为"串列四翼"(4排×2列),
 * 实际结构为: 前后2排 × 左右4列 (内外双排), 4根纵梁连接。
 * 
 * v2.0 修正内容:
 *   - 布局改为 2排(前/后) × 4列(左外/左内/右内/右外)
 *   - Roll因子分化: 外侧±1.0, 内侧±0.4706 (基于实测Y坐标比)
 *   - Pitch因子统一: 前排+1.0, 后排-1.0 (前后对称)
 *   - Yaw因子重新计算: 外侧±0.0971, 内侧±0.0592
 *   - 电机编号重新映射: P1-P4前排, P5-P8后排
 * ============================================================================
 * 
 * 设计原则:
 *   - 混控逻辑运行在ArduPilot 400Hz主控制循环,零额外延迟
 *   - 消除Lua脚本50Hz低优先级线程的单点故障
 *   - 精确处理内外双排布局的差异化Roll和Yaw因子
 *   - 支持共轴对转桨的正确扭矩配对
 * 
 * 作者: TG700 Flight Control Team
 * 日期: 2026-02-10
 * 版本: v2.0
 * 基于: ArduPilot Plane 4.5+ / AP_MotorsMatrix
 * ============================================================================
 */

#include "AP_MotorsMatrix.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;

/*
 * ============================================================================
 * 天工700 电机布局定义 v2.0 (俯视图)
 * 
 * 【关键修正】v1.0版本错误地将P1-P2/P3-P4描述为横轴(翼展方向)连接。
 * 实际上 P1与P5、P2与P6、P3与P7、P4与P8 之间通过纵轴(机身方向)纵梁
 * 连接,形成4根前后纵梁,每根纵梁前后各一个共轴电机位置。
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
 *   翼展:               11.0 m
 *   P1-P4 间距 (外侧):   6.8 m  → Y_outer = ±3.4 m
 *   P2-P3 间距 (内侧):   3.2 m  → Y_inner = ±1.6 m
 *   P1-P5 间距 (前后):   3.33 m → X_front = +1.665 m, X_rear = -1.665 m
 * 
 * 4根纵梁连接关系:
 *   纵梁1 (右外): P1 ←→ P5    Y = +3.4 m
 *   纵梁2 (右内): P2 ←→ P6    Y = +1.6 m
 *   纵梁3 (左内): P3 ←→ P7    Y = -1.6 m
 *   纵梁4 (左外): P4 ←→ P8    Y = -3.4 m
 * 
 * 每个位置P有上(Upper)和下(Lower)两个共轴电机:
 *   Upper = CW旋转 (俯视顺时针)
 *   Lower = CCW旋转 (俯视逆时针)
 * 
 * 电机编号映射 (Motor Index → Position):
 *   M1  = P1 Upper (前右外上, CW)     M2  = P1 Lower (前右外下, CCW)
 *   M3  = P2 Upper (前右内上, CW)     M4  = P2 Lower (前右内下, CCW)
 *   M5  = P3 Upper (前左内上, CW)     M6  = P3 Lower (前左内下, CCW)
 *   M7  = P4 Upper (前左外上, CW)     M8  = P4 Lower (前左外下, CCW)
 *   M9  = P5 Upper (后右外上, CW)     M10 = P5 Lower (后右外下, CCW)
 *   M11 = P6 Upper (后右内上, CW)     M12 = P6 Lower (后右内下, CCW)
 *   M13 = P7 Upper (后左内上, CW)     M14 = P7 Lower (后左内下, CCW)
 *   M15 = P8 Upper (后左外上, CW)     M16 = P8 Lower (后左外下, CCW)
 * 
 * ============================================================================
 */

/*
 * ============================================================================
 * 关键参数定义 (v2.0 全部基于实测几何重新计算)
 * ============================================================================
 */

// ---------- 物理安装角度 (度) ----------
// 切向倾斜: 产生偏航力矩的主要手段
#define TG700_TILT_TANGENTIAL_DEG   5.0f

// 内倾角度: 提供被动位置恢复力
#define TG700_TILT_INWARD_DEG       2.0f

// ---------- 预计算三角函数值 ----------
// sin(5°) = 0.08716
#define TG700_SIN_TANG              0.08716f
// cos(5°) = 0.99619
#define TG700_COS_TANG              0.99619f
// sin(2°) = 0.03490
#define TG700_SIN_INWARD            0.03490f
// cos(2°) = 0.99939
#define TG700_COS_INWARD            0.99939f

// ---------- 油门因子 (有效垂直推力比) ----------
// throttle_factor = cos(5°) × cos(2°) ≈ 0.9956
// 略低于1.0是因为推力矢量不完全垂直
#define TG700_THROTTLE_FACTOR       0.9956f

// ---------- 实测几何坐标 ----------
// Y方向 (翼展方向):
//   外侧电机 Y = ±3.4 m   (P1/P4/P5/P8)
//   内侧电机 Y = ±1.6 m   (P2/P3/P6/P7)
// X方向 (纵轴方向):
//   前排 X = +1.665 m      (P1/P2/P3/P4)
//   后排 X = -1.665 m      (P5/P6/P7/P8)
//
// 归一化基准: Y_max = 3.4m (外侧翼尖为归一化基准)
//
#define TG700_Y_OUTER               3.4f    // 外侧电机Y坐标绝对值 (m)
#define TG700_Y_INNER               1.6f    // 内侧电机Y坐标绝对值 (m)
#define TG700_X_FRONT               1.665f  // 前排X坐标 (m)
#define TG700_X_REAR                1.665f  // 后排X坐标绝对值 (m)

// ---------- Roll因子 (基于Y坐标归一化) ----------
// 归一化: roll_fac = Y / Y_max
// 右侧为正, 左侧为负
#define TG700_ROLL_RIGHT_OUTER      ( 1.0f)     // +3.4/3.4 = +1.0
#define TG700_ROLL_RIGHT_INNER      ( 0.4706f)  // +1.6/3.4 = +0.4706
#define TG700_ROLL_LEFT_INNER       (-0.4706f)  // -1.6/3.4 = -0.4706
#define TG700_ROLL_LEFT_OUTER       (-1.0f)     // -3.4/3.4 = -1.0

// ---------- Pitch因子 (前后对称) ----------
// 前排和后排距CG等距 (各1.665m), 归一化为±1.0
// 前方为正 (机头抬起时前排电机增推)
#define TG700_PITCH_FRONT           ( 1.0f)     // 前排
#define TG700_PITCH_REAR            (-1.0f)     // 后排

// ---------- 偏航因子 (v2.0 差异化: 外侧 vs 内侧) ----------
//
// 偏航力矩 = 推力 × sin(切向倾斜角) × 到CG的距离
//
// 各位置到CG的距离:
//   外侧(P1/P4/P5/P8): r = sqrt(1.665² + 3.4²) = sqrt(2.772+11.56) = 3.786 m
//   内侧(P2/P3/P6/P7): r = sqrt(1.665² + 1.6²) = sqrt(2.772+2.56)  = 2.309 m
//
// 偏航因子归一化 (以Y_max=3.4m为基准, 与Roll因子一致):
//   外侧: sin(5°) × 3.786/3.4 = 0.08716 × 1.1135 = 0.0971
//   内侧: sin(5°) × 2.309/3.4 = 0.08716 × 0.6791 = 0.0592
//
// 外/内比值: 0.0971/0.0592 = 1.641 (外侧电机偏航贡献大64.1%)
//
// 符号约定:
//   CW旋转 (上层电机) → 产生负偏航力矩 → 负值
//   CCW旋转 (下层电机) → 产生正偏航力矩 → 正值
//

#define TG700_YAW_OUTER_CW         (-0.0971f)   // 外侧CW上层电机
#define TG700_YAW_OUTER_CCW        ( 0.0971f)   // 外侧CCW下层电机
#define TG700_YAW_INNER_CW         (-0.0592f)   // 内侧CW上层电机
#define TG700_YAW_INNER_CCW        ( 0.0592f)   // 内侧CCW下层电机


/*
 * ============================================================================
 * setup_motors_tg700()
 * 
 * 在 AP_MotorsMatrix::setup_motors() 的 switch 语句中调用:
 * 
 *   case MOTOR_FRAME_TG700:
 *       setup_motors_tg700();
 *       break;
 * ============================================================================
 */

void AP_MotorsMatrix::setup_motors_tg700()
{
    // 清除所有已有电机配置
    remove_all_motors();

    /*
     * ========================================================================
     * 前排电机 (Front Row, X = +1.665m)
     * ========================================================================
     */

    /*
     * Position 1: 前右外 (Front Right Outer) - 外侧位置
     * 纵梁1: P1 ←→ P5, Y = +3.4m
     */
    // M1: P1 Upper, CW旋转
    add_motor_raw(
        AP_MOTORS_MOT_1,                // motor_num: 0
        TG700_ROLL_RIGHT_OUTER,         // roll_fac:  +1.0 (右外侧)
        TG700_PITCH_FRONT,              // pitch_fac: +1.0 (前排)
        TG700_YAW_OUTER_CW,            // yaw_fac:   -0.0971 (CW, 外侧力臂)
        1,                              // testing_order
        TG700_THROTTLE_FACTOR           // throttle_factor: 0.9956
    );

    // M2: P1 Lower, CCW旋转
    add_motor_raw(
        AP_MOTORS_MOT_2,                // motor_num: 1
        TG700_ROLL_RIGHT_OUTER,         // roll_fac:  +1.0
        TG700_PITCH_FRONT,              // pitch_fac: +1.0
        TG700_YAW_OUTER_CCW,           // yaw_fac:   +0.0971 (CCW)
        2,                              // testing_order
        TG700_THROTTLE_FACTOR
    );

    /*
     * Position 2: 前右内 (Front Right Inner) - 内侧位置
     * 纵梁2: P2 ←→ P6, Y = +1.6m
     */
    // M3: P2 Upper, CW旋转
    add_motor_raw(
        AP_MOTORS_MOT_3,                // motor_num: 2
        TG700_ROLL_RIGHT_INNER,         // roll_fac:  +0.4706 (右内侧)
        TG700_PITCH_FRONT,              // pitch_fac: +1.0 (前排)
        TG700_YAW_INNER_CW,            // yaw_fac:   -0.0592 (CW, 内侧力臂)
        3,                              // testing_order
        TG700_THROTTLE_FACTOR
    );

    // M4: P2 Lower, CCW旋转
    add_motor_raw(
        AP_MOTORS_MOT_4,                // motor_num: 3
        TG700_ROLL_RIGHT_INNER,         // roll_fac:  +0.4706
        TG700_PITCH_FRONT,              // pitch_fac: +1.0
        TG700_YAW_INNER_CCW,           // yaw_fac:   +0.0592 (CCW)
        4,                              // testing_order
        TG700_THROTTLE_FACTOR
    );

    /*
     * Position 3: 前左内 (Front Left Inner) - 内侧位置
     * 纵梁3: P3 ←→ P7, Y = -1.6m
     */
    // M5: P3 Upper, CW旋转
    add_motor_raw(
        AP_MOTORS_MOT_5,                // motor_num: 4
        TG700_ROLL_LEFT_INNER,          // roll_fac:  -0.4706 (左内侧)
        TG700_PITCH_FRONT,              // pitch_fac: +1.0 (前排)
        TG700_YAW_INNER_CW,            // yaw_fac:   -0.0592
        5,                              // testing_order
        TG700_THROTTLE_FACTOR
    );

    // M6: P3 Lower, CCW旋转
    add_motor_raw(
        AP_MOTORS_MOT_6,                // motor_num: 5
        TG700_ROLL_LEFT_INNER,          // roll_fac:  -0.4706
        TG700_PITCH_FRONT,              // pitch_fac: +1.0
        TG700_YAW_INNER_CCW,           // yaw_fac:   +0.0592
        6,                              // testing_order
        TG700_THROTTLE_FACTOR
    );

    /*
     * Position 4: 前左外 (Front Left Outer) - 外侧位置
     * 纵梁4: P4 ←→ P8, Y = -3.4m
     */
    // M7: P4 Upper, CW旋转
    add_motor_raw(
        AP_MOTORS_MOT_7,                // motor_num: 6
        TG700_ROLL_LEFT_OUTER,          // roll_fac:  -1.0 (左外侧)
        TG700_PITCH_FRONT,              // pitch_fac: +1.0 (前排)
        TG700_YAW_OUTER_CW,            // yaw_fac:   -0.0971
        7,                              // testing_order
        TG700_THROTTLE_FACTOR
    );

    // M8: P4 Lower, CCW旋转
    add_motor_raw(
        AP_MOTORS_MOT_8,                // motor_num: 7
        TG700_ROLL_LEFT_OUTER,          // roll_fac:  -1.0
        TG700_PITCH_FRONT,              // pitch_fac: +1.0
        TG700_YAW_OUTER_CCW,           // yaw_fac:   +0.0971
        8,                              // testing_order
        TG700_THROTTLE_FACTOR
    );


    /*
     * ========================================================================
     * 后排电机 (Rear Row, X = -1.665m)
     * ========================================================================
     */

    /*
     * Position 5: 后右外 (Rear Right Outer) - 外侧位置
     * 纵梁1: P1 ←→ P5, Y = +3.4m
     */
    // M9: P5 Upper, CW旋转
    add_motor_raw(
        AP_MOTORS_MOT_9,                // motor_num: 8
        TG700_ROLL_RIGHT_OUTER,         // roll_fac:  +1.0
        TG700_PITCH_REAR,              // pitch_fac: -1.0 (后排)
        TG700_YAW_OUTER_CW,            // yaw_fac:   -0.0971
        9,                              // testing_order
        TG700_THROTTLE_FACTOR
    );

    // M10: P5 Lower, CCW旋转
    add_motor_raw(
        AP_MOTORS_MOT_10,               // motor_num: 9
        TG700_ROLL_RIGHT_OUTER,         // roll_fac:  +1.0
        TG700_PITCH_REAR,              // pitch_fac: -1.0
        TG700_YAW_OUTER_CCW,           // yaw_fac:   +0.0971
        10,                             // testing_order
        TG700_THROTTLE_FACTOR
    );

    /*
     * Position 6: 后右内 (Rear Right Inner) - 内侧位置
     * 纵梁2: P2 ←→ P6, Y = +1.6m
     */
    // M11: P6 Upper, CW旋转
    add_motor_raw(
        AP_MOTORS_MOT_11,               // motor_num: 10
        TG700_ROLL_RIGHT_INNER,         // roll_fac:  +0.4706
        TG700_PITCH_REAR,              // pitch_fac: -1.0 (后排)
        TG700_YAW_INNER_CW,            // yaw_fac:   -0.0592
        11,                             // testing_order
        TG700_THROTTLE_FACTOR
    );

    // M12: P6 Lower, CCW旋转
    add_motor_raw(
        AP_MOTORS_MOT_12,               // motor_num: 11
        TG700_ROLL_RIGHT_INNER,         // roll_fac:  +0.4706
        TG700_PITCH_REAR,              // pitch_fac: -1.0
        TG700_YAW_INNER_CCW,           // yaw_fac:   +0.0592
        12,                             // testing_order
        TG700_THROTTLE_FACTOR
    );

    /*
     * Position 7: 后左内 (Rear Left Inner) - 内侧位置
     * 纵梁3: P3 ←→ P7, Y = -1.6m
     */
    // M13: P7 Upper, CW旋转
    add_motor_raw(
        AP_MOTORS_MOT_13,               // motor_num: 12
        TG700_ROLL_LEFT_INNER,          // roll_fac:  -0.4706
        TG700_PITCH_REAR,              // pitch_fac: -1.0 (后排)
        TG700_YAW_INNER_CW,            // yaw_fac:   -0.0592
        13,                             // testing_order
        TG700_THROTTLE_FACTOR
    );

    // M14: P7 Lower, CCW旋转
    add_motor_raw(
        AP_MOTORS_MOT_14,               // motor_num: 13
        TG700_ROLL_LEFT_INNER,          // roll_fac:  -0.4706
        TG700_PITCH_REAR,              // pitch_fac: -1.0
        TG700_YAW_INNER_CCW,           // yaw_fac:   +0.0592
        14,                             // testing_order
        TG700_THROTTLE_FACTOR
    );

    /*
     * Position 8: 后左外 (Rear Left Outer) - 外侧位置
     * 纵梁4: P4 ←→ P8, Y = -3.4m
     */
    // M15: P8 Upper, CW旋转
    add_motor_raw(
        AP_MOTORS_MOT_15,               // motor_num: 14
        TG700_ROLL_LEFT_OUTER,          // roll_fac:  -1.0
        TG700_PITCH_REAR,              // pitch_fac: -1.0 (后排)
        TG700_YAW_OUTER_CW,            // yaw_fac:   -0.0971
        15,                             // testing_order
        TG700_THROTTLE_FACTOR
    );

    // M16: P8 Lower, CCW旋转
    add_motor_raw(
        AP_MOTORS_MOT_16,               // motor_num: 15
        TG700_ROLL_LEFT_OUTER,          // roll_fac:  -1.0
        TG700_PITCH_REAR,              // pitch_fac: -1.0
        TG700_YAW_OUTER_CCW,           // yaw_fac:   +0.0971
        16,                             // testing_order
        TG700_THROTTLE_FACTOR
    );
}

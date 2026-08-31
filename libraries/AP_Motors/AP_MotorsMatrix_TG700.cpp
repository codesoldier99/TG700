/*
 * ============================================================================
 * TG700 PATCH: AP_MotorsMatrix_TG700.cpp  (v7.0)
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
 * 版本: v7.0 (两种转向方案由 Q_FRAME_TYPE 选择; 均为 ±X/±Y 四向倾角约束)
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
 * 两种转向方案 (由 Q_FRAME_TYPE 选择)
 * ============================================================================
 *
 * 共同前提: 电机座结构上只能沿机体 X 轴或 Y 轴倾斜 (朝机头/机尾/左/右四者之一),
 *           做不到"纯切向"(垂直于中心-电机连线)。四向里取力矩臂最大的:
 *
 *             水平力沿 ±X 时, 偏航力矩臂 = |Y|;  沿 ±Y 时, 力矩臂 = |X|
 *             外侧: |Y| = 3.400 > |X| = 1.665  -> 外侧 4 组沿 ±X (机头/机尾)
 *             内侧: |Y| = 1.600 < |X| = 1.665  -> 内侧 4 组沿 ±Y (左/右)
 *
 *           两方案的偏航权限完全相等 (解析证明比值恒为 1.0000): 每台电机的
 *           偏航贡献都是 sin(倾角)×力矩臂 + 反扭矩, 与转向排布方式无关。
 *           差别只在"哪些电机同号"以及由此带来的安装/硬件代价。
 *
 * ---------------------------------------------------------------------------
 * 方案 A — 对转 (Q_FRAME_TYPE = 0 或 1, 机架串 "COAX16_AX4")  【默认, 沿用 v6.0】
 * ---------------------------------------------------------------------------
 *   转向: 每组上层 CW / 下层 CCW  (= 现役硬件状态, 桨与 ESC 无需改动)
 *   偏航: 由同一共轴组内上下电机的差动推力产生 -> 组内推力增量恒等值反号,
 *         因此打偏航舵**完全不改变**该组总推力, 对 roll/pitch/升力零扰动。
 *   倾角: 同组上下两台必须**反向** (见下表), 这是本方案唯一的易错点。
 *
 *   臂位        坐标(X,Y)        倾斜轴   上层 CW    下层 CCW
 *   P1 前右外  (+1.665, +3.4)   ±X      机头       机尾
 *   P2 前右内  (+1.665, +1.6)   ±Y      左         右
 *   P3 前左内  (+1.665, -1.6)   ±Y      左         右
 *   P4 前左外  (+1.665, -3.4)   ±X      机尾       机头
 *   P5 后右外  (-1.665, +3.4)   ±X      机头       机尾
 *   P6 后右内  (-1.665, +1.6)   ±Y      右         左
 *   P7 后左内  (-1.665, -1.6)   ±Y      右         左
 *   P8 后左外  (-1.665, -3.4)   ±X      机尾       机头
 *
 * ---------------------------------------------------------------------------
 * 方案 B — 同转棋盘 (Q_FRAME_TYPE = 20 或 21, 机架串 "COAX16_COR")   【v5.0】
 * ---------------------------------------------------------------------------
 *   转向: 每组上下两台**同向**, 组间棋盘交替
 *           CW  组: P1, P3, P6, P8
 *           CCW 组: P2, P4, P5, P7
 *         每种转向各含 2 个外侧 + 2 个内侧组, 故 roll/pitch/升力仍精确平衡。
 *   偏航: 由**组间**差动推力产生 -> 打偏航舵会在机体上重新分配推力, 虽然
 *         线性耦合为零, 但单台电机失效时的姿态扰动比方案 A 大。
 *   倾角: 同组上下两台**同向**, 且规则极简 —— 全部朝外:
 *
 *           外侧 4 组沿 ±X, 一律背离重心 (前排朝机头, 后排朝机尾)
 *           内侧 4 组沿 ±Y, 一律背离中线 (右侧朝右,   左侧朝左)
 *
 *   臂位        坐标(X,Y)        转向   倾斜轴   上下两台同朝
 *   P1 前右外  (+1.665, +3.4)   CW     ±X      机头 (+X)
 *   P2 前右内  (+1.665, +1.6)   CCW    ±Y      右   (+Y)
 *   P3 前左内  (+1.665, -1.6)   CW     ±Y      左   (-Y)
 *   P4 前左外  (+1.665, -3.4)   CCW    ±X      机头 (+X)
 *   P5 后右外  (-1.665, +3.4)   CCW    ±X      机尾 (-X)
 *   P6 后右内  (-1.665, +1.6)   CW     ±Y      右   (+Y)
 *   P7 后左内  (-1.665, -1.6)   CCW    ±Y      左   (-Y)
 *   P8 后左外  (-1.665, -3.4)   CW     ±X      机尾 (-X)
 *
 *   硬件代价 (相对现役状态): 需要反转 8 台电机的转向 —— 即改 ESC 转向并换成
 *   反手桨: P1下, P2上, P3下, P4上, P5上, P6下, P7上, P8下。
 *   另需注意共轴同转会失去下桨回收上桨尾流旋流的收益, 悬停效率略降。
 *
 * ---------------------------------------------------------------------------
 * 提升偏航权限的唯一有效途径 (两方案相同)
 * ---------------------------------------------------------------------------
 *   加大倾角。5deg->10deg 得 1.97x, ->15deg 得 2.92x, ->18deg 得 3.48x;
 *   代价是升力损失 cos(theta): 10deg 损失 1.5% (700kg 折 10.6kg), 15deg 损失
 *   3.4% (23.9kg), 18deg 损失 4.9% (34.3kg)。
 * ============================================================================
 */

// ---------- 物理安装角度 ----------
// 单一倾角, 方向沿机体 ±X 或 ±Y (四向约束), 无向心分量
#define TG700_TILT_DEG              5.0f

// ---------- 预计算三角函数值 ----------
#define TG700_SIN_TILT              0.08716f   // sin(5°)
#define TG700_COS_TILT              0.99619f   // cos(5°)

// ---------- 油门因子 = cos(5°) ----------
// 全机一致, normalise_rpy_factors() 会把它归一化为 1.000, 仅作自证保留
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
// 倾角方向按对应方案的表安装后, 倾角力与该电机的反扭矩同号, 两者相加。
//
// 注意: AP_MotorsMatrix::normalise_rpy_factors() 会把 yaw 因子整体缩放到
//       max|yaw_fac| = 0.5, 因此这里只有"外侧:内侧"的比值 (0.0443/0.0888
//       = 0.499) 会影响最终混控; 绝对幅值仅用于自证推导。
#define TG700_YAW_OUTER             ( 0.0888f)
#define TG700_YAW_INNER             ( 0.0443f)

#define TG700_YO_CW                 (-TG700_YAW_OUTER)
#define TG700_YO_CCW                ( TG700_YAW_OUTER)
#define TG700_YI_CW                 (-TG700_YAW_INNER)
#define TG700_YI_CCW                ( TG700_YAW_INNER)


bool AP_MotorsMatrix::setup_motors_tg700(motor_frame_type frame_type)
{
    _frame_class_string = "TG700";
    _mav_type = MAV_TYPE_GENERIC;

    // 两种方案的 roll / pitch / 油门因子与测试序号完全相同, 只有 yaw 符号排布不同。
    // 把共同部分抽成一张表, 从结构上保证换方案时不可能改动非偏航轴。
    struct MotorGeom {
        uint8_t  motor;         // AP_MOTORS_MOT_x (0-based)
        float    roll;
        float    pitch;
        uint8_t  order;         // 电机测试序号
    };
    static const MotorGeom geom[16] = {
        // ---- Motor1-4 → SERVO5-8 → CAN1 ESC 0-3 → P1,P2 前右 ----
        { AP_MOTORS_MOT_1,  TG700_ROLL_RIGHT_OUTER, TG700_PITCH_FRONT,  1 },  // P1 上
        { AP_MOTORS_MOT_2,  TG700_ROLL_RIGHT_OUTER, TG700_PITCH_FRONT,  2 },  // P1 下
        { AP_MOTORS_MOT_3,  TG700_ROLL_RIGHT_INNER, TG700_PITCH_FRONT,  3 },  // P2 上
        { AP_MOTORS_MOT_4,  TG700_ROLL_RIGHT_INNER, TG700_PITCH_FRONT,  4 },  // P2 下
        // ---- Motor5-8 → SERVO13-16 → CAN2 ESC 0-3 → P3,P4 前左 ----
        { AP_MOTORS_MOT_5,  TG700_ROLL_LEFT_INNER,  TG700_PITCH_FRONT,  5 },  // P3 上
        { AP_MOTORS_MOT_6,  TG700_ROLL_LEFT_INNER,  TG700_PITCH_FRONT,  6 },  // P3 下
        { AP_MOTORS_MOT_7,  TG700_ROLL_LEFT_OUTER,  TG700_PITCH_FRONT,  7 },  // P4 上
        { AP_MOTORS_MOT_8,  TG700_ROLL_LEFT_OUTER,  TG700_PITCH_FRONT,  8 },  // P4 下
        // ---- Motor9-12 → SERVO9-12 → CAN1 ESC 4-7 → P5,P6 后右 ----
        { AP_MOTORS_MOT_9,  TG700_ROLL_RIGHT_OUTER, TG700_PITCH_REAR,   9 },  // P5 上
        { AP_MOTORS_MOT_10, TG700_ROLL_RIGHT_OUTER, TG700_PITCH_REAR,  10 },  // P5 下
        { AP_MOTORS_MOT_11, TG700_ROLL_RIGHT_INNER, TG700_PITCH_REAR,  11 },  // P6 上
        { AP_MOTORS_MOT_12, TG700_ROLL_RIGHT_INNER, TG700_PITCH_REAR,  12 },  // P6 下
        // ---- Motor13-16 → SERVO17-20 → CAN2 ESC 4-7 → P7,P8 后左 ----
        { AP_MOTORS_MOT_13, TG700_ROLL_LEFT_INNER,  TG700_PITCH_REAR,  13 },  // P7 上
        { AP_MOTORS_MOT_14, TG700_ROLL_LEFT_INNER,  TG700_PITCH_REAR,  14 },  // P7 下
        { AP_MOTORS_MOT_15, TG700_ROLL_LEFT_OUTER,  TG700_PITCH_REAR,  15 },  // P8 上
        { AP_MOTORS_MOT_16, TG700_ROLL_LEFT_OUTER,  TG700_PITCH_REAR,  16 },  // P8 下
    };

    // 方案 A: 对转 —— 组内上 CW / 下 CCW, 偏航由组内差动产生 (组内推力增量抵消)
    static const float yaw_counter_rotating[16] = {
        TG700_YO_CW,  TG700_YO_CCW,   // P1 上/下
        TG700_YI_CW,  TG700_YI_CCW,   // P2 上/下
        TG700_YI_CW,  TG700_YI_CCW,   // P3 上/下
        TG700_YO_CW,  TG700_YO_CCW,   // P4 上/下
        TG700_YO_CW,  TG700_YO_CCW,   // P5 上/下
        TG700_YI_CW,  TG700_YI_CCW,   // P6 上/下
        TG700_YI_CW,  TG700_YI_CCW,   // P7 上/下
        TG700_YO_CW,  TG700_YO_CCW,   // P8 上/下
    };

    // 方案 B: 同转棋盘 —— 组内同向, 组间交替; 偏航由组间差动产生
    //   CW 组 P1,P3,P6,P8 取负; CCW 组 P2,P4,P5,P7 取正
    static const float yaw_co_rotating[16] = {
        TG700_YO_CW,  TG700_YO_CW,    // P1 CW  (外)
        TG700_YI_CCW, TG700_YI_CCW,   // P2 CCW (内)
        TG700_YI_CW,  TG700_YI_CW,    // P3 CW  (内)
        TG700_YO_CCW, TG700_YO_CCW,   // P4 CCW (外)
        TG700_YO_CCW, TG700_YO_CCW,   // P5 CCW (外)
        TG700_YI_CW,  TG700_YI_CW,    // P6 CW  (内)
        TG700_YI_CCW, TG700_YI_CCW,   // P7 CCW (内)
        TG700_YO_CW,  TG700_YO_CW,    // P8 CW  (外)
    };

    const float *yaw = nullptr;
    switch (frame_type) {
    case MOTOR_FRAME_TYPE_PLUS:         // 0 — 兼容机上现有参数值
    case MOTOR_FRAME_TYPE_X:            // 1
        _frame_type_string = "COAX16_AX4";
        yaw = yaw_counter_rotating;
        break;
    case MOTOR_FRAME_TYPE_X_COR:        // 20 — X8 co-rotating, 语义与方案 B 一致
    case MOTOR_FRAME_TYPE_CW_X_COR:     // 21
        _frame_type_string = "COAX16_COR";
        yaw = yaw_co_rotating;
        break;
    default:
        // 不认识的机架类型: 让预解锁失败, 而不是静默套用某一方案的混控
        return false;
    }

    for (uint8_t i = 0; i < 16; i++) {
        add_motor_raw(geom[i].motor, geom[i].roll, geom[i].pitch,
                      yaw[i], geom[i].order, TG700_THROTTLE_FACTOR);
    }

    return true;
}

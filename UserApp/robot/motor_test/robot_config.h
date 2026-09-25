/*
 * robot_config.h — M3508 低速控制 (可调参)
 *
 * 本文件集中所有需要你实测/微调的参数。
 *
 * ============================ 使用说明 ============================
 * 机构是"高静摩擦"时 (低于某电流不动、高于就冲), 正确做法:
 *   1. 先把 LAUNCH_FF_CURRENT 从 0 慢慢加大, 加到电机"刚好开始动"为止
 *      (这就是克服静摩擦所需的前馈电流, 约等于启动阈值)。
 *   2. 再设 LAUNCH_TARGET_SPEED 为你想要的速度, LAUNCH_LOOP_KP 从小往大调,
 *      直到能稳住、又不会抖 (抖就减小)。
 *   3. 前馈负责"顶开摩擦", 反馈负责"稳速", 两者配合才能稳定低速。
 *
 * 如果怎么调都稳不住 (不动 <-> 冲出去 来回跳), 说明机构 stick-slip 太强,
 * 需要从机械上解决 (润滑/对中/加阻尼), 代码层面无解。
 * =================================================================
 *
 * 硬件: GIMBAL_BOARD (STM32F407xx), CAN1, M3508 ID=2
 */

#ifndef MOTOR_TEST_CONFIG_H
#define MOTOR_TEST_CONFIG_H

/* ================= 硬件 ================= */
#define LAUNCH_CAN_BUS         &hcan1   /* 挂在哪条 CAN */
#define LAUNCH_MOTOR_ID        2        /* 电调闪动次数 (1-8) */
#define LAUNCH_MOTOR_REVERSE   MOTOR_DIRECTION_NORMAL  /* 方向不对改成 MOTOR_DIRECTION_REVERSE */

/* ================= 控制模式 ================= */
/* 1 = 速度环 + 电流前馈 (推荐, 可调稳定低速)
 * 0 = 纯开环电流 (最简单, 只能停在"蠕动区"或"冲出去") */
#define LAUNCH_USE_SPEED_LOOP  1

/* ================= 速度环参数 ================= */
#define LAUNCH_TARGET_SPEED    200.0f    /* 目标转速 deg/s (转子, 3508 减速比约 19:1) */

/* 电流前馈: 用来顶掉静摩擦。先给 0, 一点点加, 加到刚好能转动 */
#define LAUNCH_FF_CURRENT      900.0f

/* 速度环增益: 只做小幅修正, 所以 Kp 要小; 抖就减小, 顶不动就加大前馈 */
#define LAUNCH_LOOP_KP         0.3f

/* 积分: 一般不用 (摩擦已由前馈补掉)。
 * 只有当"速度总是稳定偏低、离目标差一截"时才给一点, 从 0.1 试, 大了会抖/积分饱和 */
#define LAUNCH_LOOP_KI         0.0f

/* 微分: 基本不用。速度反馈有噪声, Kd 会放大噪声导致抖 */
#define LAUNCH_LOOP_KD         0.0f

/* 速度环输出限幅: 叠在前馈上的修正量上限 (电流单位, 16384≈20A) */
#define LAUNCH_LOOP_MAXOUT     300.0f
#define LAUNCH_LOOP_ILIMIT     100.0f   /* 积分限幅 */

/* 纯开环模式下使用的电流 (仅当 LAUNCH_USE_SPEED_LOOP=0 时有效) */
#define LAUNCH_OPENLOOP_CURRENT 0.0f

/* ================= 安全 ================= */
#define LAUNCH_SPEED_LIMIT     300.0f   /* 看门狗: |转速| 超过它就停机 (deg/s) */
#define LAUNCH_START_DELAY_MS  3000u    /* 上电先静止这么久再动 */
#define LAUNCH_RAMP_MS         2000u    /* 目标转速软启动时间 */

#endif  // MOTOR_TEST_CONFIG_H









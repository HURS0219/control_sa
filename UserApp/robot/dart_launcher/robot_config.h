/*
 * robot_config.h — 制导飞镖发射架 (dart_launcher) v0.1 全部可调/需实测参数
 *
 * !!! 所有标注 TODO 的参数必须在实物上单独测试后填入 !!!
 *
 * 机械/电气依据 (RM2026 凌Bug 开源技术报告):
 *   - 蓄力: 2 x M3508 (P19, 减速比 3591/187) 驱动同步带, 拉动发射滑台压缩拉簧
 *   - 拉簧: 线径3.5mm/中径30mm/55圈 共4根, 总刚度约 4006.7 N/m, 行程约 200mm
 *   - 释放: PWM 舵机拨动扳机 (F407: PE9 = TIM1_CH1, 50Hz)
 *   - Yaw : 1 x M2006 (P36, 减速比36) + 丝杆平移前轴承改变射出方向
 *
 * 注意: dji_motor 的 measure.total_angle 为【电机转子角度(deg)】,
 *       换算到输出轴需除以减速比。下面 *_DEG_PER_MM 等参数按转子角度定义。
 */
#ifndef DART_LAUNCHER_ROBOT_CONFIG_H
#define DART_LAUNCHER_ROBOT_CONFIG_H

#define DART_LAUNCHER_VERSION "0.1"

#include "main.h"
#include "tim.h"
#include "motor_def.h"
#include "controller.h"

/* ================= CAN / 电机 ID ================= */
#define LAUNCHER_CAN_HANDLE (&hcan1)  // TODO 按实际接线确认 CAN1/CAN2

#define CHARGE_MASTER_ID 1  // TODO 3508 主(蓄力同步带)
#define CHARGE_SLAVE_ID  2  // TODO 3508 从(蓄力同步带)
#define AIM_MOTOR_ID     3  // TODO 2006 (yaw 丝杆)

/* 两个 3508 若镜像安装, 其中一个需要 MOTOR_DIRECTION_REVERSE */
#define CHARGE_MASTER_REVERSE MOTOR_DIRECTION_NORMAL
#define CHARGE_SLAVE_REVERSE  MOTOR_DIRECTION_REVERSE
#define AIM_MOTOR_REVERSE     MOTOR_DIRECTION_NORMAL

/* ================= 机械换算 (TODO 全部实测) ================= */
/* 蓄力: 滑台每拉伸 1mm 对应的电机转子角度 (deg)。
 * 计算: 360 * 减速比 / 同步轮节圆周长(mm)。减速比 = 3591/187 = 19.2032 */
#define M3508_GEAR_RATIO      19.2032f
#define CHARGE_DEG_PER_MM     100.0f  // TODO 实测标定
#define CHARGE_DRAW_SIGN      1.0f    // TODO 正=拉簧拉伸方向
#define CHARGE_MIN_DRAW_MM    0.0f
#define CHARGE_MAX_DRAW_MM    200.0f  // 报告行程 200mm

/* Yaw: 2006 转子每 1deg 对应的射出方向 yaw 变化 (deg)。丝杆->前轴承->几何关系 */
#define AIM_DEG_PER_DEG       0.01f  // TODO 实测标定
#define AIM_MIN_DEG          (-30.0f)
#define AIM_MAX_DEG           (30.0f)

/* ================= 拉簧能量 (能量环) ================= */
/* E = F0*x + 0.5*k*x^2 ; 由目标能量反解拉伸量 x */
#define SPRING_K_TOTAL_N_PER_M 4006.7f  // TODO 4根合计刚度 (报告 k=1001.68/根)
#define SPRING_PRELOAD_N       0.0f     // TODO 预紧力 F0
#define CHARGE_DEFAULT_ENERGY_J 45.0f   // TODO 默认发射能量

/* ================= 运动限幅 ================= */
#define CHARGE_MAX_SPEED       8000.0f  // 蓄力最大转子速度 (deg/s)
#define CHARGE_CALI_SPEED      3000.0f  // 校准找限位速度
#define AIM_MAX_SPEED          6000.0f  // yaw 最大转子速度 (deg/s)
#define AIM_CALI_SPEED         3000.0f  // yaw 校准速度
#define CHARGE_CALI_DIR       (-1.0f)   // 找零方向 (负=向零点/收回)
#define AIM_CALI_DIR          (-1.0f)

/* ================= 校准/到位判据 ================= */
#define CALI_BACK_MM           2.0f     // 找零后回退量
#define CALI_POS_THRESHOLD     500.0f   // 到位角度阈值 (转子deg)
#define CALI_SPEED_THRESHOLD   50.0f    // 到位速度阈值 (deg/s)
#define CALI_TIMEOUT_MS        4000u    // 单步超时保护

/* ================= 扳机舵机 (PWM) ================= */
#define TRIGGER_PWM_HANDLE     (&htim1)
#define TRIGGER_PWM_CHANNEL    TIM_CHANNEL_1  // PE9
#define TRIGGER_PWM_PERIOD_S   0.02f          // 50Hz
#define TRIGGER_LOCK_US        1000.0f  // TODO 锁止脉宽
#define TRIGGER_RELEASE_US     2000.0f  // TODO 释放脉宽
#define TRIGGER_RELEASE_HOLD_MS 150u    // 释放保持时间
#define TRIGGER_SETTLE_MS      400u     // 发射后等待机构复位

/* ================= 发射时序 ================= */
#define AUTO_CHARGE_SETTLE_MS  300u     // 到位后稳定时间

/* ================= 安全 ================= */
#define LAUNCHER_ESTOP_ON_RC_LOST 1  // 遥控器离线自动急停(1=启用)

/* ================= 遥控器 ================= */
#define RC_DEADZONE            50
#define CHARGE_STICK_MAX_SPEED 8000.0f
#define AIM_STICK_MAX_SPEED    6000.0f

/* ================================================================= */
/*                        电机初始化配置宏                            */
/* ================================================================= */

/* 蓄力 3508: 位置+速度双环, 速度环启用堵转检测(PID_ErrorHandle)供校准使用 */
#define CHARGE_MOTOR_CONFIG(can_h, _id, _reverse)                                               \
  {                                                                                             \
      .motor_type = M3508,                                                                      \
      .can_init_config =                                                                        \
          {                                                                                     \
              .can_handle = can_h,                                                              \
              .tx_id = _id,                                                                     \
          },                                                                                    \
      .controller_setting_init_config =                                                         \
          {                                                                                     \
              .angle_feedback_source = MOTOR_FEED,                                              \
              .speed_feedback_source = MOTOR_FEED,                                              \
              .outer_loop_type = SPEED_LOOP,                                                    \
              .close_loop_type = SPEED_LOOP | ANGLE_LOOP,                                       \
              .motor_reverse_flag = _reverse,                                                   \
              .feedback_reverse_flag = _reverse,                                                \
          },                                                                                    \
      .controller_param_init_config =                                                           \
          {                                                                                     \
              .speed_PID =                                                                      \
                  {                                                                             \
                      .Kp = 5.0f,                                                               \
                      .Ki = 0.1f,                                                               \
                      .Kd = 0.0f,                                                               \
                      .MaxOut = 16000.0f,                                                       \
                      .IntegralLimit = 8000.0f,                                                 \
                      .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral | PID_ErrorHandle, \
                  },                                                                            \
              .angle_PID =                                                                      \
                  {                                                                             \
                      .Kp = 20.0f,                                                              \
                      .Ki = 0.0f,                                                               \
                      .Kd = 0.0f,                                                               \
                      .MaxOut = 20000.0f,                                                       \
                      .Improve = PID_Integral_Limit,                                            \
                  },                                                                            \
          },                                                                                    \
  }

/* Yaw 2006: 位置+速度双环, 速度环启用堵转检测 */
#define AIM_MOTOR_CONFIG(can_h, _id, _reverse)                                                  \
  {                                                                                             \
      .motor_type = M2006,                                                                      \
      .can_init_config =                                                                        \
          {                                                                                     \
              .can_handle = can_h,                                                              \
              .tx_id = _id,                                                                     \
          },                                                                                    \
      .controller_setting_init_config =                                                         \
          {                                                                                     \
              .angle_feedback_source = MOTOR_FEED,                                              \
              .speed_feedback_source = MOTOR_FEED,                                              \
              .outer_loop_type = SPEED_LOOP,                                                    \
              .close_loop_type = SPEED_LOOP | ANGLE_LOOP,                                       \
              .motor_reverse_flag = _reverse,                                                   \
              .feedback_reverse_flag = _reverse,                                                \
          },                                                                                    \
      .controller_param_init_config =                                                           \
          {                                                                                     \
              .speed_PID =                                                                      \
                  {                                                                             \
                      .Kp = 2.0f,                                                               \
                      .Ki = 0.1f,                                                               \
                      .Kd = 0.0f,                                                               \
                      .MaxOut = 10000.0f,                                                       \
                      .IntegralLimit = 3000.0f,                                                 \
                      .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral | PID_ErrorHandle, \
                  },                                                                            \
              .angle_PID =                                                                      \
                  {                                                                             \
                      .Kp = 15.0f,                                                              \
                      .Ki = 0.0f,                                                               \
                      .Kd = 0.3f,                                                               \
                      .MaxOut = 6000.0f,                                                        \
                      .Improve = PID_Integral_Limit,                                            \
                  },                                                                            \
          },                                                                                    \
  }

#endif  // DART_LAUNCHER_ROBOT_CONFIG_H

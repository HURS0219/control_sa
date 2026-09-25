/*
 * robot.c — M3508 低速控制 (带前馈 + 看门狗, 可调参)
 *
 * 两种模式 (由 LAUNCH_USE_SPEED_LOOP 选择):
 *   1) 速度环 + 电流前馈: 前馈顶开静摩擦, 速度环稳速 (推荐)
 *   2) 纯开环电流: 直接给固定电流 (简单, 但只能蠕动或冲)
 *
 * 安全:
 *   - 上电先静止 LAUNCH_START_DELAY_MS
 *   - 转速看门狗: |speed| > LAUNCH_SPEED_LIMIT 立即停机 (防冲车 / 防 bus-off)
 *   - 目标转速软启动 (LAUNCH_RAMP_MS)
 *
 * 目标板: GIMBAL_BOARD (STM32F407xx)
 * 集成: 把本目录放到 UserApp/robot/motor_test/, 根 CMakeLists 设
 *       set(ROBOT_TYPE "motor_test"), 用 GIMBAL_BOARD 编译即可。
 */

#include "robot.h"
#include "robot_config.h"
#include "user_lib.h"
#include "main.h"

#include <string.h>

RobotInstance* robot = NULL;

/* 电流前馈量: SDK 会把它叠加到速度环输出上 (feedforward_flag = CURRENT_FEEDFORWARD) */
static float g_ff_current = LAUNCH_FF_CURRENT;

void RobotInit(void) {
  robot = (RobotInstance*)zmalloc(sizeof(RobotInstance));
  memset(robot, 0, sizeof(RobotInstance));

  Motor_Init_Config_s launch_cfg = {
      .motor_type = M3508,
      .can_init_config = {.can_handle = LAUNCH_CAN_BUS, .tx_id = LAUNCH_MOTOR_ID},
      .controller_setting_init_config =
          {
              .angle_feedback_source = MOTOR_FEED,
              .speed_feedback_source = MOTOR_FEED,
              .outer_loop_type = SPEED_LOOP,
              .close_loop_type = SPEED_LOOP,
              .motor_reverse_flag = LAUNCH_MOTOR_REVERSE,
              .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
              .feedforward_flag = CURRENT_FEEDFORWARD,  /* 启用电流前馈 */
          },
      .controller_param_init_config =
          {
              .current_feedforward_ptr = &g_ff_current,  /* 指向上面那个前馈量 */
              .speed_PID = {.Kp = LAUNCH_LOOP_KP, .Ki = LAUNCH_LOOP_KI, .Kd = LAUNCH_LOOP_KD,
                            .MaxOut = LAUNCH_LOOP_MAXOUT,
                            .IntegralLimit = LAUNCH_LOOP_ILIMIT,
                            .Improve = PID_Integral_Limit},
          },
  };
  robot->launch_motor = DJIMotorInit(&launch_cfg);

  /* 上电默认停机 */
  DJIMotorStop(robot->launch_motor);
}

void RobotTask(void) {
  if (robot == NULL) return;
  DJIMotorInstance* m = robot->launch_motor;

  static uint32_t start_ms = 0;
  static uint32_t ramp_ms = 0;
  if (start_ms == 0) start_ms = HAL_GetTick();

  /* ---------- 安全: 转速看门狗 ---------- */
  if (abs(m->measure.speed_aps) > LAUNCH_SPEED_LIMIT) {
    DJIMotorStop(m);          /* 停机, 发 0 电流 */
    DJIMotorTask();
    return;
  }

  /* ---------- 上电静止 ---------- */
  if (HAL_GetTick() - start_ms < LAUNCH_START_DELAY_MS) {
    DJIMotorStop(m);
    DJIMotorTask();
    return;
  }

  DJIMotorEnable(m);

#if LAUNCH_USE_SPEED_LOOP
  /* ---------- 速度环 + 前馈 ---------- */
  if (ramp_ms == 0) ramp_ms = HAL_GetTick();
  float frac = (float)(HAL_GetTick() - ramp_ms) / (float)LAUNCH_RAMP_MS;
  if (frac > 1.0f) frac = 1.0f;

  DJIMotorOuterLoop(m, SPEED_LOOP);
  DJIMotorSetPIDRef(m, LAUNCH_TARGET_SPEED * frac);  /* 目标转速软启动 */

#else
  /* ---------- 纯开环电流 ---------- */
  DJIMotorSetRef(m, LAUNCH_OPENLOOP_CURRENT);
#endif

  DJIMotorTask();
}

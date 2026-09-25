/*
 * robot.c — 通用电机最小转动示例 (run_motor2)
 *
 * 用 simple_motor 通用驱动, 支持 GM6020 / M3508 / M2006。
 * 换电机只改下面两行:
 *   MOTOR_TYPE: SIMPLE_MOTOR_M3508 / SIMPLE_MOTOR_M2006 / SIMPLE_MOTOR_GM6020
 *   MOTOR_ID  : 电调拨码 ID
 *
 * 上电后每 3 秒在 0 / +值 / 0 / -值 之间切换, 电机来回转。
 * 想一直转: 把 SimpleMotorSet(值) 固定即可。
 * 注意: M3508/M2006 的输入是“电流”(范围 ±16384 / ±10000), GM6020 是“电压”(±30000)。
 */
#include "robot.h"

#include "main.h"
#include "simple_motor.h"
#include "user_lib.h"

/* ==================== 改这里换电机 ==================== */
#define MOTOR_TYPE SIMPLE_MOTOR_M3508   // M3508(默认) / M2006 / GM6020
#define MOTOR_ID   1                    // 电调拨码 ID
#define SPIN_VALUE 2000                 // 转动强度: M3508 建议 1000~5000, M2006 1000~4000
/* ====================================================== */

RobotInstance* robot = NULL;

void RobotInit(void) {
  robot = (RobotInstance*)zmalloc(sizeof(RobotInstance));

  SimpleMotorConfig_s cfg = {.type = MOTOR_TYPE, .id = MOTOR_ID};
  SimpleMotorInit(&cfg);
}

void RobotTask(void) {
  static uint32_t t0 = 0;
  if (t0 == 0) t0 = HAL_GetTick();

  uint32_t phase = ((HAL_GetTick() - t0) / 3000u) % 4u;
  int16_t value = (phase == 1) ? SPIN_VALUE : ((phase == 3) ? -SPIN_VALUE : 0);

  SimpleMotorSet(value);
  SimpleMotorReadFeedback();  // g_simple_motor.fb_count 会涨说明电机在线
}

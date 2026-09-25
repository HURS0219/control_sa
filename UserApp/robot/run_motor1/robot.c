/*
 * robot.c — 最简 GM6020 转动示例 (run_motor1)
 * 上电后每 3 秒在 0 / +8000 / 0 / -8000 之间切换, 电机来回转。
 * 想一直转: 固定一个值即可, 例如 RunMotorSet(8000);
 */
#include "robot.h"

#include "main.h"
#include "run_motor.h"
#include "user_lib.h"

RobotInstance* robot = NULL;

void RobotInit(void) {
  robot = (RobotInstance*)zmalloc(sizeof(RobotInstance));
  RunMotorInit();
}

void RobotTask(void) {
  static uint32_t t0 = 0;
  if (t0 == 0) t0 = HAL_GetTick();

  uint32_t phase = ((HAL_GetTick() - t0) / 3000u) % 4u;
  int16_t value = (phase == 1) ? 8000 : ((phase == 3) ? -8000 : 0);

  RunMotorSet(value);
  RunMotorReadFeedback();
}

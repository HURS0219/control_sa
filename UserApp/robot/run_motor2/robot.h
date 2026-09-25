/*
 * robot.h — 通用电机最小转动示例 (run_motor2)
 */
#pragma once

#ifndef RUN_MOTOR2_ROBOT_H
#define RUN_MOTOR2_ROBOT_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  void* motor;
} RobotInstance;

extern RobotInstance* robot;

void RobotInit(void);
void RobotTask(void);

#endif  // RUN_MOTOR2_ROBOT_H

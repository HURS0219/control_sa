/*
 * robot.h — 最简 GM6020 转动示例 (run_motor1)
 */
#pragma once

#ifndef RUN_MOTOR1_ROBOT_H
#define RUN_MOTOR1_ROBOT_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  void* motor;
} RobotInstance;

extern RobotInstance* robot;

void RobotInit(void);
void RobotTask(void);

#endif  // RUN_MOTOR1_ROBOT_H

/*
 * robot.h — 6020 + 3508 上电测试 (motor_test)
 */
#pragma once

#ifndef MOTOR_TEST_ROBOT_H
#define MOTOR_TEST_ROBOT_H

#include "remote_control.h"
#include "dji_motor.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  DJIMotorInstance* yaw_motor;    // GM6020 (id2)
  DJIMotorInstance* launch_motor; // M3508  (id2)
  RC_ctrl_t* rc_data;
} RobotInstance;

extern RobotInstance* robot;

void RobotInit(void);
void RobotTask(void);

#endif  // MOTOR_TEST_ROBOT_H

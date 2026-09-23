/*
 * robot.h — 制导飞镖发射架 · 网页控制版 (dart_launcher_web) v0.1 入口
 */
#pragma once

#ifndef DART_LAUNCHER_WEB_ROBOT_H
#define DART_LAUNCHER_WEB_ROBOT_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  void* motors;  // 指向 g_dart_params / g_dart_ctrl
} RobotInstance;

extern RobotInstance* robot;

void RobotInit(void);
void RobotTask(void);

#endif  // DART_LAUNCHER_WEB_ROBOT_H

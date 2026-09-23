/*
 * robot.h — 制导飞镖发射架 (dart_launcher) v0.1 应用层入口
 *
 * 分层:
 *   Module/Bsp 层: DJI 电机(dji_motor) / PWM(bsp_pwm) / 遥控(remote_control) / 守护(daemon)
 *   App 层(本目录): launcher(状态机) + launcher_charge / launcher_aim / launcher_fire
 *
 * 硬件依据: RM2026 大连理工大学凌Bug战队飞镖系统开源技术报告
 *   蓄力: 2 x M3508 同步带拉簧; Yaw: 1 x M2006 + 丝杆; 释放: 舵机扳机
 */
#pragma once

#ifndef DART_LAUNCHER_ROBOT_H
#define DART_LAUNCHER_ROBOT_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
  void* launcher;  // 指向 LauncherInstance, 由 robot.c 关联
} RobotInstance;

extern RobotInstance* robot;

void RobotInit(void);
void RobotTask(void);

#endif  // DART_LAUNCHER_ROBOT_H

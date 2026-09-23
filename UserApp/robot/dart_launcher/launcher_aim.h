/*
 * launcher_aim.h — Yaw 瞄准机构 (M2006 + 丝杆) 控制接口
 */
#ifndef DART_LAUNCHER_AIM_H
#define DART_LAUNCHER_AIM_H

#include "launcher.h"

void LauncherAimInit(LauncherInstance* l);

void LauncherAimSetSpeed(LauncherInstance* l, float speed);
void LauncherAimSetAngle(LauncherInstance* l, float angle);  // 目标为转子角度
/* 按射出方向 yaw 角度(deg)瞄准, 内部换算为电机角度并限幅 */
void LauncherAimSetYawDeg(LauncherInstance* l, float yaw_deg);

void LauncherAimHold(LauncherInstance* l);
void LauncherAimStop(LauncherInstance* l);

bool LauncherAimBlocked(LauncherInstance* l);
bool LauncherAimInPosition(LauncherInstance* l);

#endif  // DART_LAUNCHER_AIM_H

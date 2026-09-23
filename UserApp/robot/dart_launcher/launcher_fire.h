/*
 * launcher_fire.h — 扳机释放 (PWM 舵机) 控制接口
 */
#ifndef DART_LAUNCHER_FIRE_H
#define DART_LAUNCHER_FIRE_H

#include "launcher.h"

void LauncherTriggerInit(LauncherInstance* l);
void LauncherTriggerRelease(LauncherInstance* l);  // 释放拉簧能量
void LauncherTriggerLock(LauncherInstance* l);     // 锁止扳机

#endif  // DART_LAUNCHER_FIRE_H

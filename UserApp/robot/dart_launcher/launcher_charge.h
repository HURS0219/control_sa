/*
 * launcher_charge.h — 蓄力机构 (2 x M3508 同步带拉簧) 控制接口
 */
#ifndef DART_LAUNCHER_CHARGE_H
#define DART_LAUNCHER_CHARGE_H

#include "launcher.h"

void LauncherChargeInit(LauncherInstance* l);

/* 速度环 (手动调试/找零) */
void LauncherChargeSetSpeed(LauncherInstance* l, float speed);
/* 位置环: 相对零点的机械同向角度增量(转子deg) */
void LauncherChargeSetDelta(LauncherInstance* l, float delta);
/* 按拉伸量(mm)蓄力, 内部换算为角度并限幅 */
void LauncherChargeSetDrawMM(LauncherInstance* l, float mm);
/* 按目标能量(J)蓄力: 由 E = F0*x + 0.5*k*x^2 反解拉伸量 */
void LauncherChargeSetEnergy(LauncherInstance* l, float energy_j);

void LauncherChargeHold(LauncherInstance* l);   // 保持当前位置
void LauncherChargeStop(LauncherInstance* l);   // 断电(急停)

bool LauncherChargeBlocked(LauncherInstance* l);    // 是否堵转(找零用, 读取后自动清除)
bool LauncherChargeInPosition(LauncherInstance* l); // 是否到位
float LauncherChargeDrawMM(LauncherInstance* l);    // 当前实际拉伸量估计

#endif  // DART_LAUNCHER_CHARGE_H
